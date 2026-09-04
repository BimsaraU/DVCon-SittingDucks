// =============================================================================
// eth_cmd_engine.sv — parses command frames and DMAs their payload into SDRAM
//
// This is the bulk-data path. The host sends the 2.8 MB model blob once at
// startup and a 1.2 MB image bitmap per frame; both arrive as a stream of
// WRITE_MEM frames and land directly in SDRAM with no CPU in the middle.
//
// Control does NOT come through here -- registers, START and box readback go
// over JTAG (see jtag_ctrl.sv). That split is deliberate: the JTAG cable is
// already attached for programming, and keeping control off the wire means a
// dropped Ethernet frame can never start a frame or corrupt a register. It
// also means this engine needs no register access and only one reply type.
//
// ---------------------------------------------------------------------------
// FRAME FORMAT (raw L2, ethertype 0x88B5 -- the IEEE "local experimental" type)
// ---------------------------------------------------------------------------
//   [dst 6][src 6][0x88B5]  [hdr 16]  [payload <= 1408]  [FCS]
//
// eth_mac_rx checks the FCS and filters on the destination address, but does
// NOT strip the Ethernet header -- the destination MAC, source MAC and
// ethertype are all still in the stream it emits. The offsets below count from
// the first byte of the destination MAC.
//
//   hdr = ver:1 opcode:1 flags:1 rsv:1 seq:4 addr:4 len:2 rsv2:2   (big endian)
//
//   0x01 WRITE_MEM   payload -> SDRAM at addr
//   0x06 ACK_REQ     reply with the 64-bit received bitmap
//
// A payload of 1408 bytes is a multiple of 128, so every WRITE_MEM covers
// whole SDRAM bursts and no read-modify-write is ever needed.
//
// ---------------------------------------------------------------------------
// RELIABILITY
// ---------------------------------------------------------------------------
// Raw L2 has no retransmission, and a 2.8 MB blob is ~2000 frames -- at any
// realistic loss rate some will go missing, and a model with a hole in it
// produces plausible-looking wrong detections rather than an obvious failure.
//
// So: a 64-frame window. seq[5:0] indexes a bit in `rx_bitmap`; the host sends
// up to 64 WRITE_MEMs, then an ACK_REQ, and retransmits whatever bits came back
// clear. The window advances when the host moves on (detected by seq[31:6]
// changing), which is also what clears the bitmap.
// =============================================================================
`timescale 1ns/1ps

module eth_cmd_engine #(
    // Locally administered unicast (bit 1 of the first octet set, bit 0 clear),
    // so it cannot collide with a real vendor's assignment.
    parameter [47:0] MAC_ADDR  = 48'h02_00_00_D0_C0_01,
    parameter [15:0] ETHERTYPE = 16'h88B5
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- from eth_mac_rx (FCS checked, MAC filtered, header stripped) ----
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    input  wire        rx_sof,
    input  wire        rx_eof,
    input  wire        rx_err,

    // ---- to eth_mac_tx: ACK replies ----
    output reg  [7:0]  tx_data,
    output reg         tx_valid,
    output reg         tx_last,
    input  wire        tx_ready,

    // ---- Avalon-MM master into SDRAM ----
    output reg  [31:0] avm_address,
    output reg         avm_write,
    output reg  [31:0] avm_writedata,
    output reg  [3:0]  avm_byteenable,
    output reg  [6:0]  avm_burstcount,
    input  wire        avm_waitrequest,

    // ---- status, for the LEDs and for JTAG to read ----
    output reg  [31:0] stat_frames,
    output reg  [31:0] stat_bytes,
    output reg  [63:0] rx_bitmap,
    // Words the write queue could not accept. MUST be 0 after a load; any
    // other value means the model in SDRAM has holes in it. See the queue
    // below for why this counter exists at all.
    output reg  [31:0] stat_wdrop
);

    // Header byte offsets, counted from the FIRST byte the MAC emits.
    //
    // eth_mac_rx does NOT strip the Ethernet header: it checks the FCS and
    // filters on the destination address, but the destination MAC, source MAC
    // and ethertype are all still in the stream (see the comment in its R_DA
    // state). These offsets were originally written as if the MAC had been
    // stripped, putting every field 12 bytes early -- the frame passed its FCS
    // check and was then parsed as garbage, so it was silently dropped with no
    // error anywhere.
    //
    //   0..5   destination MAC (already filtered)
    //   6..11  source MAC
    //   12..13 ethertype
    //   14..   our 16-byte command header
    //
    // Verified against the MAC's actual output stream (tb_eth_path traces it):
    // ethertype 88 b5 at 12..13, opcode at 15, seq at 18..21, addr at 22..25,
    // len at 26..27, first payload byte at 30.
    localparam integer O_ETYPE  = 12;  // 2 bytes
    localparam integer O_VER    = 14;
    localparam integer O_OPCODE = 15;
    localparam integer O_FLAGS  = 16;
    localparam integer O_SEQ    = 18;  // 4 bytes
    localparam integer O_ADDR   = 22;  // 4 bytes
    localparam integer O_LEN    = 26;  // 2 bytes
    localparam integer O_PAYLOAD= 30;

    localparam [7:0] OP_WRITE_MEM = 8'h01,
                     OP_ACK_REQ   = 8'h06,
                     OP_ACK       = 8'h10;

    localparam [2:0] S_IDLE = 3'd0,
                     S_HDR  = 3'd1,
                     S_PAY  = 3'd2,
                     S_DRAIN= 3'd3,
                     S_ACK  = 3'd4;

    reg [2:0]  state;
    reg [10:0] byte_i;          // position within the frame

    reg [15:0] h_etype;
    reg [7:0]  h_opcode;
    reg [31:0] h_seq;
    reg [31:0] h_addr;
    reg [15:0] h_len;

    reg [25:0] window;          // seq[31:6] of the current window

    // Payload is packed into 32-bit words before being written, so the SDRAM
    // sees whole words rather than four byte-enabled writes per word.
    reg [31:0] pack;
    reg [1:0]  pack_n;
    reg [31:0] wr_ptr;
    reg [15:0] pay_left;

    reg [3:0]  ack_i;

    // =========================================================================
    // Write queue
    // =========================================================================
    // The receive side cannot be stalled. Bytes arrive from the PHY on their
    // own clock and there is no backpressure to apply, so a packed word has to
    // go SOMEWHERE the moment it is complete.
    //
    // This engine used to drive avm_address/avm_writedata/avm_write directly
    // from the packer. When the SDRAM was busy -- an accelerator burst holding
    // the arbiter, or, far more often, a refresh -- waitrequest stayed high and
    // the previous write was still pending, so the next completed word simply
    // OVERWROTE it. The pending word was never committed and wr_ptr had already
    // advanced past its address, leaving a four-byte hole in memory with no
    // error reported anywhere.
    //
    // Nothing catches that downstream. The frame's FCS was good, so its bit is
    // set in rx_bitmap and the host never retransmits it; stat_bytes counts
    // bytes received, not words committed. The result is a model blob in SDRAM
    // that is intact except for scattered stale words -- which is why loading
    // over JTAG (one word at a time, each one waited on) worked while loading
    // the same file over Ethernet did not.
    //
    // A queue decouples the two rates. At 100 Mbit the packer completes a word
    // roughly every 32 clk cycles; a single-beat SDRAM write costs ~15 and a
    // refresh ~10 more, so 16 entries is far deeper than any real stall. It is
    // still a bounded queue, so stat_wdrop counts what it could not take rather
    // than letting the failure go silent a second time.
    localparam integer WQ_AW = 4;                 // 16 entries
    localparam integer WQ_N  = 1 << WQ_AW;

    // {address[31:0], data[31:0], byteenable[3:0]}
    reg [67:0]     wq [0:WQ_N-1];
    reg [WQ_AW:0]  wq_wp, wq_rp;                  // one extra bit: full vs empty

    wire wq_empty = (wq_wp == wq_rp);
    wire wq_full  = (wq_wp[WQ_AW-1:0] == wq_rp[WQ_AW-1:0]) &&
                    (wq_wp[WQ_AW]     != wq_rp[WQ_AW]);

    // The head can be loaded onto the bus when nothing is in flight, or when
    // the beat in flight is being accepted this very cycle.
    wire wq_pop = !wq_empty && (!avm_write || !avm_waitrequest);
    wire [67:0] wq_head = wq[wq_rp[WQ_AW-1:0]];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            byte_i      <= 11'd0;
            h_etype     <= 16'h0;
            h_opcode    <= 8'h0;
            h_seq       <= 32'h0;
            h_addr      <= 32'h0;
            h_len       <= 16'h0;
            window      <= 26'h0;
            pack        <= 32'h0;
            pack_n      <= 2'd0;
            wr_ptr      <= 32'h0;
            pay_left    <= 16'h0;
            ack_i       <= 4'd0;
            avm_address <= 32'h0;
            avm_write   <= 1'b0;
            avm_writedata <= 32'h0;
            avm_byteenable<= 4'hF;
            avm_burstcount<= 7'd1;
            tx_data     <= 8'h0;
            tx_valid    <= 1'b0;
            tx_last     <= 1'b0;
            stat_frames <= 32'h0;
            stat_bytes  <= 32'h0;
            stat_wdrop  <= 32'h0;
            rx_bitmap   <= 64'h0;
            wq_wp       <= '0;
            wq_rp       <= '0;
        end else begin
            // ---- Avalon write issue, driven only from the queue -------------
            // Retire an accepted beat, then load the next one. Order matters:
            // the load below overrides this deassertion in the same cycle, so a
            // full queue issues back-to-back beats with no idle cycle between.
            if (avm_write && !avm_waitrequest) avm_write <= 1'b0;

            if (wq_pop) begin
                avm_address    <= wq_head[67:36];
                avm_writedata  <= wq_head[35:4];
                avm_byteenable <= wq_head[3:0];
                avm_burstcount <= 7'd1;
                avm_write      <= 1'b1;
                wq_rp          <= wq_rp + 1'b1;
            end

            if (tx_valid && tx_ready) begin
                tx_valid <= 1'b0;
                tx_last  <= 1'b0;
            end

            // A bad FCS aborts whatever was in flight. The frame is dropped
            // silently: the host will find the gap in the ACK bitmap and
            // retransmit, which is cheaper than reporting it.
            if (rx_err) begin
                state  <= S_IDLE;
                byte_i <= 11'd0;
            end else if (rx_valid) begin
                if (rx_sof) byte_i <= 11'd1;
                else        byte_i <= byte_i + 1'b1;

                case (state)
                // The first byte is the destination MAC's leading octet, not
                // the ethertype -- the MAC filters on the address but leaves
                // it in the stream. Just start counting.
                S_IDLE: if (rx_sof) state <= S_HDR;

                S_HDR: begin
                    case (byte_i)
                    O_ETYPE  : h_etype[15:8] <= rx_data;
                    O_ETYPE+1: h_etype[7:0]  <= rx_data;
                    O_OPCODE : h_opcode <= rx_data;
                    O_SEQ    : h_seq[31:24] <= rx_data;
                    O_SEQ+1  : h_seq[23:16] <= rx_data;
                    O_SEQ+2  : h_seq[15:8]  <= rx_data;
                    O_SEQ+3  : h_seq[7:0]   <= rx_data;
                    O_ADDR   : h_addr[31:24] <= rx_data;
                    O_ADDR+1 : h_addr[23:16] <= rx_data;
                    O_ADDR+2 : h_addr[15:8]  <= rx_data;
                    O_ADDR+3 : h_addr[7:0]   <= rx_data;
                    O_LEN    : h_len[15:8]   <= rx_data;
                    O_LEN+1  : h_len[7:0] <= rx_data;

                    // The header ends with two reserved bytes (rsv2) at
                    // O_LEN+2 and O_LEN+3. The dispatch happens on the LAST of
                    // them, not on the length field: transitioning to S_PAY at
                    // O_LEN+1 makes the engine treat those two reserved bytes
                    // as the first two payload bytes, so everything written to
                    // SDRAM is shifted two bytes late.
                    O_LEN+3  : begin
                        // Header complete. Anything not addressed to this
                        // protocol is dropped rather than guessed at.
                        if (h_etype != ETHERTYPE) begin
                            state <= S_DRAIN;
                        end else begin
                            // A new window clears the bitmap. Doing it here,
                            // rather than on ACK_REQ, means a retransmission
                            // inside the same window still sets its bit.
                            if (h_seq[31:6] != window) begin
                                window    <= h_seq[31:6];
                                rx_bitmap <= 64'h0;
                            end

                            case (h_opcode)
                            OP_WRITE_MEM: begin
                                wr_ptr   <= h_addr;
                                // h_len is fully latched by now -- the low byte
                                // arrived two cycles ago.
                                pay_left <= h_len;
                                pack_n   <= 2'd0;
                                state    <= S_PAY;
                            end
                            OP_ACK_REQ: begin
                                ack_i <= 4'd0;
                                state <= S_ACK;
                            end
                            default: state <= S_DRAIN;
                            endcase
                        end
                    end
                    default: ;
                    endcase
                end

                // ---- payload -> SDRAM ---------------------------------------
                // Bytes are packed little-endian into 32-bit words: the host
                // writes a byte array and the accelerator reads bytes out of
                // SDRAM, so the byte at address N must land in lane N%4.
                S_PAY: begin
                    if (pay_left != 0) begin
                        case (pack_n)
                        2'd0: pack[7:0]   <= rx_data;
                        2'd1: pack[15:8]  <= rx_data;
                        2'd2: pack[23:16] <= rx_data;
                        2'd3: pack[31:24] <= rx_data;
                        endcase
                        pay_left <= pay_left - 1'b1;

                        if (pack_n == 2'd3) begin
                            // Into the queue, never straight onto the bus --
                            // the bus may still be retiring the previous word.
                            if (!wq_full) begin
                                wq[wq_wp[WQ_AW-1:0]] <=
                                    {wr_ptr, {rx_data, pack[23:0]}, 4'hF};
                                wq_wp <= wq_wp + 1'b1;
                            end else begin
                                stat_wdrop <= stat_wdrop + 1'b1;
                            end
                            wr_ptr        <= wr_ptr + 32'd4;
                            pack_n        <= 2'd0;
                        end else begin
                            pack_n <= pack_n + 1'b1;
                        end

                        stat_bytes <= stat_bytes + 1'b1;
                    end
                end

                default: ;   // S_DRAIN, S_ACK: consume to end of frame
                endcase

            end

            // ---- end of frame -----------------------------------------------
            // Handled OUTSIDE the rx_valid branch, because eth_mac_rx asserts
            // out_eof on its own cycle AFTER the last data byte, with out_valid
            // low -- it is a boundary pulse, not a flag riding with the byte.
            // Handling it inside `if (rx_valid)` means it is never seen at all:
            // no payload tail is flushed, no seq bit is set, and no ACK is ever
            // sent. This bench-visible interface detail is exactly what an
            // engine tested against its own idealised stimulus gets wrong.
            //
            // Because EOF is its own cycle, every payload byte -- including the
            // last -- is already latched in `pack`, so the flush writes `pack`
            // directly rather than having to merge an arriving byte.
            if (rx_eof) begin
                if (state == S_PAY && pack_n != 2'd0) begin
                    if (!wq_full) begin
                        wq[wq_wp[WQ_AW-1:0]] <=
                            {wr_ptr, pack, (pack_n == 2'd1) ? 4'h1 :
                                           (pack_n == 2'd2) ? 4'h3 : 4'h7};
                        wq_wp <= wq_wp + 1'b1;
                    end else begin
                        stat_wdrop <= stat_wdrop + 1'b1;
                    end
                end

                if (state == S_PAY) begin
                    rx_bitmap[h_seq[5:0]] <= 1'b1;
                    stat_frames <= stat_frames + 1'b1;
                end

                // An ACK_REQ carries no payload, so the engine is already in
                // S_ACK by the time EOF arrives; leave it there so the reply
                // can be sent, and reset everything else.
                if (state != S_ACK) begin
                    state  <= S_IDLE;
                    byte_i <= 11'd0;
                end
            end

            if (state == S_ACK && !rx_valid) begin
                // ---- ACK reply ----------------------------------------------
                // 8 bytes of bitmap, MSB first, sent once the frame has ended.
                if (!tx_valid) begin
                    tx_data  <= rx_bitmap[8*(7-ack_i) +: 8];
                    tx_valid <= 1'b1;
                    tx_last  <= (ack_i == 4'd7);
                    if (ack_i == 4'd7) begin
                        state  <= S_IDLE;
                        byte_i <= 11'd0;
                    end else begin
                        ack_i <= ack_i + 1'b1;
                    end
                end
            end
        end
    end

endmodule
