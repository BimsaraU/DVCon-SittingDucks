// =============================================================================
// avalon_mm_master.sv — Avalon-MM burst master behind axi4_master's handshake
//
// Drop-in replacement for the accelerator's AXI4 master. Every engine
// (yolo_conv_engine, yolo_elem_engine, yolo_detect_engine, yolo_layer_sequencer)
// talks to memory through yolo_axi_arbiter, and the arbiter talks to exactly
// one master using the small internal handshake declared at axi4_master.sv:73-97:
//
//     read :  rd_start/rd_addr/rd_len  ->  rd_data/rd_data_valid/rd_done/rd_error
//     write:  wr_start/wr_addr/wr_len/wr_data/wr_strb
//                                      ->  wr_data_ready/wr_done/wr_error
//
// Reproducing that handshake exactly is what keeps the port cheap: no engine
// file and not one line of the arbiter changes. The parts of the contract that
// are easy to get subtly wrong, and that the engines actually depend on:
//
//   * rd_len / wr_len are BEAT COUNTS (1..256), not AXI's len-1 encoding. The
//     AXI master converts on the way out; this one passes the count straight to
//     Avalon's burstcount, which is also a count.
//
//   * rd_done is a single-cycle pulse asserted the cycle AFTER the last
//     rd_data_valid, not coincident with it. yolo_conv_engine's C_WLOAD latches
//     rd_done separately from the data beats and its C_ACT_W uses the final
//     rd_data_valid on the same cycle it sees rd_done, so collapsing the two
//     loses the last beat of every burst.
//
//   * wr_data is forwarded combinationally -- there is no internal latch. The
//     caller must hold the CURRENT beat on wr_data whenever wr_data_ready is
//     high and advance to the next beat on the following cycle. Avalon's
//     waitrequest maps onto this directly: a beat is accepted exactly when
//     write && !waitrequest, which is what wr_data_ready reports.
//
//   * wr_strb is per-beat byte enables, forwarded alongside wr_data. It is not
//     decoration: the conv and elem engines write a single INT8 per beat and
//     drive the one lane they mean. Tying byteenable high here would reproduce
//     the defect where each store wrote its 7 neighbours as zero.
//
// What the move to Avalon deletes
// -------------------------------
// AXI4 forbids a burst from crossing a 4 KB boundary. Bursts here reach 1152 B
// and nothing in the tree ever split them (docs/HANDOFF.md:103-122) -- a real
// defect that simply does not exist on Avalon-MM, which has no such rule. The
// Avalon SDRAM controller does cap burstcount, so MAX_BURST clamps and splits
// instead; see the read FSM.
//
// Width note: this is a 32-bit master (DATA_WIDTH=32) to match the DE2-115
// SDRAM controller natively, so Platform Designer inserts no width adapter.
// The engines are parameterised on a 64-bit data path today, so the 64/32
// gearbox lives here rather than in any engine -- see g_gearbox.
// =============================================================================
`timescale 1ns/1ps

module avalon_mm_master #(
    parameter integer ADDR_WIDTH = 32,
    parameter integer DATA_WIDTH = 32,   // Avalon side, matches SDRAM
    parameter integer CORE_WIDTH = 64,   // engine side, matches the AXI master
    // Largest burst the SDRAM controller accepts. Bursts longer than this are
    // split transparently; the caller never sees the seam.
    parameter integer MAX_BURST  = 64
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // ---------------- Avalon-MM master ----------------
    output wire [ADDR_WIDTH-1:0]         avm_address,
    output reg                           avm_read,
    output reg                           avm_write,
    // Combinational, exactly like the AXI master's m_wdata/m_wstrb: they must
    // track wr_sub in the same cycle it selects the slice, not a cycle later.
    output wire [DATA_WIDTH-1:0]         avm_writedata,
    output wire [DATA_WIDTH/8-1:0]       avm_byteenable,
    output wire [$clog2(MAX_BURST+1)-1:0] avm_burstcount,
    input  wire [DATA_WIDTH-1:0]         avm_readdata,
    input  wire                          avm_readdatavalid,
    input  wire                          avm_waitrequest,

    // ---------------- internal handshake: READ ----------------
    input  wire                          rd_start,
    input  wire [63:0]                   rd_addr,
    input  wire [7:0]                    rd_len,        // # beats (1-256)
    output reg  [CORE_WIDTH-1:0]         rd_data,
    output reg                           rd_data_valid,
    output reg                           rd_done,
    output reg                           rd_error,

    // ---------------- internal handshake: WRITE ----------------
    input  wire                          wr_start,
    input  wire [63:0]                   wr_addr,
    input  wire [7:0]                    wr_len,        // # beats
    input  wire [CORE_WIDTH-1:0]         wr_data,       // forwarded combinationally
    input  wire [CORE_WIDTH/8-1:0]       wr_strb,       // forwarded combinationally
    output wire                          wr_data_ready, // beat accepted -> advance
    output reg                           wr_done,
    output reg                           wr_error
);

    // Avalon beats per engine beat. 64-bit engine over a 32-bit bus = 2.
    localparam integer SUB = CORE_WIDTH / DATA_WIDTH;

    // Address and burstcount are driven by whichever channel owns the bus.
    //
    // They were previously assigned from BOTH always blocks. Simulation is
    // happy with that -- the last write in the cycle wins and the read and
    // write FSMs are never active together -- but synthesis is not:
    //   Error (10028): Can't resolve multiple constant drivers for net
    //                  "avm_address[31]"
    // repeated for every bit of both buses. A net with two drivers is not a
    // mux the tool can build; it has to be written as one.
    //
    // Reads take priority only in the sense that the two never overlap: the
    // arbiter upstream grants one channel at a time, and each FSM holds its
    // request until its own done.
    reg [ADDR_WIDTH-1:0]          rd_avm_address;
    reg [$clog2(MAX_BURST+1)-1:0] rd_avm_burstcount;
    reg [ADDR_WIDTH-1:0]          wr_avm_address;
    reg [$clog2(MAX_BURST+1)-1:0] wr_avm_burstcount;

    assign avm_address    = avm_write ? wr_avm_address    : rd_avm_address;
    assign avm_burstcount = avm_write ? wr_avm_burstcount : rd_avm_burstcount;
    localparam integer SUB_AW = (SUB <= 1) ? 1 : $clog2(SUB);

    // =========================================================================
    // Read FSM
    //
    // One engine beat is SUB Avalon beats, assembled little-endian to match the
    // AXI master's 64-bit word (byte 0 in the low lane). rd_data_valid pulses
    // once per assembled ENGINE beat, not once per Avalon beat.
    // =========================================================================
    localparam [1:0] RD_IDLE = 2'd0,
                     RD_REQ  = 2'd1,
                     RD_DATA = 2'd2,
                     RD_DONE = 2'd3;

    reg [1:0]  rd_state;
    reg [15:0] rd_av_left;       // Avalon beats still to fetch for this request
    reg [15:0] rd_burst_left;    // Avalon beats still expected in this burst
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [SUB_AW-1:0] rd_sub;
    reg [CORE_WIDTH-1:0] rd_acc;

    // Avalon beats still outstanding for this request, and how many of them the
    // next burst can carry.
    wire [31:0] rd_this = (rd_av_left > MAX_BURST) ? MAX_BURST : {16'h0, rd_av_left};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state      <= RD_IDLE;
            avm_read      <= 1'b0;
            rd_avm_address    <= {ADDR_WIDTH{1'b0}};
            rd_avm_burstcount <= '0;
            rd_data       <= {CORE_WIDTH{1'b0}};
            rd_data_valid <= 1'b0;
            rd_done       <= 1'b0;
            rd_error      <= 1'b0;
            rd_av_left    <= 16'd0;
            rd_burst_left <= 16'd0;
            rd_ptr        <= {ADDR_WIDTH{1'b0}};
            rd_sub        <= '0;
            rd_acc        <= {CORE_WIDTH{1'b0}};
        end else begin
            // Both are single-cycle pulses, exactly as the AXI master drives
            // them; the engines sample them as pulses.
            rd_done       <= 1'b0;
            rd_data_valid <= 1'b0;

            case (rd_state)
            RD_IDLE: begin
                avm_read <= 1'b0;
                if (rd_start) begin
                    rd_ptr     <= rd_addr[ADDR_WIDTH-1:0];
                    rd_av_left <= ((rd_len == 8'd0) ? 16'd1 : {8'd0, rd_len})
                                  * SUB;
                    rd_sub     <= '0;
                    rd_error      <= 1'b0;
                    rd_state      <= RD_REQ;
                end
            end

            // Issue one burst. Avalon holds read+address+burstcount until
            // waitrequest drops; the address must NOT advance during the wait.
            RD_REQ: begin
                rd_avm_address    <= rd_ptr;
                rd_avm_burstcount <= rd_this[$clog2(MAX_BURST+1)-1:0];
                avm_read       <= 1'b1;
                if (avm_read && !avm_waitrequest) begin
                    avm_read      <= 1'b0;
                    rd_ptr        <= rd_ptr + (rd_this * (DATA_WIDTH/8));
                    rd_state      <= RD_DATA;

                    // THE FIRST BEAT CAN ARRIVE IN THIS VERY CYCLE.
                    //
                    // Avalon does not require readdatavalid to wait for the
                    // cycle after acceptance, and sdram_ctrl on the board
                    // returns it immediately when the row is already open.
                    // Capturing beats only in RD_DATA therefore DROPS that
                    // first beat, and because SUB=2 packs two Avalon beats
                    // into one engine word, everything after it is shifted by
                    // one 32-bit word for the rest of the burst.
                    //
                    // On hardware that made the sequencer read desc[0] out of
                    // the descriptor's WORD 1: flags (2) instead of op (1), so
                    // every layer dispatched to the elementwise engine and
                    // desc[4] (dst) came back 0, sending conv output to
                    // address 0 and over the descriptor table. sdram_model in
                    // simulation always takes an extra cycle, which is why
                    // every bench passed.
                    if (avm_readdatavalid) begin
                        rd_acc[rd_sub*DATA_WIDTH +: DATA_WIDTH] <= avm_readdata;
                        rd_burst_left <= rd_this[15:0] - 16'd1;
                        rd_av_left    <= rd_av_left - 16'd1;

                        if (SUB == 1 || rd_sub == SUB-1) begin
                            rd_sub  <= '0;
                            rd_data <= (SUB == 1)
                                     ? {{(CORE_WIDTH-DATA_WIDTH){1'b0}}, avm_readdata}
                                     : {avm_readdata,
                                        rd_acc[CORE_WIDTH-DATA_WIDTH-1:0]};
                            rd_data_valid <= 1'b1;
                        end else begin
                            rd_sub <= rd_sub + 1'b1;
                        end

                        if (rd_av_left == 16'd1)          rd_state <= RD_DONE;
                        else if (rd_this[15:0] == 16'd1)  rd_state <= RD_REQ;
                    end else begin
                        rd_burst_left <= rd_this[15:0];
                    end
                end
            end

            // readdatavalid is decoupled from waitrequest: beats arrive when
            // they arrive, and there is no ready to withhold.
            // Everything is counted in AVALON beats (av_left). The engine beat
            // is derived from rd_sub alone, so a burst boundary that lands on a
            // sub-beat boundary -- which it always does when SUB divides the
            // burst -- cannot desynchronise the two counters.
            RD_DATA: begin
                if (avm_readdatavalid) begin
                    rd_acc[rd_sub*DATA_WIDTH +: DATA_WIDTH] <= avm_readdata;
                    rd_burst_left <= rd_burst_left - 16'd1;
                    rd_av_left    <= rd_av_left - 16'd1;

                    if (SUB == 1 || rd_sub == SUB-1) begin
                        rd_sub <= '0;
                        // Present the whole engine beat this cycle. rd_acc's
                        // other lanes were written on earlier cycles, so the
                        // final lane is merged in here rather than read back.
                        rd_data <= (SUB == 1)
                                 ? {{(CORE_WIDTH-DATA_WIDTH){1'b0}}, avm_readdata}
                                 : {avm_readdata,
                                    rd_acc[CORE_WIDTH-DATA_WIDTH-1:0]};
                        rd_data_valid <= 1'b1;
                    end else begin
                        rd_sub <= rd_sub + 1'b1;
                    end

                    if (rd_av_left == 16'd1)
                        rd_state <= RD_DONE;          // whole request served
                    else if (rd_burst_left == 16'd1)
                        rd_state <= RD_REQ;           // split: fetch the rest
                end
            end

            // rd_done lands the cycle AFTER the last rd_data_valid. The engines
            // rely on that ordering; see the header.
            RD_DONE: begin
                rd_done  <= 1'b1;
                rd_state <= RD_IDLE;
            end

            default: rd_state <= RD_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Write FSM
    //
    // wr_data/wr_strb are forwarded combinationally and wr_data_ready reports
    // acceptance, so the caller's "present current beat, advance when taken"
    // loop is unchanged from the AXI master.
    //
    // With SUB > 1 one engine beat becomes SUB Avalon beats. The engine may
    // only advance after the LAST of them, so wr_data_ready is gated on the
    // final sub-beat.
    // =========================================================================
    localparam [1:0] WR_IDLE = 2'd0,
                     WR_REQ  = 2'd1,
                     WR_DATA = 2'd2,
                     WR_DONE = 2'd3;

    reg [1:0]  wr_state;
    reg [15:0] wr_beats_left;
    reg [15:0] wr_burst_left;
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [SUB_AW-1:0] wr_sub;

    wire [31:0] wr_want = wr_beats_left * SUB;
    wire [31:0] wr_this = (wr_want > MAX_BURST) ? MAX_BURST : wr_want;

    // Slice of the engine beat currently on the bus.
    wire [DATA_WIDTH-1:0]   wr_slice  = wr_data[wr_sub*DATA_WIDTH +: DATA_WIDTH];
    wire [DATA_WIDTH/8-1:0] wr_bslice = wr_strb[wr_sub*(DATA_WIDTH/8) +: DATA_WIDTH/8];

    // A beat is accepted exactly when write is asserted and the slave is not
    // stalling. Report it to the caller only on the last sub-beat, so the
    // caller advances one ENGINE beat per pulse.
    assign avm_writedata  = wr_slice;
    assign avm_byteenable = wr_bslice;

    wire beat_taken = (wr_state == WR_DATA) && avm_write && !avm_waitrequest;
    assign wr_data_ready = beat_taken && (SUB == 1 || wr_sub == SUB-1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_state      <= WR_IDLE;
            avm_write         <= 1'b0;
            wr_avm_address    <= {ADDR_WIDTH{1'b0}};
            wr_avm_burstcount <= '0;
            wr_done           <= 1'b0;
            wr_error      <= 1'b0;
            wr_beats_left <= 16'd0;
            wr_burst_left <= 16'd0;
            wr_ptr        <= {ADDR_WIDTH{1'b0}};
            wr_sub        <= '0;
        end else begin
            wr_done <= 1'b0;

            case (wr_state)
            WR_IDLE: begin
                avm_write <= 1'b0;
                if (wr_start) begin
                    wr_ptr        <= wr_addr[ADDR_WIDTH-1:0];
                    wr_beats_left <= (wr_len == 8'd0) ? 16'd1 : {8'd0, wr_len};
                    wr_sub        <= '0;
                    wr_error      <= 1'b0;
                    wr_state      <= WR_REQ;
                end
            end

            // Avalon writes present address and burstcount with the FIRST beat;
            // there is no separate address phase to wait on.
            WR_REQ: begin
                wr_avm_address    <= wr_ptr;
                wr_avm_burstcount <= wr_this[$clog2(MAX_BURST+1)-1:0];
                wr_burst_left  <= wr_this[15:0];
                wr_ptr         <= wr_ptr + (wr_this * (DATA_WIDTH/8));
                wr_state       <= WR_DATA;
            end

            WR_DATA: begin
                avm_write <= 1'b1;

                if (beat_taken) begin
                    wr_burst_left <= wr_burst_left - 16'd1;

                    if (SUB == 1 || wr_sub == SUB-1) begin
                        wr_sub        <= '0;
                        wr_beats_left <= wr_beats_left - 16'd1;
                        if (wr_beats_left == 16'd1) begin
                            avm_write <= 1'b0;
                            wr_state  <= WR_DONE;
                        end else if (wr_burst_left == 16'd1) begin
                            avm_write <= 1'b0;
                            wr_state  <= WR_REQ;
                        end
                    end else begin
                        wr_sub <= wr_sub + 1'b1;
                    end
                end
            end

            WR_DONE: begin
                wr_done  <= 1'b1;
                wr_state <= WR_IDLE;
            end

            default: wr_state <= WR_IDLE;
            endcase
        end
    end

endmodule
