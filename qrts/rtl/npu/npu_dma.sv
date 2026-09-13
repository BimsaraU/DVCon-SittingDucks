// =============================================================================
// npu_dma.sv - the NPU's only path to SDRAM
//
// One command at a time: read or write a run of 32-bit words. The run is
// split into Avalon bursts of at most 64 words that never cross a 256-byte
// boundary, so a burst also never crosses sdram_ctrl's 4 KB row (its column
// counter wraps inside the open row rather than moving to the next one).
//
// READ   words stream out on rd_data/rd_valid in address order, one per
//        cycle as SDRAM returns them. There is no backpressure: every client
//        writes them straight into on-chip RAM.
//
// WRITE  the DMA pulls words with wr_pull and the client presents each one
//        exactly WR_LAT cycles later. A whole burst is gathered in the FIFO
//        before it is issued, because sdram_ctrl takes one beat per cycle for
//        the whole burst and has no notion of a bubble in the middle of one.
// =============================================================================
`timescale 1ns/1ps

module npu_dma #(
    parameter integer WR_LAT = 2
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        cmd_go,          // pulse; only while !busy
    input  wire        cmd_wr,
    input  wire [31:0] cmd_addr,        // byte address, word aligned
    input  wire [23:0] cmd_words,
    output reg         busy,
    output reg         done,            // one-cycle pulse

    output reg  [31:0] rd_data,
    output reg         rd_valid,

    output wire        wr_pull,
    input  wire [31:0] wr_data,

    output reg  [31:0] avm_address,
    output reg         avm_read,
    output reg         avm_write,
    output wire [31:0] avm_writedata,
    output wire [3:0]  avm_byteenable,
    output reg  [6:0]  avm_burstcount,
    input  wire [31:0] avm_readdata,
    input  wire        avm_readdatavalid,
    input  wire        avm_waitrequest,

    output reg  [31:0] stat_words      // words moved since reset
);
    assign avm_byteenable = 4'hF;

    reg        is_wr;
    reg [31:0] req_addr;               // next burst address
    reg [23:0] req_left;               // words not yet requested / sent
    reg [23:0] ret_left;               // read words not yet returned

    // words to the end of the current 64-word block
    wire [6:0]  space   = 7'd64 - {1'b0, req_addr[7:2]};
    wire [6:0]  blen    = (req_left < {17'd0, space}) ? req_left[6:0] : space;

    // ---------------------------------------------------------------- FIFO
    reg  [6:0]  f_wp, f_rp;
    reg  [7:0]  fill;
    reg  [23:0] pull_left;
    reg  [7:0]  pend;                  // pulled, not yet arrived
    reg  [WR_LAT-1:0] pv;
    wire        push = pv[WR_LAT-1];
    reg  [6:0]  beats;                 // beats left in the burst in flight
    wire        pop  = avm_write && !avm_waitrequest;
    wire [6:0]  f_ra = pop ? f_rp + 7'd1 : f_rp;

    assign wr_pull = busy && is_wr && (pull_left != 0) &&
                     ({1'b0, fill} + {1'b0, pend} < 9'd120);

    npu_ram #(.W(32), .D(128)) u_fifo (
        .clk(clk), .we(push), .waddr(f_wp), .wdata(wr_data),
        .raddr(f_ra), .q(avm_writedata)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; done <= 1'b0; is_wr <= 1'b0;
            req_addr <= 32'd0; req_left <= 24'd0; ret_left <= 24'd0;
            avm_address <= 32'd0; avm_read <= 1'b0; avm_write <= 1'b0;
            avm_burstcount <= 7'd0;
            rd_data <= 32'd0; rd_valid <= 1'b0;
            f_wp <= 7'd0; f_rp <= 7'd0; fill <= 8'd0;
            pull_left <= 24'd0; pend <= 8'd0; pv <= {WR_LAT{1'b0}};
            beats <= 7'd0; stat_words <= 32'd0;
        end else begin
            done     <= 1'b0;
            rd_valid <= 1'b0;

            // ---- write FIFO bookkeeping ----
            pv <= {pv[WR_LAT-2:0], wr_pull};
            if (push) f_wp <= f_wp + 7'd1;
            if (pop)  f_rp <= f_rp + 7'd1;
            fill <= fill + {7'd0, push} - {7'd0, pop};
            pend <= pend + {7'd0, wr_pull} - {7'd0, push};
            if (wr_pull) pull_left <= pull_left - 24'd1;

            if (cmd_go && !busy) begin
                busy      <= 1'b1;
                is_wr     <= cmd_wr;
                req_addr  <= cmd_addr;
                req_left  <= cmd_words;
                ret_left  <= cmd_words;
                pull_left <= cmd_wr ? cmd_words : 24'd0;
                if (cmd_words == 24'd0) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end else if (busy && !is_wr) begin
                // ---- read: issue bursts back to back ----
                if (avm_read && !avm_waitrequest)
                    avm_read <= 1'b0;
                else if (!avm_read && req_left != 0) begin
                    avm_read       <= 1'b1;
                    avm_address    <= req_addr;
                    avm_burstcount <= blen;
                    req_addr       <= req_addr + {23'd0, blen, 2'b00};
                    req_left       <= req_left - {17'd0, blen};
                end
                if (avm_readdatavalid) begin
                    rd_data    <= avm_readdata;
                    rd_valid   <= 1'b1;
                    stat_words <= stat_words + 32'd1;
                    ret_left   <= ret_left - 24'd1;
                    if (ret_left == 24'd1) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                    end
                end
            end else if (busy && is_wr) begin
                // ---- write: one gathered burst at a time ----
                if (!avm_write) begin
                    if (req_left != 0 && {1'b0, fill} >= {2'b0, blen}) begin
                        avm_write      <= 1'b1;
                        avm_address    <= req_addr;
                        avm_burstcount <= blen;
                        beats          <= blen;
                    end else if (req_left == 0) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                    end
                end else if (!avm_waitrequest) begin
                    stat_words <= stat_words + 32'd1;
                    if (beats == 7'd1) begin
                        avm_write <= 1'b0;
                        req_addr  <= req_addr + {23'd0, avm_burstcount, 2'b00};
                        req_left  <= req_left - {17'd0, avm_burstcount};
                    end
                    beats <= beats - 7'd1;
                end
            end
        end
    end
endmodule
