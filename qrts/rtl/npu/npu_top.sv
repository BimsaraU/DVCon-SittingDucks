// =============================================================================
// npu_top.sv - DVCon NPU: registers, descriptor sequencer, engines, DMA
//
//   jtag_ctrl --(6-bit register port)--> registers
//                                            |
//              sequencer: FETCH -> CONV / ELEM -> ... -> END
//                  |              |          |
//                  +------ npu_dma (one owner at a time) ----> Avalon -> SDRAM
//
// The host writes DESC_ADDR / IMG_ADDR / BOX_ADDR / CONF and pulses START.
// The sequencer then walks the descriptor table with no host involvement and
// raises done with the box count. Nothing overlaps: one descriptor at a time,
// one engine at a time, so DMA ownership is simply the sequencer's state.
//
// REGISTERS (word addresses, the same as the old dvcon_regs where they overlap
// so older host tools still read IDENT, STATUS and the box count)
//   0x00 CTRL       W  bit0 START (on a written 1), bit1 ABORT
//   0x01 STATUS     R  [0] busy [1] done [2] error [7:4] sequencer state
//                      [10:8] conv phase [13:11] elem phase [23:16] error code
//   0x06 DESC_ADDR  RW descriptor table (blob base + desc_off)
//   0x07 IMG_ADDR   RW the host's INT8 3 x H x W frame
//   0x08 BOX_ADDR   RW box list destination
//   0x09 CONF       RW detection threshold, signed Q8.8 logit
//   0x0A NUM_BOXES  R
//   0x0B LAYER_IDX  R  descriptor index being executed
//   0x0C IDENT      R  0xDC10_vvvv: magic, array edge 16, version
//   0x0D CYCLES     R  clock cycles of the current / last run
//   0x0E SLOTS      R  pixel slots through the array (utilisation)
//   0x0F DMA_WORDS  R  32-bit words moved to and from SDRAM
//   0x10 TAG        R  model layer of the current descriptor
//   0x11 OP         R  opcode and flags of the current descriptor
//   0x12 FLAGS      RW [0] model loaded [1] frame loaded. Set by the host after
//                      a verified transfer; reset clears them, as it does SDRAM.
//                      Shown on LEDG1 / LEDG2.
//
// ext_start (KEY3, debounced) starts a frame exactly like CTRL.START, with
// whatever DESC/IMG/BOX/CONF hold: the reset values are the standard memory
// map and a 0.25 confidence, so the button works with no host attached.
// =============================================================================
`timescale 1ns/1ps

`include "npu_pkg.svh"

module npu_top (
    input  wire         clk,
    input  wire         rst_n,

    input  wire [5:0]   reg_addr,
    input  wire         reg_read,
    input  wire         reg_write,
    input  wire [31:0]  reg_wdata,
    output reg  [31:0]  reg_rdata,
    output wire         reg_waitrequest,

    output wire [31:0]  avm_address,
    output wire         avm_read,
    output wire         avm_write,
    output wire [31:0]  avm_writedata,
    output wire [3:0]   avm_byteenable,
    output wire [6:0]   avm_burstcount,
    input  wire [31:0]  avm_readdata,
    input  wire         avm_readdatavalid,
    input  wire         avm_waitrequest,

    input  wire         ext_start,       // one-cycle pulse (KEY3)
    output wire [1:0]   o_flags,

    // observability for the LEDs and 7-segment displays
    output wire         o_busy,
    output wire [3:0]   o_state,
    output wire [7:0]   o_op,
    output wire [2:0]   o_conv_phase,
    output wire [2:0]   o_elem_phase,
    output wire [15:0]  o_tag,
    output wire [15:0]  o_idx,
    output wire [15:0]  o_boxes,
    output wire         o_array,
    output wire         o_dma_rd,
    output wire         o_dma_wr
);
    assign reg_waitrequest = 1'b0;

    // ------------------------------------------------------------ registers
    reg  [31:0] desc_addr, img_addr, box_addr;
    reg  signed [15:0] conf;
    reg  [31:0] cycles;
    reg         busy, done_f, err_f;
    reg  [7:0]  err_code;
    reg  [15:0] idx;
    reg  [511:0] desc;
    reg         wr_q;
    reg  [1:0]  flags;
    assign o_flags = flags;
    wire        start = (reg_write && !wr_q && reg_addr == 6'h00 && reg_wdata[0])
                        || ext_start;
    wire        abort = reg_write && !wr_q && reg_addr == 6'h00 && reg_wdata[1];

    wire [15:0] num_boxes;
    wire [31:0] slots, dma_words_stat;
    wire [2:0]  cph, eph;

    localparam [3:0] Q_IDLE = 4'd0, Q_FETCH = 4'd1, Q_CONV = 4'd2,
                     Q_ELEM = 4'd3, Q_DONE = 4'd4, Q_ERR = 4'd5, Q_DEC = 4'd6;
    reg  [3:0] q;

    always @(*) begin
        case (reg_addr)
        6'h01: reg_rdata = {8'd0, err_code, 2'd0, eph, cph, q, 1'b0, err_f, done_f, busy};
        6'h06: reg_rdata = desc_addr;
        6'h07: reg_rdata = img_addr;
        6'h08: reg_rdata = box_addr;
        6'h09: reg_rdata = {{16{conf[15]}}, conf};
        6'h0A: reg_rdata = {16'd0, num_boxes};
        6'h0B: reg_rdata = {16'd0, idx};
        6'h0C: reg_rdata = NPU_IDENT;
        6'h0D: reg_rdata = cycles;
        6'h0E: reg_rdata = slots;
        6'h0F: reg_rdata = dma_words_stat;
        6'h10: reg_rdata = {16'd0, desc[495:480]};
        6'h11: reg_rdata = {16'd0, desc[15:0]};
        6'h12: reg_rdata = {30'd0, flags};
        default: reg_rdata = 32'd0;
        endcase
    end

    // ------------------------------------------------------------------ DMA
    wire        d_busy, d_done, d_rvalid, d_pull;
    wire [31:0] d_rdata;
    reg         d_go, d_wr;
    reg  [31:0] d_addr, d_wdata;
    reg  [23:0] d_words;

    wire        c_go, c_wr, e_go, e_wr;
    wire [31:0] c_addr, e_addr, c_wdata, e_wdata;
    wire [23:0] c_words, e_words;
    reg         s_go;
    reg  [31:0] s_addr;

    wire own_c = (q == Q_CONV);
    wire own_e = (q == Q_ELEM);

    always @(*) begin
        if (own_c) begin
            d_go = c_go; d_wr = c_wr; d_addr = c_addr; d_words = c_words; d_wdata = c_wdata;
        end else if (own_e) begin
            d_go = e_go; d_wr = e_wr; d_addr = e_addr; d_words = e_words; d_wdata = e_wdata;
        end else begin
            d_go = s_go; d_wr = 1'b0; d_addr = s_addr; d_words = 24'd16; d_wdata = 32'd0;
        end
    end

    npu_dma #(.WR_LAT(2)) u_dma (
        .clk(clk), .rst_n(rst_n),
        .cmd_go(d_go), .cmd_wr(d_wr), .cmd_addr(d_addr), .cmd_words(d_words),
        .busy(d_busy), .done(d_done),
        .rd_data(d_rdata), .rd_valid(d_rvalid),
        .wr_pull(d_pull), .wr_data(d_wdata),
        .avm_address(avm_address), .avm_read(avm_read), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount), .avm_readdata(avm_readdata),
        .avm_readdatavalid(avm_readdatavalid), .avm_waitrequest(avm_waitrequest),
        .stat_words(dma_words_stat));

    // -------------------------------------------------------------- engines
    reg  c_start, e_start;
    wire c_busy, c_done, e_busy, e_done, c_arr;

    npu_conv u_conv (
        .clk(clk), .rst_n(rst_n), .clr(abort),
        .start(c_start), .desc(desc), .busy(c_busy), .done(c_done), .phase(cph),
        .dma_go(c_go), .dma_wr(c_wr), .dma_addr(c_addr), .dma_words(c_words),
        .dma_done(d_done && own_c), .dma_rdata(d_rdata),
        .dma_rvalid(d_rvalid && own_c), .dma_pull(d_pull && own_c),
        .dma_wdata(c_wdata), .stat_slots(slots), .array_active(c_arr));

    npu_elem u_elem (
        .clk(clk), .rst_n(rst_n), .clr(abort),
        .start(e_start), .desc(desc), .busy(e_busy), .done(e_done), .phase(eph),
        .img_addr(img_addr), .box_addr(box_addr), .conf_logit(conf),
        .box_clear(start), .num_boxes(num_boxes),
        .dma_go(e_go), .dma_wr(e_wr), .dma_addr(e_addr), .dma_words(e_words),
        .dma_done(d_done && own_e), .dma_rdata(d_rdata),
        .dma_rvalid(d_rvalid && own_e), .dma_pull(d_pull && own_e),
        .dma_wdata(e_wdata));

    // ------------------------------------------------------------ sequencer
    reg  [3:0]  fk;                      // descriptor word being fetched
    reg  [31:0] pc;
    wire [7:0]  op = desc[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            desc_addr <= 32'h0000_0100; img_addr <= 32'h0040_0000;
            box_addr <= 32'h0060_0000; conf <= -16'sd281;   // logit(0.25) in Q8.8
            flags <= 2'b00;
            q <= Q_IDLE; busy <= 1'b0; done_f <= 1'b0; err_f <= 1'b0;
            err_code <= 8'd0; idx <= 16'd0; cycles <= 32'd0; pc <= 32'd0;
            s_go <= 1'b0; s_addr <= 32'd0; fk <= 4'd0; wr_q <= 1'b0;
            c_start <= 1'b0; e_start <= 1'b0; desc <= 512'd0;
        end else begin
            wr_q    <= reg_write;
            s_go    <= 1'b0;
            c_start <= 1'b0;
            e_start <= 1'b0;
            if (busy) cycles <= cycles + 32'd1;

            if (reg_write && !wr_q) begin
                case (reg_addr)
                6'h06: desc_addr <= reg_wdata;
                6'h07: img_addr  <= reg_wdata;
                6'h08: box_addr  <= reg_wdata;
                6'h09: conf      <= reg_wdata[15:0];
                6'h12: flags     <= reg_wdata[1:0];
                default: ;
                endcase
            end

            case (q)
            Q_IDLE: if (start) begin
                busy <= 1'b1; done_f <= 1'b0; err_f <= 1'b0; err_code <= 8'd0;
                cycles <= 32'd0; idx <= 16'd0; pc <= desc_addr;
                s_addr <= desc_addr; s_go <= 1'b1; fk <= 4'd0;
                q <= Q_FETCH;
            end
            Q_FETCH: begin
                if (d_rvalid) begin
                    desc[32*fk +: 32] <= d_rdata;
                    fk <= fk + 4'd1;
                end
                if (d_done) q <= Q_DEC;
            end
            Q_DEC: begin
                if (op == OP_END) begin
                    q <= Q_DONE;
                end else if (op == OP_CONV) begin
                    c_start <= 1'b1; q <= Q_CONV;
                end else if (op >= OP_ADD && op <= OP_DETECT) begin
                    e_start <= 1'b1; q <= Q_ELEM;
                end else begin
                    err_code <= 8'd1; q <= Q_ERR;
                end
            end
            Q_CONV, Q_ELEM: if ((q == Q_CONV && c_done) || (q == Q_ELEM && e_done)) begin
                idx <= idx + 16'd1;
                pc <= pc + 32'd64;
                s_addr <= pc + 32'd64; s_go <= 1'b1; fk <= 4'd0;
                q <= Q_FETCH;
            end
            Q_DONE: begin busy <= 1'b0; done_f <= 1'b1; q <= Q_IDLE; end
            Q_ERR:  begin busy <= 1'b0; done_f <= 1'b1; err_f <= 1'b1; q <= Q_IDLE; end
            default: q <= Q_IDLE;
            endcase

            if (abort) begin
                q <= Q_ERR; err_code <= 8'd2;
            end
        end
    end

    assign o_busy = busy;
    assign o_state = q;
    assign o_op = op;
    assign o_conv_phase = cph;
    assign o_elem_phase = eph;
    assign o_tag = desc[495:480];
    assign o_idx = idx;
    assign o_boxes = num_boxes;
    assign o_array = c_arr;
    assign o_dma_rd = d_busy && !d_wr;
    assign o_dma_wr = d_busy && d_wr;
endmodule
