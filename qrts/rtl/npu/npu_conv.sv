// =============================================================================
// npu_conv.sv - convolution engine: band buffers, systolic array, vector unit
//
// One CONV descriptor (dense, depthwise, or a matmul whose weights are a
// runtime tensor) runs as
//
//   load LUT                                      130 x {icpt, slope}
//   [RES] load every weight tile of the layer
//   for each band of `rows` output rows:
//       load the input rows the band needs, all channel blocks -> IBUF
//       for each output-channel tile t (16 channels):
//           [!RES] load tile t's weights                        -> WBUF
//           load tile t's 16 {bias, mult, shift}
//           stream nchunks passes of P pixels through the array
//               pass j multiplies reduction chunk j (16 channels of one tap)
//               partial sums accumulate per pixel in ACC (one RAM per column)
//               the last pass goes through the VPU instead          -> OUTB
//           write OUTB (P pixels x 16 channels, contiguous)         -> SDRAM
//
// TIMING OF ONE SLOT (cycle numbers relative to the slot generator, S0)
//   S0   address + control computed, IBUF address registered
//   S1   IBUF data -> padding mux -> av (registered)
//   S2   row 0 enters the array; row r enters at S2 + r (skew registers)
//   S2+16+c   column c's partial sum leaves the array
//   S2+15+c   ACC column c read address        S2+16+c  sum with bias / ACC
//   S2+17+c   ACC write-back, or VPU stage 1 on the last pass
//   S2+22+c   OUTB write of column c
// Every column has its own ACC, LUT and OUTB RAM, so the skew never needs
// undoing: each column simply taps the shared control delay line later.
//
// Passes run back to back. A tag on the last slot of a pass swaps the shadow
// weights in as that slot moves through each PE (see npu_array.sv); the next
// tile is loaded into the shadow once the tag has cleared PE[15][15], which is
// why a pass is padded with bubbles to at least P_MIN = 48 slots.
// =============================================================================
`timescale 1ns/1ps

`include "npu_pkg.svh"

module npu_conv (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         clr,

    input  wire         start,
    input  wire [511:0] desc,
    output reg          busy,
    output reg          done,
    output reg  [2:0]   phase,          // 0 idle 1 lut/params 2 input 3 weights 4 run 5 store

    output reg          dma_go,
    output reg          dma_wr,
    output reg  [31:0]  dma_addr,
    output reg  [23:0]  dma_words,
    input  wire         dma_done,
    input  wire [31:0]  dma_rdata,
    input  wire         dma_rvalid,
    input  wire         dma_pull,
    output reg  [31:0]  dma_wdata,

    output reg  [31:0]  stat_slots,     // valid slots through the array
    output wire         array_active
);
    localparam integer L = NPU_LANES;

    // ------------------------------------------------------------ descriptor
    wire [31:0] dw [0:15];
    genvar gi;
    generate for (gi = 0; gi < 16; gi = gi + 1) begin : g_dw
        assign dw[gi] = desc[32*gi +: 32];
    end endgenerate

    reg  [7:0]  g_flags;
    reg  [31:0] g_src0, g_src1, g_dst, g_wgt, g_qp, g_lut;
    reg  [15:0] g_in_w, g_in_h, g_in_cb, g_out_cb, g_out_w, g_out_h;
    reg  [3:0]  g_k, g_s, g_pad;
    reg  [4:0]  g_psh;
    reg  [11:0] g_rows;
    reg  [15:0] g_nch, g_rtn;
    reg  [7:0]  g_padv;

    wire f_dw  = g_flags[FB_DW];
    wire f_rt  = g_flags[FB_RT];
    wire f_tr  = g_flags[FB_TR];
    wire f_res = g_flags[FB_RES];

    // ------------------------------------------------------ shared multiplier
    reg  [15:0] mul_a, mul_b;
    wire [31:0] mul_q = mul_a * mul_b;

    // ---------------------------------------------------------- layer values
    reg  [31:0] plane_in, plane_out;     // bytes per channel block
    reg  [15:0] rin, cbs, rows_s;        // band rows in, IBUF rows per block
    wire [31:0] tile_b   = {8'd0, g_nch, 8'd0};      // nch * 256
    wire [15:0] wrows_t  = {g_nch[11:0], 4'd0};      // nch * 16
    wire [31:0] rt_j_step = f_tr ? {12'd0, g_rtn, 4'd0} : 32'd256;
    wire [31:0] rt_t_step = f_tr ? 32'd256 : {12'd0, g_rtn, 4'd0};

    // ----------------------------------------------------------- band values
    reg  [15:0] oy0;
    reg  signed [16:0] iy_start;
    reg  [15:0] rows_b, rin_b;
    reg  [15:0] iy_lo, iy_hi, nr;
    reg  [15:0] P, Pe;
    reg  [31:0] words_cb, dram_off, out_off;
    reg  [15:0] ibuf_row0;

    // ------------------------------------------------------------ loop state
    reg  [15:0] cb;                      // input block being loaded
    reg  [31:0] cb_plane;                // cb * plane_in
    reg  [15:0] cb_row;                  // cb * cbs
    reg  [15:0] t;                       // output tile
    reg  [31:0] t_wgt, t_qp, t_out;      // per-tile addresses
    reg  [31:0] rt_tbase;                // runtime tile base for tile t
    reg  [15:0] t_dwcbb;                 // t * cbs (depthwise channel block)
    reg  [15:0] t_wrow;                  // WBUF row of tile t, chunk 0
    reg  [15:0] lj, lt;                  // runtime-weight load loop
    reg  [31:0] lt_base, lt_addr;        // tile base, current chunk address
    reg  [4:0]  rtw_ret;                 // where S_RTW goes after the last chunk
    reg  [23:0] wall_words;              // out_cb * nch * 64
    reg  [15:0] rows_row_step;           // s * in_w: IBUF rows per output row

    // ------------------------------------------------------------ DMA sinks
    localparam [2:0] K_NONE = 3'd0, K_LUT = 3'd1, K_W = 3'd2, K_P = 3'd3,
                     K_IN = 3'd4;
    reg  [2:0]  sink;
    reg  [15:0] wr_row;                  // IBUF/WBUF write row
    reg  [1:0]  wr_lane;
    reg  [8:0]  wr_k;                    // word index for LUT / params
    reg  [31:0] lut_icpt_tmp;

    // IBUF / WBUF: four 32-bit lanes of 4096 rows each
    wire [127:0] ib_q, wb_q;
    reg  [11:0]  ib_raddr_n, wb_raddr_n;

    generate for (gi = 0; gi < 4; gi = gi + 1) begin : g_buf
        npu_ram #(.W(32), .D(NPU_IBUF_ROWS)) u_ib (
            .clk(clk), .we(dma_rvalid && sink == K_IN && wr_lane == gi),
            .waddr(wr_row[11:0]), .wdata(dma_rdata),
            .raddr(ib_raddr_n), .q(ib_q[32*gi +: 32]));
        npu_ram #(.W(32), .D(NPU_WBUF_ROWS)) u_wb (
            .clk(clk), .we(dma_rvalid && sink == K_W && wr_lane == gi),
            .waddr(wr_row[11:0]), .wdata(dma_rdata),
            .raddr(wb_raddr_n), .q(wb_q[32*gi +: 32]));
    end endgenerate

    // per-channel parameters of the current tile
    reg signed [31:0] bias [0:L-1];
    reg        [15:0] mult [0:L-1];
    reg        [5:0]  shv  [0:L-1];

    // ---------------------------------------------------------------- FSM
    localparam [4:0]
        S_IDLE  = 5'd0,  S_SET   = 5'd1,  S_LUT   = 5'd2,  S_WALL = 5'd3,
        S_BAND  = 5'd4,  S_IN    = 5'd5,  S_TILE  = 5'd6,  S_W    = 5'd7,
        S_P     = 5'd8,  S_PRE   = 5'd9,  S_RUN   = 5'd10, S_DRAIN = 5'd11,
        S_ST    = 5'd12, S_NEXT  = 5'd13, S_WAIT  = 5'd14, S_RTW  = 5'd15;
    reg [4:0] state, ret;               // S_WAIT returns to `ret`
    reg [3:0] step;
    reg [7:0] drain;

    // ---------------------------------------------------- slot generator
    reg         sg_on, sg_bub;           // running, emitting the lead bubble
    reg  [15:0] pj;                      // pass (chunk) index
    reg  [15:0] ps;                      // slot within pass
    reg  [15:0] pix, px_ox, px_row;
    reg  [15:0] col_acc;                 // ox * s
    reg  [15:0] row_acc;                 // row * s * in_w
    reg  signed [16:0] iy_row;           // (oy0 + row) * s - pad
    reg  [3:0]  p_kx, p_ky;
    reg  [15:0] p_cb;
    reg  [15:0] p_cbb, p_kyb;            // cb*cbs (+dw base), ky*in_w
    wire signed [17:0] base_j = $signed({2'b00, p_cbb}) + $signed({2'b00, p_kyb})
                              + $signed({14'd0, p_kx}) - $signed({14'd0, g_pad});
    wire signed [17:0] iy = iy_row + $signed({14'd0, p_ky});
    wire signed [17:0] ix = $signed({2'b00, col_acc}) + $signed({14'd0, p_kx})
                          - $signed({14'd0, g_pad});
    wire in_r = (iy >= 0) && (iy < $signed({2'b00, g_in_h})) &&
                (ix >= 0) && (ix < $signed({2'b00, g_in_w}));
    wire signed [17:0] sg_addr = base_j + $signed({2'b00, row_acc})
                               + $signed({2'b00, col_acc});
    wire last_pass = (pj == g_nch - 16'd1);
    wire pass_end  = (ps == Pe - 16'd1);
    wire sg_valid  = sg_on && !sg_bub && (ps < P);
    wire sg_tag    = sg_on && (sg_bub || (pass_end && !last_pass));

    // stage S1 control (with IBUF data)
    reg         c1_v, c1_in, c1_tag, c1_first, c1_last;
    reg  [9:0]  c1_pix;
    // stage S2
    reg  [127:0] av;
    reg          av_tag;

    // ---------------------------------------------------- shadow loader
    reg        ld_run;                  // issuing WBUF addresses
    reg  [4:0] ld_step;
    reg  [5:0] ld_wait;                 // counts to 32 after a tag
    reg        ld_arm;
    reg  [15:0] ld_tile, ld_row;        // next tile to load, its WBUF row
    reg        ld_en_q;                 // data from WBUF lands this cycle
    wire [8*L-1:0] ld_data = wb_q;

    // ----------------------------------------------------------- the array
    wire [8*L-1:0] a_in;
    wire [L-1:0]   t_in;
    wire [20*L-1:0] p_out;

    npu_array #(.L(L), .PW(20)) u_array (
        .clk(clk), .a_in(a_in), .t_in(t_in),
        .ld_en(ld_en_q), .ld_col(f_tr), .ld_data(ld_data), .p_out(p_out));

    // input skew: row r delayed r cycles
    generate for (gi = 0; gi < L; gi = gi + 1) begin : g_skew
        if (gi == 0) begin : g0
            assign a_in[7:0] = av[7:0];
            assign t_in[0]   = av_tag;
        end else begin : gn
            reg [8:0] sk [0:gi-1];
            integer m;
            always @(posedge clk) begin
                sk[0] <= {av_tag, av[8*gi +: 8]};
                for (m = 1; m < gi; m = m + 1) sk[m] <= sk[m-1];
            end
            assign a_in[8*gi +: 8] = sk[gi-1][7:0];
            assign t_in[gi]        = sk[gi-1][8];
        end
    end endgenerate

    // ------------------------------------------------ control delay line
    // {valid, first, last, pix[9:0]} from S2 onward
    localparam integer DL = 38;
    reg  [12:0] dl [0:DL-1];
    integer di;
    always @(posedge clk) begin
        dl[0] <= {c1_v, c1_first, c1_last, c1_pix};
        for (di = 1; di < DL; di = di + 1) dl[di] <= dl[di-1];
    end

    // ----------------------------------------------- per-column datapath
    reg  [31:0] lut_wdata_hi;
    wire        lut_we = dma_rvalid && sink == K_LUT && wr_k[0];
    wire [7:0]  lut_waddr = wr_k[8:1];

    wire [7:0]  ob_q [0:L-1];
    reg  [9:0]  ob_raddr;

    generate for (gi = 0; gi < L; gi = gi + 1) begin : g_col
        // ACC
        wire [12:0] d_rd  = dl[15 + gi];
        wire [12:0] d_add = dl[16 + gi];
        wire [12:0] d_wr  = dl[17 + gi];
        wire [12:0] d_ob  = dl[22 + gi];
        wire [31:0] acc_q;
        reg  signed [31:0] sum_r;
        wire signed [19:0] ps_c = p_out[20*gi +: 20];
        wire signed [31:0] sum_c = (d_add[11] ? bias[gi] : $signed(acc_q)) + ps_c;

        always @(posedge clk) sum_r <= sum_c;

        npu_ram #(.W(32), .D(NPU_ACC_DEPTH)) u_acc (
            .clk(clk), .we(d_wr[12] && !d_wr[10]), .waddr(d_wr[9:0]),
            .wdata(sum_r), .raddr(d_rd[9:0]), .q(acc_q));

        // VPU lane
        reg  signed [47:0] prod;
        reg  signed [15:0] v, v_d;
        reg  signed [33:0] y;
        reg  signed [7:0]  code;
        wire [47:0] lut_q;
        wire signed [31:0] l_icpt = lut_q[47:16];
        wire signed [15:0] l_slope = lut_q[15:0];

        // Products sized exactly to their operands, so synthesis maps one
        // 32x17 and one 16x16 multiply rather than something LHS-wide.
        wire signed [48:0] mp = sum_r * $signed({1'b0, mult[gi]});
        wire signed [31:0] lp = l_slope * v_d;
        wire [5:0] sh = shv[gi];
        wire signed [48:0] rnd1 = (sh == 6'd0) ? 49'sd0 : (49'sd1 <<< (sh - 6'd1));
        // $signed() is load-bearing: a concatenation is UNSIGNED in Verilog,
        // and one unsigned operand makes the whole expression unsigned, so
        // >>> would shift in zeros and every negative result would saturate
        // to the positive rail. That is exactly what the first bench run
        // showed: positives bit-exact, every negative output 127.
        wire signed [48:0] vs = ($signed({prod[47], prod}) + rnd1) >>> sh;
        wire signed [15:0] v_n = (vs > 49'sd32767)  ? 16'sh7FFF :
                                 (vs < -49'sd32768) ? 16'sh8000 : vs[15:0];
        wire [7:0] idx = ($signed(v) < -16'sd2048) ? 8'd128 :
                         ($signed(v) >=  16'sd2048) ? 8'd129 :
                         {1'b0, 7'((v + 16'sd2048) >>> 5)};
        wire signed [34:0] rnd2 = (g_psh == 5'd0) ? 35'sd0 : (35'sd1 <<< (g_psh - 5'd1));
        wire signed [34:0] ys = ($signed({y[33], y}) + rnd2) >>> g_psh;
        wire signed [7:0] code_n = (ys > 35'sd127)  ? 8'sd127 :
                                   (ys < -35'sd128) ? -8'sd128 : ys[7:0];

        always @(posedge clk) begin
            prod <= mp[47:0];
            v    <= v_n;
            v_d  <= v;
            y    <= $signed({{2{lp[31]}}, lp}) + $signed({{2{l_icpt[31]}}, l_icpt});
            code <= code_n;
        end

        npu_ram #(.W(48), .D(256)) u_lut (
            .clk(clk), .we(lut_we), .waddr(lut_waddr),
            .wdata({lut_icpt_tmp, dma_rdata[15:0]}), .raddr(idx), .q(lut_q));

        npu_ram #(.W(8), .D(NPU_ACC_DEPTH)) u_ob (
            .clk(clk), .we(d_ob[12] && d_ob[10]), .waddr(d_ob[9:0]),
            .wdata(code), .raddr(ob_raddr), .q(ob_q[gi]));
    end endgenerate

    // store path: word k = pixel k/4, channels 4(k%4) .. 4(k%4)+3
    reg  [23:0] st_k;
    reg  [1:0]  st_g_d;
    always @(posedge clk) begin
        st_g_d    <= st_k[1:0];
        dma_wdata <= {ob_q[4*st_g_d + 3], ob_q[4*st_g_d + 2],
                      ob_q[4*st_g_d + 1], ob_q[4*st_g_d + 0]};
    end
    always @(*) ob_raddr = st_k[11:2];

    assign array_active = sg_on;

    // ---------------------------------------------- IBUF / WBUF read ports
    always @(*) begin
        ib_raddr_n = sg_addr[11:0];
        wb_raddr_n = ld_row[11:0];
    end

    // ------------------------------------------------------ main process
    integer ci;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; ret <= S_IDLE; step <= 4'd0;
            busy <= 1'b0; done <= 1'b0; phase <= 3'd0;
            dma_go <= 1'b0; dma_wr <= 1'b0; dma_addr <= 32'd0; dma_words <= 24'd0;
            sink <= K_NONE; wr_row <= 16'd0; wr_lane <= 2'd0; wr_k <= 9'd0;
            sg_on <= 1'b0; sg_bub <= 1'b0;
            c1_v <= 1'b0; c1_tag <= 1'b0; c1_in <= 1'b0; c1_first <= 1'b0;
            c1_last <= 1'b0; c1_pix <= 10'd0; av <= 128'd0; av_tag <= 1'b0;
            ld_run <= 1'b0; ld_arm <= 1'b0; ld_wait <= 6'd0; ld_step <= 5'd0;
            ld_en_q <= 1'b0; ld_tile <= 16'd0; ld_row <= 16'd0;
            st_k <= 24'd0; stat_slots <= 32'd0; drain <= 8'd0;
        end else begin
            done   <= 1'b0;
            dma_go <= 1'b0;

            // ---- DMA read sinks -------------------------------------------
            if (dma_rvalid) begin
                case (sink)
                K_IN, K_W: begin
                    wr_lane <= wr_lane + 2'd1;
                    if (wr_lane == 2'd3) wr_row <= wr_row + 16'd1;
                end
                K_LUT: begin
                    if (!wr_k[0]) lut_icpt_tmp <= dma_rdata;
                    wr_k <= wr_k + 9'd1;
                end
                K_P: begin
                    if (!wr_k[0]) bias[wr_k[4:1]] <= dma_rdata;
                    else begin
                        mult[wr_k[4:1]] <= dma_rdata[15:0];
                        shv[wr_k[4:1]]  <= dma_rdata[21:16];
                    end
                    wr_k <= wr_k + 9'd1;
                end
                default: ;
                endcase
            end
            if (dma_pull) st_k <= st_k + 24'd1;

            // ---- S1: IBUF data -> padding mux -> av (S2) -------------------
            c1_v <= sg_valid;  c1_in <= in_r;  c1_tag <= sg_tag;
            c1_first <= (pj == 16'd0); c1_last <= last_pass;
            c1_pix <= pix[9:0];
            for (ci = 0; ci < L; ci = ci + 1)
                av[8*ci +: 8] <= !c1_v ? 8'd0 : (c1_in ? ib_q[8*ci +: 8] : g_padv);
            av_tag <= c1_tag;
            if (sg_valid) stat_slots <= stat_slots + 32'd1;

            // ---- shadow loader ----------------------------------------------
            ld_en_q <= ld_run;
            if (ld_run) begin
                ld_step <= ld_step + 5'd1;
                ld_row  <= ld_row - 16'd1;
                if (ld_step == 5'd15) begin
                    ld_run  <= 1'b0;
                    ld_tile <= ld_tile + 16'd1;
                end
            end else if (ld_arm) begin
                ld_wait <= ld_wait + 6'd1;
                if (ld_wait == 6'd31) begin
                    ld_arm  <= 1'b0;
                    if (ld_tile < g_nch) begin
                        ld_run  <= 1'b1;
                        ld_step <= 5'd0;
                        ld_row  <= t_wrow + {ld_tile[11:0], 4'd0} + 16'd15;
                    end
                end
            end
            if (sg_tag && state == S_RUN) begin
                ld_arm  <= 1'b1;
                ld_wait <= 6'd0;
            end

            case (state)
            // ------------------------------------------------------------
            S_IDLE: begin
                busy <= 1'b0; phase <= 3'd0;
                if (start) begin
                    g_flags <= dw[0][15:8];
                    g_src0 <= dw[1]; g_src1 <= dw[2]; g_dst <= dw[3];
                    g_wgt <= dw[4]; g_qp <= dw[5]; g_lut <= dw[6];
                    g_in_w <= dw[7][15:0];  g_in_h <= dw[7][31:16];
                    g_in_cb <= dw[8][15:0]; g_out_cb <= dw[8][31:16];
                    g_out_w <= dw[9][15:0]; g_out_h <= dw[9][31:16];
                    g_k <= dw[10][3:0]; g_s <= dw[10][7:4]; g_pad <= dw[10][11:8];
                    g_psh <= dw[10][16:12]; g_rows <= dw[10][31:20];
                    g_nch <= dw[11][15:0]; g_rtn <= dw[11][31:16];
                    g_padv <= dw[12][7:0];
                    busy <= 1'b1; phase <= 3'd1;
                    step <= 4'd0; state <= S_SET;
                end
            end

            // ------------------------------------------------------------
            // layer constants, one product per step through the shared mult
            S_SET: begin
                step <= step + 4'd1;
                case (step)
                4'd0: begin mul_a <= g_in_h;  mul_b <= g_in_w; end
                4'd1: begin plane_in <= {mul_q[27:0], 4'd0};
                            mul_a <= g_out_h; mul_b <= g_out_w; end
                4'd2: begin plane_out <= {mul_q[27:0], 4'd0};
                            mul_a <= {4'd0, g_rows} - 16'd1; mul_b <= {12'd0, g_s}; end
                4'd3: begin rin <= mul_q[15:0] + {12'd0, g_k};
                            mul_a <= {4'd0, g_rows}; mul_b <= {12'd0, g_s}; end
                4'd4: begin rows_s <= mul_q[15:0];
                            mul_a <= rin; mul_b <= g_in_w; end
                4'd5: begin cbs <= mul_q[15:0];
                            mul_a <= {12'd0, g_s}; mul_b <= g_in_w; end
                4'd6: begin rows_row_step <= mul_q[15:0];
                            mul_a <= g_out_cb; mul_b <= g_nch; end
                4'd7: begin
                    wall_words <= {mul_q[17:0], 6'd0};
                    oy0 <= 16'd0;
                    iy_start <= -$signed({13'd0, g_pad});
                    t_wrow <= 16'd0;
                    // LUT: 260 words
                    sink <= K_LUT; wr_k <= 9'd0;
                    dma_wr <= 1'b0; dma_addr <= g_lut; dma_words <= 24'd260;
                    dma_go <= 1'b1;
                    state <= S_WAIT;
                    ret <= f_res ? S_WALL : S_BAND;
                    lt <= 16'd0; lj <= 16'd0; lt_base <= g_src1;
                    wr_row <= 16'd0; wr_lane <= 2'd0;
                    step <= 4'd0;
                end
                default: ;
                endcase
            end

            // resident weights: the whole layer, once
            S_WALL: begin
                phase <= 3'd3;
                sink <= K_W; wr_row <= 16'd0; wr_lane <= 2'd0;
                if (!f_rt) begin
                    dma_addr  <= g_wgt;
                    dma_words <= wall_words;
                    dma_wr <= 1'b0; dma_go <= 1'b1;
                    state <= S_WAIT; ret <= S_BAND;
                end else begin
                    lt <= 16'd0; lj <= 16'd0;
                    lt_base <= g_src1; lt_addr <= g_src1;
                    rtw_ret <= S_BAND;
                    state <= S_RTW;
                end
            end

            // runtime-weight tiles: one 64-word DMA per chunk. From S_WALL it
            // walks every tile (lt runs to out_cb); from S_TILE only tile t.
            // S_WAIT clears the sink after each DMA, so it is set every time;
            // wr_row is NOT reset, so the chunks land one after another.
            S_RTW: begin
                sink      <= K_W;
                dma_addr  <= lt_addr;
                dma_words <= 24'd64;
                dma_wr <= 1'b0; dma_go <= 1'b1;
                state <= S_WAIT;
                if (lj == g_nch - 16'd1) begin
                    lj <= 16'd0;
                    if (f_res && lt != g_out_cb - 16'd1) begin
                        lt      <= lt + 16'd1;
                        lt_base <= lt_base + rt_t_step;
                        lt_addr <= lt_base + rt_t_step;
                        ret     <= S_RTW;
                    end else begin
                        ret     <= rtw_ret;
                    end
                end else begin
                    lj      <= lj + 16'd1;
                    lt_addr <= lt_addr + rt_j_step;
                    ret     <= S_RTW;
                end
            end

            // ------------------------------------------------------------
            S_BAND: begin
                step <= step + 4'd1;
                case (step)
                4'd0: begin
                    rows_b <= ({4'd0, g_rows} < g_out_h - oy0) ? {4'd0, g_rows}
                                                              : g_out_h - oy0;
                end
                4'd1: begin mul_a <= rows_b; mul_b <= g_out_w; end
                4'd2: begin P <= mul_q[15:0];
                            mul_a <= rows_b - 16'd1; mul_b <= {12'd0, g_s}; end
                4'd3: begin
                    rin_b <= mul_q[15:0] + {12'd0, g_k};
                    Pe <= (P < NPU_P_MIN) ? NPU_P_MIN[15:0] : P;
                    iy_lo <= (iy_start < 0) ? 16'd0 : iy_start[15:0];
                end
                4'd4: begin
                    iy_hi <= (iy_start + $signed({1'b0, rin_b}) - 17'sd1 >
                              $signed({1'b0, g_in_h}) - 17'sd1)
                           ? g_in_h - 16'd1
                           : 16'(iy_start + $signed({1'b0, rin_b}) - 17'sd1);
                end
                4'd5: begin nr <= iy_hi - iy_lo + 16'd1;
                            mul_a <= iy_hi - iy_lo + 16'd1; mul_b <= g_in_w; end
                4'd6: begin words_cb <= {mul_q[29:0], 2'b00};
                            mul_a <= iy_lo; mul_b <= g_in_w; end
                4'd7: begin dram_off <= {mul_q[27:0], 4'd0};
                            mul_a <= 16'(iy_lo - iy_start); mul_b <= g_in_w; end
                4'd8: begin ibuf_row0 <= mul_q[15:0];
                            mul_a <= oy0; mul_b <= g_out_w; end
                4'd9: begin
                    out_off <= {mul_q[27:0], 4'd0};
                    cb <= 16'd0; cb_plane <= 32'd0; cb_row <= 16'd0;
                    step <= 4'd0;
                    state <= S_IN;
                end
                default: ;
                endcase
            end

            S_IN: begin
                phase <= 3'd2;
                if (cb == g_in_cb) begin
                    t <= 16'd0;
                    t_wgt <= g_wgt; t_qp <= g_qp; t_out <= g_dst + out_off;
                    t_dwcbb <= 16'd0;
                    t_wrow <= 16'd0;
                    rt_tbase <= g_src1;
                    state <= S_TILE;
                end else begin
                    sink <= K_IN; wr_lane <= 2'd0;
                    wr_row <= cb_row + ibuf_row0;
                    dma_addr <= g_src0 + cb_plane + dram_off;
                    dma_words <= words_cb[23:0];
                    dma_wr <= 1'b0; dma_go <= 1'b1;
                    cb <= cb + 16'd1;
                    cb_plane <= cb_plane + plane_in;
                    cb_row <= cb_row + cbs;
                    state <= S_WAIT; ret <= S_IN;
                end
            end

            S_TILE: begin
                if (f_res) begin
                    state <= S_P;
                end else if (f_rt) begin
                    phase <= 3'd3;
                    sink <= K_W; wr_row <= 16'd0; wr_lane <= 2'd0;
                    lj <= 16'd0; lt <= t;
                    lt_base <= rt_tbase; lt_addr <= rt_tbase;
                    rtw_ret <= S_P;
                    state <= S_RTW;
                end else begin
                    phase <= 3'd3;
                    sink <= K_W; wr_row <= 16'd0; wr_lane <= 2'd0;
                    dma_addr <= t_wgt; dma_words <= {6'd0, g_nch[11:0], 6'd0};
                    dma_wr <= 1'b0; dma_go <= 1'b1;
                    state <= S_WAIT; ret <= S_P;
                end
            end

            S_P: begin
                phase <= 3'd1;
                sink <= K_P; wr_k <= 9'd0;
                dma_addr <= t_qp; dma_words <= 24'd32;
                dma_wr <= 1'b0; dma_go <= 1'b1;
                state <= S_WAIT; ret <= S_PRE;
                // preload tile 0 into the shadow right after the params
                ld_tile <= 16'd0;
            end

            // preload chunk 0's weights into the shadow registers
            S_PRE: begin
                phase <= 3'd4;
                if (!ld_run && !ld_en_q && ld_tile == 16'd0 && step == 4'd0) begin
                    ld_run  <= 1'b1;
                    ld_step <= 5'd0;
                    ld_row  <= t_wrow + 16'd15;
                    step    <= 4'd1;
                end else if (step == 4'd1 && !ld_run && !ld_en_q) begin
                    // chunk 0 is in the shadow; the lead bubble swaps it in
                    step   <= 4'd0;
                    sg_on  <= 1'b1; sg_bub <= 1'b1;
                    pj <= 16'd0; ps <= 16'd0;
                    pix <= 16'd0; px_ox <= 16'd0; px_row <= 16'd0;
                    col_acc <= 16'd0; row_acc <= 16'd0;
                    iy_row <= iy_start;
                    p_kx <= 4'd0; p_ky <= 4'd0; p_cb <= 16'd0;
                    p_cbb <= f_dw ? t_dwcbb : 16'd0;
                    p_kyb <= 16'd0;
                    state <= S_RUN;
                end
            end

            // ------------------------------------------------------------
            S_RUN: begin
                if (sg_bub) begin
                    sg_bub <= 1'b0;
                end else begin
                    // pixel walk
                    if (pass_end) begin
                        ps <= 16'd0;
                        pix <= 16'd0; px_ox <= 16'd0; px_row <= 16'd0;
                        col_acc <= 16'd0; row_acc <= 16'd0;
                        iy_row <= iy_start;
                        if (last_pass) begin
                            sg_on <= 1'b0;
                            drain <= 8'd0;
                            state <= S_DRAIN;
                        end else begin
                            pj <= pj + 16'd1;
                            // tap odometer: kx, ky, then channel block
                            if (p_kx == g_k - 4'd1) begin
                                p_kx <= 4'd0;
                                if (p_ky == g_k - 4'd1) begin
                                    p_ky <= 4'd0; p_kyb <= 16'd0;
                                    p_cb <= p_cb + 16'd1;
                                    p_cbb <= p_cbb + cbs;
                                end else begin
                                    p_ky <= p_ky + 4'd1;
                                    p_kyb <= p_kyb + g_in_w;
                                end
                            end else begin
                                p_kx <= p_kx + 4'd1;
                            end
                        end
                    end else begin
                        ps <= ps + 16'd1;
                        if (ps < P - 16'd1) begin
                            pix <= pix + 16'd1;
                            if (px_ox == g_out_w - 16'd1) begin
                                px_ox <= 16'd0; col_acc <= 16'd0;
                                px_row <= px_row + 16'd1;
                                row_acc <= row_acc + rows_row_step;
                                iy_row <= iy_row + $signed({13'd0, g_s});
                            end else begin
                                px_ox <= px_ox + 16'd1;
                                col_acc <= col_acc + {12'd0, g_s};
                            end
                        end
                    end
                end
            end

            S_DRAIN: begin
                drain <= drain + 8'd1;
                if (drain == 8'd48) begin
                    phase <= 3'd5;
                    st_k <= 24'd0;
                    dma_addr <= t_out; dma_words <= {6'd0, P, 2'b00};
                    dma_wr <= 1'b1; dma_go <= 1'b1;
                    state <= S_WAIT; ret <= S_NEXT;
                end
            end

            S_NEXT: begin
                if (t != g_out_cb - 16'd1) begin
                    t <= t + 16'd1;
                    t_wgt <= t_wgt + tile_b;
                    t_qp <= t_qp + 32'd128;
                    t_out <= t_out + plane_out;
                    t_dwcbb <= t_dwcbb + cbs;
                    rt_tbase <= rt_tbase + rt_t_step;
                    if (f_res) t_wrow <= t_wrow + wrows_t;
                    state <= S_TILE;
                end else if (oy0 + {4'd0, g_rows} < g_out_h) begin
                    oy0 <= oy0 + {4'd0, g_rows};
                    iy_start <= iy_start + $signed({1'b0, rows_s});
                    step <= 4'd0;
                    state <= S_BAND;
                end else begin
                    done <= 1'b1; busy <= 1'b0; phase <= 3'd0;
                    state <= S_IDLE;
                end
            end

            S_WAIT: if (dma_done) begin
                sink <= K_NONE;
                state <= ret;
            end

            default: state <= S_IDLE;
            endcase

            if (clr) begin
                state <= S_IDLE; busy <= 1'b0; sg_on <= 1'b0;
                ld_run <= 1'b0; ld_arm <= 1'b0;
            end
        end
    end

endmodule
