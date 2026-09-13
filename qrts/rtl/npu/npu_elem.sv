// =============================================================================
// npu_elem.sv - everything that is not a convolution
//
//   ADD       residual add with per-input requantisation and zero points
//   UPSAMPLE  nearest, x2
//   MAXPOOL   k x k, stride 1, pad k/2 (SPPF), separable: rows then columns
//   SOFTMAX   over channels at each pixel (attention), exp from a per-layer LUT
//   PACK      the host's 3 x H x W INT8 planes -> one C16-blocked tensor
//   DETECT    argmax over classes, threshold on the logit, box decode, and the
//             16-byte records appended to the box list
//
// Every op is the same three steps on two 2048-word scratch banks: DMA in,
// compute bank to bank, DMA out. ADD and UPSAMPLE stream one word per cycle;
// the others use a read subroutine (issue, wait, use) that is slower but easy
// to check. They are a few percent of the frame. Semantics are
// compiler/npu_isa.py's, bit for bit.
//
// Scratch reads have a latency of 2: an address registered here in cycle t is
// sampled by the RAM at the end of t+1 and its data is valid in t+2.
// =============================================================================
`timescale 1ns/1ps

`include "npu_pkg.svh"

module npu_elem (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         clr,

    input  wire         start,
    input  wire [511:0] desc,
    output reg          busy,
    output reg          done,
    output reg  [2:0]   phase,           // 0 idle 1 load 2 compute 3 store

    input  wire [31:0]  img_addr,
    input  wire [31:0]  box_addr,
    input  wire signed [15:0] conf_logit,
    input  wire         box_clear,       // START of a frame
    output reg  [15:0]  num_boxes,

    output reg          dma_go,
    output reg          dma_wr,
    output reg  [31:0]  dma_addr,
    output reg  [23:0]  dma_words,
    input  wire         dma_done,
    input  wire [31:0]  dma_rdata,
    input  wire         dma_rvalid,
    input  wire         dma_pull,
    output reg  [31:0]  dma_wdata
);
    // ------------------------------------------------------------ descriptor
    wire [31:0] dw [0:15];
    genvar gi;
    generate for (gi = 0; gi < 16; gi = gi + 1) begin : g_dw
        assign dw[gi] = desc[32*gi +: 32];
    end endgenerate

    reg  [7:0]  g_op;
    reg  [31:0] g_src0, g_src1, g_dst, g_lut, g_a, g_b, g_c;
    reg  [15:0] g_w, g_h, g_cb, g_ocb;
    reg  [3:0]  g_k;

    // ----------------------------------------------------- shared multiplier
    reg  [15:0] mul_a, mul_b;
    wire [31:0] mul_q = mul_a * mul_b;
    reg  [31:0] hw, plane, tw;           // H*W, H*W*16 bytes, words in tensor

    // ------------------------------------------------------------ scratch
    reg  [10:0] c_ra0, c_ra1, c_wa0, c_wa1;
    reg  [31:0] c_wd0, c_wd1;
    reg         c_we0, c_we1;
    wire [31:0] q0, q1;

    localparam [1:0] K_NONE = 2'd0, K_B0 = 2'd1, K_B1 = 2'd2, K_LUT = 2'd3;
    reg  [1:0]  sink;
    reg  [10:0] rd_ptr;
    reg         storing, st_bank;
    reg  [10:0] st_ptr;

    wire d0 = dma_rvalid && sink == K_B0;
    wire d1 = dma_rvalid && sink == K_B1;

    npu_ram #(.W(32), .D(NPU_BANK_WORDS)) u_b0 (
        .clk(clk), .we(d0 | c_we0), .waddr(d0 ? rd_ptr : c_wa0),
        .wdata(d0 ? dma_rdata : c_wd0),
        .raddr(storing ? st_ptr : c_ra0), .q(q0));
    npu_ram #(.W(32), .D(NPU_BANK_WORDS)) u_b1 (
        .clk(clk), .we(d1 | c_we1), .waddr(d1 ? rd_ptr : c_wa1),
        .wdata(d1 ? dma_rdata : c_wd1),
        .raddr(storing ? st_ptr : c_ra1), .q(q1));

    // store path: pull at t -> RAM address at t -> q at t+1 -> wdata at t+2
    reg st_bank_d;
    always @(posedge clk) begin
        st_bank_d <= st_bank;
        dma_wdata <= st_bank_d ? q1 : q0;
    end

    // softmax exp LUT: 256 x 16 stored as 128 x 32, one copy per byte lane
    reg  [7:0]  lut_ix [0:3];
    wire [31:0] lut_q  [0:3];
    reg  [3:0]  lut_hi_d1;
    generate for (gi = 0; gi < 4; gi = gi + 1) begin : g_lutram
        npu_ram #(.W(32), .D(128)) u_l (
            .clk(clk), .we(dma_rvalid && sink == K_LUT), .waddr(rd_ptr[6:0]),
            .wdata(dma_rdata), .raddr(lut_ix[gi][7:1]), .q(lut_q[gi]));
    end endgenerate
    always @(posedge clk) begin
        lut_hi_d1 <= {lut_ix[3][0], lut_ix[2][0], lut_ix[1][0], lut_ix[0][0]};
    end
    // lut_ix is held while the result is used, so the one-cycle-delayed half
    // select lines up with the RAM's registered output.
    wire [15:0] lut_e [0:3];
    generate for (gi = 0; gi < 4; gi = gi + 1) begin : g_le
        assign lut_e[gi] = lut_hi_d1[gi] ? lut_q[gi][31:16] : lut_q[gi][15:0];
    end endgenerate

    // ---------------------------------------------------------------- FSM
    localparam [5:0]
        S_IDLE = 6'd0,  S_SET  = 6'd1,  S_WAIT = 6'd2,  S_R1   = 6'd3,
        S_DONE = 6'd4,
        A_LOOP = 6'd5,  A_RDB  = 6'd6,  A_CMP  = 6'd7,  A_ST   = 6'd8,
        U_LOOP = 6'd9,  U_CMP  = 6'd10, U_ST2  = 6'd11, U_NEXT = 6'd12,
        M_LOOP = 6'd13, M_WORD = 6'd14, M_TAP  = 6'd15, M_FOLD = 6'd16,
        M_ST   = 6'd17,
        X_GRP  = 6'd18, X_LD   = 6'd19, X_POS  = 6'd20, X_WALK = 6'd21,
        X_USE  = 6'd22, X_LUT  = 6'd23, X_DIV  = 6'd24, X_ST   = 6'd25,
        P_ZERO = 6'd26, P_LOOP = 6'd27, P_LD2  = 6'd28, P_LD3  = 6'd29,
        P_WORD = 6'd30, P_RG   = 6'd31, P_RB   = 6'd32, P_WR   = 6'd33,
        P_ST   = 6'd34, P_USE  = 6'd35,
        D_ROW  = 6'd36, D_LDC  = 6'd37, D_X    = 6'd38, D_ARG  = 6'd39,
        D_EVAL = 6'd40, D_REC  = 6'd41, D_ST   = 6'd42, D_USE  = 6'd43,
        D_BOX  = 6'd44;
    reg [5:0] state, ret, rback;
    reg [3:0] step;

    // one-read subroutine: returns to `rback` with q0/q1 valid
    task automatic rd(input bank, input [10:0] a, input [5:0] back);
        begin
            if (bank) c_ra1 <= a; else c_ra0 <= a;
            rback <= back; state <= S_R1;
        end
    endtask

    task automatic dma_rd(input [31:0] a, input [23:0] nw, input [1:0] sk,
                          input [10:0] base, input [5:0] back);
        begin
            dma_addr <= a; dma_words <= nw; dma_wr <= 1'b0; dma_go <= 1'b1;
            sink <= sk; rd_ptr <= base; storing <= 1'b0;
            ret <= back; state <= S_WAIT; phase <= 3'd1;
        end
    endtask

    task automatic dma_st(input [31:0] a, input [23:0] nw, input bank,
                          input [10:0] base, input [5:0] back);
        begin
            dma_addr <= a; dma_words <= nw; dma_wr <= 1'b1; dma_go <= 1'b1;
            sink <= K_NONE; storing <= 1'b1; st_bank <= bank; st_ptr <= base;
            ret <= back; state <= S_WAIT; phase <= 3'd3;
        end
    endtask

    function automatic signed [7:0] byt(input [31:0] w, input integer b);
        byt = w[8*b +: 8];
    endfunction

    // loop registers shared by the ops
    reg  [31:0] off, n, sa, da, pl;
    reg  [15:0] r, rows, i, j;
    reg  [4:0]  pv;
    reg  [10:0] pa [0:4];
    integer li;

    // ------------------------------------------------------------- ADD
    wire signed [15:0] ka = g_a[15:0];
    wire signed [15:0] kb = g_a[31:16];
    wire signed [31:0] kc = g_c;
    wire [5:0]         ash = g_b[5:0];
    wire signed [31:0] arnd = (ash == 6'd0) ? 32'sd0 : (32'sd1 <<< (ash - 6'd1));
    reg  [31:0]        ares;
    wire [31:0]        ares_n;
    // Per-lane products as wires, registered here, rather than assigned from
    // a loop in the FSM: that form left the product register X in xsim.
    generate for (gi = 0; gi < 4; gi = gi + 1) begin : g_add
        wire signed [31:0] p_n = $signed(q0[8*gi +: 8]) * ka
                               + $signed(q1[8*gi +: 8]) * kb + kc;
        reg  signed [31:0] apa;
        always @(posedge clk) apa <= p_n;
        wire signed [31:0] rs = (apa + arnd) >>> ash;
        assign ares_n[8*gi +: 8] = (rs > 32'sd127) ? 8'h7F :
                                   (rs < -32'sd128) ? 8'h80 : rs[7:0];
    end endgenerate

    // --------------------------------------------------------- MAXPOOL
    reg         mp_v;                    // 0 row pass b0->b1, 1 column pass b1->b0
    reg  [15:0] mx, my;
    reg  [1:0]  mq;
    reg  [3:0]  md;
    reg  [10:0] mo;                      // word index = (my*W + mx)*4 + mq
    reg  [31:0] mmax;
    wire [15:0] mstep = mp_v ? {g_w[13:0], 2'b00} : 16'd4;
    wire signed [17:0] mcc = (mp_v ? $signed({2'b00, my}) : $signed({2'b00, mx}))
                           + $signed({14'd0, md}) - $signed({15'd0, g_k[3:1]});
    wire        m_in = (mcc >= 0) &&
                       (mcc < $signed({2'b00, (mp_v ? g_h : g_w)}));
    reg  signed [17:0] mtap;             // (md - pad) * mstep
    wire [31:0] mword = mp_v ? q1 : q0;

    // --------------------------------------------------------- SOFTMAX
    wire [15:0] sm_cv = g_a[15:0];
    wire [15:0] sm_g  = g_a[31:16];
    wire [39:0] sm_nq = {g_c[7:0], g_b};
    reg  [15:0] n0, gn, gpos, cbi;
    reg  [1:0]  sq, spass;               // word in block, pass 0/1/2
    reg  [10:0] sbase, sw;               // gpos*4, word address
    reg  [15:0] sch;                     // first channel of the word
    reg  signed [7:0] smax;
    reg  [25:0] ssum;
    reg  [39:0] drem;
    reg  [39:0] dnum;
    reg  [39:0] rq;                      // floor(nq / sum), < 2^25 in practice
    reg  [5:0]  dbit;
    reg  [31:0] sword;
    reg  [10:0] cbase;                   // cbi * gn * 4
    reg  [31:0] cplane;                  // cbi * plane
    wire [17:0] lut_sum4 = ((sch + 16'd0 < sm_cv) ? lut_e[0] : 16'd0)
                         + ((sch + 16'd1 < sm_cv) ? lut_e[1] : 16'd0)
                         + ((sch + 16'd2 < sm_cv) ? lut_e[2] : 16'd0)
                         + ((sch + 16'd3 < sm_cv) ? lut_e[3] : 16'd0);

    // ------------------------------------------------------------ PACK
    reg  [31:0] pr, pg;

    // ---------------------------------------------------------- DETECT
    wire [15:0] d_kb   = g_a[15:0];
    wire [15:0] d_kc   = g_a[31:16];
    wire [3:0]  d_sl   = g_b[3:0];
    wire [3:0]  d_lvl  = g_b[7:4];
    wire [7:0]  d_ncls = g_b[15:8];
    reg  [15:0] dy, dx, nrow;
    reg  signed [8:0] best;
    reg  [7:0]  bidx;
    reg  [31:0] boxw;
    reg  [1:0]  ri;
    reg  [10:0] dwb;                     // class word address
    reg  [31:0] drow;                    // dy * W * 16

    wire signed [7:0]  bl = boxw[7:0],   bt = boxw[15:8];
    wire signed [7:0]  br = boxw[23:16], bb = boxw[31:24];
    wire signed [25:0] kbs = $signed({10'd0, d_kb});
    wire signed [25:0] lq0 = (bl * kbs + 26'sd2048) >>> 12;
    wire signed [25:0] lq1 = (bt * kbs + 26'sd2048) >>> 12;
    wire signed [25:0] lq2 = (br * kbs + 26'sd2048) >>> 12;
    wire signed [25:0] lq3 = (bb * kbs + 26'sd2048) >>> 12;
    wire signed [25:0] cxq = $signed({6'd0, dx, 4'd0}) + 26'sd8;
    wire signed [25:0] cyq = $signed({6'd0, dy, 4'd0}) + 26'sd8;
    wire signed [31:0] x1 = $signed(cxq - lq0) <<< d_sl;
    wire signed [31:0] y1 = $signed(cyq - lq1) <<< d_sl;
    wire signed [31:0] x2 = $signed(cxq + lq2) <<< d_sl;
    wire signed [31:0] y2 = $signed(cyq + lq3) <<< d_sl;
    wire signed [31:0] lg = (best * $signed({16'd0, d_kc}) + 32'sd128) >>> 8;
    function automatic [15:0] c16(input signed [31:0] v);
        c16 = (v > 32'sd32767) ? 16'h7FFF : (v < -32'sd32768) ? 16'h8000 : v[15:0];
    endfunction
    wire [31:0] rec0 = {c16(y1), c16(x1)};
    wire [31:0] rec1 = {c16(y2), c16(x2)};
    wire [31:0] rec2 = {8'd0, bidx, c16(lg)};
    wire [31:0] rec3 = {dy[11:0], dx[11:0], 4'd0, d_lvl};

    // argmax over one class word (first max wins: lane order, strict >)
    reg signed [8:0] am_best;
    reg [7:0]        am_idx;
    integer ab;
    always @(*) begin
        am_best = best; am_idx = bidx;
        for (ab = 0; ab < 4; ab = ab + 1)
            if (sch + ab < {8'd0, d_ncls} &&
                $signed(byt(q1, ab)) > am_best) begin
                am_best = byt(q1, ab);
                am_idx  = sch[7:0] + 8'(ab);
            end
    end

    integer bi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; ret <= S_IDLE; rback <= S_IDLE; step <= 4'd0;
            busy <= 1'b0; done <= 1'b0; phase <= 3'd0; num_boxes <= 16'd0;
            dma_go <= 1'b0; dma_wr <= 1'b0; dma_addr <= 32'd0; dma_words <= 24'd0;
            sink <= K_NONE; rd_ptr <= 11'd0; storing <= 1'b0; st_bank <= 1'b0;
            st_ptr <= 11'd0; c_we0 <= 1'b0; c_we1 <= 1'b0; pv <= 5'd0;
        end else begin
            done   <= 1'b0;
            dma_go <= 1'b0;
            c_we0  <= 1'b0;
            c_we1  <= 1'b0;
            if (box_clear) num_boxes <= 16'd0;
            if (dma_rvalid && sink != K_NONE) rd_ptr <= rd_ptr + 11'd1;
            if (dma_pull) st_ptr <= st_ptr + 11'd1;

            case (state)
            S_IDLE: begin
                busy <= 1'b0; phase <= 3'd0;
                if (start) begin
                    g_op <= dw[0][7:0];
                    g_src0 <= dw[1]; g_src1 <= dw[2]; g_dst <= dw[3]; g_lut <= dw[6];
                    g_w <= dw[7][15:0]; g_h <= dw[7][31:16];
                    g_cb <= dw[8][15:0]; g_ocb <= dw[8][31:16];
                    g_k <= dw[10][3:0];
                    g_a <= dw[12]; g_b <= dw[13]; g_c <= dw[14];
                    busy <= 1'b1; step <= 4'd0; state <= S_SET;
                end
            end

            S_SET: begin
                step <= step + 4'd1;
                case (step)
                4'd0: begin mul_a <= g_h; mul_b <= g_w; end
                4'd1: begin hw <= mul_q; plane <= {mul_q[27:0], 4'd0};
                            mul_a <= mul_q[15:0]; mul_b <= g_cb; end
                4'd2: begin tw <= {mul_q[29:0], 2'b00};
                            mul_a <= g_h; mul_b <= g_cb; end
                4'd3: begin
                    rows <= mul_q[15:0];
                    off <= 32'd0; r <= 16'd0; sa <= g_src0; da <= g_dst;
                    pl <= 32'd0; step <= 4'd0; phase <= 3'd2;
                    case (g_op)
                    OP_ADD:      state <= A_LOOP;
                    OP_UPSAMPLE: state <= U_LOOP;
                    OP_MAXPOOL:  state <= M_LOOP;
                    OP_SOFTMAX:  dma_rd(g_lut, 24'd128, K_LUT, 11'd0, X_GRP);
                    OP_PACK: begin
                        sa <= (g_src0 == 32'd0) ? img_addr : g_src0;
                        i <= 16'd0; state <= P_ZERO;
                    end
                    OP_DETECT: begin dy <= 16'd0; drow <= 32'd0; state <= D_ROW; end
                    default:     state <= S_DONE;
                    endcase
                    if (g_op == OP_SOFTMAX) n0 <= 16'd0;
                end
                default: ;
                endcase
            end

            S_WAIT: if (dma_done) begin
                sink <= K_NONE; storing <= 1'b0; phase <= 3'd2;
                state <= ret;
            end

            S_R1: state <= rback;

            S_DONE: begin busy <= 1'b0; done <= 1'b1; phase <= 3'd0; state <= S_IDLE; end

            // ============================================================ ADD
            // chunks of up to 2048 words: a -> bank0, b -> bank1, bank0 = f(a, b)
            A_LOOP: begin
                if (off == tw) state <= S_DONE;
                else begin
                    n <= (tw - off > 32'd2048) ? 32'd2048 : tw - off;
                    dma_rd(g_src0 + {off[29:0], 2'b00},
                           (tw - off > 32'd2048) ? 24'd2048 : 24'(tw - off),
                           K_B0, 11'd0, A_RDB);
                end
            end
            A_RDB: begin
                dma_rd(g_src1 + {off[29:0], 2'b00}, n[23:0], K_B1, 11'd0, A_CMP);
                i <= 16'd0; pv <= 5'd0;
            end
            A_CMP: begin
                // issue i | pv1: q valid, products | pv2: result | pv3: write
                pv <= {pv[3:0], (i < n[15:0])};
                c_ra0 <= i[10:0]; c_ra1 <= i[10:0];
                pa[0] <= i[10:0];
                for (li = 1; li < 5; li = li + 1) pa[li] <= pa[li-1];
                if (i < n[15:0]) i <= i + 16'd1;
                ares <= ares_n;
                if (pv[3]) begin c_we0 <= 1'b1; c_wa0 <= pa[3]; c_wd0 <= ares; end
                if (i == n[15:0] && pv == 5'd0)
                    dma_st(g_dst + {off[29:0], 2'b00}, n[23:0], 1'b0, 11'd0, A_ST);
            end
            A_ST: begin off <= off + n; state <= A_LOOP; end

            // ======================================================= UPSAMPLE
            // one input row -> one output row in bank1, written out twice
            U_LOOP: begin
                if (r == rows) state <= S_DONE;
                else begin
                    dma_rd(sa, {6'd0, g_w, 2'b00}, K_B0, 11'd0, U_CMP);
                    i <= 16'd0; pv <= 5'd0;
                end
            end
            U_CMP: begin
                pv <= {pv[3:0], (i < {g_w[12:0], 3'b000})};
                c_ra0 <= {i[11:3], i[1:0]};      // ((o >> 3) << 2) | (o & 3)
                pa[0] <= i[10:0];
                for (li = 1; li < 5; li = li + 1) pa[li] <= pa[li-1];
                if (i < {g_w[12:0], 3'b000}) i <= i + 16'd1;
                if (pv[1]) begin c_we1 <= 1'b1; c_wa1 <= pa[1]; c_wd1 <= q0; end
                if (i == {g_w[12:0], 3'b000} && pv == 5'd0)
                    dma_st(da, {5'd0, g_w, 3'b000}, 1'b1, 11'd0, U_ST2);
            end
            U_ST2: dma_st(da + {g_w, 5'd0}, {5'd0, g_w, 3'b000}, 1'b1, 11'd0, U_NEXT);
            U_NEXT: begin
                r <= r + 16'd1;
                sa <= sa + {g_w, 4'd0};
                da <= da + {g_w, 6'd0};
                state <= U_LOOP;
            end

            // ======================================================== MAXPOOL
            M_LOOP: begin
                if (r == g_cb) state <= S_DONE;
                else begin
                    mp_v <= 1'b0;
                    dma_rd(g_src0 + pl, {hw[21:0], 2'b00}, K_B0, 11'd0, M_WORD);
                end
                mx <= 16'd0; my <= 16'd0; mq <= 2'd0; mo <= 11'd0;
            end
            M_WORD: begin                // start a word: -128 in every lane
                mmax <= 32'h80808080;
                md   <= 4'd0;
                mtap <= -$signed({14'd0, g_k[3:1]}) *
                        $signed({2'b00, mstep});
                state <= M_TAP;
            end
            M_TAP: begin
                if (md == g_k) begin
                    if (mp_v) begin c_we0 <= 1'b1; c_wa0 <= mo; c_wd0 <= mmax; end
                    else      begin c_we1 <= 1'b1; c_wa1 <= mo; c_wd1 <= mmax; end
                    // next word, raster over (y, x, q)
                    mo <= mo + 11'd1;
                    mq <= mq + 2'd1;
                    if (mq == 2'd3) begin
                        if (mx == g_w - 16'd1) begin
                            mx <= 16'd0;
                            if (my == g_h - 16'd1) begin
                                my <= 16'd0; mo <= 11'd0;
                                if (!mp_v) mp_v <= 1'b1;
                                else begin
                                    state <= M_ST;
                                end
                            end else my <= my + 16'd1;
                        end else mx <= mx + 16'd1;
                    end
                    if (!(mq == 2'd3 && mx == g_w - 16'd1 &&
                          my == g_h - 16'd1 && mp_v))
                        state <= M_WORD;
                end else if (m_in) begin
                    rd(mp_v, 11'($signed({1'b0, mo}) + mtap), M_FOLD);
                end else begin
                    md <= md + 4'd1;
                    mtap <= mtap + $signed({2'b00, mstep});
                end
            end
            M_FOLD: begin
                for (bi = 0; bi < 4; bi = bi + 1)
                    if ($signed(mword[8*bi +: 8]) > $signed(mmax[8*bi +: 8]))
                        mmax[8*bi +: 8] <= mword[8*bi +: 8];
                md <= md + 4'd1;
                mtap <= mtap + $signed({2'b00, mstep});
                state <= M_TAP;
            end
            M_ST: begin
                dma_st(g_dst + pl, {hw[21:0], 2'b00}, 1'b0, 11'd0, M_LOOP);
                r <= r + 16'd1;
                pl <= pl + plane;
            end

            // ======================================================== SOFTMAX
            X_GRP: begin
                if (n0 >= hw[15:0]) state <= S_DONE;
                else begin
                    gn <= (hw[15:0] - n0 > sm_g) ? sm_g : hw[15:0] - n0;
                    cbi <= 16'd0; cbase <= 11'd0; cplane <= 32'd0;
                    state <= X_LD;
                end
            end
            X_LD: begin                  // block cbi, gn pixels -> bank0 @ cbi*gn*4
                if (cbi == g_cb) begin
                    gpos <= 16'd0; sbase <= 11'd0;
                    spass <= 2'd0;
                    state <= X_POS;
                end else begin
                    dma_rd(g_src0 + cplane + {n0, 4'd0}, {6'd0, gn, 2'b00},
                           K_B0, cbase, X_LD);
                    cbi <= cbi + 16'd1;
                    cbase <= cbase + {gn[8:0], 2'b00};
                    cplane <= cplane + plane;
                end
            end
            X_POS: begin                 // start a pass over one pixel's channels
                if (gpos == gn) begin
                    cbi <= 16'd0; cbase <= 11'd0; cplane <= 32'd0;
                    state <= X_ST;
                end else begin
                    cbi <= 16'd0; sq <= 2'd0; sch <= 16'd0;
                    sw <= sbase;
                    if (spass == 2'd0) smax <= -8'sd128;
                    if (spass == 2'd1) ssum <= 26'd0;
                    state <= X_WALK;
                end
            end
            X_WALK: begin
                if (cbi == g_cb) begin
                    if (spass == 2'd0) begin spass <= 2'd1; state <= X_POS; end
                    else if (spass == 2'd1) begin
                        // R = floor(nq / sum), restoring, 40 steps
                        drem <= 40'd0; dnum <= sm_nq; rq <= 40'd0; dbit <= 6'd39;
                        state <= X_DIV;
                    end else begin
                        spass <= 2'd0; gpos <= gpos + 16'd1;
                        sbase <= sbase + 11'd4;
                        state <= X_POS;
                    end
                end else rd(1'b0, sw, X_USE);
            end
            X_USE: begin
                if (spass == 2'd0) begin
                    begin : sm_max
                        reg signed [7:0] m;
                        m = smax;
                        for (bi = 0; bi < 4; bi = bi + 1)
                            if (sch + bi < sm_cv && $signed(q0[8*bi +: 8]) > m)
                                m = q0[8*bi +: 8];
                        smax <= m;
                    end
                    step <= 4'd9;            // nothing to wait for
                end else begin
                    for (bi = 0; bi < 4; bi = bi + 1)
                        lut_ix[bi] <= 8'($signed(smax) - $signed(q0[8*bi +: 8]));
                    step <= 4'd0;
                end
                state <= X_LUT;
            end
            X_LUT: begin
                // pass 0 skips straight through; passes 1 and 2 wait two
                // cycles for the four LUT reads, then fold them in
                if (step == 4'd9 || step == 4'd2) begin
                    if (spass == 2'd1) begin
                        ssum <= ssum + {8'd0, lut_sum4};
                    end else if (spass == 2'd2) begin
                        c_we1 <= 1'b1; c_wa1 <= sw;
                        for (bi = 0; bi < 4; bi = bi + 1) begin : sm_out
                            reg [41:0] pp;
                            pp = ({26'd0, lut_e[bi]} * {16'd0, rq[25:0]} + 42'd8388608) >> 24;
                            c_wd1[8*bi +: 8] <= (sch + bi >= sm_cv) ? 8'd0 :
                                                (pp > 42'd127) ? 8'd127 : pp[7:0];
                        end
                    end
                    // next word: q, then block
                    sq <= sq + 2'd1;
                    sch <= sch + 16'd4;
                    if (sq == 2'd3) begin
                        cbi <= cbi + 16'd1;
                        sw <= sw + {gn[8:0], 2'b00} - 11'd3;
                    end else sw <= sw + 11'd1;
                    state <= X_WALK;
                end else step <= step + 4'd1;
            end
            X_DIV: begin
                begin : div_step
                    reg [39:0] t;
                    t = {drem[38:0], dnum[dbit]};
                    if (t >= {14'd0, ssum}) begin
                        drem <= t - {14'd0, ssum};
                        rq[dbit] <= 1'b1;
                    end else drem <= t;
                end
                if (dbit == 6'd0) begin spass <= 2'd2; state <= X_POS; end
                else dbit <= dbit - 6'd1;
            end
            X_ST: begin                  // bank1 block cbi -> dst
                if (cbi == g_cb) begin
                    n0 <= n0 + gn;
                    state <= X_GRP;
                end else begin
                    dma_st(g_dst + cplane + {n0, 4'd0}, {6'd0, gn, 2'b00}, 1'b1,
                           cbase, X_ST);
                    cbi <= cbi + 16'd1;
                    cbase <= cbase + {gn[8:0], 2'b00};
                    cplane <= cplane + plane;
                end
            end

            // =========================================================== PACK
            P_ZERO: begin
                // words 1..3 of every output pixel are zero: clear bank1 once
                c_we1 <= 1'b1; c_wa1 <= i[10:0]; c_wd1 <= 32'd0;
                i <= i + 16'd1;
                if (i == 16'd2047) begin off <= 32'd0; state <= P_LOOP; end
            end
            P_LOOP: begin
                if (off == hw) state <= S_DONE;
                else begin
                    n <= (hw - off > 32'd512) ? 32'd512 : hw - off;
                    dma_rd(sa + off,
                           (hw - off > 32'd512) ? 24'd128 : 24'((hw - off) >> 2),
                           K_B0, 11'd0, P_LD2);
                end
            end
            P_LD2: dma_rd(sa + hw + off, n[25:2], K_B0, 11'd128, P_LD3);
            P_LD3: begin
                dma_rd(sa + {hw[30:0], 1'b0} + off, n[25:2], K_B0, 11'd256, P_WORD);
                j <= 16'd0;
            end
            P_WORD: begin                // four pixels per plane word
                if (j == n[17:2]) state <= P_ST;
                else rd(1'b0, j[10:0], P_RG);
            end
            P_RG: begin pr <= q0; rd(1'b0, 11'd128 + j[10:0], P_RB); end
            P_RB: begin pg <= q0; rd(1'b0, 11'd256 + j[10:0], P_USE); end
            P_USE: begin sword <= q0; i <= 16'd0; state <= P_WR; end
            P_WR: begin
                c_we1 <= 1'b1;
                c_wa1 <= {j[6:0], i[1:0], 2'b00};
                c_wd1 <= {8'd0, sword[8*i[1:0] +: 8], pg[8*i[1:0] +: 8],
                          pr[8*i[1:0] +: 8]};
                i <= i + 16'd1;
                if (i[1:0] == 2'd3) begin j <= j + 16'd1; state <= P_WORD; end
            end
            P_ST: begin
                dma_st(g_dst + {off[27:0], 4'd0}, {n[21:0], 2'b00}, 1'b1,
                       11'd0, P_LOOP);
                off <= off + n;
            end

            // ========================================================= DETECT
            D_ROW: begin
                if (dy == g_h) state <= S_DONE;
                else begin
                    dma_rd(g_src0 + drow, {6'd0, g_w, 2'b00}, K_B0, 11'd0, D_LDC);
                    cbi <= 16'd0; cbase <= 11'd0; cplane <= 32'd0;
                end
            end
            D_LDC: begin                 // class blocks -> bank1 @ cbi*W*4
                if (cbi == g_ocb) begin
                    dx <= 16'd0; nrow <= 16'd0;
                    state <= D_X;
                end else begin
                    dma_rd(g_src1 + cplane + drow, {6'd0, g_w, 2'b00},
                           K_B1, cbase, D_LDC);
                    cbi <= cbi + 16'd1;
                    cbase <= cbase + {g_w[8:0], 2'b00};
                    cplane <= cplane + plane;
                end
            end
            D_X: begin
                if (dx == g_w) begin
                    if (nrow != 16'd0)
                        dma_st(box_addr + {(num_boxes - nrow), 4'd0},
                               {6'd0, nrow, 2'b00}, 1'b0, 11'd1024, D_ST);
                    else state <= D_ST;
                end else begin
                    best <= -9'sd129; bidx <= 8'd0;
                    cbi <= 16'd0; sq <= 2'd0; sch <= 16'd0;
                    dwb <= {dx[8:0], 2'b00};
                    state <= D_ARG;
                end
            end
            D_ARG: begin
                if (cbi == g_ocb) state <= D_EVAL;
                else rd(1'b1, dwb, D_USE);
            end
            D_USE: begin
                best <= am_best; bidx <= am_idx;
                sq <= sq + 2'd1; sch <= sch + 16'd4;
                if (sq == 2'd3) begin
                    cbi <= cbi + 16'd1;
                    dwb <= dwb + {g_w[8:0], 2'b00} - 11'd3;
                end else dwb <= dwb + 11'd1;
                state <= D_ARG;
            end
            D_EVAL: begin
                if ($signed(lg) >= $signed({{16{conf_logit[15]}}, conf_logit}) &&
                    num_boxes < NPU_MAX_BOXES[15:0])
                    rd(1'b0, {dx[8:0], 2'b00}, D_BOX);
                else begin dx <= dx + 16'd1; state <= D_X; end
            end
            D_BOX: begin boxw <= q0; ri <= 2'd0; state <= D_REC; end
            D_REC: begin
                c_we0 <= 1'b1;
                c_wa0 <= 11'd1024 + {nrow[6:0], ri};
                c_wd0 <= (ri == 2'd0) ? rec0 : (ri == 2'd1) ? rec1 :
                         (ri == 2'd2) ? rec2 : rec3;
                ri <= ri + 2'd1;
                if (ri == 2'd3) begin
                    nrow <= nrow + 16'd1;
                    num_boxes <= num_boxes + 16'd1;
                    dx <= dx + 16'd1;
                    state <= D_X;
                end
            end
            D_ST: begin
                dy <= dy + 16'd1;
                drow <= drow + {g_w, 4'd0};
                state <= D_ROW;
            end

            default: state <= S_IDLE;
            endcase

            if (clr) begin state <= S_IDLE; busy <= 1'b0; storing <= 1'b0; end
        end
    end
endmodule
