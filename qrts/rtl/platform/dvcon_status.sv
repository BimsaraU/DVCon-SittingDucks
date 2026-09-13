// =============================================================================
// dvcon_status.sv - what the board's LEDs and 7-segment displays show
//
// LEDR, one lamp per NPU state (stretched to ~84 ms so short states show):
//   R0  IDLE, ready for START          R9  SOFTMAX (attention)
//   R1  FETCH descriptor               R10 PACK (input frame)
//   R2  CONV: loading input band       R11 DETECT (box decode)
//   R3  CONV: loading weights/params   R12 DONE (held until next START)
//   R4  CONV: systolic array running   R13 ERROR (held until next START)
//   R5  CONV: storing output           R14 DMA read
//   R6  ADD                            R15 DMA write
//   R7  UPSAMPLE                       R16 array MAC slot this instant
//   R8  MAXPOOL                        R17 last frame found boxes
//
// LEDG, the host-facing view:
//   G0 heartbeat ~1.5 Hz   G1 model loaded    G2 image loaded   G3 Ethernet RX
//   G4 Ethernet TX         G5 Ethernet RX error                 G6 JTAG transfer
//   G7 NPU busy            G8 reset released
//   G1/G2 are the NPU FLAGS register: steady, set by the host after a verified
//   load, cleared by reset (which is also when SDRAM loses its contents).
//
// HEX7..HEX0:
//   running  [layer tens][layer ones][op letter][ ][descriptor index, 4 hex]
//            op letters: C conv, A add, U upsample, P pool, S softmax,
//            I input pack, d detect
//   done     d o n E  [box count, 4 hex]
//   error    E r r [ ] [error code]
//   idle     - - - -  - - - -
// =============================================================================
`timescale 1ns/1ps

module dvcon_status (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        n_busy,
    input  wire [3:0]  n_state,
    input  wire [7:0]  n_op,
    input  wire [2:0]  n_cph,
    input  wire [2:0]  n_eph,
    input  wire [15:0] n_tag,
    input  wire [15:0] n_idx,
    input  wire [15:0] n_boxes,
    input  wire        n_array,
    input  wire        n_drd,
    input  wire        n_dwr,
    input  wire [1:0]  ld_flags,         // [0] model loaded [1] image loaded
    input  wire        ev_rx, ev_tx, ev_err, ev_jmem,
    output wire [17:0] LEDR,
    output wire [8:0]  LEDG,
    output wire [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7
);
    localparam integer SW = 22;         // 2^22 cycles = 84 ms at 50 MHz

    // sequencer states (npu_top)
    wire s_idle  = n_state == 4'd0;
    wire s_fetch = n_state == 4'd1 || n_state == 4'd6;
    wire s_conv  = n_state == 4'd2;
    wire s_elem  = n_state == 4'd3;

    reg done_l, err_l, busy_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin done_l <= 1'b0; err_l <= 1'b0; busy_q <= 1'b0; end
        else begin
            busy_q <= n_busy;
            if (n_busy && !busy_q) begin done_l <= 1'b0; err_l <= 1'b0; end
            if (n_state == 4'd4) done_l <= 1'b1;
            if (n_state == 4'd5) err_l  <= 1'b1;
        end
    end

    wire [17:0] ev = {
        done_l && n_boxes != 16'd0,                  // 17
        n_array,                                     // 16
        n_dwr,                                       // 15
        n_drd,                                       // 14
        err_l,                                       // 13
        done_l && !err_l,                            // 12
        s_elem && n_op == 8'd7,                      // 11 detect
        s_elem && n_op == 8'd6,                      // 10 pack
        s_elem && n_op == 8'd5,                      // 9  softmax
        s_elem && n_op == 8'd4,                      // 8  maxpool
        s_elem && n_op == 8'd3,                      // 7  upsample
        s_elem && n_op == 8'd2,                      // 6  add
        s_conv && n_cph == 3'd5,                     // 5  store
        s_conv && n_cph == 3'd4,                     // 4  array
        s_conv && (n_cph == 3'd1 || n_cph == 3'd3),  // 3  weights / params
        s_conv && n_cph == 3'd2,                     // 2  input band
        s_fetch,                                     // 1
        s_idle && !n_busy                            // 0
    };

    genvar i;
    generate for (i = 0; i < 18; i = i + 1) begin : g_r
        reg [SW-1:0] c;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n)      c <= {SW{1'b0}};
            else if (ev[i])  c <= {SW{1'b1}};
            else if (|c)     c <= c - 1'b1;
        end
        // held states show while true; transient ones stretch
        assign LEDR[i] = ev[i] | (|c);
    end endgenerate

    reg [24:0] hb;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) hb <= 25'd0; else hb <= hb + 25'd1;

    wire [3:0] gev = {ev_jmem, ev_err, ev_tx, ev_rx};
    wire [3:0] gled;
    generate for (i = 0; i < 4; i = i + 1) begin : g_g
        reg [SW-1:0] c;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n)      c <= {SW{1'b0}};
            else if (gev[i]) c <= {SW{1'b1}};
            else if (|c)     c <= c - 1'b1;
        end
        assign gled[i] = |c;
    end endgenerate
    assign LEDG = {rst_n, n_busy, gled, ld_flags[1], ld_flags[0], hb[24]};

    // ------------------------------------------------------- 7-segment
    // active-high patterns, segment a = bit 0; inverted at the pins
    function automatic [6:0] hexd(input [3:0] d);
        case (d)
        4'h0: hexd = 7'h3F; 4'h1: hexd = 7'h06; 4'h2: hexd = 7'h5B; 4'h3: hexd = 7'h4F;
        4'h4: hexd = 7'h66; 4'h5: hexd = 7'h6D; 4'h6: hexd = 7'h7D; 4'h7: hexd = 7'h07;
        4'h8: hexd = 7'h7F; 4'h9: hexd = 7'h6F; 4'hA: hexd = 7'h77; 4'hB: hexd = 7'h7C;
        4'hC: hexd = 7'h39; 4'hD: hexd = 7'h5E; 4'hE: hexd = 7'h79; default: hexd = 7'h71;
        endcase
    endfunction

    localparam [6:0] L_DASH = 7'h40, L_BLANK = 7'h00, L_n = 7'h54, L_o = 7'h5C,
                     L_r = 7'h50, L_E = 7'h79, L_d = 7'h5E, L_C = 7'h39,
                     L_A = 7'h77, L_U = 7'h3E, L_P = 7'h73, L_S = 7'h6D,
                     L_I = 7'h30;

    function automatic [6:0] opl(input [7:0] op);
        case (op)
        8'd1: opl = L_C; 8'd2: opl = L_A; 8'd3: opl = L_U; 8'd4: opl = L_P;
        8'd5: opl = L_S; 8'd6: opl = L_I; 8'd7: opl = L_d; default: opl = L_DASH;
        endcase
    endfunction

    // layer number in decimal (tags are 0..99)
    wire [6:0] t7 = n_tag[6:0];
    wire [3:0] tens = (t7 >= 90) ? 4'd9 : (t7 >= 80) ? 4'd8 : (t7 >= 70) ? 4'd7 :
                      (t7 >= 60) ? 4'd6 : (t7 >= 50) ? 4'd5 : (t7 >= 40) ? 4'd4 :
                      (t7 >= 30) ? 4'd3 : (t7 >= 20) ? 4'd2 : (t7 >= 10) ? 4'd1 : 4'd0;
    wire [6:0] ones7 = t7 - {tens, 3'b000} - {2'b00, tens, 1'b0};

    reg [6:0] h [0:7];
    always @(*) begin
        if (n_busy) begin
            h[7] = hexd(tens); h[6] = hexd(ones7[3:0]); h[5] = opl(n_op);
            h[4] = L_BLANK;
            h[3] = hexd(n_idx[15:12]); h[2] = hexd(n_idx[11:8]);
            h[1] = hexd(n_idx[7:4]);   h[0] = hexd(n_idx[3:0]);
        end else if (err_l) begin
            h[7] = L_E; h[6] = L_r; h[5] = L_r; h[4] = L_BLANK;
            h[3] = hexd(n_idx[15:12]); h[2] = hexd(n_idx[11:8]);
            h[1] = hexd(n_idx[7:4]);   h[0] = hexd(n_idx[3:0]);
        end else if (done_l) begin
            h[7] = L_d; h[6] = L_o; h[5] = L_n; h[4] = L_E;
            h[3] = hexd(n_boxes[15:12]); h[2] = hexd(n_boxes[11:8]);
            h[1] = hexd(n_boxes[7:4]);   h[0] = hexd(n_boxes[3:0]);
        end else begin
            h[7] = L_DASH; h[6] = L_DASH; h[5] = L_DASH; h[4] = L_DASH;
            h[3] = L_DASH; h[2] = L_DASH; h[1] = L_DASH; h[0] = L_DASH;
        end
    end

    assign HEX0 = ~h[0]; assign HEX1 = ~h[1]; assign HEX2 = ~h[2]; assign HEX3 = ~h[3];
    assign HEX4 = ~h[4]; assign HEX5 = ~h[5]; assign HEX6 = ~h[6]; assign HEX7 = ~h[7];
endmodule
