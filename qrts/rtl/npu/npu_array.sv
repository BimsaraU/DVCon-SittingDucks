// =============================================================================
// npu_array.sv - 16x16 weight-stationary systolic array
//
//          a_in[0] -> PE00 -> PE01 -> ... -> PE0F        activations move right
//          a_in[1] ->  PE10 -> PE11 -> ...                partial sums move down
//             ...        |       |
//                      p_out[0] p_out[1] ...
//
// PE[r][c] holds weight W[r][c] (reduction row r, output channel c). Row r of
// the input is delayed r cycles by the caller, so the activation of one pixel
// meets its partial sum at every PE on the same cycle and column c's sum for a
// pixel leaves the bottom 16 + c cycles after row 0 entered.
//
// DOUBLE-BUFFERED WEIGHTS. Each PE has an active weight and a shadow. The
// shadow is loaded while the array computes, 16 bytes per cycle, either as
// rows shifting down (ld_col = 0, the compile-time tile layout) or as columns
// shifting right (ld_col = 1, for attention's runtime tiles, which arrive
// transposed). A tag travelling with the activations swaps shadow into active
// at each PE exactly when the last pixel of the old pass has gone through it,
// so consecutive passes stream with no gap.
//
// Partial sums are 20 bits: 16 products of at most 128 x 127 fit in 19.
// =============================================================================
`timescale 1ns/1ps

module npu_array #(
    parameter integer L  = 16,
    parameter integer PW = 20
)(
    input  wire              clk,
    input  wire [8*L-1:0]    a_in,     // row r at [8r +: 8], skewed by r
    input  wire [L-1:0]      t_in,     // swap tag per row, skewed like a_in
    input  wire              ld_en,    // shift the shadow chain one step
    input  wire              ld_col,   // 0: rows move down, 1: columns move right
    input  wire [8*L-1:0]    ld_data,  // byte i -> column i (rows) / row i (cols)
    output wire [PW*L-1:0]   p_out     // column c at [PW*c +: PW]
);
    wire [8*L*L-1:0]  a_bus;
    wire [L*L-1:0]    t_bus;
    wire [PW*L*L-1:0] p_bus;
    wire [8*L*L-1:0]  s_bus;

    genvar r, c;
    generate
        for (r = 0; r < L; r = r + 1) begin : g_row
            for (c = 0; c < L; c = c + 1) begin : g_col
                wire signed [7:0]    a_i;
                wire                 t_i;
                wire signed [PW-1:0] p_i;
                wire signed [7:0]    s_from_row;   // shadow source, row mode
                wire signed [7:0]    s_from_col;   // shadow source, column mode

                if (c == 0) begin : g_l
                    assign a_i        = a_in[8*r +: 8];
                    assign t_i        = t_in[r];
                    assign s_from_col = ld_data[8*r +: 8];
                end else begin : g_i
                    assign a_i        = a_bus[(r*L + c - 1)*8 +: 8];
                    assign t_i        = t_bus[r*L + c - 1];
                    assign s_from_col = s_bus[(r*L + c - 1)*8 +: 8];
                end
                if (r == 0) begin : g_t
                    assign p_i        = {PW{1'b0}};
                    assign s_from_row = ld_data[8*c +: 8];
                end else begin : g_b
                    assign p_i        = p_bus[((r - 1)*L + c)*PW +: PW];
                    assign s_from_row = s_bus[((r - 1)*L + c)*8 +: 8];
                end

                reg signed [7:0]    a_q, wa_q, ws_q;
                reg                 t_q;
                reg signed [PW-1:0] p_q;
                wire signed [15:0]  prod = a_i * wa_q;

                always @(posedge clk) begin
                    a_q <= a_i;
                    t_q <= t_i;
                    p_q <= p_i + prod;
                    // The tagged slot is the LAST of the old pass: it still
                    // multiplies by the old weight this cycle.
                    if (t_i)   wa_q <= ws_q;
                    if (ld_en) ws_q <= ld_col ? s_from_col : s_from_row;
                end

                assign a_bus[(r*L + c)*8 +: 8]   = a_q;
                assign t_bus[r*L + c]            = t_q;
                assign p_bus[(r*L + c)*PW +: PW] = p_q;
                assign s_bus[(r*L + c)*8 +: 8]   = ws_q;
            end
        end
        for (c = 0; c < L; c = c + 1) begin : g_out
            assign p_out[PW*c +: PW] = p_bus[((L - 1)*L + c)*PW +: PW];
        end
    endgenerate
endmodule
