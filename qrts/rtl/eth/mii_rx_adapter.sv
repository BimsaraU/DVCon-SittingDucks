// =============================================================================
// mii_rx_adapter.sv — MII nibble stream -> byte stream, into the system clock
//
// The DE2-115's ENET0 is wired for MII: 4 data bits clocked by ENET0_RX_CLK,
// which the PHY generates at 25 MHz for 100BASE-TX (2.5 MHz at 10BASE-T). The
// MAC downstream is byte-wide and runs on clk_sys. This does both halves of
// that gap.
//
//   nibble pairs -> bytes    MII sends the LOW nibble first
//   rx_clk -> clk_sys        asynchronous, unrelated frequencies
//
// ---------------------------------------------------------------------------
// WHY A FIFO AND NOT A SYNCHRONISER
// ---------------------------------------------------------------------------
// A two-flop synchroniser works for a level or a toggle, not for a stream: at
// 25 MHz a byte arrives every 8 rx_clk edges, and clk_sys is free-running at
// 100 MHz with no fixed phase. Sampling the byte register directly would work
// most of the time and drop or duplicate a byte whenever the two edges landed
// close together -- which shows up as an occasional bad FCS, once every few
// thousand frames, and looks exactly like a cable problem.
//
// So: a small asynchronous FIFO with Gray-coded pointers. Depth 16 is ample --
// the reader is 4x faster than the writer, so the FIFO only ever holds the
// jitter between the two clocks, never a backlog.
//
// ---------------------------------------------------------------------------
// FRAME BOUNDARY
// ---------------------------------------------------------------------------
// eth_mac_rx tests `rx_valid && rx_last`, so the boundary must ride WITH the
// final byte, not follow it. An earlier version pushed a separate zero-data
// beat with last set and valid clear; the MAC then never reached its FCS check
// and every frame was silently discarded -- stat_good and stat_bad_fcs both
// stayed at zero, which looks like a dead cable rather than a protocol
// mismatch.
//
// The last byte is identifiable one rx_clk in advance: RX_DV falls on the edge
// after the frame's final nibble. So the byte is held back one edge and pushed
// with `last` set when DV drops.
//
// (Note that the MAC is not symmetric here: it CONSUMES last-with-valid but
// PRODUCES eof on its own cycle with valid low. eth_cmd_engine handles the
// producing side separately -- see the note there.)
// =============================================================================
`timescale 1ns/1ps

module mii_rx_adapter #(
    parameter integer AW = 4          // FIFO depth = 2**AW bytes
)(
    // ---- MII side, ENET0_RX_CLK domain ----
    input  wire       rx_clk,
    input  wire       rst_n,
    input  wire [3:0] mii_rxd,
    input  wire       mii_rx_dv,

    // ---- byte side, system clock ----
    input  wire       clk,
    output reg  [7:0] out_data,
    output reg        out_valid,
    output reg        out_last
);

    localparam integer DEPTH = 1 << AW;

    // ---- rx_clk domain: nibbles to bytes ------------------------------------
    reg       phase;              // 0 = expecting low nibble
    reg [3:0] lo_nib;
    reg [7:0] wr_byte;
    reg       wr_push;
    reg       wr_last;

    // The assembled byte is held one edge rather than pushed immediately, so
    // that when RX_DV falls the byte still in hand can be pushed WITH its last
    // flag -- which is the form eth_mac_rx requires.
    reg [7:0] hold;
    reg       hold_v;

    always @(posedge rx_clk or negedge rst_n) begin
        if (!rst_n) begin
            phase   <= 1'b0;
            lo_nib  <= 4'h0;
            wr_byte <= 8'h0;
            wr_push <= 1'b0;
            wr_last <= 1'b0;
            hold    <= 8'h0;
            hold_v  <= 1'b0;
        end else begin
            wr_push <= 1'b0;
            wr_last <= 1'b0;

            if (mii_rx_dv) begin
                if (!phase) begin
                    // MII sends the low nibble of each byte first.
                    lo_nib <= mii_rxd;
                    phase  <= 1'b1;
                end else begin
                    // Push the PREVIOUS byte, keep this one back in case it
                    // turns out to be the last.
                    if (hold_v) begin
                        wr_byte <= hold;
                        wr_push <= 1'b1;
                    end
                    hold   <= {mii_rxd, lo_nib};
                    hold_v <= 1'b1;
                    phase  <= 1'b0;
                end
            end else begin
                phase <= 1'b0;
                // DV has fallen: the held byte was the frame's last.
                if (hold_v) begin
                    wr_byte <= hold;
                    wr_push <= 1'b1;
                    wr_last <= 1'b1;
                    hold_v  <= 1'b0;
                end
            end
        end
    end

    // ---- asynchronous FIFO --------------------------------------------------
    // 9 bits wide: 8 of data plus the last flag, so the boundary cannot arrive
    // out of step with the byte it belongs to.
    reg [8:0] fifo [0:DEPTH-1];

    reg [AW:0] wptr_bin, wptr_gray;
    reg [AW:0] rptr_bin, rptr_gray;
    reg [AW:0] wptr_gray_s1, wptr_gray_s2;
    reg [AW:0] rptr_gray_s1, rptr_gray_s2;

    function automatic [AW:0] bin2gray(input [AW:0] b);
        bin2gray = b ^ (b >> 1);
    endfunction

    wire [AW:0] wptr_next = wptr_bin + 1'b1;
    wire full = (bin2gray(wptr_next) == {~rptr_gray_s2[AW:AW-1],
                                         rptr_gray_s2[AW-2:0]});

    // write side
    always @(posedge rx_clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr_bin  <= '0;
            wptr_gray <= '0;
        end else if (wr_push && !full) begin
            fifo[wptr_bin[AW-1:0]] <= {wr_last, wr_byte};
            wptr_bin  <= wptr_next;
            wptr_gray <= bin2gray(wptr_next);
        end
    end

    // pointer crossings
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr_gray_s1 <= '0;
            wptr_gray_s2 <= '0;
        end else begin
            wptr_gray_s1 <= wptr_gray;
            wptr_gray_s2 <= wptr_gray_s1;
        end
    end

    always @(posedge rx_clk or negedge rst_n) begin
        if (!rst_n) begin
            rptr_gray_s1 <= '0;
            rptr_gray_s2 <= '0;
        end else begin
            rptr_gray_s1 <= rptr_gray;
            rptr_gray_s2 <= rptr_gray_s1;
        end
    end

    wire empty = (rptr_gray == wptr_gray_s2);

    // read side
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rptr_bin  <= '0;
            rptr_gray <= '0;
            out_data  <= 8'h0;
            out_valid <= 1'b0;
            out_last  <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            out_last  <= 1'b0;
            if (!empty) begin
                // Every entry is a real byte, including the one carrying
                // `last`, so valid is asserted unconditionally. eth_mac_rx
                // tests `rx_valid && rx_last` together and would never see the
                // end of a frame otherwise.
                {out_last, out_data} <= fifo[rptr_bin[AW-1:0]];
                out_valid <= 1'b1;
                rptr_bin  <= rptr_bin + 1'b1;
                rptr_gray <= bin2gray(rptr_bin + 1'b1);
            end
        end
    end

endmodule
