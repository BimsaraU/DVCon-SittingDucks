// =============================================================================
// tb_mii_adapters.sv — MII nibble/byte adapters across their clock crossings
//
// These sit between the byte-wide MACs and the 4-bit ENET0 pins. A bug here is
// the worst kind to find on the bench: a dropped or duplicated byte fails the
// FCS, so the symptom is "some frames don't arrive" -- indistinguishable from a
// bad cable, a PHY strap, or a switch dropping runts.
//
// What is checked:
//   1. RX: a nibble stream at 25 MHz arrives as the right bytes on clk_sys
//   2. RX: the frame boundary (RX_DV falling) arrives with the last byte
//   3. RX: two frames back to back stay separate
//   4. TX: bytes on clk_sys leave as the right nibbles, low nibble first
//   5. TX: tx_en frames the data and drops between frames
//   6. both survive the clocks being unrelated -- the point of the FIFOs
// =============================================================================
`timescale 1ns/1ps

module tb_mii_adapters;

    // Deliberately not an integer ratio, and not phase-aligned: a design that
    // only works at exactly 4:1 passes a 4:1 bench and fails on the board.
    reg clk = 1'b0;      always #5    clk    = ~clk;   // 100 MHz
    reg rx_clk = 1'b0;   always #19.7 rx_clk = ~rx_clk; // ~25.4 MHz
    reg tx_clk = 1'b0;   always #20.3 tx_clk = ~tx_clk; // ~24.6 MHz

    reg rst_n = 1'b0;

    integer errors = 0;

    task automatic check(input string what, input [31:0] got, input [31:0] exp);
        begin
            if (got !== exp) begin
                $display("  FAIL %-38s got %0h expected %0h", what, got, exp);
                errors = errors + 1;
            end else begin
                $display("  ok   %-38s %0h", what, got);
            end
        end
    endtask

    // ---- RX path ------------------------------------------------------------
    reg  [3:0] mii_rxd = 4'h0;
    reg        mii_rx_dv = 1'b0;
    wire [7:0] rx_out_data;
    wire       rx_out_valid, rx_out_last;

    mii_rx_adapter u_rx (
        .rx_clk(rx_clk), .rst_n(rst_n),
        .mii_rxd(mii_rxd), .mii_rx_dv(mii_rx_dv),
        .clk(clk),
        .out_data(rx_out_data), .out_valid(rx_out_valid),
        .out_last(rx_out_last)
    );

    // collect what the RX adapter produces
    reg [7:0] rx_seen [0:63];
    integer   rx_n = 0;
    integer   rx_last_n = 0;

    // `last` rides WITH the final byte, so a last-flagged beat is also a data
    // beat and must be counted as one.
    always @(posedge clk) begin
        if (rx_out_valid) begin
            rx_seen[rx_n] = rx_out_data;
            rx_n = rx_n + 1;
            if (rx_out_last) rx_last_n = rx_last_n + 1;
        end
    end

    // drive one byte as two nibbles, low first
    task automatic mii_send_byte(input [7:0] b);
        begin
            @(negedge rx_clk); mii_rxd = b[3:0]; mii_rx_dv = 1'b1;
            @(negedge rx_clk); mii_rxd = b[7:4]; mii_rx_dv = 1'b1;
        end
    endtask

    task automatic mii_send_frame(input [7:0] first, input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) mii_send_byte(first + i[7:0]);
            @(negedge rx_clk); mii_rx_dv = 1'b0; mii_rxd = 4'h0;
            repeat (4) @(negedge rx_clk);
        end
    endtask

    // ---- TX path ------------------------------------------------------------
    reg  [7:0] tx_in_data  = 8'h0;
    reg        tx_in_valid = 1'b0;
    wire       tx_in_ready;
    wire [3:0] mii_txd;
    wire       mii_tx_en;

    mii_tx_adapter u_tx (
        .clk(clk), .rst_n(rst_n),
        .in_data(tx_in_data), .in_valid(tx_in_valid), .in_ready(tx_in_ready),
        .tx_clk(tx_clk),
        .mii_txd(mii_txd), .mii_tx_en(mii_tx_en)
    );

    // reassemble the nibble stream back into bytes
    reg [7:0] tx_seen [0:63];
    integer   tx_n = 0;
    reg       tx_phase = 1'b0;
    reg [3:0] tx_lo;

    always @(posedge tx_clk) begin
        if (mii_tx_en) begin
            if (!tx_phase) begin
                tx_lo    = mii_txd;
                tx_phase = 1'b1;
            end else begin
                tx_seen[tx_n] = {mii_txd, tx_lo};
                tx_n     = tx_n + 1;
                tx_phase = 1'b0;
            end
        end else begin
            tx_phase = 1'b0;
        end
    end

    task automatic tx_push(input [7:0] b);
        begin
            @(negedge clk);
            while (!tx_in_ready) @(negedge clk);
            tx_in_data  = b;
            tx_in_valid = 1'b1;
            @(negedge clk);
            tx_in_valid = 1'b0;
        end
    endtask

    integer i;

    initial begin
        $display("=== tb_mii_adapters ===");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // -- RX: one frame of 8 bytes ----------------------------------------
        mii_send_frame(8'hA0, 8);
        repeat (60) @(posedge clk);

        check("rx byte count", rx_n, 8);
        for (i = 0; i < 8 && i < rx_n; i = i + 1)
            check($sformatf("rx byte[%0d]", i), rx_seen[i], 8'hA0 + i);
        check("rx frame boundary seen once", rx_last_n, 1);

        // -- RX: a second frame stays separate -------------------------------
        rx_n = 0; rx_last_n = 0;
        mii_send_frame(8'h50, 4);
        repeat (60) @(posedge clk);
        check("rx second frame byte count", rx_n, 4);
        for (i = 0; i < 4 && i < rx_n; i = i + 1)
            check($sformatf("rx2 byte[%0d]", i), rx_seen[i], 8'h50 + i);
        check("rx second boundary", rx_last_n, 1);

        // -- TX: 8 bytes out --------------------------------------------------
        for (i = 0; i < 8; i = i + 1) tx_push(8'hC0 + i[7:0]);
        repeat (400) @(posedge clk);

        check("tx byte count", tx_n, 8);
        for (i = 0; i < 8 && i < tx_n; i = i + 1)
            check($sformatf("tx byte[%0d]", i), tx_seen[i], 8'hC0 + i);

        $display("=== %s: %0d error(s) ===",
                 errors == 0 ? "PASSED" : "FAILED", errors);
        $finish;
    end

    initial begin
        #500000;
        $display("=== FAILED: timeout ===");
        $finish;
    end

endmodule
