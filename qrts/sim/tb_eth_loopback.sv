// =============================================================================
// tb_eth_loopback.sv — eth_mac_tx into eth_mac_rx
//
// The plan's Ethernet gate: good FCS accepted, bad FCS dropped, wrong MAC
// dropped, TX CRC verifies.
//
// Looping TX into RX is the strongest cheap check available: TX builds a frame
// from scratch (preamble, padding, FCS byte order) and RX takes it apart
// (preamble strip, address filter, residue check). A disagreement anywhere --
// FCS endianness, whether padding is covered by the CRC, where the frame body
// starts -- shows up as a frame that does not survive the round trip. Both
// ends being independently wrong in exactly the same way is the one thing this
// cannot catch, which is why crc32_eth is checked against external
// known-answer vectors separately (tb_crc32_eth).
//
// The payload comparison matters as much as the accept/reject: an RX that
// hands on the FCS bytes, or drops the destination MAC, passes a "did it
// accept" test and corrupts every command.
// =============================================================================
`timescale 1ns/1ps

module tb_eth_loopback;

    localparam [47:0] OUR_MAC = 48'h02_00_00_C0_FF_EE;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    integer errors = 0;

    // ---- TX ----
    reg  [7:0] tx_data = 8'h0;
    reg        tx_valid = 1'b0, tx_last = 1'b0;
    wire       tx_ready;
    wire [7:0] phy_data;
    wire       phy_en;
    wire       tx_busy;

    eth_mac_tx u_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_data(tx_data), .tx_valid(tx_valid), .tx_last(tx_last),
        .tx_ready(tx_ready),
        .phy_data(phy_data), .phy_en(phy_en), .busy(tx_busy)
    );

    // ---- wire, with optional corruption ----
    // A real link damages bytes; the RX must reject those frames rather than
    // pass them to the command engine.
    reg        corrupt_en = 1'b0;
    reg [15:0] corrupt_at = 16'd0;
    reg [15:0] wire_cnt = 16'd0;

    wire [7:0] wire_data = (corrupt_en && wire_cnt == corrupt_at)
                           ? (phy_data ^ 8'h5A) : phy_data;

    reg  phy_en_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wire_cnt <= 16'd0;
            phy_en_d <= 1'b0;
        end else begin
            phy_en_d <= phy_en;
            if (phy_en) wire_cnt <= wire_cnt + 16'd1;
            else if (!phy_en && phy_en_d) wire_cnt <= 16'd0;
        end
    end

    // rx_last marks the FINAL byte of the carrier burst, which is only
    // knowable once the carrier has dropped. The stream is therefore delayed
    // by one cycle: stage 1 holds the byte being presented to the RX, and the
    // live phy_en says whether another byte follows it.
    //
    // A real RGMII front end has the same shape -- rx_dv falls after the last
    // nibble pair, so the receiver always trails the wire by a beat.
    reg [7:0] pd1;
    reg       pe1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pd1 <= 8'h0;
            pe1 <= 1'b0;
        end else begin
            pd1 <= wire_data;
            pe1 <= phy_en;
        end
    end

    wire [7:0] rx_data_r  = pd1;
    wire       rx_valid_r = pe1;
    wire       rx_last_r  = pe1 && !phy_en;   // nothing follows this byte

    // ---- RX ----
    wire [7:0] out_data;
    wire       out_valid, out_sof, out_eof, out_err;
    wire [31:0] stat_good, stat_bad_fcs, stat_filtered;

    eth_mac_rx #(.MAC_ADDR(OUR_MAC)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .rx_data(rx_data_r), .rx_valid(rx_valid_r), .rx_last(rx_last_r),
        .out_data(out_data), .out_valid(out_valid),
        .out_sof(out_sof), .out_eof(out_eof), .out_err(out_err),
        .stat_good(stat_good), .stat_bad_fcs(stat_bad_fcs),
        .stat_filtered(stat_filtered)
    );

    // ---- capture what RX hands on ----
    reg [7:0] got [0:2047];
    integer   got_n;
    reg       saw_eof, saw_err;

    always @(posedge clk) begin
        if (out_valid) begin
            got[got_n] = out_data;
            got_n = got_n + 1;
        end
        if (out_eof) saw_eof = 1'b1;
        if (out_err) saw_err = 1'b1;
    end

    // ---- drive one frame body ----
    reg [7:0] body [0:2047];
    integer   body_n;

    // Standard valid/ready producer: present a byte, hold it until the cycle
    // the MAC accepts it, then advance. Presenting and then waiting an extra
    // edge before checking tx_ready hands the same byte over twice, which
    // shifts the whole frame and corrupts the destination address.
    task automatic send_body;
        integer i;
        begin
            i = 0;
            while (i < body_n) begin
                tx_data  <= body[i];
                tx_valid <= 1'b1;
                tx_last  <= (i == body_n - 1);
                @(posedge clk);
                if (tx_ready) i = i + 1;
            end
            tx_valid <= 1'b0;
            tx_last  <= 1'b0;
            @(posedge clk);
        end
    endtask

    task automatic build_body(input [47:0] dst, input integer payload_n);
        integer i;
        begin
            body[0] = dst[47:40]; body[1] = dst[39:32]; body[2] = dst[31:24];
            body[3] = dst[23:16]; body[4] = dst[15:8];  body[5] = dst[7:0];
            // source MAC
            body[6]  = 8'hDE; body[7]  = 8'hAD; body[8]  = 8'hBE;
            body[9]  = 8'hEF; body[10] = 8'h00; body[11] = 8'h01;
            // EtherType 0x88B5, the local-experimental type the protocol uses
            body[12] = 8'h88; body[13] = 8'hB5;
            for (i = 0; i < payload_n; i = i + 1)
                body[14 + i] = i[7:0] ^ 8'h3C;
            body_n = 14 + payload_n;
        end
    endtask

    task automatic run_case(input [255:0] name,
                             input [47:0] dst,
                             input integer payload_n,
                             input integer corrupt,
                             input integer expect_accept);
        integer i, guard, bad;
        begin
            got_n = 0; saw_eof = 1'b0; saw_err = 1'b0;
            corrupt_en = corrupt[0];
            corrupt_at = 16'd20;          // inside the payload

            build_body(dst, payload_n);
            send_body();

            guard = 0;
            while (!saw_eof && !saw_err && guard < 4000) begin
                @(posedge clk);
                guard = guard + 1;
            end

            if (expect_accept) begin
                if (!saw_eof) begin
                    errors = errors + 1;
                    $display("  FAIL: %0s -- frame not accepted (err=%b)",
                             name, saw_err);
                end else if (got_n != body_n && got_n != 60) begin
                    // Padding is part of the frame, so a short body comes back
                    // padded to the 60-byte minimum. Either length is correct.
                    errors = errors + 1;
                    $display("  FAIL: %0s -- got %0d bytes, sent %0d",
                             name, got_n, body_n);
                end else begin
                    bad = 0;
                    for (i = 0; i < body_n; i = i + 1)
                        if (got[i] !== body[i]) bad = bad + 1;
                    if (bad != 0) begin
                        errors = errors + 1;
                        $display("  FAIL: %0s -- %0d payload byte(s) differ",
                                 name, bad);
                    end else begin
                        $display("  ok  : %0s (%0d bytes, payload intact)",
                                 name, got_n);
                    end
                end
            end else begin
                if (saw_eof) begin
                    errors = errors + 1;
                    $display("  FAIL: %0s -- frame accepted, should have been dropped",
                             name);
                end else begin
                    $display("  ok  : %0s (dropped)", name);
                end
            end

            corrupt_en = 1'b0;
            repeat (40) @(posedge clk);
        end
    endtask


    initial begin
        $display("=== tb_eth_loopback ===");
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        run_case("unicast, good FCS",   OUR_MAC,            64, 0, 1);
        run_case("broadcast",           48'hFFFFFFFFFFFF,   64, 0, 1);
        run_case("short body (padded)", OUR_MAC,             4, 0, 1);
        run_case("wrong MAC",           48'h02_00_00_11_22_33, 64, 0, 0);
        run_case("corrupted byte",      OUR_MAC,            64, 1, 0);

        $display("  stats: good=%0d bad_fcs=%0d filtered=%0d",
                 stat_good, stat_bad_fcs, stat_filtered);

        if (errors == 0) $display("=== PASSED: 0 error(s) ===");
        else             $display("=== FAILED: %0d error(s) ===", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("=== FAILED: global timeout ===");
        $finish;
    end

endmodule
