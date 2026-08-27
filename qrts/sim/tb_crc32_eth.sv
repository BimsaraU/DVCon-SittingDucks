// =============================================================================
// tb_crc32_eth.sv — CRC against known-answer vectors
//
// A wrong FCS is the worst kind of Ethernet bug: the link looks alive, frames
// leave the FPGA, and the host's NIC drops every one of them in hardware with
// no error anywhere the FPGA can see. So this checks against values computed
// independently (zlib.crc32 in the generator comment), not against another
// copy of the same algorithm.
//
// Two properties, both load-bearing:
//   1. crc_out over a payload equals the standard CRC-32 of that payload.
//   2. Feeding payload + its FCS back in leaves the residue, so crc_ok rises.
//      That is how eth_mac_rx validates without buffering the last four bytes.
// =============================================================================
`timescale 1ns/1ps

module tb_crc32_eth;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    reg        init = 1'b0, en = 1'b0;
    reg  [7:0] data_in = 8'h0;
    wire [31:0] crc, crc_out;
    wire        crc_ok;

    crc32_eth dut (
        .clk(clk), .rst_n(rst_n),
        .init(init), .en(en), .data_in(data_in),
        .crc(crc), .crc_ok(crc_ok), .crc_out(crc_out)
    );

    integer errors = 0;

    task automatic feed(input [7:0] b);
        begin
            @(posedge clk);
            data_in <= b;
            en      <= 1'b1;
            @(posedge clk);
            en <= 1'b0;
        end
    endtask

    task automatic restart;
        begin
            @(posedge clk);
            init <= 1'b1;
            @(posedge clk);
            init <= 1'b0;
        end
    endtask

    // Known answers, from zlib.crc32:
    //   crc32(b"123456789")            = 0xCBF43926
    //   crc32(bytes(range(16)))        = 0xCECEE288
    //   crc32(b"\x00"*4)               = 0x2144DF1C
    task automatic check_payload(input [255:0] name,
                                  input integer n,
                                  input [31:0] expect_crc);
        integer i;
        reg [7:0] b;
        reg [31:0] fcs;
        begin
            restart();
            for (i = 0; i < n; i = i + 1) begin
                case (name[7:0])
                8'd1: b = "1" + i[7:0];            // "123456789"
                8'd2: b = i[7:0];                  // 0x00..0x0F
                default: b = 8'h00;                // zeros
                endcase
                feed(b);
            end
            @(posedge clk);

            if (crc_out !== expect_crc) begin
                errors = errors + 1;
                $display("  FAIL: crc_out = %08h, expected %08h", crc_out, expect_crc);
            end else begin
                $display("  ok  : crc_out = %08h", crc_out);
            end

            // Now append the FCS little-endian, as it goes on the wire, and
            // confirm the residue check fires.
            fcs = crc_out;
            feed(fcs[7:0]);
            feed(fcs[15:8]);
            feed(fcs[23:16]);
            feed(fcs[31:24]);
            @(posedge clk);
            if (crc_ok !== 1'b1) begin
                errors = errors + 1;
                $display("  FAIL: residue check did not fire (crc=%08h)", crc);
            end else begin
                $display("  ok  : residue check fires on a good FCS");
            end

            // One corrupted bit must break it -- otherwise the check is
            // decorative and bad frames reach the command engine.
            feed(8'h01);
            @(posedge clk);
            if (crc_ok === 1'b1) begin
                errors = errors + 1;
                $display("  FAIL: residue still valid after a trailing byte");
            end else begin
                $display("  ok  : residue clears when the frame is disturbed");
            end
        end
    endtask

    initial begin
        $display("=== tb_crc32_eth ===");
        rst_n = 1'b0;
        repeat (6) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        $display("-- \"123456789\" --");
        check_payload({248'd0, 8'd1}, 9,  32'hCBF43926);
        $display("-- 0x00..0x0F --");
        check_payload({248'd0, 8'd2}, 16, 32'hCECEE288);
        $display("-- four zero bytes --");
        check_payload({248'd0, 8'd3}, 4,  32'h2144DF1C);

        if (errors == 0) $display("=== PASSED: 0 error(s) ===");
        else             $display("=== FAILED: %0d error(s) ===", errors);
        $finish;
    end

endmodule
