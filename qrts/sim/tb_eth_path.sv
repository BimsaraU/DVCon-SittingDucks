// =============================================================================
// tb_eth_path.sv — the whole Ethernet path, nibbles in to nibbles out
//
//   MII nibbles -> mii_rx_adapter -> eth_mac_rx -> eth_cmd_engine -> SDRAM
//                                                        |
//   MII nibbles <- mii_tx_adapter <- eth_mac_tx  <--------+
//
// Every block here passed its own bench. That is exactly why this one exists:
// eth_cmd_engine passed 30/30 against stimulus that asserted rx_eof together
// with rx_valid, and eth_mac_rx emits eof as a SEPARATE cycle with valid low.
// The engine was therefore unable to see the end of any frame -- no payload
// tail written, no seq bit set, no ACK ever sent -- and no unit bench could
// have shown it, because each side was individually correct.
//
// This bench builds a real frame (preamble, SFD, MAC header, FCS) and drives
// it in as nibbles, then checks that the payload reached memory and that a
// reply came back out as nibbles.
// =============================================================================
`timescale 1ns/1ps

module tb_eth_path;

    reg clk = 1'b0;      always #5    clk    = ~clk;   // 100 MHz core
    reg rx_clk = 1'b0;   always #19.7 rx_clk = ~rx_clk; // ~25 MHz from the PHY
    reg tx_clk = 1'b0;   always #20.3 tx_clk = ~tx_clk;

    reg rst_n = 1'b0;

    localparam [47:0] MAC_ADDR = 48'h02_00_00_C0_FF_EE;

    // ---- RX chain -----------------------------------------------------------
    reg  [3:0] mii_rxd   = 4'h0;
    reg        mii_rx_dv = 1'b0;

    wire [7:0] rx_byte;
    wire       rx_valid, rx_last;

    mii_rx_adapter u_mii_rx (
        .rx_clk(rx_clk), .rst_n(rst_n),
        .mii_rxd(mii_rxd), .mii_rx_dv(mii_rx_dv),
        .clk(clk),
        .out_data(rx_byte), .out_valid(rx_valid), .out_last(rx_last)
    );

    wire [7:0]  mac_data;
    wire        mac_valid, mac_sof, mac_eof, mac_err;
    wire [31:0] stat_good, stat_bad_fcs, stat_filtered;

    eth_mac_rx #(.MAC_ADDR(MAC_ADDR)) u_mac_rx (
        .clk(clk), .rst_n(rst_n),
        .rx_data(rx_byte), .rx_valid(rx_valid), .rx_last(rx_last),
        .out_data(mac_data), .out_valid(mac_valid),
        .out_sof(mac_sof), .out_eof(mac_eof), .out_err(mac_err),
        .stat_good(stat_good), .stat_bad_fcs(stat_bad_fcs),
        .stat_filtered(stat_filtered)
    );

    wire [7:0]  cmd_tx_data;
    wire        cmd_tx_valid, cmd_tx_last, cmd_tx_ready;
    wire [31:0] avm_address, avm_writedata;
    wire        avm_write;
    wire [3:0]  avm_byteenable;
    wire [6:0]  avm_burstcount;
    wire [31:0] stat_frames, stat_bytes;
    wire [63:0] rx_bitmap;

    eth_cmd_engine #(.MAC_ADDR(MAC_ADDR)) u_cmd (
        .clk(clk), .rst_n(rst_n),
        .rx_data(mac_data), .rx_valid(mac_valid),
        .rx_sof(mac_sof), .rx_eof(mac_eof), .rx_err(mac_err),
        .tx_data(cmd_tx_data), .tx_valid(cmd_tx_valid),
        .tx_last(cmd_tx_last), .tx_ready(cmd_tx_ready),
        .avm_address(avm_address), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount), .avm_waitrequest(1'b0),
        .stat_frames(stat_frames), .stat_bytes(stat_bytes),
        .rx_bitmap(rx_bitmap)
    );

    // ---- TX chain -----------------------------------------------------------
    wire [7:0] tx_byte;
    wire       tx_en;
    wire [3:0] mii_txd;
    wire       mii_tx_en;

    eth_mac_tx u_mac_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_data(cmd_tx_data), .tx_valid(cmd_tx_valid),
        .tx_last(cmd_tx_last), .tx_ready(cmd_tx_ready),
        .phy_data(tx_byte), .phy_en(tx_en), .busy()
    );

    mii_tx_adapter u_mii_tx (
        .clk(clk), .rst_n(rst_n),
        .in_data(tx_byte), .in_valid(tx_en), .in_ready(),
        .tx_clk(tx_clk),
        .mii_txd(mii_txd), .mii_tx_en(mii_tx_en)
    );

    // Trace what the MAC emits and what the engine parses.
    integer bidx = 0;
    always @(posedge clk) begin
        if (mac_valid) begin
            if (mac_sof) bidx = 0;
            if (bidx < 34)
                $display("  [mac] byte_i=%0d data=%02x sof=%b", bidx, mac_data, mac_sof);
            bidx = bidx + 1;
        end
        if (mac_eof) $display("  [mac] EOF after %0d bytes", bidx);
    end

    // ---- model memory -------------------------------------------------------
    reg [7:0] mem [0:65535];
    always @(posedge clk)
        if (avm_write) begin
            if (avm_byteenable[0]) mem[avm_address+0] = avm_writedata[7:0];
            if (avm_byteenable[1]) mem[avm_address+1] = avm_writedata[15:8];
            if (avm_byteenable[2]) mem[avm_address+2] = avm_writedata[23:16];
            if (avm_byteenable[3]) mem[avm_address+3] = avm_writedata[31:24];
        end

    // ---- count what leaves on the wire --------------------------------------
    integer tx_nibbles = 0;
    always @(posedge tx_clk) if (mii_tx_en) tx_nibbles = tx_nibbles + 1;

    // ---- CRC32, matching the MAC's -----------------------------------------
    function automatic [31:0] crc32_byte(input [31:0] crc, input [7:0] d);
        integer b;
        reg [31:0] c;
        begin
            c = crc ^ {24'h0, d};
            for (b = 0; b < 8; b = b + 1)
                c = c[0] ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
            crc32_byte = c;
        end
    endfunction

    integer errors = 0;
    task automatic check(input string what, input [31:0] got, input [31:0] exp);
        begin
            if (got !== exp) begin
                $display("  FAIL %-40s got %0h expected %0h", what, got, exp);
                errors = errors + 1;
            end else begin
                $display("  ok   %-40s %0h", what, got);
            end
        end
    endtask

    // ---- drive one byte as two MII nibbles, low first -----------------------
    task automatic put_byte(input [7:0] b);
        begin
            @(negedge rx_clk); mii_rxd = b[3:0]; mii_rx_dv = 1'b1;
            @(negedge rx_clk); mii_rxd = b[7:4]; mii_rx_dv = 1'b1;
        end
    endtask

    // A complete frame: preamble, SFD, dst, src, ethertype, our 16-byte
    // header, payload, FCS. The MAC checks the FCS and filters on dst, so
    // anything wrong here is rejected before the engine sees it.
    reg [31:0] crc;
    task automatic send_frame(input [7:0] opcode, input [31:0] seq,
                              input [31:0] addr, input [15:0] len,
                              input [7:0] first);
        integer i;
        reg [7:0] b;
        begin
            crc = 32'hFFFF_FFFF;

            repeat (7) put_byte(8'h55);
            put_byte(8'hD5);

            for (i = 5; i >= 0; i = i - 1) begin
                b = MAC_ADDR[8*i +: 8]; put_byte(b); crc = crc32_byte(crc, b);
            end
            for (i = 0; i < 6; i = i + 1) begin
                b = 8'h10 + i[7:0];     put_byte(b); crc = crc32_byte(crc, b);
            end

            b = 8'h88; put_byte(b); crc = crc32_byte(crc, b);
            b = 8'hB5; put_byte(b); crc = crc32_byte(crc, b);

            b = 8'h01;         put_byte(b); crc = crc32_byte(crc, b); // ver
            b = opcode;        put_byte(b); crc = crc32_byte(crc, b);
            b = 8'h00;         put_byte(b); crc = crc32_byte(crc, b); // flags
            b = 8'h00;         put_byte(b); crc = crc32_byte(crc, b); // rsv
            for (i = 3; i >= 0; i = i - 1) begin
                b = seq[8*i +: 8];  put_byte(b); crc = crc32_byte(crc, b);
            end
            for (i = 3; i >= 0; i = i - 1) begin
                b = addr[8*i +: 8]; put_byte(b); crc = crc32_byte(crc, b);
            end
            b = len[15:8];     put_byte(b); crc = crc32_byte(crc, b);
            b = len[7:0];      put_byte(b); crc = crc32_byte(crc, b);
            b = 8'h00;         put_byte(b); crc = crc32_byte(crc, b); // rsv2
            b = 8'h00;         put_byte(b); crc = crc32_byte(crc, b);

            for (i = 0; i < len; i = i + 1) begin
                b = first + i[7:0]; put_byte(b); crc = crc32_byte(crc, b);
            end

            // FCS: the residue, inverted, little-endian on the wire.
            crc = ~crc;
            put_byte(crc[7:0]);
            put_byte(crc[15:8]);
            put_byte(crc[23:16]);
            put_byte(crc[31:24]);

            @(negedge rx_clk); mii_rx_dv = 1'b0; mii_rxd = 4'h0;
            repeat (8) @(negedge rx_clk);
            repeat (200) @(posedge clk);
        end
    endtask

    integer i;

    initial begin
        $display("=== tb_eth_path ===");
        for (i = 0; i < 65536; i = i + 1) mem[i] = 8'h00;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);

        // -- a real WRITE_MEM frame all the way to memory ---------------------
        send_frame(8'h01, 32'd0, 32'h0000_1000, 16'd16, 8'hA0);

        check("MAC accepted the frame (good FCS)", stat_good, 32'd1);
        check("no FCS failures",                   stat_bad_fcs, 32'd0);
        check("engine counted the frame",          stat_frames, 32'd1);
        for (i = 0; i < 16; i = i + 1)
            check($sformatf("payload[%0d] in memory", i),
                  mem[32'h1000 + i], 8'hA0 + i);
        check("seq 0 recorded", rx_bitmap[0], 1'b1);

        // -- a second frame, unaligned length ---------------------------------
        send_frame(8'h01, 32'd1, 32'h0000_2000, 16'd7, 8'h60);
        for (i = 0; i < 7; i = i + 1)
            check($sformatf("frame2 payload[%0d]", i),
                  mem[32'h2000 + i], 8'h60 + i);
        check("byte past the tail untouched", mem[32'h2007], 8'h00);
        check("seq 1 recorded", rx_bitmap[1], 1'b1);

        // -- ACK_REQ comes back out on the wire -------------------------------
        // The reply has to traverse eth_mac_tx (which generates its own
        // preamble and FCS) and mii_tx_adapter. Checking that nibbles actually
        // leave is what proves the transmit handshake works end to end.
        tx_nibbles = 0;
        send_frame(8'h06, 32'd2, 32'h0, 16'd0, 8'h0);
        repeat (3000) @(posedge clk);

        if (tx_nibbles == 0) begin
            $display("  FAIL no reply left on the wire");
            errors = errors + 1;
        end else begin
            // preamble 8 + dst 6 + src 6 + type 2 + 8 payload + pad + FCS 4,
            // padded to the 60-byte minimum = 68 bytes = 136 nibbles.
            $display("  ok   reply transmitted (%0d nibbles)", tx_nibbles);
        end

        $display("=== %s: %0d error(s) ===",
                 errors == 0 ? "PASSED" : "FAILED", errors);
        $finish;
    end

    initial begin
        #8000000;
        $display("=== FAILED: timeout ===");
        $finish;
    end

endmodule
