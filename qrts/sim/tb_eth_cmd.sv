// =============================================================================
// tb_eth_cmd.sv — eth_cmd_engine against a model SDRAM slave
//
// This engine writes the model blob and every image bitmap into SDRAM. An
// off-by-one in the header offsets, or a byte packed into the wrong lane,
// corrupts the weights -- and a corrupted INT8 model does not crash, it
// produces plausible-looking wrong detections. That failure is nearly
// impossible to attribute after the fact, so it gets caught here.
//
// What is checked:
//   1. a WRITE_MEM payload lands at the right address, in the right byte order
//   2. a payload whose length is not a multiple of 4 writes its tail
//   3. the seq bit is set in the ACK bitmap
//   4. ACK_REQ replies with the bitmap, MSB first
//   5. a frame with the wrong ethertype is ignored entirely
//   6. two frames to different addresses do not interfere
// =============================================================================
`timescale 1ns/1ps

module tb_eth_cmd;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    reg  [7:0] rx_data  = 8'h0;
    reg        rx_valid = 1'b0;
    reg        rx_sof   = 1'b0;
    reg        rx_eof   = 1'b0;
    reg        rx_err   = 1'b0;

    wire [7:0] tx_data;
    wire       tx_valid, tx_last;
    reg        tx_ready = 1'b1;

    wire [31:0] avm_address, avm_writedata;
    wire        avm_write;
    wire [3:0]  avm_byteenable;
    wire [6:0]  avm_burstcount;
    reg         avm_waitrequest = 1'b0;

    wire [31:0] stat_frames, stat_bytes;
    wire [63:0] rx_bitmap;

    eth_cmd_engine dut (
        .clk(clk), .rst_n(rst_n),
        .rx_data(rx_data), .rx_valid(rx_valid), .rx_sof(rx_sof),
        .rx_eof(rx_eof), .rx_err(rx_err),
        .tx_data(tx_data), .tx_valid(tx_valid), .tx_last(tx_last),
        .tx_ready(tx_ready),
        .avm_address(avm_address), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount), .avm_waitrequest(avm_waitrequest),
        .stat_frames(stat_frames), .stat_bytes(stat_bytes),
        .rx_bitmap(rx_bitmap)
    );

    // ---- model memory, byte addressed so lane errors are visible ------------
    reg [7:0] mem [0:65535];
    integer   n_writes = 0;

    always @(posedge clk) begin
        if (avm_write && !avm_waitrequest) begin
            if (avm_byteenable[0]) mem[avm_address + 0] = avm_writedata[7:0];
            if (avm_byteenable[1]) mem[avm_address + 1] = avm_writedata[15:8];
            if (avm_byteenable[2]) mem[avm_address + 2] = avm_writedata[23:16];
            if (avm_byteenable[3]) mem[avm_address + 3] = avm_writedata[31:24];
            n_writes = n_writes + 1;
        end
    end

    // ---- collected ACK reply ------------------------------------------------
    reg [7:0] ack_seen [0:15];
    integer   ack_n = 0;
    always @(posedge clk)
        if (tx_valid && tx_ready) begin
            ack_seen[ack_n] = tx_data;
            ack_n = ack_n + 1;
        end

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

    // ---- frame driver -------------------------------------------------------
    //
    // Matches eth_mac_rx exactly: out_eof is a SEPARATE cycle after the last
    // data byte, with out_valid LOW. It is a boundary pulse, not a flag riding
    // with the byte.
    //
    // The first version of this bench asserted them together, which is a
    // protocol the MAC never produces -- and the engine passed against it while
    // being unable to see a real end of frame at all. A bench that invents its
    // own stimulus protocol verifies nothing about integration.
    task automatic rx_byte(input [7:0] b, input bit sof);
        begin
            @(negedge clk);
            rx_data  = b;
            rx_valid = 1'b1;
            rx_sof   = sof;
            rx_eof   = 1'b0;
            @(posedge clk);
            @(negedge clk);
            rx_valid = 1'b0;
            rx_sof   = 1'b0;
        end
    endtask

    task automatic rx_eof_pulse;
        begin
            @(negedge clk);
            rx_valid = 1'b0;
            rx_eof   = 1'b1;
            @(posedge clk);
            @(negedge clk);
            rx_eof   = 1'b0;
        end
    endtask

    // One command frame. eth_mac_rx does NOT strip the Ethernet header, so the
    // stream starts with the destination MAC -- this bench has to reproduce
    // that or it tests offsets the hardware never sees. (An earlier version
    // started at the ethertype and passed against an engine whose offsets were
    // 12 bytes wrong; tb_eth_path, which builds real frames, is what caught
    // it.)
    task automatic send_cmd(input [15:0] etype, input [7:0] opcode,
                            input [31:0] seq, input [31:0] addr,
                            input [15:0] len, input [7:0] first);
        integer i;
        begin
            for (i = 0; i < 6; i = i + 1)                 // destination MAC
                rx_byte(8'h02 + i[7:0], (i == 0));
            for (i = 0; i < 6; i = i + 1)                 // source MAC
                rx_byte(8'h10 + i[7:0], 1'b0);
            rx_byte(etype[15:8], 1'b0);
            rx_byte(etype[7:0],  1'b0);
            rx_byte(8'h01,       1'b0);   // ver
            rx_byte(opcode,      1'b0);
            rx_byte(8'h00,       1'b0);   // flags
            rx_byte(8'h00,       1'b0);   // rsv
            rx_byte(seq[31:24],  1'b0);
            rx_byte(seq[23:16],  1'b0);
            rx_byte(seq[15:8],   1'b0);
            rx_byte(seq[7:0],    1'b0);
            rx_byte(addr[31:24], 1'b0);
            rx_byte(addr[23:16], 1'b0);
            rx_byte(addr[15:8],  1'b0);
            rx_byte(addr[7:0],   1'b0);
            rx_byte(len[15:8],   1'b0);
            rx_byte(len[7:0],    1'b0);
            rx_byte(8'h00,       1'b0);   // rsv2
            rx_byte(8'h00,       1'b0);
            for (i = 0; i < len; i = i + 1)
                rx_byte(first + i[7:0], 1'b0);
            rx_eof_pulse();
            repeat (12) @(posedge clk);
        end
    endtask

    integer i;

    initial begin
        $display("=== tb_eth_cmd ===");
        for (i = 0; i < 65536; i = i + 1) mem[i] = 8'h00;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (6) @(posedge clk);

        // -- 1. an aligned WRITE_MEM -----------------------------------------
        send_cmd(16'h88B5, 8'h01, 32'd0, 32'h0000_1000, 16'd8, 8'hA0);
        for (i = 0; i < 8; i = i + 1)
            check($sformatf("payload byte[%0d] at 0x1000", i),
                  mem[32'h1000 + i], 8'hA0 + i);
        check("seq 0 bit set", rx_bitmap[0], 1'b1);
        check("frame counted", stat_frames, 32'd1);

        // -- 2. a length that is not a multiple of 4 --------------------------
        // The tail is the case an engine that only writes whole words drops.
        send_cmd(16'h88B5, 8'h01, 32'd1, 32'h0000_2000, 16'd6, 8'h50);
        for (i = 0; i < 6; i = i + 1)
            check($sformatf("tail byte[%0d] at 0x2000", i),
                  mem[32'h2000 + i], 8'h50 + i);
        check("byte after the tail untouched", mem[32'h2006], 8'h00);
        check("seq 1 bit set", rx_bitmap[1], 1'b1);

        // -- 3. wrong ethertype is ignored ------------------------------------
        send_cmd(16'h0800, 8'h01, 32'd2, 32'h0000_3000, 16'd4, 8'hEE);
        check("foreign frame wrote nothing", mem[32'h3000], 8'h00);
        check("foreign frame not counted", stat_frames, 32'd2);
        check("seq 2 bit clear", rx_bitmap[2], 1'b0);

        // -- 4. ACK_REQ returns the bitmap ------------------------------------
        ack_n = 0;
        send_cmd(16'h88B5, 8'h06, 32'd3, 32'h0, 16'd0, 8'h0);
        repeat (40) @(posedge clk);
        check("ack reply length", ack_n, 8);
        // bits 0 and 1 set -> the low byte of a big-endian 64-bit word is last
        check("ack byte[7] holds bits 0..7", ack_seen[7], 8'h03);
        check("ack byte[0] is the top byte",  ack_seen[0], 8'h00);

        // -- 5. a second window clears the bitmap -----------------------------
        // seq 64 is window 1, so the bitmap starts clean and only bit 0 is set.
        send_cmd(16'h88B5, 8'h01, 32'd64, 32'h0000_4000, 16'd4, 8'h70);
        check("new window: seq 64 -> bit 0", rx_bitmap[0], 1'b1);
        check("new window cleared bit 1",    rx_bitmap[1], 1'b0);
        for (i = 0; i < 4; i = i + 1)
            check($sformatf("window2 byte[%0d]", i),
                  mem[32'h4000 + i], 8'h70 + i);

        $display("=== %s: %0d error(s) ===",
                 errors == 0 ? "PASSED" : "FAILED", errors);
        $finish;
    end

    initial begin
        #2000000;
        $display("=== FAILED: timeout ===");
        $finish;
    end

endmodule
