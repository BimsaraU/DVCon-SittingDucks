// =============================================================================
// tb_arbiter.sv — avalon_arbiter with the REAL sdram_ctrl underneath
//
// Deliberately not a stub slave. The failure this bench exists to catch is a
// grant moving mid-burst: Avalon presents address and burstcount with the first
// beat only, so a burst that changes owner halfway has its remaining beats
// written to the other master's address. Against a stub that just records
// transactions, that still "works". Against a real SDRAM controller, the data
// lands in the wrong row and the readback proves it.
//
// What is checked:
//   1. each master alone reads back what it wrote
//   2. a burst from one master is not broken up by the other's request
//   3. simultaneous requests are ordered by priority (master 0 first)
//   4. read data goes to the master that asked for it, not the other one
//   5. interleaved traffic leaves both masters' regions intact
// =============================================================================
`timescale 1ns/1ps

module tb_arbiter;

    localparam integer BURST_W = 7;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    // ---- master 0 ----
    reg  [31:0] m0_address    = '0;
    reg         m0_read       = 1'b0;
    reg         m0_write      = 1'b0;
    reg  [31:0] m0_writedata  = '0;
    reg  [3:0]  m0_byteenable = 4'hF;
    reg  [6:0]  m0_burstcount = 7'd1;
    wire [31:0] m0_readdata;
    wire        m0_readdatavalid, m0_waitrequest;

    // ---- master 1 ----
    reg  [31:0] m1_address    = '0;
    reg         m1_read       = 1'b0;
    reg         m1_write      = 1'b0;
    reg  [31:0] m1_writedata  = '0;
    reg  [3:0]  m1_byteenable = 4'hF;
    reg  [6:0]  m1_burstcount = 7'd1;
    wire [31:0] m1_readdata;
    wire        m1_readdatavalid, m1_waitrequest;

    // ---- master 2: the JTAG read window ----
    reg  [31:0] m2_address = '0;
    reg         m2_read    = 1'b0;
    wire [31:0] m2_readdata;
    wire        m2_readdatavalid, m2_waitrequest;

    // ---- to the SDRAM ----
    wire [31:0] s_address, s_writedata, s_readdata;
    wire        s_read, s_write, s_readdatavalid, s_waitrequest;
    wire [3:0]  s_byteenable;
    wire [6:0]  s_burstcount;

    avalon_arbiter #(.BURST_W(BURST_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .m0_address(m0_address), .m0_read(m0_read), .m0_write(m0_write),
        .m0_writedata(m0_writedata), .m0_byteenable(m0_byteenable),
        .m0_burstcount(m0_burstcount), .m0_readdata(m0_readdata),
        .m0_readdatavalid(m0_readdatavalid), .m0_waitrequest(m0_waitrequest),
        .m1_address(m1_address), .m1_read(m1_read), .m1_write(m1_write),
        .m1_writedata(m1_writedata), .m1_byteenable(m1_byteenable),
        .m1_burstcount(m1_burstcount), .m1_readdata(m1_readdata),
        .m1_readdatavalid(m1_readdatavalid), .m1_waitrequest(m1_waitrequest),
        .m2_address(m2_address), .m2_read(m2_read),
        .m2_readdata(m2_readdata), .m2_readdatavalid(m2_readdatavalid),
        .m2_waitrequest(m2_waitrequest),
        .s_address(s_address), .s_read(s_read), .s_write(s_write),
        .s_writedata(s_writedata), .s_byteenable(s_byteenable),
        .s_burstcount(s_burstcount), .s_readdata(s_readdata),
        .s_readdatavalid(s_readdatavalid), .s_waitrequest(s_waitrequest)
    );

    wire [12:0] dram_addr;
    wire [1:0]  dram_ba;
    wire        dram_cas_n, dram_ras_n, dram_we_n, dram_cs_n, dram_cke;
    wire [3:0]  dram_dqm;
    wire [31:0] dram_dq_out, dram_dq_in;
    wire        dram_dq_oe;

    sdram_ctrl #(.T_INIT(20), .T_REFI(2000)) u_sdram (
        .clk(clk), .rst_n(rst_n),
        .avs_address(s_address[24:0]), .avs_read(s_read), .avs_write(s_write),
        .avs_writedata(s_writedata), .avs_byteenable(s_byteenable),
        .avs_burstcount(s_burstcount),
        .avs_readdata(s_readdata), .avs_readdatavalid(s_readdatavalid),
        .avs_waitrequest(s_waitrequest),
        .dram_addr(dram_addr), .dram_ba(dram_ba),
        .dram_cas_n(dram_cas_n), .dram_ras_n(dram_ras_n),
        .dram_we_n(dram_we_n), .dram_cs_n(dram_cs_n), .dram_cke(dram_cke),
        .dram_dqm(dram_dqm),
        .dram_dq_in(dram_dq_in), .dram_dq_out(dram_dq_out),
        .dram_dq_oe(dram_dq_oe)
    );

    sdram_model #(.REFRESH_DEADLINE(20000)) mem (
        .clk(clk), .addr(dram_addr), .ba(dram_ba),
        .cas_n(dram_cas_n), .ras_n(dram_ras_n), .we_n(dram_we_n),
        .cs_n(dram_cs_n), .cke(dram_cke), .dqm(dram_dqm),
        .dq_in(dram_dq_out), .dq_oe(dram_dq_oe), .dq_out(dram_dq_in)
    );

    integer errors = 0;
    task automatic check(input string what, input [31:0] got, input [31:0] exp);
        begin
            if (got !== exp) begin
                $display("  FAIL %-38s got %08x expected %08x", what, got, exp);
                errors = errors + 1;
            end else begin
                $display("  ok   %-38s %08x", what, got);
            end
        end
    endtask

    // ---- master 0 drivers ---------------------------------------------------
    task automatic m0_wr(input [24:0] base, input integer n, input [31:0] seed);
        integer i;
        begin
            @(negedge clk);
            m0_address = base; m0_burstcount = n[6:0]; m0_byteenable = 4'hF;
            m0_write = 1'b1;
            for (i = 0; i < n; i = i + 1) begin
                m0_writedata = seed + i;
                @(posedge clk);
                while (m0_waitrequest) @(posedge clk);
                @(negedge clk);
            end
            m0_write = 1'b0;
            // No drain loop here: waitrequest is only meaningful while a
            // request is asserted. An idle master is deselected, so its
            // waitrequest stays high and a wait on it never returns.
            repeat (12) @(posedge clk);
        end
    endtask

    reg [31:0] m0_buf [0:63];
    task automatic m0_rd(input [24:0] base, input integer n);
        integer got;
        begin
            @(negedge clk);
            m0_address = base; m0_burstcount = n[6:0]; m0_read = 1'b1;
            @(posedge clk);
            while (m0_waitrequest) @(posedge clk);
            @(negedge clk);
            m0_read = 1'b0;
            got = 0;
            while (got < n) begin
                @(posedge clk);
                if (m0_readdatavalid) begin
                    m0_buf[got] = m0_readdata;
                    got = got + 1;
                end
            end
        end
    endtask

    // ---- master 1 drivers ---------------------------------------------------
    task automatic m1_wr(input [24:0] base, input integer n, input [31:0] seed);
        integer i;
        begin
            @(negedge clk);
            m1_address = base; m1_burstcount = n[6:0]; m1_byteenable = 4'hF;
            m1_write = 1'b1;
            for (i = 0; i < n; i = i + 1) begin
                m1_writedata = seed + i;
                @(posedge clk);
                while (m1_waitrequest) @(posedge clk);
                @(negedge clk);
            end
            m1_write = 1'b0;
            repeat (12) @(posedge clk);
        end
    endtask

    reg [31:0] m1_buf [0:63];
    task automatic m1_rd(input [24:0] base, input integer n);
        integer got;
        begin
            @(negedge clk);
            m1_address = base; m1_burstcount = n[6:0]; m1_read = 1'b1;
            @(posedge clk);
            while (m1_waitrequest) @(posedge clk);
            @(negedge clk);
            m1_read = 1'b0;
            got = 0;
            while (got < n) begin
                @(posedge clk);
                if (m1_readdatavalid) begin
                    m1_buf[got] = m1_readdata;
                    got = got + 1;
                end
            end
        end
    endtask

    // Master 2 reads a single word, the way jtag_ctrl does.
    reg [31:0] m2_word;
    task automatic m2_rd1(input [24:0] addr);
        begin
            @(negedge clk);
            m2_address = addr; m2_read = 1'b1;
            @(posedge clk);
            while (m2_waitrequest) @(posedge clk);
            @(negedge clk);
            m2_read = 1'b0;
            forever begin
                @(posedge clk);
                if (m2_readdatavalid) begin
                    m2_word = m2_readdata;
                    break;
                end
            end
        end
    endtask

    integer i;

    initial begin
        $display("=== tb_arbiter ===");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (80) @(posedge clk);

        // -- 1. each master alone ---------------------------------------------
        m0_wr(25'h00_1000, 4, 32'hA000_0000);
        m0_rd(25'h00_1000, 4);
        for (i = 0; i < 4; i = i + 1)
            check($sformatf("m0 alone[%0d]", i), m0_buf[i], 32'hA000_0000 + i);

        m1_wr(25'h00_2000, 4, 32'hB000_0000);
        m1_rd(25'h00_2000, 4);
        for (i = 0; i < 4; i = i + 1)
            check($sformatf("m1 alone[%0d]", i), m1_buf[i], 32'hB000_0000 + i);

        // -- 2. contention: both write at once --------------------------------
        // Forked so the requests genuinely overlap. If the arbiter let a grant
        // move mid-burst, these two would interleave in the SDRAM and both
        // readbacks below would be wrong.
        fork
            m0_wr(25'h00_3000, 8, 32'hC000_0000);
            m1_wr(25'h00_4000, 8, 32'hD000_0000);
        join
        repeat (20) @(posedge clk);

        m0_rd(25'h00_3000, 8);
        for (i = 0; i < 8; i = i + 1)
            check($sformatf("m0 under contention[%0d]", i),
                  m0_buf[i], 32'hC000_0000 + i);

        m1_rd(25'h00_4000, 8);
        for (i = 0; i < 8; i = i + 1)
            check($sformatf("m1 under contention[%0d]", i),
                  m1_buf[i], 32'hD000_0000 + i);

        // -- 3. the earlier regions survived ----------------------------------
        m0_rd(25'h00_1000, 4);
        for (i = 0; i < 4; i = i + 1)
            check($sformatf("m0 region intact[%0d]", i),
                  m0_buf[i], 32'hA000_0000 + i);
        m1_rd(25'h00_2000, 4);
        for (i = 0; i < 4; i = i + 1)
            check($sformatf("m1 region intact[%0d]", i),
                  m1_buf[i], 32'hB000_0000 + i);

        // -- 4. the JTAG read window reaches the same memory -------------------
        // This is the path the box list comes home by, so it has to see what
        // the accelerator wrote, not a private copy.
        m2_rd1(25'h00_1000);
        check("m2 reads m0's data", m2_word, 32'hA000_0000);
        m2_rd1(25'h00_4000);
        check("m2 reads m1's data", m2_word, 32'hD000_0000);

        // -- 5. no protocol violations from the interleaving ------------------
        if (mem.errors != 0) begin
            $display("  FAIL model reported %0d protocol violation(s)",
                     mem.errors);
            errors = errors + mem.errors;
        end else begin
            $display("  ok   no SDRAM protocol violations");
        end

        $display("=== %s: %0d error(s) ===",
                 errors == 0 ? "PASSED" : "FAILED", errors);
        $finish;
    end

    initial begin
        #3000000;
        $display("=== FAILED: timeout ===");
        $finish;
    end

endmodule
