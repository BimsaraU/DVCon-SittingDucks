// =============================================================================
// tb_sdram.sv — sdram_ctrl against the behavioural IS42S16320D pair
//
// The controller replaces avalon_onchip_ram, which is the module that built
// 4096x32 bits out of 131,145 flip-flops and pushed dvcon_top to 357,518 logic
// elements on a 114,480-LE part. Everything downstream -- the ethernet DMA,
// the JTAG control path, the whole frame -- assumes memory works, so this is
// the first thing that has to be right.
//
// What is checked:
//   1. single word write then read back
//   2. burst write then burst read, in order
//   3. byte enables leave the untouched lanes alone
//   4. a burst that crosses a column boundary into the next bank
//   5. reads still correct with refresh preempting traffic
//   6. the model's own protocol assertions (no read from a closed row, no
//      refresh with a bank open, CAS latency as configured)
// =============================================================================
`timescale 1ns/1ps

module tb_sdram;

    localparam integer DATA_W  = 32;
    localparam integer MAX_BURST = 64;
    localparam integer CAS_LAT = 3;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;              // 100 MHz

    reg  [24:0] avs_address    = '0;
    reg         avs_read       = 1'b0;
    reg         avs_write      = 1'b0;
    reg  [31:0] avs_writedata  = '0;
    reg  [3:0]  avs_byteenable = 4'hF;
    reg  [6:0]  avs_burstcount = 7'd1;
    wire [31:0] avs_readdata;
    wire        avs_readdatavalid;
    wire        avs_waitrequest;

    wire [12:0] dram_addr;
    wire [1:0]  dram_ba;
    wire        dram_cas_n, dram_ras_n, dram_we_n, dram_cs_n, dram_cke;
    wire [3:0]  dram_dqm;
    wire [31:0] dram_dq_out, dram_dq_in;
    wire        dram_dq_oe;

    // T_INIT shrunk from 20,000 cycles (200 us) to 20: the power-up wait is
    // real on silicon and pointless in a bench.
    sdram_ctrl #(
        .DATA_W(DATA_W), .MAX_BURST(MAX_BURST), .CAS_LAT(CAS_LAT),
        .T_INIT(20), .T_REFI(300)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .avs_address(avs_address), .avs_read(avs_read), .avs_write(avs_write),
        .avs_writedata(avs_writedata), .avs_byteenable(avs_byteenable),
        .avs_burstcount(avs_burstcount),
        .avs_readdata(avs_readdata), .avs_readdatavalid(avs_readdatavalid),
        .avs_waitrequest(avs_waitrequest),
        .dram_addr(dram_addr), .dram_ba(dram_ba),
        .dram_cas_n(dram_cas_n), .dram_ras_n(dram_ras_n),
        .dram_we_n(dram_we_n), .dram_cs_n(dram_cs_n), .dram_cke(dram_cke),
        .dram_dqm(dram_dqm),
        .dram_dq_in(dram_dq_in), .dram_dq_out(dram_dq_out),
        .dram_dq_oe(dram_dq_oe)
    );

    sdram_model #(
        .DATA_W(DATA_W), .CAS_LAT(CAS_LAT), .REFRESH_DEADLINE(2000)
    ) mem (
        .clk(clk), .addr(dram_addr), .ba(dram_ba),
        .cas_n(dram_cas_n), .ras_n(dram_ras_n), .we_n(dram_we_n),
        .cs_n(dram_cs_n), .cke(dram_cke), .dqm(dram_dqm),
        .dq_in(dram_dq_out), .dq_oe(dram_dq_oe),
        .dq_out(dram_dq_in)
    );

    integer errors = 0;

    task automatic check(input string what, input [31:0] got, input [31:0] exp);
        begin
            if (got !== exp) begin
                $display("  FAIL %-40s got %08x expected %08x", what, got, exp);
                errors = errors + 1;
            end else begin
                $display("  ok   %-40s %08x", what, got);
            end
        end
    endtask

    // ---- Avalon write burst -------------------------------------------------
    // A beat is transferred on a rising edge where waitrequest is low. The
    // driver must HOLD address/data/write across a stalled cycle rather than
    // advancing, which is what an earlier version of this task got wrong: it
    // asserted write while waitrequest was still high from the init sequence
    // and treated the beat as accepted, so the whole bench hung on beat one.
    task automatic wr_burst(input [24:0] base, input integer n,
                            input [31:0] seed, input [3:0] be);
        integer i;
        begin
            // Stimulus changes on the falling edge so it is stable at the
            // rising edge the DUT samples. Driving on the same edge the DUT
            // samples is a race: an earlier version did that and the address
            // reached the controller as 0 instead of 0x100.
            @(negedge clk);
            avs_address    = base;
            avs_burstcount = n[6:0];
            avs_byteenable = be;
            avs_write      = 1'b1;
            for (i = 0; i < n; i = i + 1) begin
                avs_writedata = seed + i;
                // hold this beat until it is accepted
                @(posedge clk);
                while (avs_waitrequest) @(posedge clk);
                @(negedge clk);
            end
            @(negedge clk);
            avs_write     = 1'b0;
            avs_writedata = '0;
            // Wait for the controller to retire the burst (precharge, back to
            // idle) before returning. Without this the next task asserts its
            // request while the write is still in flight, and the controller
            // samples it in a state that ignores it -- the request is lost and
            // the bench waits forever for data that was never requested.
            @(posedge clk);
            while (avs_waitrequest) @(posedge clk);
        end
    endtask

    // ---- Avalon read burst; collects into rdbuf -----------------------------
    reg [31:0] rdbuf [0:127];
    integer    rdn;

    // A read burst is one accepted command; data comes back later as
    // readdatavalid beats, so collection runs after the handshake.
    task automatic rd_burst(input [24:0] base, input integer n);
        integer got;
        begin
            rdn = 0;
            @(negedge clk);
            avs_address    = base;
            avs_burstcount = n[6:0];
            avs_read       = 1'b1;
            @(posedge clk);
            while (avs_waitrequest) @(posedge clk);
            @(negedge clk);
            avs_read = 1'b0;

            got = 0;
            while (got < n) begin
                @(posedge clk);
                if (avs_readdatavalid) begin
                    rdbuf[got] = avs_readdata;
                    got = got + 1;
                end
            end
            rdn = got;
        end
    endtask

    integer i;
    initial begin
        $display("=== tb_sdram ===");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // Let the (shortened) init sequence finish.
        repeat (80) @(posedge clk);

        // -- 1. single word ---------------------------------------------------
        wr_burst(25'h00_0100, 1, 32'hDEAD_BEEF, 4'hF);
        rd_burst(25'h00_0100, 1);
        check("single word readback", rdbuf[0], 32'hDEAD_BEEF);

        // -- 2. burst of 8 ----------------------------------------------------
        wr_burst(25'h00_0200, 8, 32'h1000_0000, 4'hF);
        rd_burst(25'h00_0200, 8);
        for (i = 0; i < 8; i = i + 1)
            check($sformatf("burst[%0d]", i), rdbuf[i], 32'h1000_0000 + i);

        // -- 3. byte enables --------------------------------------------------
        wr_burst(25'h00_0300, 1, 32'hFFFF_FFFF, 4'hF);
        wr_burst(25'h00_0300, 1, 32'h0000_00AA, 4'h1);   // low byte only
        rd_burst(25'h00_0300, 1);
        check("byte enable leaves upper lanes", rdbuf[0], 32'hFFFF_FFAA);

        // -- 4. crossing a column boundary into the next bank -----------------
        // col is 10 bits, so word 0x3FF is the last of a column.
        wr_burst(25'h00_03FE, 4, 32'hC0DE_0000, 4'hF);
        rd_burst(25'h00_03FE, 4);
        for (i = 0; i < 4; i = i + 1)
            check($sformatf("cross-boundary[%0d]", i), rdbuf[i], 32'hC0DE_0000 + i);

        // -- 5. survive refresh -----------------------------------------------
        // T_REFI is 300 cycles in this bench, so idling here forces several
        // refreshes between the write and the read.
        wr_burst(25'h00_0400, 4, 32'hBEEF_0000, 4'hF);
        repeat (1200) @(posedge clk);
        rd_burst(25'h00_0400, 4);
        for (i = 0; i < 4; i = i + 1)
            check($sformatf("after refresh[%0d]", i), rdbuf[i], 32'hBEEF_0000 + i);

        // -- 6. the model's protocol assertions -------------------------------
        if (mem.errors != 0) begin
            $display("  FAIL %0d protocol violation(s) reported by the model",
                     mem.errors);
            errors = errors + mem.errors;
        end else begin
            $display("  ok   no protocol violations (%0d reads, %0d writes)",
                     mem.n_reads, mem.n_writes);
        end

        $display("=== %s: %0d error(s) ===", errors == 0 ? "PASSED" : "FAILED",
                 errors);
        $finish;
    end

    initial begin
        #500000;
        $display("=== FAILED: timeout ===");
        $finish;
    end

endmodule
