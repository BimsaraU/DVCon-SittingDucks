// =============================================================================
// tb_jtag_ctrl.sv — the JTAG control path against a model register slave
//
// jtag_ctrl replaces the boot sequencer that hardcoded four addresses and a
// START. Everything the host does to drive a frame goes through here, so a
// silent bug in it is a board that ignores the host -- and the JTAG cable is
// exactly the instrument you would try to debug that with.
//
// What is checked:
//   1. a register write reaches the Avalon side with the right address/data
//   2. a register read returns what the slave holds
//   3. several writes in sequence do not corrupt each other
//   4. the clock-domain crossing survives tck being much slower than clk,
//      which is the real ratio (USB-Blaster ~10 MHz against 100 MHz)
// =============================================================================
`timescale 1ns/1ps

module tb_jtag_ctrl;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;                 // 100 MHz system clock

    wire [5:0]  avm_address;
    wire        avm_read, avm_write;
    wire [31:0] avm_writedata;
    reg  [31:0] avm_readdata;
    reg         avm_waitrequest;   // combinational, driven below

    // SDRAM read window: a model that returns address-derived data, so a
    // wrong address is visible in the value.
    wire [31:0] mem_address;
    wire        mem_read;
    reg  [31:0] mem_readdata = 32'h0;
    reg         mem_readdatavalid = 1'b0;

    always @(posedge clk) begin
        mem_readdatavalid <= mem_read;
        if (mem_read) mem_readdata <= 32'hBB00_0000 | mem_address;
    end

    jtag_ctrl dut (
        .clk(clk), .rst_n(rst_n),
        .avm_address(avm_address), .avm_read(avm_read), .avm_write(avm_write),
        .avm_writedata(avm_writedata),
        .avm_readdata(avm_readdata), .avm_waitrequest(avm_waitrequest),
        .mem_address(mem_address), .mem_read(mem_read),
        .mem_readdata(mem_readdata),
        .mem_readdatavalid(mem_readdatavalid),
        .mem_waitrequest(1'b0),
        .eth_good(32'h0000_1234), .eth_bad(32'h0000_0005),
        .eth_frames(32'h0000_00AB), .eth_bitmap(64'h0000_0000_DEAD_F00D)
    );

    // ---- model register file ------------------------------------------------
    // Stalls for a couple of cycles so the bench exercises waitrequest rather
    // than assuming a zero-latency slave.
    reg [31:0] regs [0:63];
    integer    stall;
    integer    n_writes = 0;

    initial begin
        for (int i = 0; i < 64; i = i + 1) regs[i] = 32'h0;
        avm_readdata    = 32'h0;
        stall           = 0;
    end

    // waitrequest is COMBINATIONAL, as Avalon requires: a master samples it in
    // the same cycle it asserts the command. Registering it (as an earlier
    // version of this model did) lets the master see it low on the first cycle
    // and complete a transfer the slave has not accepted -- the write is
    // simply lost, and the symptom is a register file that never changes.
    always @(*) avm_waitrequest = (avm_write || avm_read) && (stall < 2);

    always @(posedge clk) begin
        if (avm_write || avm_read) begin
            if (stall < 2) begin
                stall <= stall + 1;
            end else begin
                stall <= 0;
                if (avm_write) begin
                    regs[avm_address] <= avm_writedata;
                    n_writes          <= n_writes + 1;
                end
                if (avm_read) avm_readdata <= regs[avm_address];
            end
        end else begin
            stall <= 0;
        end
    end

    // ---- host-side helpers --------------------------------------------------
    localparam [1:0] IR_ADDR = 2'd0, IR_DATA = 2'd1;

    reg [31:0] dout;

    task automatic jt_write(input [5:0] a, input [31:0] d);
        begin
            // ADDR scan: 8 bits of { write, addr[5:0], 1'b0 } placed so that
            // after 8 shifts the payload sits in the top byte.
            dut.u_vjtag.scan_ir(IR_ADDR);
            dut.u_vjtag.scan_dr(8, {24'h0, 1'b1, a, 1'b0}, dout);
            dut.u_vjtag.scan_ir(IR_DATA);
            dut.u_vjtag.scan_dr(32, d, dout);
            // let the crossing settle
            repeat (20) @(posedge clk);
        end
    endtask

    task automatic jt_read(input [5:0] a, output [31:0] d);
        begin
            dut.u_vjtag.scan_ir(IR_ADDR);
            dut.u_vjtag.scan_dr(8, {24'h0, 1'b0, a, 1'b0}, dout);
            // The read is issued one tck AFTER ADDR's Update_DR (see jtag_ctrl's
            // `arm`), and then has to cross into the clk domain, run on Avalon,
            // and cross back. Waiting in clk cycles alone is not enough when
            // tck is the slower clock -- wait out the tck side too, or the
            // DATA scan captures whatever the previous read left behind.
            repeat (8) dut.u_vjtag.tick();
            repeat (20) @(posedge clk);
            dut.u_vjtag.scan_ir(IR_DATA);
            dut.u_vjtag.scan_dr(32, 32'h0, d);
        end
    endtask

    integer errors = 0;
    task automatic check(input string what, input [31:0] got,
                         input [31:0] exp);
        begin
            if (got !== exp) begin
                $display("  FAIL %-34s got %08x expected %08x", what, got, exp);
                errors = errors + 1;
            end else begin
                $display("  ok   %-34s %08x", what, got);
            end
        end
    endtask

    reg [31:0] rd;

    initial begin
        $display("=== tb_jtag_ctrl ===");
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // -- 1. a write lands -------------------------------------------------
        jt_write(6'h06, 32'h0000_1000);
        check("write reached slave (reg 0x06)", regs[6], 32'h0000_1000);

        // -- 2. more writes, no cross-talk ------------------------------------
        jt_write(6'h07, 32'h0040_0000);
        jt_write(6'h08, 32'h0060_0000);
        check("reg 0x07", regs[7], 32'h0040_0000);
        check("reg 0x08", regs[8], 32'h0060_0000);
        check("reg 0x06 undisturbed", regs[6], 32'h0000_1000);

        // -- 3. read back -----------------------------------------------------
        jt_read(6'h07, rd);
        check("read reg 0x07", rd, 32'h0040_0000);

        jt_read(6'h08, rd);
        check("read reg 0x08", rd, 32'h0060_0000);

        // Re-read the register whose FIRST read failed. If this passes, the
        // failure is position-dependent (first read after writes), not
        // address-dependent.
        jt_read(6'h07, rd);
        check("re-read reg 0x07", rd, 32'h0040_0000);

        // -- 4. a START-shaped write ------------------------------------------
        jt_write(6'h00, 32'h0000_0003);
        check("CTRL start|yolo", regs[0], 32'h0000_0003);

        // -- 5. Ethernet diagnostics -------------------------------------------
        // Answered from wires, not through Avalon, so they must not disturb the
        // register path either.
        jt_read(6'h3A, rd);  check("ETH_GOOD",   rd, 32'h0000_1234);
        jt_read(6'h3B, rd);  check("ETH_BAD",    rd, 32'h0000_0005);
        jt_read(6'h3C, rd);  check("ETH_CMD",    rd, 32'h0000_00AB);
        jt_read(6'h3D, rd);  check("ETH_BITMAP", rd, 32'hDEAD_F00D);

        jt_read(6'h07, rd);
        check("register path still works after", rd, 32'h0040_0000);

        // -- 6. the SDRAM read window -----------------------------------------
        // MEMADDR is set once; each MEMDATA read post-increments it by 4, which
        // is what lets the host walk the box list.
        jt_write(6'h3E, 32'h0060_0000);
        jt_read(6'h3F, rd);
        check("MEMDATA at base",        rd, 32'hBB60_0000);
        jt_read(6'h3F, rd);
        check("MEMDATA post-incremented", rd, 32'hBB60_0004);
        jt_read(6'h3F, rd);
        check("MEMDATA again",          rd, 32'hBB60_0008);

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
