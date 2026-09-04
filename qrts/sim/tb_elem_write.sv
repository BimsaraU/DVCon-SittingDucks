// =============================================================================
// tb_elem_write.sv — the elementwise engine's single-byte writes, through the
// real AXI master, bridge, arbiter and SDRAM controller.
//
// On hardware a 32-element LAYER_OP_ADD wrote only 16 of its 32 bytes, and the
// ones that landed were 4 bytes BELOW where they belong:
//
//     elem  4 -> dst+0      elem  0..3   never written
//     elem 12 -> dst+8      elem  8..11  never written
//     elem 20 -> dst+16     elem 16..19  never written
//     elem 28 -> dst+24     elem 24..27  never written
//
// Exactly the elements with addr[2]==1 survive, each landing at addr-4, and
// each overwriting the element that should have been there. That is the
// signature of the two 32-bit sub-beats of one 64-bit core write being
// collapsed onto a single address.
//
// The engine's own encoding is the thing under test. yolo_elem_engine drives
//
//     wr_addr = dst + idx              (NOT 8-byte aligned)
//     wr_data = byte << (addr[2:0]*8)  (lane within a 64-bit beat)
//     wr_strb = 8'h01 << addr[2:0]     (lane within a 64-bit beat)
//
// The data and strobe are positioned relative to an 8-byte-ALIGNED beat while
// the address still carries those same low bits, so the low 3 bits are applied
// twice. Whether that is benign depends entirely on what the master downstream
// does with an unaligned address, which is what this bench pins down.
//
// Writes a 0x40+idx ramp so every element is distinguishable, then reads the
// region back through a direct controller access -- not through the master --
// so a read-path defect cannot be mistaken for a write-path one.
// =============================================================================
`timescale 1ns/1ps

module tb_elem_write;

    localparam [31:0] DST = 32'h0000_1000;
    localparam integer N  = 32;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    integer errors = 0;

    // ---- engine-side handles into axi4_master -------------------------------
    reg         wr_start = 1'b0;
    reg  [63:0] wr_addr  = 64'd0;
    reg  [7:0]  wr_len   = 8'd0;
    reg  [63:0] wr_data  = 64'd0;
    reg  [7:0]  wr_strb  = 8'h00;
    wire        wr_data_ready, wr_done, wr_error;

    // ---- AXI between master and bridge --------------------------------------
    wire [11:0] awid, bid, arid, rid;
    wire [63:0] awaddr, araddr;
    wire [7:0]  awlen, arlen;
    wire [2:0]  awsize, arsize, awprot, arprot;
    wire [1:0]  awburst, arburst, bresp, rresp;
    wire        awvalid, awready, wvalid, wready, wlast, bvalid, bready;
    wire        arvalid, arready, rvalid, rready, rlast;
    wire [63:0] wdata, rdata;
    wire [7:0]  wstrb;

    axi4_master #(.ADDR_WIDTH(64), .DATA_WIDTH(64), .ID_WIDTH(12)) u_m (
        .clk(clk), .rst_n(rst_n),
        .m_arid(arid), .m_araddr(araddr), .m_arlen(arlen), .m_arsize(arsize),
        .m_arburst(arburst), .m_arprot(arprot), .m_arvalid(arvalid),
        .m_arready(arready),
        .m_rid(rid), .m_rdata(rdata), .m_rresp(rresp), .m_rlast(rlast),
        .m_rvalid(rvalid), .m_rready(rready),
        .m_awid(awid), .m_awaddr(awaddr), .m_awlen(awlen), .m_awsize(awsize),
        .m_awburst(awburst), .m_awprot(awprot), .m_awvalid(awvalid),
        .m_awready(awready),
        .m_wdata(wdata), .m_wstrb(wstrb), .m_wlast(wlast), .m_wvalid(wvalid),
        .m_wready(wready),
        .m_bready(bready), .m_bvalid(bvalid), .m_bid(bid), .m_bresp(bresp),
        .rd_start(1'b0), .rd_addr(64'd0), .rd_len(8'd0),
        .rd_data(), .rd_data_valid(), .rd_done(), .rd_error(),
        .wr_start(wr_start), .wr_addr(wr_addr), .wr_len(wr_len),
        .wr_data(wr_data), .wr_strb(wr_strb),
        .wr_data_ready(wr_data_ready), .wr_done(wr_done), .wr_error(wr_error)
    );

    // ---- bridge -------------------------------------------------------------
    wire [31:0] avm_address, avm_writedata, avm_readdata;
    wire        avm_read, avm_write, avm_readdatavalid, avm_waitrequest;
    wire [3:0]  avm_byteenable;
    wire [6:0]  avm_burstcount;

    axi_to_avalon #(
        .AXI_DATA_W(64), .AVM_DATA_W(32), .AVM_ADDR_W(32), .MAX_BURST(64)
    ) u_bridge (
        .clk(clk), .rst_n(rst_n),
        .s_awvalid(awvalid), .s_awid(awid), .s_awlen(awlen), .s_awsize(awsize),
        .s_awburst(awburst), .s_awaddr(awaddr), .s_awready(awready),
        .s_wvalid(wvalid), .s_wlast(wlast), .s_wdata(wdata), .s_wstrb(wstrb),
        .s_wready(wready),
        .s_bready(bready), .s_bvalid(bvalid), .s_bid(bid), .s_bresp(bresp),
        .s_arvalid(arvalid), .s_arid(arid), .s_arlen(arlen), .s_arsize(arsize),
        .s_arburst(arburst), .s_araddr(araddr), .s_arready(arready),
        .s_rready(rready), .s_rvalid(rvalid), .s_rid(rid), .s_rdata(rdata),
        .s_rresp(rresp), .s_rlast(rlast),
        .avm_address(avm_address), .avm_read(avm_read), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount), .avm_readdata(avm_readdata),
        .avm_readdatavalid(avm_readdatavalid), .avm_waitrequest(avm_waitrequest)
    );

    // ---- a back door onto the controller, for seeding and for readback ------
    // Readback deliberately does NOT go through the master: a read defect must
    // not be able to masquerade as a write defect here.
    reg         probe = 1'b0;
    reg  [31:0] pr_address, pr_writedata;
    reg         pr_write, pr_read;

    wire [31:0] s_address    = probe ? pr_address   : avm_address;
    wire        s_read       = probe ? pr_read      : avm_read;
    wire        s_write      = probe ? pr_write     : avm_write;
    wire [31:0] s_writedata  = probe ? pr_writedata : avm_writedata;
    wire [3:0]  s_byteenable = probe ? 4'hF         : avm_byteenable;
    wire [6:0]  s_burstcount = probe ? 7'd1         : avm_burstcount;

    wire [31:0] arb_address, arb_writedata;
    wire        arb_read, arb_write;
    wire [3:0]  arb_byteenable;
    wire [6:0]  arb_burstcount;
    wire [31:0] sdc_readdata;
    wire        sdc_readdatavalid, sdc_waitrequest;

    avalon_arbiter #(.BURST_W(7)) u_arb (
        .clk(clk), .rst_n(rst_n),
        .m0_address(s_address), .m0_read(s_read), .m0_write(s_write),
        .m0_writedata(s_writedata), .m0_byteenable(s_byteenable),
        .m0_burstcount(s_burstcount), .m0_readdata(avm_readdata),
        .m0_readdatavalid(avm_readdatavalid), .m0_waitrequest(avm_waitrequest),
        .m1_address(32'h0), .m1_read(1'b0), .m1_write(1'b0),
        .m1_writedata(32'h0), .m1_byteenable(4'h0), .m1_burstcount(7'd1),
        .m1_readdata(), .m1_readdatavalid(), .m1_waitrequest(),
        .m2_address(32'h0), .m2_read(1'b0),
        .m2_write(1'b0), .m2_writedata(32'h0), .m2_byteenable(4'h0),
        .m2_readdata(), .m2_readdatavalid(), .m2_waitrequest(),
        .s_address(arb_address), .s_read(arb_read), .s_write(arb_write),
        .s_writedata(arb_writedata), .s_byteenable(arb_byteenable),
        .s_burstcount(arb_burstcount), .s_readdata(sdc_readdata),
        .s_readdatavalid(sdc_readdatavalid), .s_waitrequest(sdc_waitrequest)
    );

    wire [12:0] dram_addr; wire [1:0] dram_ba;
    wire dram_cas_n, dram_ras_n, dram_we_n, dram_cs_n, dram_cke;
    wire [3:0]  dram_dqm;
    wire [31:0] dram_dq_in, dram_dq_out; wire dram_dq_oe;

    sdram_ctrl #(.T_INIT(20), .T_REFI(20000)) u_sdram (
        .clk(clk), .rst_n(rst_n),
        .cap_sel(2'd1), .avs_address(arb_address[26:2]), .avs_read(arb_read),
        .avs_write(arb_write), .avs_writedata(arb_writedata),
        .avs_byteenable(arb_byteenable), .avs_burstcount(arb_burstcount),
        .avs_readdata(sdc_readdata), .avs_readdatavalid(sdc_readdatavalid),
        .avs_waitrequest(sdc_waitrequest),
        .dram_addr(dram_addr), .dram_ba(dram_ba),
        .dram_cas_n(dram_cas_n), .dram_ras_n(dram_ras_n),
        .dram_we_n(dram_we_n), .dram_cs_n(dram_cs_n), .dram_cke(dram_cke),
        .dram_dqm(dram_dqm),
        .dram_dq_in(dram_dq_in), .dram_dq_out(dram_dq_out),
        .dram_dq_oe(dram_dq_oe)
    );

    sdram_model #(.REFRESH_DEADLINE(200000)) mem (
        .clk(clk), .addr(dram_addr), .ba(dram_ba),
        .cas_n(dram_cas_n), .ras_n(dram_ras_n), .we_n(dram_we_n),
        .cs_n(dram_cs_n), .cke(dram_cke), .dqm(dram_dqm),
        .dq_in(dram_dq_out), .dq_oe(dram_dq_oe), .dq_out(dram_dq_in)
    );

    // -------------------------------------------------------------------------
    task automatic probe_write(input [31:0] byte_addr, input [31:0] val);
        begin
            @(posedge clk);
            pr_address   <= byte_addr;
            pr_writedata <= val;
            pr_write     <= 1'b1;
            pr_read      <= 1'b0;
            @(posedge clk);
            while (sdc_waitrequest) @(posedge clk);
            pr_write <= 1'b0;
            @(posedge clk);
        end
    endtask

    reg [31:0] pr_result;
    task automatic probe_read(input [31:0] byte_addr);
        integer guard;
        begin
            @(posedge clk);
            pr_address <= byte_addr;
            pr_read    <= 1'b1;
            pr_write   <= 1'b0;
            @(posedge clk);
            while (sdc_waitrequest) @(posedge clk);
            pr_read <= 1'b0;
            guard = 0;
            while (!sdc_readdatavalid && guard < 200) begin
                @(posedge clk);
                guard = guard + 1;
            end
            pr_result = sdc_readdata;
            @(posedge clk);
        end
    endtask

    // One engine-style single-byte write, encoded exactly as yolo_elem_engine
    // does it.
    task automatic elem_write(input [31:0] byte_addr, input [7:0] val);
        integer guard;
        begin
            @(posedge clk);
            // The address is 8-byte ALIGNED and the byte's position within the
            // beat is carried only by wr_data/wr_strb. Passing byte_addr
            // unaligned as well -- which is what the engines used to do --
            // applies the low three bits twice and is the defect this bench
            // was written to reproduce.
            wr_addr  <= {32'h0, byte_addr & 32'hFFFF_FFF8};
            wr_data  <= {56'h0, val} << {byte_addr[2:0], 3'b000};
            wr_strb  <= 8'h01 << byte_addr[2:0];
            wr_len   <= 8'd1;
            wr_start <= 1'b1;
            @(posedge clk);
            wr_start <= 1'b0;
            guard = 0;
            while (!wr_done && guard < 4000) begin
                @(posedge clk);
                guard = guard + 1;
            end
            if (guard >= 4000) begin
                $display("  [FAIL] write to %h never completed", byte_addr);
                errors = errors + 1;
            end
        end
    endtask

    integer i, off, elem;
    reg [31:0] w;
    reg [7:0]  b;
    integer written, misplaced;

    initial begin
        probe = 1'b0; pr_write = 1'b0; pr_read = 1'b0;
        pr_address = 32'h0; pr_writedata = 32'h0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (400) @(posedge clk);      // controller init

        $display("=== tb_elem_write ===");

        // ---- sentinel ------------------------------------------------------
        probe = 1'b1;
        for (i = 0; i < 16; i = i + 1)
            probe_write(DST + i*4, 32'hDEAD_BEEF);
        probe = 1'b0;
        repeat (10) @(posedge clk);

        // ---- 32 engine-style single-byte writes ----------------------------
        for (i = 0; i < N; i = i + 1)
            elem_write(DST + i, 8'h40 + i[7:0]);
        repeat (20) @(posedge clk);

        // ---- read back, not through the master -----------------------------
        probe = 1'b1;
        written   = 0;
        misplaced = 0;
        $display("  misplaced bytes:");
        for (i = 0; i < 16; i = i + 1) begin
            probe_read(DST + i*4);
            w = pr_result;
            for (off = 0; off < 4; off = off + 1) begin
                b = w[off*8 +: 8];
                if (b >= 8'h40 && b < 8'h40 + N) begin
                    elem    = b - 8'h40;
                    written = written + 1;
                    if (elem != i*4 + off) begin
                        $display("    elem %0d landed at +%0d   (delta %0d)",
                                 elem, i*4 + off, (i*4 + off) - elem);
                        misplaced = misplaced + 1;
                    end
                end
            end
        end
        probe = 1'b0;

        $display("  %0d/%0d bytes present, %0d misplaced", written, N, misplaced);

        if (written != N || misplaced != 0) begin
            $display("  [FAIL] the hardware write pattern REPRODUCES in simulation");
            errors = errors + 1;
        end else begin
            $display("  [ok  ] all %0d bytes landed at dst+idx", N);
        end

        if (errors == 0) $display("=== PASSED: 0 error(s) ===");
        else             $display("=== FAILED: %0d error(s) ===", errors);
        $finish;
    end

    initial begin
        #5000000;
        $display("=== FAILED: global timeout ===");
        $finish;
    end

endmodule
