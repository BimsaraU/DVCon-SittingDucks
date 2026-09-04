// =============================================================================
// tb_desc_path.sv -- the WHOLE descriptor read path, end to end
//
// tb_desc_fetch covers axi_to_avalon -> avalon_arbiter -> sdram_ctrl by driving
// the AXI side from the testbench. That leaves the half of the path that is
// actually inside the accelerator untested on this platform:
//
//   yolo_layer_sequencer -> yolo_axi_arbiter -> axi4_master -> AXI
//        -> axi_to_avalon -> avalon_mm_master -> avalon_arbiter -> sdram_ctrl
//
// Two AXI translations and two arbiters, none of them exercised together. The
// board symptom is that desc[0] comes back holding the descriptor's WORD 1, so
// every layer decodes flags(2) as op(2)=LAYER_OP_ADD and hangs in S_ELEM_W.
//
// Rather than test one hypothesis, this loads a descriptor whose word i is
// 0xA0000000+i and prints the whole 16-word array the sequencer latched. A
// clean run reproduces the table exactly; any shift, drop or duplicate names
// itself.
// =============================================================================
`timescale 1ns/1ps

module tb_desc_path;

    localparam integer ARRAY_SIZE = 16;
    localparam [31:0]  DESC_ADDR  = 32'h0000_1000;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;              // 100 MHz

    // ---- accelerator master port ----
    wire        m_awvalid, m_wvalid, m_wlast, m_bready, m_arvalid, m_rready;
    wire [11:0] m_awid, m_arid;
    wire [7:0]  m_awlen, m_arlen;
    wire [2:0]  m_awsize, m_arsize, m_awprot, m_arprot;
    wire [1:0]  m_awburst, m_arburst;
    wire [0:0]  m_awlock, m_arlock;
    wire [3:0]  m_awcache, m_awqos, m_arcache, m_arqos;
    wire [63:0] m_awaddr, m_araddr, m_wdata;
    wire [7:0]  m_wstrb;
    wire        m_awready, m_wready, m_bvalid, m_arready, m_rvalid, m_rlast;
    wire [11:0] m_bid, m_rid;
    wire [1:0]  m_bresp, m_rresp;
    wire [63:0] m_rdata;

    // ---- accelerator slave (config) port, driven by this bench ----
    reg         s_awvalid = 1'b0, s_wvalid = 1'b0, s_bready = 1'b0;
    reg  [63:0] s_awaddr  = 64'h0, s_wdata = 64'h0;
    reg  [7:0]  s_wstrb   = 8'h0;
    wire        s_awready, s_wready, s_bvalid;

    Accelerator_Top #(.ARRAY_SIZE(ARRAY_SIZE)) u_acc (
        .s_axi_aclk(clk), .s_axi_aresetn(rst_n),

        .m_axi_awvalid(m_awvalid), .m_axi_awid(m_awid), .m_axi_awlen(m_awlen),
        .m_axi_awsize(m_awsize), .m_axi_awburst(m_awburst),
        .m_axi_awlock(m_awlock), .m_axi_awcache(m_awcache),
        .m_axi_awqos(m_awqos), .m_axi_awaddr(m_awaddr), .m_axi_awprot(m_awprot),
        .m_axi_awready(m_awready),
        .m_axi_wvalid(m_wvalid), .m_axi_wlast(m_wlast), .m_axi_wdata(m_wdata),
        .m_axi_wstrb(m_wstrb), .m_axi_wready(m_wready),
        .m_axi_bready(m_bready), .m_axi_bvalid(m_bvalid), .m_axi_bid(m_bid),
        .m_axi_bresp(m_bresp),
        .m_axi_arvalid(m_arvalid), .m_axi_arid(m_arid), .m_axi_arlen(m_arlen),
        .m_axi_arsize(m_arsize), .m_axi_arburst(m_arburst),
        .m_axi_arlock(m_arlock), .m_axi_arcache(m_arcache),
        .m_axi_arqos(m_arqos), .m_axi_araddr(m_araddr), .m_axi_arprot(m_arprot),
        .m_axi_arready(m_arready),
        .m_axi_rready(m_rready), .m_axi_rvalid(m_rvalid), .m_axi_rid(m_rid),
        .m_axi_rlast(m_rlast), .m_axi_rresp(m_rresp), .m_axi_rdata(m_rdata),

        .s_axi_awid(12'h0), .s_axi_awaddr(s_awaddr), .s_axi_awlen(8'h0),
        .s_axi_awsize(3'b011), .s_axi_awburst(2'b01), .s_axi_awlock(1'b0),
        .s_axi_awcache(4'h0), .s_axi_awprot(3'h0), .s_axi_awqos(4'h0),
        .s_axi_awvalid(s_awvalid), .s_axi_awready(s_awready),
        .s_axi_wdata(s_wdata), .s_axi_wstrb(s_wstrb), .s_axi_wlast(1'b1),
        .s_axi_wvalid(s_wvalid), .s_axi_wready(s_wready),
        .s_axi_bready(s_bready), .s_axi_bid(), .s_axi_bresp(), .s_axi_bvalid(s_bvalid),
        .s_axi_arid(12'h0), .s_axi_araddr(64'h0), .s_axi_arlen(8'h0),
        .s_axi_arsize(3'b011), .s_axi_arburst(2'b01), .s_axi_arlock(1'b0),
        .s_axi_arcache(4'h0), .s_axi_arprot(3'h0), .s_axi_arqos(4'h0),
        .s_axi_arvalid(1'b0), .s_axi_arready(),
        .s_axi_rready(1'b0), .s_axi_rid(), .s_axi_rdata(), .s_axi_rresp(),
        .s_axi_rlast(), .s_axi_rvalid()
    );

    // ---- AXI -> Avalon ----
    wire [31:0] avm_address, avm_writedata, avm_readdata;
    wire        avm_read, avm_write, avm_readdatavalid, avm_waitrequest;
    wire [3:0]  avm_byteenable;
    wire [6:0]  avm_burstcount;

    axi_to_avalon #(.MAX_BURST(64)) u_bridge (
        .clk(clk), .rst_n(rst_n),
        .s_awvalid(m_awvalid), .s_awid(m_awid), .s_awlen(m_awlen),
        .s_awsize(m_awsize), .s_awburst(m_awburst), .s_awaddr(m_awaddr),
        .s_awready(m_awready),
        .s_wvalid(m_wvalid), .s_wlast(m_wlast), .s_wdata(m_wdata),
        .s_wstrb(m_wstrb), .s_wready(m_wready),
        .s_bready(m_bready), .s_bvalid(m_bvalid), .s_bid(m_bid), .s_bresp(m_bresp),
        .s_arvalid(m_arvalid), .s_arid(m_arid), .s_arlen(m_arlen),
        .s_arsize(m_arsize), .s_arburst(m_arburst), .s_araddr(m_araddr),
        .s_arready(m_arready),
        .s_rready(m_rready), .s_rvalid(m_rvalid), .s_rid(m_rid),
        .s_rdata(m_rdata), .s_rresp(m_rresp), .s_rlast(m_rlast),
        .avm_address(avm_address), .avm_read(avm_read), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount), .avm_readdata(avm_readdata),
        .avm_readdatavalid(avm_readdatavalid), .avm_waitrequest(avm_waitrequest)
    );

    // ---- arbiter + SDRAM, exactly as dvcon_top wires them ----
    wire [31:0] arb_address, arb_writedata, arb_readdata;
    wire        arb_read, arb_write, arb_readdatavalid, arb_waitrequest;
    wire [3:0]  arb_byteenable;
    wire [6:0]  arb_burstcount;

    avalon_arbiter #(.BURST_W(7)) u_arb (
        .clk(clk), .rst_n(rst_n),
        .m0_address(avm_address), .m0_read(avm_read), .m0_write(avm_write),
        .m0_writedata(avm_writedata), .m0_byteenable(avm_byteenable),
        .m0_burstcount(avm_burstcount), .m0_readdata(avm_readdata),
        .m0_readdatavalid(avm_readdatavalid), .m0_waitrequest(avm_waitrequest),
        .m1_address(32'h0), .m1_read(1'b0), .m1_write(1'b0),
        .m1_writedata(32'h0), .m1_byteenable(4'h0), .m1_burstcount(7'd1),
        .m1_readdata(), .m1_readdatavalid(), .m1_waitrequest(),
        .m2_address(32'h0), .m2_read(1'b0), .m2_write(1'b0),
        .m2_writedata(32'h0), .m2_byteenable(4'h0),
        .m2_readdata(), .m2_readdatavalid(), .m2_waitrequest(),
        .s_address(arb_address), .s_read(arb_read), .s_write(arb_write),
        .s_writedata(arb_writedata), .s_byteenable(arb_byteenable),
        .s_burstcount(arb_burstcount), .s_readdata(arb_readdata),
        .s_readdatavalid(arb_readdatavalid), .s_waitrequest(arb_waitrequest)
    );

    wire [12:0] dram_addr;
    wire [1:0]  dram_ba;
    wire        dram_cas_n, dram_ras_n, dram_we_n, dram_cs_n, dram_cke;
    wire [3:0]  dram_dqm;
    wire [31:0] dram_dq_out;
    wire        dram_dq_oe;
    wire [31:0] dram_dq;

    sdram_ctrl #(.DATA_W(32), .MAX_BURST(64), .T_INIT(20), .T_REFI(20000)) u_sdram (
        .clk(clk), .rst_n(rst_n),
        .cap_sel(2'd1), .avs_address(arb_address[26:2]), .avs_read(arb_read), .avs_write(arb_write),
        .avs_writedata(arb_writedata), .avs_byteenable(arb_byteenable),
        .avs_burstcount(arb_burstcount), .avs_readdata(arb_readdata),
        .avs_readdatavalid(arb_readdatavalid), .avs_waitrequest(arb_waitrequest),
        .dram_addr(dram_addr), .dram_ba(dram_ba), .dram_cas_n(dram_cas_n),
        .dram_ras_n(dram_ras_n), .dram_we_n(dram_we_n), .dram_cs_n(dram_cs_n),
        .dram_cke(dram_cke), .dram_dqm(dram_dqm),
        .dram_dq_in(dram_dq), .dram_dq_out(dram_dq_out), .dram_dq_oe(dram_dq_oe)
    );

    assign dram_dq = dram_dq_oe ? dram_dq_out : 32'bz;

    sdram_model #(.CHECK_REFRESH(0)) u_mem (
        .clk(clk), .addr(dram_addr), .ba(dram_ba), .cas_n(dram_cas_n),
        .ras_n(dram_ras_n), .we_n(dram_we_n), .cs_n(dram_cs_n), .cke(dram_cke),
        .dqm(dram_dqm), .dq_in(dram_dq), .dq_oe(dram_dq_oe), .dq_out(dram_dq)
    );

    // =========================================================================
    // Stimulus changes on the NEGEDGE and ready is sampled there too.
    //
    // Sampling on the posedge does not work in either direction: the slave
    // registers awready/wready, so reading them at the edge races that edge's
    // own update, and deasserting valid the moment ready is seen high kills the
    // handshake that was about to complete on the next edge. Both mistakes
    // leave the slave parked in W_DATA with wready high forever and the
    // register write silently never happening -- the sequencer then just sits
    // in S_IDLE with no error anywhere to explain it.
    //
    // At the negedge, ready holds the value it has for the whole of the current
    // cycle, and the transfer completes on the posedge that ends it.
    //
    // The slave is 64 bits wide and decodes on the 8-byte-ALIGNED offset, using
    // WSTRB to pick the half. So an odd word such as OFF_IMGA (0x1C) is not
    // addressed as 0x1C at all: it is 0x18 with the UPPER strobe. Sending it as
    // 0x18/lower -- which is what addressing it naively does -- writes OFF_DESC
    // instead, and the first thing that shows is the descriptor fetch reading
    // from the image base.
    task automatic cfg_write(input [5:0] off, input [31:0] val);
        begin
            // ---- AW ----
            @(negedge clk);
            s_awaddr  = {58'h0, off[5:3], 3'b000};
            s_awvalid = 1'b1;
            while (!s_awready) @(negedge clk);
            @(posedge clk); #1;
            s_awvalid = 1'b0;

            // ---- W ----
            @(negedge clk);
            s_wdata  = off[2] ? {val, 32'h0} : {32'h0, val};
            s_wstrb  = off[2] ? 8'hF0 : 8'h0F;
            s_wvalid = 1'b1;
            while (!s_wready) @(negedge clk);
            @(posedge clk); #1;
            s_wvalid = 1'b0;

            // ---- B ----
            @(negedge clk);
            s_bready = 1'b1;
            while (!s_bvalid) @(negedge clk);
            @(posedge clk); #1;
            s_bready = 1'b0;
            $display("%7t  cfg_write off=%02h val=%08h done", $time, off, val);
        end
    endtask

    // A hung handshake should say so rather than run the simulator until it is
    // killed from outside.
    initial begin
        #400000;
        $display("  tb_desc_path: TIMEOUT -- handshake stuck");
        $display("    wstate=%0d awready=%b wready=%b bvalid=%b seq_fsm=%0d",
                 u_acc.u_regs.wstate, s_awready, s_wready, s_bvalid,
                 u_acc.u_seq.fsm_state);
        $finish;
    end

    // ---- beat-level trace ---------------------------------------------------
    // Every stage the data crosses, so a beat that goes missing names the stage
    // that dropped it instead of only showing up as a wrong descriptor.
    reg trace = 1'b0;
    integer n_avs, n_avm, n_axi, n_seq;

    always @(posedge clk) if (rst_n && trace) begin
        if (arb_read && !arb_waitrequest)
            $display("%7t  CMD   avalon read addr=%08h burst=%0d",
                     $time, arb_address, arb_burstcount);
        if (arb_readdatavalid) begin
            n_avs = n_avs + 1;
            $display("%7t  SDRAM beat %0d = %08h", $time, n_avs, arb_readdata);
        end
        if (u_bridge.m_rd_data_valid) begin
            n_avm = n_avm + 1;
            $display("%7t  GEARBOX beat %0d = %016h",
                     $time, n_avm, u_bridge.m_rd_data);
        end
        if (m_rvalid && m_rready) begin
            n_axi = n_axi + 1;
            $display("%7t  AXI-R beat %0d = %016h last=%b",
                     $time, n_axi, m_rdata, m_rlast);
        end
        if (u_acc.u_seq.rd_data_valid) begin
            n_seq = n_seq + 1;
            $display("%7t  SEQ  beat %0d = %016h -> desc[%0d],desc[%0d]",
                     $time, n_seq, u_acc.u_seq.rd_data,
                     {u_acc.u_seq.fetch_beat[2:0], 1'b0},
                     {u_acc.u_seq.fetch_beat[2:0], 1'b1});
        end
        if (u_acc.u_seq.rd_done)
            $display("%7t  SEQ  rd_done", $time);
    end

    integer i, bad;
    reg [31:0] got, exp;

    initial begin
        // Descriptor: word i = 0xA0000000 + i. Any shift is self-evident.
        for (i = 0; i < 16; i = i + 1)
            u_mem.mem[(DESC_ADDR >> 2) + i] = 32'hA000_0000 + i;
        // Guard words either side, so a read that strays outside is obvious.
        u_mem.mem[(DESC_ADDR >> 2) - 1]  = 32'hDEAD_BEEF;
        u_mem.mem[(DESC_ADDR >> 2) + 16] = 32'hFEED_FACE;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (400) @(posedge clk);      // let sdram_ctrl finish its init

        cfg_write(6'h18, DESC_ADDR);      // OFF_DESC
        cfg_write(6'h1C, 32'h0080_0000);  // OFF_IMGA
        cfg_write(6'h20, 32'h0090_0000);  // OFF_BOX
        n_avs = 0; n_avm = 0; n_axi = 0; n_seq = 0;
        trace = 1'b1;
        cfg_write(6'h00, 32'h0000_0003);  // OFF_CTRL: START | mode_yolo

        // The fetch is 8 beats; give it room, then look at what landed.
        repeat (400) @(posedge clk);
        trace = 1'b0;
        repeat (3600) @(posedge clk);

        $display("");
        $display("  idx  expected    got");
        bad = 0;
        for (i = 0; i < 16; i = i + 1) begin
            exp = 32'hA000_0000 + i;
            got = u_acc.u_seq.desc[i];
            $display("  %2d   %08h    %08h %s", i, exp, got,
                     (got === exp) ? "" : "  <-- MISMATCH");
            if (got !== exp) bad = bad + 1;
        end
        $display("");
        $display("  sequencer fsm_state = %0d", u_acc.u_seq.fsm_state);

        // run_all.sh greps for this exact verdict form.
        if (bad == 0)
            $display("=== PASSED: 0 error(s) ===");
        else
            $display("=== FAILED: %0d error(s) (descriptor words wrong) ===", bad);
        $finish;
    end

endmodule
