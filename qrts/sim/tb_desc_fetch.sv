// =============================================================================
// tb_desc_fetch.sv — an AXI burst read from the core, through the real bridge,
// into the real SDRAM controller.
//
// This is the bench that did not exist, and its absence is why a broken
// descriptor fetch reached hardware looking like a clean run.
//
// The pieces were each covered on their own:
//   tb_avalon_master  avalon_mm_master against axi4_master -- same stimulus into
//                     both sides of the SAME internal handshake, so a defect in
//                     the AXI-facing wrapper is invisible to it.
//   tb_arbiter        avalon_arbiter + the real sdram_ctrl, driven by
//                     hand-written Avalon masters -- never through the bridge.
//
// Nothing drove an AXI READ BURST through axi_to_avalon into memory and checked
// that every beat came back, so that whole path was unproven end to end.
//
// It is proven now, and the result is a negative worth recording: with the
// arbiter in place and the real controller underneath, an 8-beat descriptor
// fetch returns all 8 beats with the right data and rlast in the right place.
// The hardware failure -- desc[2..15] arriving as zeros, so DESC_NEXT read 0
// and the walk ended after layer 0 with done=1 and error=0 -- is NOT
// reproducible here. Whatever causes it is something this bench does not
// model: real SDRAM timing, or the missing DRAM_CLK phase shift (there is no
// PLL; DRAM_CLK is just ~clk_sys), rather than the digital handshake.
//
// A theory that the bridge held s_rvalid too long was tested and disproved:
// the bench passes identically with that code reverted, because s_rvalid is
// visible only the cycle after it is set and clears at the end of that cycle.
//
// Checked: a 64-byte (8 x 64-bit) read returns 8 beats, in order, with the
// right data and rlast on the last one -- the exact shape of a descriptor
// fetch. Then 2- and 32-beat bursts, because beat-accounting errors often pass
// at one length and fail at another.
// =============================================================================
`timescale 1ns/1ps

module tb_desc_fetch;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    integer errors = 0;
    task check(input string what, input integer got, input integer exp);
        begin
            if (got === exp) $display("  ok   %-32s %0d", what, got);
            else begin
                $display("  FAIL %-32s got %0d want %0d", what, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    task check64(input string what, input [63:0] got, input [63:0] exp);
        begin
            if (got === exp) $display("  ok   %-32s %016h", what, got);
            else begin
                $display("  FAIL %-32s got %016h want %016h", what, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    // ---- AXI master side, driven as the core master would ------------------
    reg         arvalid = 1'b0;
    reg  [7:0]  arlen   = 8'd0;
    reg  [63:0] araddr  = 64'd0;
    wire        arready;
    reg         rready  = 1'b1;      // the accelerator consumers hold this high
    wire        rvalid, rlast;
    wire [63:0] rdata;

    // ---- bridge to Avalon ---------------------------------------------------
    wire [31:0] avm_address, avm_writedata, avm_readdata;
    wire        avm_read, avm_write, avm_readdatavalid, avm_waitrequest;
    wire [3:0]  avm_byteenable;
    wire [6:0]  avm_burstcount;

    // Seeding drives the controller directly; the bridge is muxed out while
    // seeding so the bench never needs back-door access to the memory model.
    reg         seeding = 1'b0;
    reg  [31:0] sd_address, sd_writedata;
    reg         sd_write;

    wire [31:0] s_address    = seeding ? sd_address    : avm_address;
    wire        s_read       = seeding ? 1'b0          : avm_read;
    wire        s_write      = seeding ? sd_write      : avm_write;
    wire [31:0] s_writedata  = seeding ? sd_writedata  : avm_writedata;
    wire [3:0]  s_byteenable = seeding ? 4'hF          : avm_byteenable;
    wire [6:0]  s_burstcount = seeding ? 7'd1          : avm_burstcount;

    axi_to_avalon #(
        .AXI_DATA_W(64), .AVM_DATA_W(32), .AVM_ADDR_W(32), .MAX_BURST(64)
    ) u_bridge (
        .clk(clk), .rst_n(rst_n),
        .s_awvalid(1'b0), .s_awid(12'h0), .s_awlen(8'h0), .s_awsize(3'b011),
        .s_awburst(2'b01), .s_awaddr(64'h0), .s_awready(),
        .s_wvalid(1'b0), .s_wlast(1'b0), .s_wdata(64'h0), .s_wstrb(8'h0),
        .s_wready(),
        .s_bready(1'b1), .s_bvalid(), .s_bid(), .s_bresp(),
        .s_arvalid(arvalid), .s_arid(12'h0), .s_arlen(arlen), .s_arsize(3'b011),
        .s_arburst(2'b01), .s_araddr(araddr), .s_arready(arready),
        .s_rready(rready), .s_rvalid(rvalid), .s_rid(), .s_rdata(rdata),
        .s_rresp(), .s_rlast(rlast),
        .avm_address(avm_address), .avm_read(avm_read), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount), .avm_readdata(avm_readdata),
        .avm_readdatavalid(avm_readdatavalid), .avm_waitrequest(avm_waitrequest)
    );

    // ---- the real controller and memory model -------------------------------
    // Wired exactly as tb_arbiter does. sdram_ctrl has no dram_clk output --
    // dvcon_top generates DRAM_CLK itself -- so the model runs off the same clk.
    wire [12:0] dram_addr; wire [1:0] dram_ba;
    wire dram_cas_n, dram_ras_n, dram_we_n, dram_cs_n, dram_cke;
    wire [3:0]  dram_dqm;
    wire [31:0] dram_dq_in, dram_dq_out; wire dram_dq_oe;

    wire [31:0] sdc_readdata;
    wire        sdc_readdatavalid, sdc_waitrequest;

    // The arbiter sits between the bridge and the controller in dvcon_top, and
    // it is the one structural difference that a bridge-to-controller bench
    // does not model. It matters for reads specifically: `busy` is only set for
    // WRITE bursts, so once the master drops avm_read after the command is
    // accepted, `sel` swings away from master 0 mid-burst and the returning
    // beats are steered by the latched rd_owner alone.
    wire [31:0] arb_address, arb_writedata;
    wire        arb_read, arb_write;
    wire [3:0]  arb_byteenable;
    wire [6:0]  arb_burstcount;

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

    sdram_ctrl #(.T_INIT(20), .T_REFI(20000)) u_sdram (
        .clk(clk), .rst_n(rst_n),
        // BYTE address in, word address out -- the [26:2] slice dvcon_top uses.
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

    // ---- capture every accepted R beat --------------------------------------
    reg [63:0] got [0:63];
    integer    nbeats  = 0;
    reg        saw_last = 1'b0;
    integer    last_at  = -1;

    always @(posedge clk) begin
        if (rvalid && rready) begin
            if (nbeats < 64) got[nbeats] <= rdata;
            if (rlast) begin saw_last <= 1'b1; last_at <= nbeats; end
            nbeats <= nbeats + 1;
        end
    end

    task automatic do_read(input [31:0] addr, input integer beats);
        integer guard;
        begin
            nbeats = 0; saw_last = 1'b0; last_at = -1;
            @(negedge clk);
            araddr  = {32'h0, addr};
            arlen   = beats - 1;          // AXI carries len-1
            arvalid = 1'b1;
            @(posedge clk);
            while (!arready) @(posedge clk);
            @(negedge clk);
            arvalid = 1'b0;
            // Bounded wait: a hang must be a reported failure, not a freeze.
            guard = 0;
            while (!saw_last && guard < 200000) begin
                @(posedge clk);
                guard = guard + 1;
            end
            repeat (6) @(posedge clk);
        end
    endtask

    task automatic seed(input [31:0] addr, input integer nwords);
        integer i;
        begin
            seeding = 1'b1;
            for (i = 0; i < nwords; i = i + 1) begin
                @(negedge clk);
                sd_address   = addr + i*4;
                sd_writedata = 32'hC0DE_0000 + i;
                sd_write     = 1'b1;
                @(posedge clk);
                while (avm_waitrequest) @(posedge clk);
                @(negedge clk);
                sd_write = 1'b0;
                @(posedge clk);
            end
            seeding = 1'b0;
        end
    endtask

    integer k;
    initial begin
        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (3000) @(posedge clk);     // controller init sequence

        seed(32'h0000_0000, 64);

        // ---- the descriptor-fetch shape: 8 x 64-bit = 64 bytes -------------
        do_read(32'h0000_0000, 8);
        check("desc fetch: beat count", nbeats, 8);
        check("desc fetch: rlast position", last_at, 7);
        for (k = 0; k < 8; k = k + 1)
            if (k < nbeats)
                check64($sformatf("desc fetch: beat %0d", k), got[k],
                        {32'hC0DE_0000 + (2*k+1), 32'hC0DE_0000 + (2*k)});

        // ---- other lengths: off-by-one often passes at exactly one length ---
        do_read(32'h0000_0000, 2);
        check("2-beat burst: beat count", nbeats, 2);
        check("2-beat burst: rlast position", last_at, 1);

        do_read(32'h0000_0000, 32);
        check("32-beat burst: beat count", nbeats, 32);
        check("32-beat burst: rlast position", last_at, 31);

        $display("");
        $display("=== %s: %0d error(s) ===",
                 errors == 0 ? "PASSED" : "FAILED", errors);
        $finish;
    end

    initial begin
        #20000000;
        $display("=== FAILED: timeout ===");
        $finish;
    end

endmodule
