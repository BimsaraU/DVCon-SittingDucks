// =============================================================================
// tb_dvcon_regs.sv — the Avalon register slave against the real AXI register file
//
// dvcon_regs turns Avalon accesses into the AXI4 writes Accelerator_Top's
// slave port expects. Two things it has to get right, and both are easy to get
// silently wrong:
//
//   1. Avalon is WORD addressed and the AXI slave is BYTE addressed, so
//      avs_address N is byte offset 4N. Getting that wrong aliases the whole
//      map to four times its size.
//
//   2. The AXI slave decodes a 64-bit word address and selects the 32-bit half
//      with WSTRB -- a write to 0x0C arrives as AWADDR=0x08 with the UPPER
//      strobe set, not as AWADDR=0x0C with a full strobe. Decoding it the
//      naive way aliases 0x04/0x0C/0x14 onto 0x00/0x08/0x10 and silently drops
//      half the register map. That exact bug cost hours in Stage 3A and is
//      called out in yolo_axi4_slave_regs' own header.
//
// So this drives dvcon_regs against the REAL yolo_axi4_slave_regs and reads
// the values back out of its datapath outputs. A bench that checked only the
// AXI waveform would pass with the halves swapped.
// =============================================================================
`timescale 1ns/1ps

module tb_dvcon_regs;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    integer errors = 0;

    // ---- Avalon side ----
    reg  [5:0]  avs_address = 6'h0;
    reg         avs_read = 1'b0, avs_write = 1'b0;
    reg  [31:0] avs_writedata = 32'h0;
    wire [31:0] avs_readdata;
    wire        avs_waitrequest;

    // ---- AXI between the two ----
    wire [11:0] awid;  wire [63:0] awaddr; wire [7:0] awlen;
    wire [2:0]  awsize; wire [1:0] awburst; wire awlock;
    wire [3:0]  awcache, awqos; wire [2:0] awprot;
    wire        awvalid; wire awready;
    wire [63:0] wdata; wire [7:0] wstrb; wire wlast, wvalid; wire wready;
    wire        bready; wire bvalid;
    wire [11:0] arid; wire [63:0] araddr; wire [7:0] arlen;
    wire [2:0]  arsize; wire [1:0] arburst; wire arlock;
    wire [3:0]  arcache, arqos; wire [2:0] arprot;
    wire        arvalid; wire arready;
    wire        rready; wire rvalid; wire [63:0] rdata;

    dvcon_regs #(.ARRAY_SIZE(16), .BUILD_ID(16'hBEEF)) dut (
        .clk(clk), .rst_n(rst_n),
        .avs_address(avs_address), .avs_read(avs_read), .avs_write(avs_write),
        .avs_writedata(avs_writedata),
        .avs_readdata(avs_readdata), .avs_waitrequest(avs_waitrequest),
        .m_awid(awid), .m_awaddr(awaddr), .m_awlen(awlen),
        .m_awsize(awsize), .m_awburst(awburst), .m_awlock(awlock),
        .m_awcache(awcache), .m_awprot(awprot), .m_awqos(awqos),
        .m_awvalid(awvalid), .m_awready(awready),
        .m_wdata(wdata), .m_wstrb(wstrb), .m_wlast(wlast),
        .m_wvalid(wvalid), .m_wready(wready),
        .m_bready(bready), .m_bvalid(bvalid),
        .m_arid(arid), .m_araddr(araddr), .m_arlen(arlen),
        .m_arsize(arsize), .m_arburst(arburst), .m_arlock(arlock),
        .m_arcache(arcache), .m_arprot(arprot), .m_arqos(arqos),
        .m_arvalid(arvalid), .m_arready(arready),
        .m_rready(rready), .m_rvalid(rvalid), .m_rdata(rdata)
    );

    // The real register file, so the datapath outputs can be checked directly.
    wire        start_pulse, mode_yolo, mode_ucode;
    wire [31:0] src_addr, dst_addr, img_dim, weight_addr;
    wire [31:0] desc_addr, img_addr, box_addr;
    wire [7:0]  conf_thresh;
    wire        ucode_we; wire [9:0] ucode_waddr; wire [63:0] ucode_wdata;

    yolo_axi4_slave_regs #(.ID_WIDTH(12), .ADDR_WIDTH(64), .DATA_WIDTH(64))
    u_regs (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awid(awid), .s_axi_awaddr(awaddr), .s_axi_awlen(awlen),
        .s_axi_awsize(awsize), .s_axi_awburst(awburst), .s_axi_awlock(awlock),
        .s_axi_awcache(awcache), .s_axi_awprot(awprot), .s_axi_awqos(awqos),
        .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wlast(wlast),
        .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bready(bready), .s_axi_bid(), .s_axi_bresp(),
        .s_axi_bvalid(bvalid),
        .s_axi_arid(arid), .s_axi_araddr(araddr), .s_axi_arlen(arlen),
        .s_axi_arsize(arsize), .s_axi_arburst(arburst), .s_axi_arlock(arlock),
        .s_axi_arcache(arcache), .s_axi_arprot(arprot), .s_axi_arqos(arqos),
        .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rready(rready), .s_axi_rid(), .s_axi_rdata(rdata),
        .s_axi_rresp(), .s_axi_rlast(), .s_axi_rvalid(rvalid),
        .start_pulse(start_pulse), .mode_yolo(mode_yolo),
        .mode_ucode(mode_ucode),
        .src_addr(src_addr), .dst_addr(dst_addr),
        .img_dim(img_dim), .weight_addr(weight_addr),
        .desc_addr(desc_addr), .img_addr(img_addr), .box_addr(box_addr),
        .conf_thresh(conf_thresh),
        .busy_in(1'b0), .done_in(1'b1), .error_in(1'b0), .fsm_in(4'h5),
        .num_boxes_in(16'd42), .layer_idx_in(16'd7),
        .ucode_pc_in(16'h0),
        .ucode_we(ucode_we), .ucode_waddr(ucode_waddr),
        .ucode_wdata(ucode_wdata)
    );

    task automatic avs_wr(input [5:0] a, input [31:0] d);
        begin
            @(posedge clk);
            avs_address   <= a;
            avs_writedata <= d;
            avs_write     <= 1'b1;
            @(posedge clk);
            // waitrequest rises the cycle AFTER the request is seen, so wait
            // for it to go high before waiting for it to fall. Skipping that
            // returns while the AXI transaction is still in flight and the
            // register reads back its OLD value -- which looks like a broken
            // decode and is a broken testbench.
            while (!avs_waitrequest) @(posedge clk);
            while (avs_waitrequest)  @(posedge clk);
            avs_write <= 1'b0;
            @(posedge clk);
            @(posedge clk);
        end
    endtask

    // Reading is done by watching, not by counting edges.
    //
    // Trying to sample avs_readdata at a computed offset from waitrequest went
    // wrong three different ways -- one transaction early, one late, and
    // correct-by-accident for the one register answered without an AXI cycle.
    // The robust form is to hold the request until the slave has clearly
    // finished, then read the port: avs_readdata is a register that keeps its
    // value until the next transaction, so there is no race once the
    // transaction is over.
    task automatic avs_rd(input [5:0] a, output [31:0] d);
        integer g;
        begin
            @(posedge clk);
            avs_address <= a;
            avs_read    <= 1'b1;

            // Hold the request until waitrequest has risen AND fallen again,
            // or until it is clear this access never stalls (IDENT is answered
            // combinationally inside dvcon_regs).
            g = 0;
            @(posedge clk);
            while (!avs_waitrequest && g < 4) begin
                @(posedge clk);
                g = g + 1;
            end
            while (avs_waitrequest) @(posedge clk);

            // One more edge with the request still asserted, so a slave that
            // registers its output on the completing edge has presented it.
            @(posedge clk);
            d = avs_readdata;

            avs_read <= 1'b0;
            @(posedge clk);
        end
    endtask

    task automatic check(input [255:0] what, input [31:0] got,
                          input [31:0] exp_);
        begin
            if (got !== exp_) begin
                errors = errors + 1;
                $display("  FAIL: %0s = %08h, expected %08h", what, got, exp_);
            end else begin
                $display("  ok  : %0s = %08h", what, got);
            end
        end
    endtask

    reg [31:0] rd;

    initial begin
        $display("=== tb_dvcon_regs ===");
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // Word address 6 == byte offset 0x18 == DESC_ADDR. This is the check
        // that the word/byte conversion is right; if it is off by a factor of
        // four this lands somewhere else entirely.
        $display("-- writes reach the right registers --");
        avs_wr(6'd6, 32'hDEAD_0018);
        check("desc_addr", desc_addr, 32'hDEAD_0018);

        // Word 7 == 0x1C: an ODD 32-bit word, so it arrives at AWADDR 0x18
        // with the UPPER strobe. Getting the half wrong writes DESC_ADDR
        // instead -- and DESC_ADDR was just written, so a naive decode would
        // silently overwrite it here and still look plausible.
        avs_wr(6'd7, 32'hBEEF_001C);
        check("img_addr",  img_addr,  32'hBEEF_001C);
        check("desc_addr survived", desc_addr, 32'hDEAD_0018);

        avs_wr(6'd8, 32'h1234_0020);
        check("box_addr",  box_addr,  32'h1234_0020);

        avs_wr(6'd2, 32'hAAAA_0008);
        check("src_addr",  src_addr,  32'hAAAA_0008);
        avs_wr(6'd3, 32'hBBBB_000C);
        check("dst_addr",  dst_addr,  32'hBBBB_000C);
        check("src_addr survived", src_addr, 32'hAAAA_0008);

        avs_wr(6'd9, 32'h0000_007F);
        check("conf_thresh", {24'h0, conf_thresh}, 32'h0000_007F);

        // CTRL: START self-clears, MODE and ENGINE latch.
        $display("-- control --");
        avs_wr(6'd0, 32'h0000_0003);      // START | MODE_YOLO
        check("mode_yolo",  {31'h0, mode_yolo},  32'h1);
        check("mode_ucode", {31'h0, mode_ucode}, 32'h0);

        avs_wr(6'd0, 32'h0000_0007);      // START | MODE_YOLO | ENGINE_UCODE
        check("mode_ucode after bit2", {31'h0, mode_ucode}, 32'h1);

        // Reads come back through the same halving.
        $display("-- reads --");
        avs_rd(6'd1, rd);                 // STATUS, odd word -> upper half
        check("status done|fsm", rd & 32'h0000_00F2, 32'h0000_0052);

        avs_rd(6'd10, rd);                // NUM_BOXES
        check("num_boxes", rd & 32'h0000_FFFF, 32'd42);

        avs_rd(6'd11, rd);                // LAYER_IDX, odd word
        check("layer_idx", rd & 32'h0000_FFFF, 32'd7);

        // IDENT is answered by dvcon_regs itself, not forwarded: the host needs
        // an answer even when the accelerator is wedged, and it carries
        // ARRAY_SIZE so a blob built for a different array is refused.
        avs_rd(6'd12, rd);
        check("ident", rd, 32'hDC10_BEEF);

        if (errors == 0) $display("=== PASSED: 0 error(s) ===");
        else             $display("=== FAILED: %0d error(s) ===", errors);
        $finish;
    end

    initial begin
        #2_000_000;
        $display("=== FAILED: timeout ===");
        $finish;
    end

endmodule
