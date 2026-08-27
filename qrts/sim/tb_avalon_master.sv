// =============================================================================
// tb_avalon_master.sv — does the Avalon shim behave like the AXI master?
//
// The whole premise of the port is that the engines never see the bus: they see
// rd_start/rd_len -> rd_data/rd_data_valid/rd_done and
// wr_start/wr_data/wr_strb -> wr_data_ready/wr_done. Swapping axi4_master for
// avalon_mm_master is only safe if that handshake is bit- and cycle-compatible
// in the ways the engines actually depend on.
//
// This drives both masters with identical stimulus against equivalent memory
// models and compares what comes back:
//
//   reads   the sequence of rd_data beats, and that rd_done arrives AFTER the
//           last rd_data_valid rather than with it (yolo_conv_engine's C_ACT_W
//           consumes the final beat on the same cycle it sees rd_done, so
//           collapsing the two silently drops the last element of every
//           activation column)
//
//   writes  the bytes actually committed to memory, honouring wr_strb. This is
//           the check that would have caught the all-ones-byteenable defect:
//           an all-lanes memory model shows nothing wrong.
//
// Deliberately covers the burst lengths the engines really use: 1 (the conv
// engine's per-element reads and per-byte stores), ARRAY_SIZE^2/8 (a weight
// tile), and a burst past MAX_BURST so the Avalon-side split is exercised --
// the split is invisible to the caller, which is exactly what needs proving.
// =============================================================================
`timescale 1ns/1ps

module tb_avalon_master;

    localparam integer CORE_W   = 64;
    localparam integer AV_W     = 32;
    localparam integer MAX_BURST= 64;
    localparam integer MEM_BYTES= 1 << 16;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    integer errors = 0;
    integer checks = 0;

    // Two independent copies of the same initial memory, so a write test can
    // compare the committed bytes side by side.
    reg [7:0] mem_axi [0:MEM_BYTES-1];
    reg [7:0] mem_avl [0:MEM_BYTES-1];

    // =========================================================================
    // Shared stimulus
    // =========================================================================
    reg         rd_start = 1'b0;
    reg  [63:0] rd_addr  = 64'h0;
    reg  [7:0]  rd_len   = 8'd0;

    reg         wr_start = 1'b0;
    reg  [63:0] wr_addr  = 64'h0;
    reg  [7:0]  wr_len   = 8'd0;
    reg  [7:0]  wr_strb  = 8'h0;

    // Each master consumes beats at its own rate, so each gets its own data
    // register driven by its own accepted-beat count. Sharing one register and
    // advancing it on either ready lets the faster master pull the data out
    // from under the slower one -- which looks exactly like a DUT bug.
    reg  [63:0] wr_data_axi = 64'h0;
    reg  [63:0] wr_data_avl = 64'h0;

    // =========================================================================
    // DUT A: the existing AXI4 master, with a behavioural AXI slave
    // =========================================================================
    wire [11:0] a_arid, a_awid;
    wire [63:0] a_araddr, a_awaddr;
    wire [7:0]  a_arlen, a_awlen;
    wire [2:0]  a_arsize, a_awsize, a_arprot, a_awprot;
    wire [1:0]  a_arburst, a_awburst;
    wire        a_arvalid, a_awvalid;
    reg         a_arready = 1'b0, a_awready = 1'b0;

    reg  [63:0] a_rdata = 64'h0;
    reg  [1:0]  a_rresp = 2'b00;
    reg         a_rlast = 1'b0, a_rvalid = 1'b0;
    wire        a_rready;
    reg  [11:0] a_rid = 12'h0;

    wire [63:0] a_wdata;
    wire [7:0]  a_wstrb;
    wire        a_wlast, a_wvalid;
    reg         a_wready = 1'b0;

    wire        a_bready;
    reg         a_bvalid = 1'b0;
    reg  [11:0] a_bid = 12'h0;
    reg  [1:0]  a_bresp = 2'b00;

    wire [63:0] axi_rd_data;
    wire        axi_rd_data_valid, axi_rd_done, axi_rd_error;
    wire        axi_wr_data_ready, axi_wr_done, axi_wr_error;

    axi4_master #(.ADDR_WIDTH(64), .DATA_WIDTH(64), .ID_WIDTH(12)) u_axi (
        .clk(clk), .rst_n(rst_n),
        .m_arid(a_arid), .m_araddr(a_araddr), .m_arlen(a_arlen),
        .m_arsize(a_arsize), .m_arburst(a_arburst), .m_arprot(a_arprot),
        .m_arvalid(a_arvalid), .m_arready(a_arready),
        .m_rid(a_rid), .m_rdata(a_rdata), .m_rresp(a_rresp),
        .m_rlast(a_rlast), .m_rvalid(a_rvalid), .m_rready(a_rready),
        .m_awid(a_awid), .m_awaddr(a_awaddr), .m_awlen(a_awlen),
        .m_awsize(a_awsize), .m_awburst(a_awburst), .m_awprot(a_awprot),
        .m_awvalid(a_awvalid), .m_awready(a_awready),
        .m_wdata(a_wdata), .m_wstrb(a_wstrb), .m_wlast(a_wlast),
        .m_wvalid(a_wvalid), .m_wready(a_wready),
        .m_bready(a_bready), .m_bvalid(a_bvalid), .m_bid(a_bid), .m_bresp(a_bresp),
        .rd_start(rd_start), .rd_addr(rd_addr), .rd_len(rd_len),
        .rd_data(axi_rd_data), .rd_data_valid(axi_rd_data_valid),
        .rd_done(axi_rd_done), .rd_error(axi_rd_error),
        .wr_start(wr_start), .wr_addr(wr_addr), .wr_len(wr_len),
        .wr_data(wr_data_axi), .wr_strb(wr_strb),
        .wr_data_ready(axi_wr_data_ready),
        .wr_done(axi_wr_done), .wr_error(axi_wr_error)
    );

    // ---- behavioural AXI slave ----
    integer ax_rbeat, ax_rtotal, ax_wbeat, ax_i;
    reg [63:0] ax_rbase, ax_wbase;
    reg ax_rbusy, ax_wbusy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_arready <= 1'b0; a_rvalid <= 1'b0; a_rlast <= 1'b0;
            a_awready <= 1'b0; a_wready <= 1'b0; a_bvalid <= 1'b0;
            ax_rbusy <= 1'b0; ax_wbusy <= 1'b0;
            ax_rbeat <= 0; ax_rtotal <= 0; ax_wbeat <= 0;
        end else begin
            // --- read ---
            a_arready <= 1'b0;
            a_rvalid  <= 1'b0;
            a_rlast   <= 1'b0;
            if (a_arvalid && !ax_rbusy) begin
                a_arready <= 1'b1;
                ax_rbase  <= a_araddr;
                ax_rtotal <= a_arlen + 1;
                ax_rbeat  <= 0;
                ax_rbusy  <= 1'b1;
            end else if (ax_rbusy && a_rready) begin
                for (ax_i = 0; ax_i < 8; ax_i = ax_i + 1)
                    a_rdata[ax_i*8 +: 8] <= mem_axi[ax_rbase + ax_rbeat*8 + ax_i];
                a_rvalid <= 1'b1;
                a_rlast  <= (ax_rbeat == ax_rtotal-1);
                if (ax_rbeat == ax_rtotal-1) ax_rbusy <= 1'b0;
                else ax_rbeat <= ax_rbeat + 1;
            end

            // --- write ---
            a_awready <= 1'b0;
            a_bvalid  <= 1'b0;
            if (a_awvalid && !ax_wbusy) begin
                a_awready <= 1'b1;
                ax_wbase  <= a_awaddr;
                ax_wbeat  <= 0;
                ax_wbusy  <= 1'b1;
                a_wready  <= 1'b1;
            end else if (ax_wbusy && a_wvalid && a_wready) begin
                // A real slave applies the strobe at the aligned word.
                for (ax_i = 0; ax_i < 8; ax_i = ax_i + 1)
                    if (a_wstrb[ax_i])
                        mem_axi[((ax_wbase + ax_wbeat*8) & ~64'h7) + ax_i]
                            <= a_wdata[ax_i*8 +: 8];
                if (a_wlast) begin
                    a_wready <= 1'b0;
                    a_bvalid <= 1'b1;
                    ax_wbusy <= 1'b0;
                end else begin
                    ax_wbeat <= ax_wbeat + 1;
                end
            end
        end
    end

    // =========================================================================
    // DUT B: the Avalon shim, with a behavioural Avalon slave
    // =========================================================================
    wire [31:0] v_address;
    wire        v_read, v_write;
    wire [31:0] v_writedata;
    wire [3:0]  v_byteenable;
    wire [6:0]  v_burstcount;
    reg  [31:0] v_readdata = 32'h0;
    reg         v_readdatavalid = 1'b0;
    reg         v_waitrequest = 1'b0;

    wire [63:0] avl_rd_data;
    wire        avl_rd_data_valid, avl_rd_done, avl_rd_error;
    wire        avl_wr_data_ready, avl_wr_done, avl_wr_error;

    avalon_mm_master #(
        .ADDR_WIDTH(32), .DATA_WIDTH(AV_W), .CORE_WIDTH(CORE_W),
        .MAX_BURST(MAX_BURST)
    ) u_avl (
        .clk(clk), .rst_n(rst_n),
        .avm_address(v_address), .avm_read(v_read), .avm_write(v_write),
        .avm_writedata(v_writedata), .avm_byteenable(v_byteenable),
        .avm_burstcount(v_burstcount),
        .avm_readdata(v_readdata), .avm_readdatavalid(v_readdatavalid),
        .avm_waitrequest(v_waitrequest),
        .rd_start(rd_start), .rd_addr(rd_addr), .rd_len(rd_len),
        .rd_data(avl_rd_data), .rd_data_valid(avl_rd_data_valid),
        .rd_done(avl_rd_done), .rd_error(avl_rd_error),
        .wr_start(wr_start), .wr_addr(wr_addr), .wr_len(wr_len),
        .wr_data(wr_data_avl), .wr_strb(wr_strb),
        .wr_data_ready(avl_wr_data_ready),
        .wr_done(avl_wr_done), .wr_error(avl_wr_error)
    );

    // ---- behavioural Avalon slave ----
    //
    // Deliberately stalls with waitrequest and delays readdatavalid: a slave
    // that never stalls hides exactly the bugs this bench is for.
    integer av_rleft, av_i;
    reg [31:0] av_rptr;
    reg        av_rbusy;
    integer    av_stall;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_readdatavalid <= 1'b0;
            v_waitrequest   <= 1'b0;
            av_rbusy <= 1'b0; av_rleft <= 0; av_rptr <= 32'h0;
            av_stall <= 0;
        end else begin
            v_readdatavalid <= 1'b0;

            // Stall every third cycle while a request is on the bus.
            av_stall      <= (av_stall == 2) ? 0 : av_stall + 1;
            v_waitrequest <= (v_read || v_write) && (av_stall == 0);

            if (v_read && !v_waitrequest && !av_rbusy) begin
                av_rptr  <= v_address;
                av_rleft <= v_burstcount;
                av_rbusy <= 1'b1;
            end else if (av_rbusy) begin
                for (av_i = 0; av_i < 4; av_i = av_i + 1)
                    v_readdata[av_i*8 +: 8] <= mem_avl[av_rptr + av_i];
                v_readdatavalid <= 1'b1;
                av_rptr  <= av_rptr + 4;
                av_rleft <= av_rleft - 1;
                if (av_rleft == 1) av_rbusy <= 1'b0;
            end

            // Writes are committed by the burst-tracking block below, which
            // owns the address pointer. Committing here as well would write
            // every beat twice and to the wrong place after the first.
        end
    end

    // The Avalon slave above must advance its own write pointer across a burst;
    // Avalon holds one address for the whole burst and the slave increments.
    // Modelled by latching the base on the first accepted beat.
    reg [31:0] av_wptr;
    reg        av_wbusy;
    integer    av_wleft, av_j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            av_wbusy <= 1'b0; av_wptr <= 32'h0; av_wleft <= 0;
        end else if (v_write && !v_waitrequest) begin
            if (!av_wbusy) begin
                av_wptr  <= v_address + 4;
                av_wleft <= v_burstcount - 1;
                av_wbusy <= (v_burstcount > 1);
                for (av_j = 0; av_j < 4; av_j = av_j + 1)
                    if (v_byteenable[av_j])
                        mem_avl[v_address + av_j] <= v_writedata[av_j*8 +: 8];
            end else begin
                for (av_j = 0; av_j < 4; av_j = av_j + 1)
                    if (v_byteenable[av_j])
                        mem_avl[av_wptr + av_j] <= v_writedata[av_j*8 +: 8];
                av_wptr  <= av_wptr + 4;
                av_wleft <= av_wleft - 1;
                if (av_wleft == 1) av_wbusy <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Read comparison
    // =========================================================================
    reg [63:0] axi_beats [0:511];
    reg [63:0] avl_beats [0:511];
    integer    axi_n, avl_n;
    integer    axi_done_gap, avl_done_gap;   // cycles from last beat to done

    task automatic compare_read(input [63:0] addr, input [7:0] len);
        integer k;
        integer guard;
        begin
            axi_n = 0; avl_n = 0;
            axi_done_gap = -1; avl_done_gap = -1;

            @(posedge clk);
            rd_addr <= addr; rd_len <= len; rd_start <= 1'b1;
            @(posedge clk);
            rd_start <= 1'b0;

            // Run until BOTH masters have pulsed rd_done, collecting beats as
            // they arrive. done_gap records how many cycles separated the last
            // rd_data_valid from rd_done: the engines require it to be >= 1,
            // because C_ACT_W consumes the final beat on the cycle it sees
            // rd_done. A master that asserts both together drops that beat.
            begin : rd_wait
                reg axi_fin, avl_fin;
                integer axi_since, avl_since;
                axi_fin = 1'b0; avl_fin = 1'b0;
                axi_since = 0;  avl_since = 0;
                guard = 0;
                while ((!axi_fin || !avl_fin) && guard < 20000) begin
                    @(posedge clk);
                    guard = guard + 1;

                    if (!axi_fin) begin
                        if (axi_rd_data_valid) begin
                            axi_beats[axi_n] = axi_rd_data; axi_n = axi_n + 1;
                            axi_since = 0;
                        end else begin
                            axi_since = axi_since + 1;
                        end
                        if (axi_rd_done) begin
                            axi_fin = 1'b1; axi_done_gap = axi_since;
                        end
                    end

                    if (!avl_fin) begin
                        if (avl_rd_data_valid) begin
                            avl_beats[avl_n] = avl_rd_data; avl_n = avl_n + 1;
                            avl_since = 0;
                        end else begin
                            avl_since = avl_since + 1;
                        end
                        if (avl_rd_done) begin
                            avl_fin = 1'b1; avl_done_gap = avl_since;
                        end
                    end
                end
                if (guard >= 20000) begin
                    errors = errors + 1;
                    $display("  FAIL read addr=%0h len=%0d: timeout (axi_fin=%b avl_fin=%b)",
                             addr, len, axi_fin, avl_fin);
                end
            end

            checks = checks + 1;
            if (avl_done_gap < 1) begin
                errors = errors + 1;
                $display("  FAIL read addr=%0h len=%0d: rd_done coincides with the last beat (gap=%0d, axi gap=%0d)",
                         addr, len, avl_done_gap, axi_done_gap);
            end
            if (axi_n != avl_n) begin
                errors = errors + 1;
                $display("  FAIL read addr=%0h len=%0d: %0d beats vs %0d",
                         addr, len, axi_n, avl_n);
            end else begin
                for (k = 0; k < axi_n; k = k + 1)
                    if (axi_beats[k] !== avl_beats[k]) begin
                        errors = errors + 1;
                        if (errors <= 8)
                            $display("  FAIL read addr=%0h beat %0d: axi=%h avl=%h",
                                     addr, k, axi_beats[k], avl_beats[k]);
                    end
            end
            repeat (8) @(posedge clk);
        end
    endtask

    // =========================================================================
    // Write comparison
    // =========================================================================
    task automatic compare_write(input [63:0] addr, input [7:0] len,
                                  input [7:0] strb);
        integer guard, k, bad;
        integer axi_beat, avl_beat;
        reg axi_fin, avl_fin;
        begin
            axi_fin = 1'b0; avl_fin = 1'b0;
            axi_beat = 0;    avl_beat = 0;

            @(posedge clk);
            wr_addr <= addr; wr_len <= len; wr_strb <= strb;
            wr_data_axi <= BEAT(0);
            wr_data_avl <= BEAT(0);
            wr_start <= 1'b1;
            @(posedge clk);
            wr_start <= 1'b0;

            // Beat N carries the same value for both masters -- it is a
            // function of the beat index, exactly as an engine's store loop
            // makes it a function of its counter, not of time.
            guard = 0;
            while ((!axi_fin || !avl_fin) && guard < 40000) begin
                @(posedge clk);
                guard = guard + 1;
                if (axi_wr_data_ready) begin
                    axi_beat = axi_beat + 1;
                    wr_data_axi <= BEAT(axi_beat);
                end
                if (avl_wr_data_ready) begin
                    avl_beat = avl_beat + 1;
                    wr_data_avl <= BEAT(avl_beat);
                end
                if (axi_wr_done) axi_fin = 1'b1;
                if (avl_wr_done) avl_fin = 1'b1;
            end
            if (guard >= 40000) begin
                errors = errors + 1;
                $display("  FAIL write addr=%0h len=%0d: timeout", addr, len);
            end

            checks = checks + 1;
            bad = 0;
            for (k = 0; k < len*8; k = k + 1)
                if (mem_axi[addr + k] !== mem_avl[addr + k]) begin
                    bad = bad + 1;
                    if (bad <= 4)
                        $display("      byte +%0d: axi=%h avl=%h",
                                 k, mem_axi[addr+k], mem_avl[addr+k]);
                end
            if (bad != 0) begin
                errors = errors + 1;
                $display("  FAIL write addr=%0h len=%0d strb=%b: %0d byte(s) differ",
                         addr, len, strb, bad);
            end
            repeat (8) @(posedge clk);
        end
    endtask

    // Value carried by write beat n. Deterministic in n so both masters see
    // the same payload for the same beat regardless of when they consume it.
    function automatic [63:0] BEAT(input integer n);
        BEAT = {8{8'hA0}} ^ (64'h0001020304050607 + n);
    endfunction

    integer i, seed = 32'h5EED;

    initial begin
        $display("=== tb_avalon_master ===");
        for (i = 0; i < MEM_BYTES; i = i + 1) begin
            mem_axi[i] = $random(seed);
            mem_avl[i] = mem_axi[i];
        end

        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // Reads. len=1 is the conv engine's per-element activation fetch;
        // len=32 is a 16x16 weight tile; len=96 forces an Avalon burst split.
        compare_read(64'h0000_0100, 8'd1);
        compare_read(64'h0000_0200, 8'd2);
        compare_read(64'h0000_0400, 8'd32);
        compare_read(64'h0000_0800, 8'd96);

        // Writes. Single-byte lanes are the case that the old all-ones
        // byteenable destroyed, so every lane gets its own check.
        for (i = 0; i < 8; i = i + 1)
            compare_write(64'h0000_1000 + i*8, 8'd1, 8'h01 << i);
        compare_write(64'h0000_2000, 8'd4,  8'hFF);
        compare_write(64'h0000_3000, 8'd40, 8'hFF);

        if (errors == 0)
            $display("=== PASSED: %0d checks, masters agree ===", checks);
        else
            $display("=== FAILED: %0d error(s) over %0d checks ===", errors, checks);
        $finish;
    end

endmodule
