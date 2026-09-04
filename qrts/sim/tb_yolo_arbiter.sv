// =============================================================================
// tb_yolo_arbiter.sv — yolo_axi_arbiter, and specifically the dropped-pulse bug
//
// This module had NO testbench, which is how the bug below survived to the
// board. tb_arbiter covers avalon_arbiter (the memory-side one); nothing
// covered the 4-to-1 mux the accelerator's engines and sequencer share.
//
// ---------------------------------------------------------------------------
// THE BUG THIS EXISTS TO CATCH
// ---------------------------------------------------------------------------
// Requesters assert *_rd_start for exactly ONE cycle. The old arbiter routed
// that pulse combinationally, which only reaches the master while the channel
// is UNLOCKED. Pulse while another requester holds the lock and the request is
// gone: the master never sees it, no done ever comes back, and the requester
// waits forever.
//
// On hardware that presented as the sequencer parked in S_FETCH_W with busy=1
// and error=0 after two layers -- the sequencer is requester 0 and issues its
// next descriptor fetch one cycle after an engine reports done, while that
// engine's last burst can still hold the lock.
//
// TEST 3 is the regression: requester 1 takes the channel, requester 0 pulses
// start mid-burst, and both must complete. On the old RTL requester 0 never
// sees rd_done and the bench times out.
//
// The downstream master is modelled, not real: this is about arbitration, and
// tb_desc_fetch already drives the real bridge/arbiter/sdram_ctrl stack.
// =============================================================================
`timescale 1ns/1ps

module tb_yolo_arbiter;

    localparam int TIMEOUT_CYC = 20000;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;               // 100 MHz

    // ---- requester side ---------------------------------------------------
    reg  [3:0]  rd_start;
    reg  [63:0] rd_addr [0:3];
    reg  [7:0]  rd_len  [0:3];
    wire [3:0]  rd_data_valid, rd_done, rd_error;
    wire [63:0] rd_data [0:3];

    reg  [3:0]  wr_start;
    reg  [63:0] wr_addr [0:3];
    reg  [7:0]  wr_len  [0:3];
    reg  [63:0] wr_data [0:3];
    reg  [7:0]  wr_strb [0:3];
    wire [3:0]  wr_data_ready, wr_done, wr_error;

    // ---- downstream master ------------------------------------------------
    wire        m_rd_start;
    wire [63:0] m_rd_addr;
    wire [7:0]  m_rd_len;
    reg  [63:0] m_rd_data;
    reg         m_rd_data_valid, m_rd_done, m_rd_error;

    wire        m_wr_start;
    wire [63:0] m_wr_addr;
    wire [7:0]  m_wr_len;
    wire [63:0] m_wr_data;
    wire [7:0]  m_wr_strb;
    reg         m_wr_data_ready, m_wr_done, m_wr_error;

    yolo_axi_arbiter dut (
        .clk(clk), .rst_n(rst_n),

        .s0_rd_start(rd_start[0]), .s0_rd_addr(rd_addr[0]), .s0_rd_len(rd_len[0]),
        .s0_rd_data(rd_data[0]), .s0_rd_data_valid(rd_data_valid[0]),
        .s0_rd_done(rd_done[0]), .s0_rd_error(rd_error[0]),
        .s0_wr_start(wr_start[0]), .s0_wr_addr(wr_addr[0]), .s0_wr_len(wr_len[0]),
        .s0_wr_data(wr_data[0]), .s0_wr_strb(wr_strb[0]),
        .s0_wr_data_ready(wr_data_ready[0]), .s0_wr_done(wr_done[0]), .s0_wr_error(wr_error[0]),

        .s1_rd_start(rd_start[1]), .s1_rd_addr(rd_addr[1]), .s1_rd_len(rd_len[1]),
        .s1_rd_data(rd_data[1]), .s1_rd_data_valid(rd_data_valid[1]),
        .s1_rd_done(rd_done[1]), .s1_rd_error(rd_error[1]),
        .s1_wr_start(wr_start[1]), .s1_wr_addr(wr_addr[1]), .s1_wr_len(wr_len[1]),
        .s1_wr_data(wr_data[1]), .s1_wr_strb(wr_strb[1]),
        .s1_wr_data_ready(wr_data_ready[1]), .s1_wr_done(wr_done[1]), .s1_wr_error(wr_error[1]),

        .s2_rd_start(rd_start[2]), .s2_rd_addr(rd_addr[2]), .s2_rd_len(rd_len[2]),
        .s2_rd_data(rd_data[2]), .s2_rd_data_valid(rd_data_valid[2]),
        .s2_rd_done(rd_done[2]), .s2_rd_error(rd_error[2]),
        .s2_wr_start(wr_start[2]), .s2_wr_addr(wr_addr[2]), .s2_wr_len(wr_len[2]),
        .s2_wr_data(wr_data[2]), .s2_wr_strb(wr_strb[2]),
        .s2_wr_data_ready(wr_data_ready[2]), .s2_wr_done(wr_done[2]), .s2_wr_error(wr_error[2]),

        .s3_rd_start(rd_start[3]), .s3_rd_addr(rd_addr[3]), .s3_rd_len(rd_len[3]),
        .s3_rd_data(rd_data[3]), .s3_rd_data_valid(rd_data_valid[3]),
        .s3_rd_done(rd_done[3]), .s3_rd_error(rd_error[3]),
        .s3_wr_start(wr_start[3]), .s3_wr_addr(wr_addr[3]), .s3_wr_len(wr_len[3]),
        .s3_wr_data(wr_data[3]), .s3_wr_strb(wr_strb[3]),
        .s3_wr_data_ready(wr_data_ready[3]), .s3_wr_done(wr_done[3]), .s3_wr_error(wr_error[3]),

        .m_rd_start(m_rd_start), .m_rd_addr(m_rd_addr), .m_rd_len(m_rd_len),
        .m_rd_data(m_rd_data), .m_rd_data_valid(m_rd_data_valid),
        .m_rd_done(m_rd_done), .m_rd_error(m_rd_error),
        .m_wr_start(m_wr_start), .m_wr_addr(m_wr_addr), .m_wr_len(m_wr_len),
        .m_wr_data(m_wr_data), .m_wr_strb(m_wr_strb),
        .m_wr_data_ready(m_wr_data_ready), .m_wr_done(m_wr_done), .m_wr_error(m_wr_error)
    );

    // -------------------------------------------------------------------------
    // Downstream read model: on m_rd_start, return m_rd_len beats after a few
    // cycles of latency, then pulse m_rd_done one cycle AFTER the last beat --
    // the ordering avalon_mm_master documents and the sequencer relies on.
    // Data is the address plus the beat index, so a misrouted burst is visible
    // rather than merely late.
    // -------------------------------------------------------------------------
    integer errors = 0;
    reg [63:0] served_addr;
    reg [7:0]  m_rd_len_lat;

    task automatic serve_read;
        integer b;
        begin
            served_addr = m_rd_addr;
            repeat (3) @(posedge clk);
            for (b = 0; b < m_rd_len_lat; b = b + 1) begin
                @(posedge clk);
                m_rd_data       <= served_addr + b;
                m_rd_data_valid <= 1'b1;
            end
            @(posedge clk);
            m_rd_data_valid <= 1'b0;
            m_rd_done       <= 1'b1;
            @(posedge clk);
            m_rd_done       <= 1'b0;
        end
    endtask

    initial begin
        m_rd_data = 64'd0; m_rd_data_valid = 1'b0;
        m_rd_done = 1'b0;  m_rd_error = 1'b0;
        forever begin
            @(posedge clk);
            if (m_rd_start) begin
                m_rd_len_lat = m_rd_len;
                serve_read;
            end
        end
    end

    // ---- downstream write model -------------------------------------------
    initial begin
        m_wr_data_ready = 1'b0; m_wr_done = 1'b0; m_wr_error = 1'b0;
        forever begin
            @(posedge clk);
            if (m_wr_start) begin
                repeat (2) @(posedge clk);
                m_wr_data_ready <= 1'b1;
                repeat (m_wr_len) @(posedge clk);
                m_wr_data_ready <= 1'b0;
                @(posedge clk);
                m_wr_done <= 1'b1;
                @(posedge clk);
                m_wr_done <= 1'b0;
            end
        end
    end

    // ---- helpers ----------------------------------------------------------
    task automatic issue_read(input integer who, input [63:0] a, input [7:0] n);
        begin
            @(posedge clk);
            rd_addr[who]  <= a;
            rd_len[who]   <= n;
            rd_start[who] <= 1'b1;
            @(posedge clk);
            rd_start[who] <= 1'b0;
        end
    endtask

    // Wait for one requester's done, counting beats on the way.
    task automatic await_read(input integer who, input [127:0] name,
                              output integer beats, output integer timed_out);
        integer c;
        begin
            beats = 0; timed_out = 0; c = 0;
            while (!rd_done[who] && c < TIMEOUT_CYC) begin
                @(posedge clk);
                if (rd_data_valid[who]) beats = beats + 1;
                c = c + 1;
            end
            if (c >= TIMEOUT_CYC) begin
                timed_out = 1;
                $display("  [FAIL] %0s: rd_done never arrived (%0d cycles)", name, c);
                errors = errors + 1;
            end
        end
    endtask

    integer beats0, beats1, to0, to1;

    initial begin
        rd_start = 4'd0; wr_start = 4'd0;
        for (int i = 0; i < 4; i++) begin
            rd_addr[i] = 0; rd_len[i] = 0;
            wr_addr[i] = 0; wr_len[i] = 0; wr_data[i] = 0; wr_strb[i] = 8'hFF;
        end
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        $display("=== tb_yolo_arbiter ===");

        // ---- 1. a lone requester ------------------------------------------
        issue_read(0, 64'h0000_0020, 8'd8);
        await_read(0, "test1 lone read", beats0, to0);
        if (!to0 && beats0 != 8) begin
            $display("  [FAIL] test1: %0d beats, expected 8", beats0);
            errors = errors + 1;
        end else if (!to0)
            $display("  [ok  ] test1 lone read: 8 beats");

        // ---- 2. two requesters, strictly sequential ------------------------
        issue_read(1, 64'h0010_0000, 8'd4);
        await_read(1, "test2 engine read", beats1, to1);
        if (!to1 && beats1 != 4) begin
            $display("  [FAIL] test2: %0d beats, expected 4", beats1);
            errors = errors + 1;
        end else if (!to1)
            $display("  [ok  ] test2 sequential: 4 beats");

        repeat (5) @(posedge clk);

        // ---- 3. THE REGRESSION --------------------------------------------
        // Requester 1 (an engine) owns the channel. Requester 0 (the
        // sequencer) pulses start while that burst is still running. Its
        // single-cycle pulse must be remembered and served afterwards.
        //
        // Old RTL: the pulse lands while rd_locked is set for requester 1, is
        // never routed to the master, and requester 0 hangs -- exactly the
        // S_FETCH_W stall seen on the board.
        $display("  ---- test3: request while the channel is locked ----");
        fork
            begin
                issue_read(1, 64'h0020_0000, 8'd16);
                await_read(1, "test3 engine", beats1, to1);
            end
            begin
                // Land mid-burst, well after the lock is taken.
                repeat (8) @(posedge clk);
                issue_read(0, 64'h0000_0060, 8'd8);
                await_read(0, "test3 sequencer (THE BUG)", beats0, to0);
            end
        join

        if (!to0 && !to1) begin
            if (beats0 != 8 || beats1 != 16) begin
                $display("  [FAIL] test3: beats seq=%0d (want 8) eng=%0d (want 16)",
                         beats0, beats1);
                errors = errors + 1;
            end else
                $display("  [ok  ] test3: both served, seq=8 eng=16 beats");
        end

        // ---- 4. same hazard on the write channel ---------------------------
        $display("  ---- test4: write request while locked ----");
        fork
            begin
                @(posedge clk);
                wr_addr[1] <= 64'h0030_0000; wr_len[1] <= 8'd8;
                wr_start[1] <= 1'b1; @(posedge clk); wr_start[1] <= 1'b0;
                begin : w1
                    integer c; c = 0;
                    while (!wr_done[1] && c < TIMEOUT_CYC) begin @(posedge clk); c = c + 1; end
                    if (c >= TIMEOUT_CYC) begin
                        $display("  [FAIL] test4: engine wr_done never arrived");
                        errors = errors + 1;
                    end
                end
            end
            begin
                repeat (4) @(posedge clk);
                wr_addr[0] <= 64'h0000_0100; wr_len[0] <= 8'd2;
                wr_start[0] <= 1'b1; @(posedge clk); wr_start[0] <= 1'b0;
                begin : w0
                    integer c; c = 0;
                    while (!wr_done[0] && c < TIMEOUT_CYC) begin @(posedge clk); c = c + 1; end
                    if (c >= TIMEOUT_CYC) begin
                        $display("  [FAIL] test4: sequencer wr_done never arrived (THE BUG)");
                        errors = errors + 1;
                    end else
                        $display("  [ok  ] test4: both writes completed");
                end
            end
        join

        // run_all.sh scans for a line starting "=== PASSED" / "=== FAILED".
        if (errors == 0) $display("=== PASSED: 0 error(s) ===");
        else             $display("=== FAILED: %0d error(s) ===", errors);
        $finish;
    end

    // Global backstop so a hang is a failure rather than a hung job.
    initial begin
        #(TIMEOUT_CYC * 4 * 10);
        $display("  [FAIL] global timeout -- a requester never completed");
        $display("=== FAILED: global timeout ===");
        $finish;
    end

endmodule
