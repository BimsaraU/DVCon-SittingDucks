// =============================================================================
// tb_fast_slave.sv — avalon_mm_master against a ZERO-LATENCY read slave
//
// Every other bench drives avalon_mm_master through sdram_model, which always
// takes at least a cycle between accepting a read and returning its first
// beat. Avalon does not require that, and sdram_ctrl on the board returns the
// first beat in the SAME cycle it drops waitrequest when the row is already
// open.
//
// The master used to capture beats only in its RD_DATA state, which it enters
// the cycle AFTER acceptance -- so that first beat was silently dropped. With
// SUB=2 (64-bit engine over a 32-bit bus) two Avalon beats make one engine
// word, so losing one shifts every word after it by 32 bits.
//
// On hardware that shifted the descriptor table by one word: the sequencer
// read desc[0] out of the descriptor's WORD 1, getting `flags` (2) instead of
// `op` (1). Every layer dispatched to the elementwise engine, and desc[4]
// (dst) came back as word 5 (zero), so conv output was written to address 0 --
// over the descriptor table itself.
//
// This bench models the fast slave and checks the DATA, not just the count:
// a dropped beat shows up as the wrong value in the wrong lane.
// =============================================================================
`timescale 1ns/1ps

module tb_fast_slave;

    localparam integer AW   = 32;
    localparam integer DW   = 32;   // Avalon side
    localparam integer CW   = 64;   // engine side, SUB = 2
    localparam integer MAXB = 64;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    // ---- engine side ----
    reg         rd_start;
    reg  [63:0] rd_addr;
    reg  [7:0]  rd_len;
    wire [CW-1:0] rd_data;
    wire        rd_data_valid, rd_done, rd_error;

    // ---- Avalon side ----
    wire [AW-1:0] avm_address;
    wire          avm_read, avm_write;
    wire [DW-1:0] avm_writedata;
    wire [DW/8-1:0] avm_byteenable;
    wire [6:0]    avm_burstcount;
    reg  [DW-1:0] avm_readdata;
    reg           avm_readdatavalid;  // driven combinationally below
    reg           avm_waitrequest;

    avalon_mm_master #(
        .ADDR_WIDTH(AW), .DATA_WIDTH(DW), .CORE_WIDTH(CW), .MAX_BURST(MAXB)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .avm_address(avm_address), .avm_read(avm_read), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount),
        .avm_readdata(avm_readdata), .avm_readdatavalid(avm_readdatavalid),
        .avm_waitrequest(avm_waitrequest),
        .rd_start(rd_start), .rd_addr(rd_addr), .rd_len(rd_len),
        .rd_data(rd_data), .rd_data_valid(rd_data_valid),
        .rd_done(rd_done), .rd_error(rd_error),
        .wr_start(1'b0), .wr_addr(64'd0), .wr_len(8'd0),
        .wr_data(64'd0), .wr_strb(8'h00),
        .wr_data_ready(), .wr_done(), .wr_error()
    );

    // -------------------------------------------------------------------------
    // Zero-latency slave: waitrequest is low, and the first beat is presented
    // in the SAME cycle the master's read is accepted. Word N of the burst is
    // 32'hA0000000 + N so a dropped or reordered beat is visible in the value.
    // -------------------------------------------------------------------------
    integer beats_to_send = 0;
    integer send_idx      = 0;
    reg     sending       = 1'b0;
    reg [DW-1:0] q_data;
    reg          q_valid;

    // The first beat must be presented COMBINATIONALLY with the accepted read,
    // not one cycle later. A registered model gives a cycle of latency, which
    // is precisely the case that already worked -- and why this bug reached
    // hardware despite every existing bench.
    wire first_beat = avm_read && !sending;

    always @(*) begin
        if (first_beat) begin
            avm_readdatavalid = 1'b1;
            avm_readdata      = 32'hA000_0000;
        end else begin
            avm_readdatavalid = q_valid;
            avm_readdata      = q_data;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_valid  <= 1'b0;
            q_data   <= 32'd0;
            sending  <= 1'b0;
            send_idx <= 0;
        end else begin
            q_valid <= 1'b0;
            if (first_beat) begin
                sending       <= 1'b1;
                beats_to_send <= avm_burstcount;
                send_idx      <= 1;
            end else if (sending) begin
                if (send_idx < beats_to_send) begin
                    q_data   <= 32'hA000_0000 + send_idx;
                    q_valid  <= 1'b1;
                    send_idx <= send_idx + 1;
                end else begin
                    sending <= 1'b0;
                end
            end
        end
    end

    initial avm_waitrequest = 1'b0;

    // ---- checker ----------------------------------------------------------
    integer errors = 0;
    integer words_seen = 0;
    reg [CW-1:0] expect_word;

    always @(posedge clk) begin
        if (rst_n && rd_data_valid) begin
            // Little-endian packing: the first Avalon beat is the LOW half.
            expect_word = {32'hA000_0000 + (words_seen*2 + 1),
                           32'hA000_0000 + (words_seen*2)};
            if (rd_data !== expect_word) begin
                $display("  [FAIL] word %0d: got %h expected %h",
                         words_seen, rd_data, expect_word);
                errors = errors + 1;
            end
            words_seen = words_seen + 1;
        end
    end

    integer c;
    initial begin
        rd_start = 1'b0; rd_addr = 64'd0; rd_len = 8'd0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        $display("=== tb_fast_slave ===");

        // 8 engine words = 16 Avalon beats.
        @(posedge clk);
        rd_addr  <= 64'h0000_0020;
        rd_len   <= 8'd8;
        rd_start <= 1'b1;
        @(posedge clk);
        rd_start <= 1'b0;

        c = 0;
        while (!rd_done && c < 2000) begin
            @(posedge clk);
            c = c + 1;
        end

        if (c >= 2000) begin
            $display("  [FAIL] rd_done never arrived");
            errors = errors + 1;
        end else if (words_seen != 8) begin
            $display("  [FAIL] %0d engine words, expected 8", words_seen);
            errors = errors + 1;
        end else if (errors == 0) begin
            $display("  [ok  ] 8 words, all lanes correct with a same-cycle first beat");
        end

        if (errors == 0) $display("=== PASSED: 0 error(s) ===");
        else             $display("=== FAILED: %0d error(s) ===", errors);
        $finish;
    end

    initial begin
        #200000;
        $display("=== FAILED: global timeout ===");
        $finish;
    end

endmodule
