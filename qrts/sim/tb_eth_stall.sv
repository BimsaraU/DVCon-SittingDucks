// =============================================================================
// tb_eth_stall.sv — eth_cmd_engine against an SDRAM that actually says "wait"
//
// tb_eth_cmd and tb_eth_path both hold avm_waitrequest LOW for the whole run
// (tb_eth_path ties it to a literal 1'b0). Every write is therefore accepted in
// the cycle it is offered, and the one thing this engine has to get right on
// real hardware -- what happens to a completed word when the memory is busy --
// was never exercised by either of them.
//
// On the board it is busy often: a single-beat write costs ~15 cycles through
// ACTIVE/WRITE/PRECHARGE, a refresh costs ~10 more, and the accelerator has
// priority in avalon_arbiter. The engine used to drive avm_* straight from the
// packer, so a word completing during a stall simply overwrote the pending one
// and was lost. Nothing downstream notices: the frame's FCS was good so its bit
// is set in rx_bitmap and the host never retransmits, and stat_bytes counts
// bytes received rather than words committed. The model in SDRAM ends up
// intact except for scattered stale words -- which is exactly why loading over
// JTAG (one word at a time, each one waited on) worked and Ethernet did not.
//
// This bench drives a pseudo-random stall pattern, including stalls longer than
// the four cycles it takes to pack the next word, and checks every byte.
// =============================================================================
`timescale 1ns/1ps

module tb_eth_stall;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    reg  [7:0] rx_data  = 8'h0;
    reg        rx_valid = 1'b0;
    reg        rx_sof   = 1'b0;
    reg        rx_eof   = 1'b0;
    reg        rx_err   = 1'b0;

    wire [7:0] tx_data;
    wire       tx_valid, tx_last;
    reg        tx_ready = 1'b1;

    wire [31:0] avm_address, avm_writedata;
    wire        avm_write;
    wire [3:0]  avm_byteenable;
    wire [6:0]  avm_burstcount;
    reg         avm_waitrequest = 1'b0;

    wire [31:0] stat_frames, stat_bytes, stat_wdrop;
    wire [63:0] rx_bitmap;

    eth_cmd_engine dut (
        .clk(clk), .rst_n(rst_n),
        .rx_data(rx_data), .rx_valid(rx_valid), .rx_sof(rx_sof),
        .rx_eof(rx_eof), .rx_err(rx_err),
        .tx_data(tx_data), .tx_valid(tx_valid), .tx_last(tx_last),
        .tx_ready(tx_ready),
        .avm_address(avm_address), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount), .avm_waitrequest(avm_waitrequest),
        .stat_frames(stat_frames), .stat_bytes(stat_bytes),
        .stat_wdrop(stat_wdrop), .rx_bitmap(rx_bitmap)
    );

    // ---- stall generator ----------------------------------------------------
    // Bursty, like the real thing: mostly free, with occasional multi-cycle
    // stalls long enough that another word finishes packing underneath.
    integer seed = 32'h1234_5678;
    integer stall_left = 0;

    always @(posedge clk) begin
        if (!rst_n) begin
            avm_waitrequest <= 1'b0;
            stall_left      <= 0;
        end else if (stall_left > 0) begin
            stall_left      <= stall_left - 1;
            avm_waitrequest <= 1'b1;
            if (stall_left == 1) avm_waitrequest <= 1'b0;
        end else begin
            if (($random(seed) % 100) < 40) begin
                stall_left      <= 1 + ({$random(seed)} % 25);
                avm_waitrequest <= 1'b1;
            end else begin
                avm_waitrequest <= 1'b0;
            end
        end
    end

    // ---- model memory, byte addressed so lane errors stay visible -----------
    reg [7:0] mem [0:65535];
    reg       written [0:65535];
    integer   n_writes = 0;

    always @(posedge clk) begin
        if (avm_write && !avm_waitrequest) begin
            if (avm_byteenable[0]) begin
                mem[avm_address+0] = avm_writedata[7:0];   written[avm_address+0] = 1'b1;
            end
            if (avm_byteenable[1]) begin
                mem[avm_address+1] = avm_writedata[15:8];  written[avm_address+1] = 1'b1;
            end
            if (avm_byteenable[2]) begin
                mem[avm_address+2] = avm_writedata[23:16]; written[avm_address+2] = 1'b1;
            end
            if (avm_byteenable[3]) begin
                mem[avm_address+3] = avm_writedata[31:24]; written[avm_address+3] = 1'b1;
            end
            n_writes = n_writes + 1;
        end
    end

    integer errors = 0;

    // ---- stimulus, matching eth_mac_rx's real framing -----------------------
    task automatic rx_byte(input [7:0] b, input bit sof);
        begin
            @(negedge clk);
            rx_data = b; rx_valid = 1'b1; rx_sof = sof; rx_eof = 1'b0;
            @(posedge clk);
            @(negedge clk);
            rx_valid = 1'b0; rx_sof = 1'b0;
        end
    endtask

    task automatic rx_eof_pulse;
        begin
            @(negedge clk);
            rx_valid = 1'b0; rx_eof = 1'b1;
            @(posedge clk);
            @(negedge clk);
            rx_eof = 1'b0;
        end
    endtask

    task automatic send_write(input [31:0] seq, input [31:0] addr,
                              input [15:0] len, input [7:0] first);
        integer i;
        begin
            for (i = 0; i < 6; i = i + 1) rx_byte(8'h02 + i[7:0], (i == 0));
            for (i = 0; i < 6; i = i + 1) rx_byte(8'h10 + i[7:0], 1'b0);
            rx_byte(8'h88, 1'b0); rx_byte(8'hB5, 1'b0);   // ethertype
            rx_byte(8'h01, 1'b0);                          // ver
            rx_byte(8'h01, 1'b0);                          // OP_WRITE_MEM
            rx_byte(8'h00, 1'b0); rx_byte(8'h00, 1'b0);    // flags, rsv
            rx_byte(seq[31:24], 1'b0); rx_byte(seq[23:16], 1'b0);
            rx_byte(seq[15:8],  1'b0); rx_byte(seq[7:0],   1'b0);
            rx_byte(addr[31:24],1'b0); rx_byte(addr[23:16],1'b0);
            rx_byte(addr[15:8], 1'b0); rx_byte(addr[7:0],  1'b0);
            rx_byte(len[15:8],  1'b0); rx_byte(len[7:0],   1'b0);
            rx_byte(8'h00, 1'b0); rx_byte(8'h00, 1'b0);    // rsv2
            for (i = 0; i < len; i = i + 1) rx_byte(first + i[7:0], 1'b0);
            rx_eof_pulse();
            repeat (200) @(posedge clk);   // let the queue drain
        end
    endtask

    integer i, f;
    reg [31:0] base;
    reg [7:0]  expv;

    initial begin
        $display("=== tb_eth_stall ===");
        for (i = 0; i < 65536; i = i + 1) begin
            mem[i] = 8'hFF; written[i] = 1'b0;
        end
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (6) @(posedge clk);

        // Eight frames of 64 bytes each, back to back, under stall.
        for (f = 0; f < 8; f = f + 1)
            send_write(f, 32'h1000 + f*64, 16'd64, 8'h40 + f[7:0]*8);

        // A payload that is not a multiple of 4, so the EOF tail flush also
        // has to survive a stall.
        send_write(32'd8, 32'h2000, 16'd6, 8'h50);

        repeat (500) @(posedge clk);

        // ---- every payload byte must be in memory, at the right address ----
        for (f = 0; f < 8; f = f + 1) begin
            base = 32'h1000 + f*64;
            for (i = 0; i < 64; i = i + 1) begin
                expv = (8'h40 + f[7:0]*8) + i[7:0];
                if (!written[base+i]) begin
                    $display("  FAIL frame %0d byte %0d at %0h was NEVER WRITTEN",
                             f, i, base+i);
                    errors = errors + 1;
                end else if (mem[base+i] !== expv) begin
                    $display("  FAIL frame %0d byte %0d at %0h = %02h expected %02h",
                             f, i, base+i, mem[base+i], expv);
                    errors = errors + 1;
                end
            end
        end
        for (i = 0; i < 6; i = i + 1) begin
            expv = 8'h50 + i[7:0];
            if (!written[32'h2000+i] || mem[32'h2000+i] !== expv) begin
                $display("  FAIL tail byte %0d = %02h (written=%b) expected %02h",
                         i, mem[32'h2000+i], written[32'h2000+i], expv);
                errors = errors + 1;
            end
        end

        $display("  writes accepted        = %0d", n_writes);
        $display("  stat_frames            = %0d", stat_frames);
        $display("  stat_wdrop (must be 0) = %0d", stat_wdrop);
        if (stat_wdrop !== 32'd0) begin
            $display("  FAIL the write queue overflowed -- it is too shallow");
            errors = errors + 1;
        end

        if (errors == 0) $display("=== PASSED: 0 error(s) ===");
        else             $display("=== FAILED: %0d error(s) ===", errors);
        $finish;
    end

endmodule
