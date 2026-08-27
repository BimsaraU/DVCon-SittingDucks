// =============================================================================
// sdram_ctrl.sv — Avalon-MM slave for the DE2-115's 128 MB SDRAM
//
// Quartus 25.1std ships NO SDRAM controller. Every one of them --
// altera_avalon_new_sdram_controller, altera_sdram_tri_controller, and the
// three altmemphy DDR variants -- is a 272-byte tombstone under
// quartus/common/misc/outdated_ip/ pointing at a release note. The University
// Program IP has SRAM, SSRAM, flash and SD card, but no SDRAM. So this is
// written against the IS42S16320D datasheet rather than instantiated.
//
// BOARD GEOMETRY
// Two IS42S16320D-6 in parallel: each is 8M x 16 (13-bit row, 10-bit column,
// 4 banks), wired to share address, bank and control, giving a 32-bit bus and
// 128 MB total. Both chips always see the same command; only the DQ and DQM
// lines are split.
//
//   avalon word address -> { row[12:0], bank[1:0], col[9:0] }
//
// Bank in the MIDDLE, not the top: consecutive words walk the column address,
// and only when a 1024-word column wraps does the bank change. That keeps a
// burst inside one open row, which is the whole point -- a row activate costs
// ~20 ns and the accelerator streams weights linearly.
//
// TIMING (-6 speed grade at 100 MHz, 10 ns/cycle, rounded UP)
//   tRC  60 ns  activate-to-activate, same bank      6 cycles
//   tRCD 18 ns  activate-to-read/write               2 cycles
//   tRP  18 ns  precharge-to-activate                2 cycles
//   tRFC 60 ns  refresh-to-activate                  6 cycles
//   tREFI 7.8us average refresh interval           780 cycles
//   CAS latency 3
//
// REFRESH IS NOT OPTIONAL. Miss the 7.8 us average and the array decays; the
// symptom is bit rot under load, which looks exactly like a datapath bug and
// is far more expensive to chase. The refresh request here PREEMPTS a waiting
// transaction and is sticky, so it cannot be starved by back-to-back bursts.
//
// Burst length 1 in the mode register, with one explicit command per word:
// slower per word than an SDRAM page burst, but it makes the Avalon side
// trivially correct and lets byte enables work per word. Sequential streaming
// -- the access pattern that matters here -- stays inside an open row either
// way, so the row-activate cost is amortised the same.
// =============================================================================
`timescale 1ns/1ps

module sdram_ctrl #(
    parameter integer ROW_W   = 13,
    parameter integer COL_W   = 10,
    parameter integer BANK_W  = 2,
    parameter integer DATA_W  = 32,
    parameter integer MAX_BURST = 64,

    // Cycle counts at 100 MHz. Parameterised so a bench can shrink the
    // power-up wait from 20,000 cycles to something a simulation can afford.
    parameter integer T_INIT  = 20000,  // 200 us power-up
    parameter integer T_RC    = 6,
    parameter integer T_RCD   = 2,
    parameter integer T_RP    = 2,
    parameter integer T_RFC   = 6,
    parameter integer T_MRD   = 2,
    parameter integer T_REFI  = 780,
    parameter integer CAS_LAT = 3
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ---- Avalon-MM slave ----------------------------------------------------
    input  wire [24:0]              avs_address,     // 32M words = 128 MB
    input  wire                     avs_read,
    input  wire                     avs_write,
    input  wire [DATA_W-1:0]        avs_writedata,
    input  wire [DATA_W/8-1:0]      avs_byteenable,
    input  wire [$clog2(MAX_BURST+1)-1:0] avs_burstcount,
    output reg  [DATA_W-1:0]        avs_readdata,
    output reg                      avs_readdatavalid,
    output wire                     avs_waitrequest,

    // ---- SDRAM pins ---------------------------------------------------------
    // DQ is split into in/out/oe rather than an inout so this module stays
    // synthesisable and testable without a tristate in the middle of it; the
    // top level owns the actual bidirectional pin.
    output reg  [ROW_W-1:0]         dram_addr,
    output reg  [BANK_W-1:0]        dram_ba,
    output wire                     dram_cas_n,
    output wire                     dram_ras_n,
    output wire                     dram_we_n,
    output wire                     dram_cs_n,
    output reg                      dram_cke,
    output reg  [DATA_W/8-1:0]      dram_dqm,
    input  wire [DATA_W-1:0]        dram_dq_in,
    output reg  [DATA_W-1:0]        dram_dq_out,
    output reg                      dram_dq_oe
);

    // ---- command encoding {cs_n, ras_n, cas_n, we_n} ------------------------
    localparam [3:0] CMD_INHIBIT   = 4'b1111,
                     CMD_NOP       = 4'b0111,
                     CMD_ACTIVE    = 4'b0011,
                     CMD_READ      = 4'b0101,
                     CMD_WRITE     = 4'b0100,
                     CMD_PRECHARGE = 4'b0010,
                     CMD_REFRESH   = 4'b0001,
                     CMD_LOAD_MODE = 4'b0000;

    // Burst length 1, sequential, CAS 3, single-location write.
    localparam [12:0] MODE_REG = {3'b000, 1'b0, 2'b00, 3'd3, 1'b0, 3'b000};

    localparam [3:0]
        S_INIT_WAIT = 4'd0,  S_INIT_PRE  = 4'd1,  S_INIT_REF1 = 4'd2,
        S_INIT_REF2 = 4'd3,  S_INIT_MODE = 4'd4,  S_IDLE      = 4'd5,
        S_ACTIVE    = 4'd6,  S_RD        = 4'd7,  S_RD_WAIT   = 4'd8,
        S_WR        = 4'd9,  S_PRE       = 4'd10, S_REFRESH   = 4'd11;

    reg [3:0]  state;
    reg [14:0] wait_cnt;      // wide enough for T_INIT
    reg [3:0]  cmd;

    // ---- refresh ------------------------------------------------------------
    reg [9:0] refi_cnt;
    reg       refresh_req;

    // ---- address split ------------------------------------------------------
    wire [ROW_W-1:0]  a_row  = avs_address[COL_W+BANK_W +: ROW_W];
    wire [BANK_W-1:0] a_bank = avs_address[COL_W +: BANK_W];
    wire [COL_W-1:0]  a_col  = avs_address[0 +: COL_W];

    // Latched request. Avalon presents address and burstcount with the FIRST
    // beat only, so the controller owns them for the rest of the burst.
    reg [ROW_W-1:0]  r_row;
    reg [BANK_W-1:0] r_bank;
    reg [COL_W-1:0]  r_col;
    reg              r_is_write;
    reg [$clog2(MAX_BURST+1)-1:0] r_left;

    // One-beat holding register for write data. Avalon hands over a beat in
    // the cycle waitrequest is low, but the WRITE command for it is issued a
    // state later, so the beat has to be held rather than driven straight at
    // the pins. Without it the burst is issued one beat off: the first word is
    // dropped and the last written twice, which the bench sees as every read
    // shifted by one.
    reg [DATA_W-1:0]   w_data;
    reg [DATA_W/8-1:0] w_be;

    // Accept a request only when parked in S_IDLE with no refresh pending, and
    // during the data phase of a write burst.
    wire idle_ready = (state == S_IDLE) && !refresh_req && (wait_cnt == 0);
    assign avs_waitrequest = !(idle_ready || (state == S_WR && wait_cnt == 0));

    // ---- CAS-latency read pipeline ------------------------------------------
    // Data returns CAS_LAT cycles after the READ command. This shift register
    // marks which cycles carry valid data; getting the depth wrong shifts every
    // read by a word, which presents as memory corruption rather than a
    // latency bug.
    reg [CAS_LAT:0] rd_pipe;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            refi_cnt    <= '0;
            refresh_req <= 1'b0;
        end else begin
            if (refi_cnt >= T_REFI[9:0]) begin
                refi_cnt    <= '0;
                refresh_req <= 1'b1;          // sticky until serviced
            end else begin
                refi_cnt <= refi_cnt + 1'b1;
            end
            if (state == S_REFRESH) refresh_req <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_INIT_WAIT;
            wait_cnt   <= '0;
            cmd        <= CMD_INHIBIT;
            dram_cke   <= 1'b0;
            dram_dq_oe <= 1'b0;
            dram_dqm   <= '1;
            dram_addr  <= '0;
            dram_ba    <= '0;
            rd_pipe    <= '0;
            r_left     <= '0;
            r_row      <= '0;
            r_bank     <= '0;
            r_col      <= '0;
            r_is_write <= 1'b0;
            w_data     <= '0;
            w_be       <= '0;
            avs_readdatavalid <= 1'b0;
            avs_readdata      <= '0;
        end else begin
            cmd        <= CMD_NOP;
            dram_cke   <= 1'b1;
            dram_dq_oe <= 1'b0;
            dram_dqm   <= '0;

            // Read return path, independent of the command FSM below.
            //
            // rd_pipe[CAS_LAT] is high during the cycle the device is driving
            // the word on DQ, so data and flag are captured on the same edge
            // and present together on the Avalon side, as required. Sampling
            // either one an edge off returns the neighbouring word, which
            // presents as memory corruption rather than a latency bug -- the
            // bench catches it by reading back a known pattern.
            rd_pipe           <= {rd_pipe[CAS_LAT-1:0], 1'b0};
            avs_readdatavalid <= rd_pipe[CAS_LAT];
            if (rd_pipe[CAS_LAT]) avs_readdata <= dram_dq_in;

            if (wait_cnt != 0) begin
                wait_cnt <= wait_cnt - 1'b1;
            end else begin
                case (state)
                // ---- power-up sequence --------------------------------------
                S_INIT_WAIT: begin
                    dram_cke <= 1'b0;
                    cmd      <= CMD_INHIBIT;
                    wait_cnt <= T_INIT[14:0];
                    state    <= S_INIT_PRE;
                end
                S_INIT_PRE: begin
                    cmd       <= CMD_PRECHARGE;
                    dram_addr <= 13'h400;          // A10 high = all banks
                    wait_cnt  <= T_RP[14:0];
                    state     <= S_INIT_REF1;
                end
                S_INIT_REF1: begin
                    cmd      <= CMD_REFRESH;
                    wait_cnt <= T_RFC[14:0];
                    state    <= S_INIT_REF2;
                end
                S_INIT_REF2: begin
                    cmd      <= CMD_REFRESH;
                    wait_cnt <= T_RFC[14:0];
                    state    <= S_INIT_MODE;
                end
                S_INIT_MODE: begin
                    cmd       <= CMD_LOAD_MODE;
                    dram_addr <= MODE_REG;
                    dram_ba   <= '0;
                    wait_cnt  <= T_MRD[14:0];
                    state     <= S_IDLE;
                end

                // ---- idle ---------------------------------------------------
                S_IDLE: begin
                    // Refresh wins over a new transaction, and is sticky, so a
                    // stream of bursts cannot starve it.
                    if (refresh_req) begin
                        cmd       <= CMD_PRECHARGE;
                        dram_addr <= 13'h400;
                        wait_cnt  <= T_RP[14:0];
                        state     <= S_REFRESH;
                    end else if (avs_read || avs_write) begin
                        r_row      <= a_row;
                        r_bank     <= a_bank;
                        r_col      <= a_col;
                        r_is_write <= avs_write;
                        r_left     <= avs_burstcount;

                        cmd       <= CMD_ACTIVE;
                        dram_addr <= a_row;
                        dram_ba   <= a_bank;
                        wait_cnt  <= T_RCD[14:0];
                        state     <= S_ACTIVE;

                        // A write's first data beat is accepted in the same
                        // cycle as the request, because waitrequest was low.
                        // Hold it; the WRITE command that carries it is issued
                        // in S_WR.
                        if (avs_write) begin
                            w_data <= avs_writedata;
                            w_be   <= avs_byteenable;
                        end
                    end
                end

                S_REFRESH: begin
                    cmd      <= CMD_REFRESH;
                    wait_cnt <= T_RFC[14:0];
                    state    <= S_IDLE;
                end

                S_ACTIVE: state <= r_is_write ? S_WR : S_RD;

                // ---- read burst ---------------------------------------------
                S_RD: begin
                    cmd        <= CMD_READ;
                    dram_addr  <= {3'b000, r_col};  // A10 low: no auto precharge
                    dram_ba    <= r_bank;
                    rd_pipe[0] <= 1'b1;
                    r_col      <= r_col + 1'b1;
                    if (r_left <= 1) state  <= S_RD_WAIT;
                    else             r_left <= r_left - 1'b1;
                end
                S_RD_WAIT: begin
                    // Hold until the last CAS latency has drained; precharging
                    // early would cut the final word off.
                    if (rd_pipe == 0) begin
                        cmd       <= CMD_PRECHARGE;
                        dram_addr <= 13'h400;
                        wait_cnt  <= T_RP[14:0];
                        state     <= S_IDLE;
                    end
                end

                // ---- write burst --------------------------------------------
                // Issue the HELD beat -- the one latched in S_IDLE on the first
                // pass, or here on each later pass -- and accept the next in
                // the same cycle, since waitrequest is low throughout S_WR.
                //
                // dram_dqm is driven from the held byte enables rather than
                // from avs_byteenable directly: the top of this block clears
                // dram_dqm every cycle, so reading the live Avalon signal here
                // dropped the enables of the beat actually being written and
                // turned a partial write into a full one.
                S_WR: begin
                    cmd         <= CMD_WRITE;
                    dram_addr   <= {3'b000, r_col};
                    dram_ba     <= r_bank;
                    dram_dq_out <= w_data;
                    dram_dqm    <= ~w_be;
                    dram_dq_oe  <= 1'b1;
                    r_col       <= r_col + 1'b1;

                    if (r_left <= 1) begin
                        state <= S_PRE;
                    end else begin
                        r_left <= r_left - 1'b1;
                        w_data <= avs_writedata;
                        w_be   <= avs_byteenable;
                    end
                end
                S_PRE: begin
                    cmd       <= CMD_PRECHARGE;
                    dram_addr <= 13'h400;
                    wait_cnt  <= T_RP[14:0];
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
                endcase
            end
        end
    end

    assign {dram_cs_n, dram_ras_n, dram_cas_n, dram_we_n} = cmd;

endmodule
