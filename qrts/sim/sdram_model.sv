// =============================================================================
// sdram_model.sv — behavioural IS42S16320D pair, for testing sdram_ctrl
//
// Checks the protocol as well as storing data. A model that only stores would
// happily accept a controller that never refreshes, reads from an unopened
// row, or violates tRCD -- all of which work in simulation and rot on silicon.
//
// It models ONE bank's open row per bank, CAS-3 read latency, and the refresh
// interval. It is not a timing-accurate model: it checks the ordering rules
// that a controller gets wrong, not the picosecond setup windows that belong
// to static timing analysis.
// =============================================================================
`timescale 1ns/1ps

module sdram_model #(
    parameter integer ROW_W  = 13,
    parameter integer COL_W  = 10,
    parameter integer BANK_W = 2,
    parameter integer DATA_W = 32,
    parameter integer CAS_LAT = 3,
    // Refresh deadline in cycles. The real part needs 8192 rows per 64 ms; the
    // bench shortens it so a short simulation still exercises the logic.
    parameter integer REFRESH_DEADLINE = 2000,
    parameter integer CHECK_REFRESH = 1
)(
    input  wire                  clk,
    input  wire [ROW_W-1:0]      addr,
    input  wire [BANK_W-1:0]     ba,
    input  wire                  cas_n,
    input  wire                  ras_n,
    input  wire                  we_n,
    input  wire                  cs_n,
    input  wire                  cke,
    input  wire [DATA_W/8-1:0]   dqm,
    input  wire [DATA_W-1:0]     dq_in,     // controller -> memory
    input  wire                  dq_oe,
    output wire [DATA_W-1:0]     dq_out     // memory -> controller
);

    // Sparse storage: the address space is 32M words and a bench touches a
    // handful. An associative array keeps the model instant to elaborate.
    reg [DATA_W-1:0] mem [*];

    reg [ROW_W-1:0] open_row  [0:(1<<BANK_W)-1];
    reg             row_valid [0:(1<<BANK_W)-1];

    reg initialised = 1'b0;
    reg mode_set    = 1'b0;

    integer refresh_timer = 0;
    integer errors        = 0;
    integer n_reads       = 0;
    integer n_writes      = 0;

    // CAS pipeline: {valid, address} delayed CAS_LAT cycles.
    reg              rd_v   [0:CAS_LAT];
    reg [24:0]       rd_a   [0:CAS_LAT];
    integer          k;

    wire [3:0] cmd = {cs_n, ras_n, cas_n, we_n};
    localparam [3:0] C_NOP = 4'b0111, C_ACT = 4'b0011, C_RD = 4'b0101,
                     C_WR  = 4'b0100, C_PRE = 4'b0010, C_REF = 4'b0001,
                     C_MRS = 4'b0000;

    initial begin
        for (k = 0; k < (1<<BANK_W); k = k + 1) row_valid[k] = 1'b0;
        for (k = 0; k <= CAS_LAT; k = k + 1) rd_v[k] = 1'b0;
    end

    function [24:0] full_addr(input [BANK_W-1:0] b,
                              input [ROW_W-1:0]  r,
                              input [COL_W-1:0]  c);
        full_addr = {r, b, c};
    endfunction

    always @(posedge clk) begin
        // ---- CAS pipeline advance ------------------------------------------
        for (k = CAS_LAT; k > 0; k = k - 1) begin
            rd_v[k] <= rd_v[k-1];
            rd_a[k] <= rd_a[k-1];
        end
        rd_v[0] <= 1'b0;

        // (dq_out is driven combinationally below, not registered here -- see
        // the note there.)

        // ---- refresh deadline ----------------------------------------------
        if (CHECK_REFRESH && initialised) begin
            refresh_timer <= refresh_timer + 1;
            if (refresh_timer > REFRESH_DEADLINE) begin
                $display("  [model] ERROR: refresh overdue (%0d cycles) -- the array would decay",
                         refresh_timer);
                errors <= errors + 1;
                refresh_timer <= 0;
            end
        end

        if (cke && cmd != C_NOP && cs_n == 1'b0) begin
            case (cmd)
            C_MRS: begin
                if (addr[6:4] != CAS_LAT[2:0]) begin
                    $display("  [model] ERROR: mode register CAS=%0d, model built for %0d",
                             addr[6:4], CAS_LAT);
                    errors <= errors + 1;
                end
                mode_set    <= 1'b1;
                initialised <= 1'b1;
            end

            C_REF: begin
                for (k = 0; k < (1<<BANK_W); k = k + 1)
                    if (row_valid[k]) begin
                        $display("  [model] ERROR: REFRESH with bank %0d still open", k);
                        errors <= errors + 1;
                    end
                refresh_timer <= 0;
            end

            C_ACT: begin
                if (row_valid[ba]) begin
                    $display("  [model] ERROR: ACTIVE on bank %0d, already open", ba);
                    errors <= errors + 1;
                end
                open_row[ba]  <= addr;
                row_valid[ba] <= 1'b1;
            end

            C_PRE: begin
                if (addr[10]) begin
                    for (k = 0; k < (1<<BANK_W); k = k + 1) row_valid[k] <= 1'b0;
                end else begin
                    row_valid[ba] <= 1'b0;
                end
            end

            C_RD: begin
                if (!row_valid[ba]) begin
                    $display("  [model] ERROR: READ from bank %0d with no open row", ba);
                    errors <= errors + 1;
                end
                rd_v[0] <= 1'b1;
                rd_a[0] <= full_addr(ba, open_row[ba], addr[COL_W-1:0]);
                n_reads <= n_reads + 1;
            end

            C_WR: begin
                if (!row_valid[ba]) begin
                    $display("  [model] ERROR: WRITE to bank %0d with no open row", ba);
                    errors <= errors + 1;
                end else begin
                    automatic logic [24:0] wa = full_addr(ba, open_row[ba], addr[COL_W-1:0]);
                    automatic logic [DATA_W-1:0] cur =
                        mem.exists(wa) ? mem[wa] : {DATA_W{1'b0}};
                    for (k = 0; k < DATA_W/8; k = k + 1)
                        if (!dqm[k]) cur[k*8 +: 8] = dq_in[k*8 +: 8];
                    mem[wa]  = cur;
                    n_writes = n_writes + 1;
                end
            end
            default: ;
            endcase
        end
    end

    // Combinational, and from stage CAS_LAT-1 rather than CAS_LAT.
    //
    // The real device drives DQ during the cycle the word is due, so the
    // controller sees it on the same edge it captures. This model's rd_v
    // advances on the same edge as the controller's rd_pipe, so the stage
    // about to retire here is the one the controller is sampling now.
    // Registering this output, or taking it from stage CAS_LAT, puts the bus
    // one cycle behind the capture and every read returns the PREVIOUS word --
    // which presents as memory corruption rather than a latency error. Both
    // mistakes were made writing this, and the bench caught both.
    assign dq_out = rd_v[CAS_LAT-1]
                  ? (mem.exists(rd_a[CAS_LAT-1]) ? mem[rd_a[CAS_LAT-1]]
                                                 : {DATA_W{1'bx}})
                  : {DATA_W{1'bz}};

endmodule
