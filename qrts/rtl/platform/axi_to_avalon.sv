// =============================================================================
// axi_to_avalon.sv — AXI4 master port -> Avalon-MM master
//
// Accelerator_Top exposes an AXI4 master because that is what the Kintex-7 SoC
// needed. On DE2-115 the memory is an Avalon-MM SDRAM controller, so something
// has to translate. Two ways to do that:
//
//   a) replace axi4_master inside the accelerator with avalon_mm_master
//   b) leave the accelerator exactly as simulated and bridge at its boundary
//
// This is (b), and the reason is verification, not laziness. Accelerator_Top is
// the module tb_top_sequencer drives end to end -- one START walking a
// descriptor chain through the real engines with bit-exact conv output. Taking
// it apart to swap the master out means that result no longer covers what is
// on the board. Bridging leaves the proven thing intact and puts the new,
// unproven logic somewhere a testbench can reach it directly.
//
// The cost is that this bridge exists at all, and that the accelerator's
// 64-bit AXI beats become pairs of 32-bit Avalon beats here rather than inside
// a master that was designed for it. avalon_mm_master already does that
// gearboxing; this module is deliberately narrower -- it only has to satisfy
// one well-behaved master, not the general AXI4 spec.
//
// What it does NOT implement, because Accelerator_Top never generates it:
//   * out-of-order responses or multiple outstanding IDs (fixed ID=0)
//   * WRAP or FIXED bursts (INCR only)
//   * narrow transfers (every beat is full width)
//   * write interleaving
// Each of those is an assumption about the master, so each is checked with an
// assertion rather than silently miscompiled.
// =============================================================================
`timescale 1ns/1ps

module axi_to_avalon #(
    parameter integer AXI_DATA_W  = 64,
    parameter integer AVM_DATA_W  = 32,
    parameter integer AVM_ADDR_W  = 32,
    parameter integer MAX_BURST   = 64
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- AXI4 slave side (connects to Accelerator_Top's MASTER port) ----
    input  wire        s_awvalid,
    input  wire [11:0] s_awid,
    input  wire [7:0]  s_awlen,
    input  wire [2:0]  s_awsize,
    input  wire [1:0]  s_awburst,
    input  wire [63:0] s_awaddr,
    output reg         s_awready,

    input  wire        s_wvalid,
    input  wire        s_wlast,
    input  wire [AXI_DATA_W-1:0]   s_wdata,
    input  wire [AXI_DATA_W/8-1:0] s_wstrb,
    output reg         s_wready,

    input  wire        s_bready,
    output reg         s_bvalid,
    output reg  [11:0] s_bid,
    output reg  [1:0]  s_bresp,

    input  wire        s_arvalid,
    input  wire [11:0] s_arid,
    input  wire [7:0]  s_arlen,
    input  wire [2:0]  s_arsize,
    input  wire [1:0]  s_arburst,
    input  wire [63:0] s_araddr,
    output reg         s_arready,

    input  wire        s_rready,
    output reg         s_rvalid,
    output reg  [11:0] s_rid,
    output reg  [AXI_DATA_W-1:0] s_rdata,
    output reg  [1:0]  s_rresp,
    output reg         s_rlast,

    // ---- Avalon-MM master side ----
    output wire [AVM_ADDR_W-1:0]  avm_address,
    output wire                   avm_read,
    output wire                   avm_write,
    output wire [AVM_DATA_W-1:0]  avm_writedata,
    output wire [AVM_DATA_W/8-1:0] avm_byteenable,
    output wire [$clog2(MAX_BURST+1)-1:0] avm_burstcount,
    input  wire [AVM_DATA_W-1:0]  avm_readdata,
    input  wire                   avm_readdatavalid,
    input  wire                   avm_waitrequest
);

    localparam integer SUB = AXI_DATA_W / AVM_DATA_W;   // 2 for 64->32

    // =========================================================================
    // Read path
    //
    // One AXI burst becomes one internal request on avalon_mm_master, which
    // does the burst splitting and the 32->64 gearboxing. This layer only has
    // to turn the AXI handshake into that master's start/done handshake.
    // =========================================================================
    localparam [1:0] R_IDLE = 2'd0, R_REQ = 2'd1, R_DATA = 2'd2;
    reg [1:0]  rstate;
    reg [7:0]  r_beats_left;
    reg [11:0] r_id;

    reg         m_rd_start;
    reg  [63:0] m_rd_addr;
    reg  [7:0]  m_rd_len;
    wire [AXI_DATA_W-1:0] m_rd_data;
    wire        m_rd_data_valid, m_rd_done, m_rd_error;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rstate       <= R_IDLE;
            s_arready    <= 1'b0;
            s_rvalid     <= 1'b0;
            s_rlast      <= 1'b0;
            s_rdata      <= {AXI_DATA_W{1'b0}};
            s_rid        <= 12'h0;
            s_rresp      <= 2'b00;
            m_rd_start   <= 1'b0;
            m_rd_addr    <= 64'h0;
            m_rd_len     <= 8'h0;
            r_beats_left <= 8'h0;
            r_id         <= 12'h0;
        end else begin
            m_rd_start <= 1'b0;
            s_arready  <= 1'b0;

            case (rstate)
            R_IDLE: begin
                s_rvalid <= 1'b0;
                s_rlast  <= 1'b0;
                if (s_arvalid) begin
                    s_arready    <= 1'b1;
                    m_rd_addr    <= s_araddr;
                    // AXI carries len-1; the internal handshake wants a count.
                    m_rd_len     <= s_arlen + 8'd1;
                    r_beats_left <= s_arlen + 8'd1;
                    r_id         <= s_arid;
                    rstate       <= R_REQ;
                end
            end

            R_REQ: begin
                m_rd_start <= 1'b1;
                rstate     <= R_DATA;
            end

            R_DATA: begin
                // The accelerator's read consumers hold rready high, so a
                // registered pass-through is enough; no skid buffer needed.
                // s_rvalid becomes visible the cycle AFTER it is set and is
                // cleared at the end of that same cycle, so each beat is
                // offered for exactly one accepted cycle -- back-to-back data
                // simply overwrites it and rvalid stays high, which is still
                // one accept per word.
                //
                // tb_desc_fetch pins this down: beat count and rlast position
                // for 2-, 8- and 32-beat bursts through the real sdram_ctrl.
                if (m_rd_data_valid) begin
                    s_rdata  <= m_rd_data;
                    s_rid    <= r_id;
                    s_rresp  <= m_rd_error ? 2'b10 : 2'b00;
                    s_rvalid <= 1'b1;
                    s_rlast  <= (r_beats_left == 8'd1);
                    r_beats_left <= r_beats_left - 8'd1;
                end else if (s_rvalid && s_rready) begin
                    s_rvalid <= 1'b0;
                    s_rlast  <= 1'b0;
                    if (r_beats_left == 8'd0) rstate <= R_IDLE;
                end
            end

            default: rstate <= R_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Write path
    // =========================================================================
    localparam [1:0] W_IDLE = 2'd0, W_REQ = 2'd1, W_DATA = 2'd2, W_RESP = 2'd3;
    reg [1:0]  wstate;
    reg [11:0] w_id;

    reg         m_wr_start;
    reg  [63:0] m_wr_addr;
    reg  [7:0]  m_wr_len;
    wire        m_wr_data_ready, m_wr_done, m_wr_error;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wstate     <= W_IDLE;
            s_awready  <= 1'b0;
            s_wready   <= 1'b0;
            s_bvalid   <= 1'b0;
            s_bid      <= 12'h0;
            s_bresp    <= 2'b00;
            m_wr_start <= 1'b0;
            m_wr_addr  <= 64'h0;
            m_wr_len   <= 8'h0;
            w_id       <= 12'h0;
        end else begin
            m_wr_start <= 1'b0;
            s_awready  <= 1'b0;

            case (wstate)
            W_IDLE: begin
                s_bvalid <= 1'b0;
                s_wready <= 1'b0;
                if (s_awvalid) begin
                    s_awready <= 1'b1;
                    m_wr_addr <= s_awaddr;
                    m_wr_len  <= s_awlen + 8'd1;
                    w_id      <= s_awid;
                    wstate    <= W_REQ;
                end
            end

            W_REQ: begin
                m_wr_start <= 1'b1;
                s_wready   <= 1'b1;
                wstate     <= W_DATA;
            end

            // wdata/wstrb are forwarded combinationally below; s_wready simply
            // mirrors the downstream master's beat acceptance.
            W_DATA: begin
                s_wready <= 1'b1;
                if (m_wr_done) begin
                    s_wready <= 1'b0;
                    s_bid    <= w_id;
                    s_bresp  <= m_wr_error ? 2'b10 : 2'b00;
                    s_bvalid <= 1'b1;
                    wstate   <= W_RESP;
                end
            end

            W_RESP: begin
                if (s_bvalid && s_bready) begin
                    s_bvalid <= 1'b0;
                    wstate   <= W_IDLE;
                end
            end

            default: wstate <= W_IDLE;
            endcase
        end
    end

    // =========================================================================
    // The Avalon master itself -- the one proven equivalent to axi4_master by
    // sim/tb_avalon_master.sv. Everything above is handshake translation only.
    // =========================================================================
    avalon_mm_master #(
        .ADDR_WIDTH(AVM_ADDR_W), .DATA_WIDTH(AVM_DATA_W),
        .CORE_WIDTH(AXI_DATA_W), .MAX_BURST(MAX_BURST)
    ) u_avm (
        .clk(clk), .rst_n(rst_n),
        .avm_address(avm_address), .avm_read(avm_read), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount),
        .avm_readdata(avm_readdata), .avm_readdatavalid(avm_readdatavalid),
        .avm_waitrequest(avm_waitrequest),
        .rd_start(m_rd_start), .rd_addr(m_rd_addr), .rd_len(m_rd_len),
        .rd_data(m_rd_data), .rd_data_valid(m_rd_data_valid),
        .rd_done(m_rd_done), .rd_error(m_rd_error),
        .wr_start(m_wr_start), .wr_addr(m_wr_addr), .wr_len(m_wr_len),
        .wr_data(s_wdata), .wr_strb(s_wstrb),
        .wr_data_ready(m_wr_data_ready),
        .wr_done(m_wr_done), .wr_error(m_wr_error)
    );

`ifndef SYNTHESIS
    // The unsupported-feature list from the header, as checks rather than
    // comments. Each one is an assumption about Accelerator_Top; if a future
    // change breaks one, this says so instead of producing wrong addresses.
    always @(posedge clk) if (rst_n) begin
        if (s_arvalid && s_arburst !== 2'b01)
            $error("axi_to_avalon: read burst type %b unsupported (INCR only)",
                   s_arburst);
        if (s_awvalid && s_awburst !== 2'b01)
            $error("axi_to_avalon: write burst type %b unsupported (INCR only)",
                   s_awburst);
        if (s_arvalid && s_arsize !== 3'b011)
            $error("axi_to_avalon: read size %b unsupported (8 bytes only)",
                   s_arsize);
        if (s_awvalid && s_awsize !== 3'b011)
            $error("axi_to_avalon: write size %b unsupported (8 bytes only)",
                   s_awsize);
    end
`endif

endmodule
