// =============================================================================
// dvcon_regs.sv — control registers, Avalon-MM slave -> AXI4-Lite-ish master
//
// The host writes DESC_ADDR / IMG_ADDR / BOX_ADDR / CONF and one START, then
// polls STATUS. On the Kintex-7 build a RISC-V core did that over AXI4. Here
// the Ethernet command engine does it over Avalon, and this module turns those
// accesses into the AXI4 writes Accelerator_Top's slave port expects.
//
// Register map -- deliberately IDENTICAL to yolo_axi4_slave_regs, because
// sw/accel.h and sw/yolo_model.h mirror it and those are the files the host
// application reuses:
//
//   0x00 CTRL       bit0 START (self-clearing), bit1 MODE (0=GEMM, 1=YOLO)
//                   bit2 ENGINE (0=sequencer, 1=microcode)
//   0x04 STATUS     [0]busy [1]done [2]error [7:4]fsm      read-only
//   0x08 SRC_ADDR   GEMM activations
//   0x0C DST_ADDR   GEMM results
//   0x10 IMG_DIM    {K, ARRAY_SIZE}
//   0x14 WEIGHT_ADDR
//   0x18 DESC_ADDR  layer descriptor table base
//   0x1C IMG_ADDR   input bitmap base
//   0x20 BOX_ADDR   output box list base
//   0x24 CONF       [7:0] confidence threshold
//   0x28 NUM_BOXES  read-only
//   0x2C LAYER_IDX  read-only, which descriptor is executing
//   0x30 IDENT      read-only, {8'hDC, ARRAY_SIZE[7:0], build[15:0]}
//
// IDENT is new and is not decoration. ARRAY_SIZE couples the RTL to the
// exporter -- export_yolo26n.py emits weight tiles in the engine's exact loop
// order -- so a model blob built for one array size and run on another reads
// the wrong tile for every conv, silently, with no error flag. The host reads
// IDENT and refuses to run a mismatched blob.
//
// Avalon is word addressed: avs_address counts 32-bit words, so the byte
// offsets above are avs_address*4. Getting that wrong aliases the whole map to
// four times its size, which is the same class of bug as the WSTRB decode that
// cost hours in Stage 3A.
// =============================================================================
`timescale 1ns/1ps

module dvcon_regs #(
    parameter integer ARRAY_SIZE = 16,
    parameter [15:0]  BUILD_ID   = 16'h0001
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- Avalon-MM slave (word addressed) ----
    input  wire [5:0]  avs_address,
    input  wire        avs_read,
    input  wire        avs_write,
    input  wire [31:0] avs_writedata,
    output reg  [31:0] avs_readdata,
    output wire        avs_waitrequest,

    // ---- AXI4 master into Accelerator_Top's slave port ----
    output reg  [11:0] m_awid,
    output reg  [63:0] m_awaddr,
    output reg  [7:0]  m_awlen,
    output wire [2:0]  m_awsize,
    output wire [1:0]  m_awburst,
    output wire        m_awlock,
    output wire [3:0]  m_awcache,
    output wire [2:0]  m_awprot,
    output wire [3:0]  m_awqos,
    output reg         m_awvalid,
    input  wire        m_awready,

    output reg  [63:0] m_wdata,
    output reg  [7:0]  m_wstrb,
    output reg         m_wlast,
    output reg         m_wvalid,
    input  wire        m_wready,

    output reg         m_bready,
    input  wire        m_bvalid,

    output reg  [11:0] m_arid,
    output reg  [63:0] m_araddr,
    output reg  [7:0]  m_arlen,
    output wire [2:0]  m_arsize,
    output wire [1:0]  m_arburst,
    output wire        m_arlock,
    output wire [3:0]  m_arcache,
    output wire [2:0]  m_arprot,
    output wire [3:0]  m_arqos,
    output reg         m_arvalid,
    input  wire        m_arready,

    output reg         m_rready,
    input  wire        m_rvalid,
    input  wire [63:0] m_rdata
);

    // Fixed AXI attributes. Accelerator_Top's slave is a register file: single
    // beats, full width, no bursting.
    assign m_awsize  = 3'b011;      // 8 bytes
    assign m_awburst = 2'b01;       // INCR
    assign m_awlock  = 1'b0;
    assign m_awcache = 4'h0;
    assign m_awprot  = 3'h0;
    assign m_awqos   = 4'h0;
    assign m_arsize  = 3'b011;
    assign m_arburst = 2'b01;
    assign m_arlock  = 1'b0;
    assign m_arcache = 4'h0;
    assign m_arprot  = 3'h0;
    assign m_arqos   = 4'h0;

    // Word address -> byte offset. The AXI slave decodes a 64-bit word address
    // and selects the 32-bit half with WSTRB (see yolo_axi4_slave_regs) -- so a
    // write to an odd word goes to the ALIGNED address with the upper strobe
    // set, not to that address with a full strobe.
    wire [7:0] byte_off = {avs_address, 2'b00};
    wire       upper    = byte_off[2];

    // Which half the IN-FLIGHT transaction wants, latched when the request is
    // accepted.
    //
    // Using the live `upper` in S_R is a real bug: Avalon does not require the
    // requester to hold address valid for the whole transaction, so by the time
    // the AXI read data comes back the address may already be the NEXT one.
    // The half is then chosen from the wrong offset and the read returns the
    // other register of the 64-bit pair -- which looks like a decode error and
    // is a latching error.
    reg upper_lat;

    localparam [2:0] S_IDLE = 3'd0,
                     S_AW   = 3'd1,
                     S_W    = 3'd2,
                     S_B    = 3'd3,
                     S_AR   = 3'd4,
                     S_R    = 3'd5;
    reg [2:0] state;

    // Avalon stalls the requester until the AXI transaction completes. That is
    // the whole reason waitrequest exists, and it keeps this module from
    // needing a queue.
    assign avs_waitrequest = (state != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            m_awvalid   <= 1'b0; m_awaddr <= 64'h0; m_awlen <= 8'h0;
            m_awid      <= 12'h0;
            m_wvalid    <= 1'b0; m_wdata  <= 64'h0; m_wstrb <= 8'h0;
            m_wlast     <= 1'b0;
            m_bready    <= 1'b0;
            m_arvalid   <= 1'b0; m_araddr <= 64'h0; m_arlen <= 8'h0;
            m_arid      <= 12'h0;
            m_rready    <= 1'b0;
            upper_lat   <= 1'b0;
            avs_readdata<= 32'h0;
        end else begin
            case (state)
            S_IDLE: begin
                m_bready <= 1'b0;
                m_rready <= 1'b0;

                // IDENT is answered here rather than forwarded: the
                // accelerator does not have it, and the host needs an answer
                // even when the accelerator is wedged.
                if (avs_read && byte_off == 8'h30) begin
                    // 0xDC = "DVCon", ARRAY_SIZE so the host can refuse a
                    // blob compiled for a different array, then the build id.
                    avs_readdata <= {8'hDC, ARRAY_SIZE[7:0], BUILD_ID};
                end else if (avs_write) begin
                    m_awaddr  <= {56'h0, byte_off & 8'hF8};
                    m_awlen   <= 8'd0;
                    m_awvalid <= 1'b1;
                    // Place the data in the half the offset selects and strobe
                    // only that half.
                    m_wdata   <= upper ? {avs_writedata, 32'h0}
                                       : {32'h0, avs_writedata};
                    m_wstrb   <= upper ? 8'hF0 : 8'h0F;
                    m_wlast   <= 1'b1;
                    state     <= S_AW;
                end else if (avs_read) begin
                    upper_lat <= upper;
                    m_araddr  <= {56'h0, byte_off & 8'hF8};
                    m_arlen   <= 8'd0;
                    m_arvalid <= 1'b1;
                    state     <= S_AR;
                end
            end

            // The AXI slave accepts AW and W in SEPARATE states, so they
            // cannot be offered together -- awready and wready are never high
            // in the same cycle.
            S_AW: begin
                if (m_awvalid && m_awready) begin
                    m_awvalid <= 1'b0;
                    m_wvalid  <= 1'b1;
                    state     <= S_W;
                end
            end

            S_W: begin
                if (m_wvalid && m_wready) begin
                    m_wvalid <= 1'b0;
                    m_wlast  <= 1'b0;
                    m_bready <= 1'b1;
                    state    <= S_B;
                end
            end

            S_B: begin
                if (m_bvalid && m_bready) begin
                    m_bready <= 1'b0;
                    state    <= S_IDLE;
                end
            end

            S_AR: begin
                if (m_arvalid && m_arready) begin
                    m_arvalid <= 1'b0;
                    m_rready  <= 1'b1;
                    state     <= S_R;
                end
            end

            S_R: begin
                if (m_rvalid && m_rready) begin
                    avs_readdata <= upper_lat ? m_rdata[63:32]
                                              : m_rdata[31:0];
                    m_rready     <= 1'b0;
                    state        <= S_IDLE;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
