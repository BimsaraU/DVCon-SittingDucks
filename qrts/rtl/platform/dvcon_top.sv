// =============================================================================
// dvcon_top.sv — DE2-115 top level
//
// Replaces Accelerator_Top's ROLE as the chip top, not Accelerator_Top itself:
// the accelerator is instantiated here unchanged, with shims either side.
//
//     ENET1 --- mii_rx/tx --- eth_mac_rx/tx --- eth_cmd_engine ---+
//                                                                  |
//     JTAG --- jtag_ctrl --- dvcon_regs ---AXI--> Accelerator_Top   |
//                                                        |          |
//                                                  axi_to_avalon    |
//                                                        |          |
//                                                  avalon_arbiter <-+
//                                                        |
//                                                   sdram_ctrl --> 128 MB
//
// Control is over JTAG; bulk data (the model blob and each image) is over
// Ethernet. Splitting them that way means a dropped Ethernet frame can never
// start a frame or corrupt a register, and the JTAG cable used for programming
// is the same one that drives the accelerator.
//
// Why the accelerator is instantiated whole rather than rebuilt from its
// engines: Accelerator_Top is the module tb_top_sequencer drives end to end --
// one START walking a descriptor chain through the real engines with bit-exact
// conv output. Rebuilding the hierarchy here would mean that result no longer
// describes what is on the board. The shims are new, so they are the parts
// that carry risk, and they are small enough to test on their own.
//
// ---------------------------------------------------------------------------
// STATUS
// ---------------------------------------------------------------------------
// Every block here is real -- no stand-ins. Each was verified on its own
// bench before being wired up (tb_sdram, tb_jtag_ctrl, tb_mii_adapters,
// tb_eth_cmd, tb_arbiter), and the accelerator by tb_top_sequencer.
//
// Not yet done: timing has not been read, and there is no PLL -- the design
// runs directly off the 50 MHz oscillator, so the SDRAM controller's timing
// constants (written for 100 MHz) are conservative rather than wrong.
// =============================================================================
`timescale 1ns/1ps

module dvcon_top #(
    // 16, not the 24 the Xilinx build used. Each PE is a signed 8x8 multiply,
    // two per 18x18 block: 24x24 = 576 MACs = 288 blocks against the 266 this
    // part has. Measured at 16: 256 multiplier elements, 25,509 LE.
    //
    // NOT a hardware-only knob -- export_yolo26n.py emits weight tiles in the
    // engine's exact loop order, so the model must be re-exported for 16 or
    // every conv silently reads the wrong tile. IDENT carries this value so the
    // host can refuse a mismatched blob.
    parameter integer ARRAY_SIZE = 16,
    parameter [47:0]  MAC_ADDR   = 48'h02_00_00_C0_FF_EE
)(
    // ---- clocks and reset ----
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,

    // ---- SDRAM (IS42S16320D x2, 128 MB, 32-bit) ----
    output wire [12:0] DRAM_ADDR,
    output wire [1:0]  DRAM_BA,
    output wire        DRAM_CAS_N,
    output wire        DRAM_RAS_N,
    output wire        DRAM_WE_N,
    output wire        DRAM_CS_N,
    output wire        DRAM_CKE,
    output wire        DRAM_CLK,
    inout  wire [31:0] DRAM_DQ,
    output wire [3:0]  DRAM_DQM,

    // ---- Ethernet PHY 1 (Marvell 88E1111, connector J5), MII ----
    // ENET1, not ENET0. The 88E1111's interface mode is strapped by a jumper
    // at power-on -- JP1 for PHY 0, JP2 for PHY 1, short 2-3 for MII. JP1 was
    // not reachable on this board, so JP2 was moved and the cable went into
    // J5. Both PHYs are the same part on the same nets; only the pin
    // assignments differ (see pin/de2_115_pins.tcl).
    output wire        ENET1_GTX_CLK,
    input  wire        ENET1_TX_CLK,
    input  wire        ENET1_RX_CLK,
    input  wire [3:0]  ENET1_RX_DATA,
    input  wire        ENET1_RX_DV,
    output wire [3:0]  ENET1_TX_DATA,
    output wire        ENET1_TX_EN,
    output wire        ENET1_RST_N,
    inout  wire        ENET1_MDIO,
    output wire        ENET1_MDC,

    // ---- status ----
    output wire [17:0] LEDR,
    output wire [8:0]  LEDG
);

    // =========================================================================
    // Clocking and reset
    //
    // The core runs directly off the 50 MHz oscillator. The PLL (100 MHz core,
    // 100 MHz -3ns to the SDRAM pins, 125 MHz + 125 MHz@90 for RGMII) is a
    // Quartus megafunction that has to be generated from the IP catalogue for
    // this device; running at 50 MHz until then is legal, slower, and lets the
    // design fit and be timed.
    // =========================================================================
    wire clk_sys = CLOCK_50;

    // KEY is a mechanical button: assert reset asynchronously, RELEASE it
    // synchronously. An asynchronous release that violates recovery time on
    // one flop and not another starts the design in an inconsistent state.
    reg [3:0] rst_sync;
    always @(posedge clk_sys or negedge KEY[0]) begin
        if (!KEY[0]) rst_sync <= 4'b0000;
        else         rst_sync <= {rst_sync[2:0], 1'b1};
    end
    wire rst_n = rst_sync[3];

    // =========================================================================
    // Ethernet: MII front end, MAC, command engine
    //
    //   ENET1 pins --4-bit--> mii_rx_adapter --byte--> eth_mac_rx
    //                                                       |
    //                                              eth_cmd_engine --Avalon--> SDRAM
    //                                                       |
    //   ENET1 pins <--4-bit-- mii_tx_adapter <--byte-- eth_mac_tx
    //
    // MII, not RGMII. 100 Mbit is enough here -- the largest transfer is the
    // 2.8 MB model blob, about a quarter of a second, once at startup.
    //
    // The transmit clock comes FROM the PHY on ENET1_TX_CLK (PIN_C22), which
    // the DE2-115 manual lists as "MII transmit clock". This used to drive
    // ENET1_GTX_CLK (PIN_C23) with a locally divided 25 MHz instead -- but the
    // manual calls that the "GMII transmit clock", used only in GMII/RGMII. In
    // MII the 88E1111 samples TXD against the clock it generates, so the board
    // could never transmit, and the MII clock was not even a port.
    //
    // NOTE: the PHY only sources this clock in MII mode, which for PHY 1 is
    // JP2 shorted 2-3. The board ships in RGMII (JP2 1-2), and there TX_CLK is
    // silent and nothing is sent. Moving the jumper needs a hardware reset.
    // =========================================================================

    wire [7:0] rx_byte;
    wire       rx_valid;
    wire       rx_last;

    mii_rx_adapter u_mii_rx (
        .rx_clk(ENET1_RX_CLK), .rst_n(rst_n),
        .mii_rxd(ENET1_RX_DATA), .mii_rx_dv(ENET1_RX_DV),
        .clk(clk_sys),
        .out_data(rx_byte), .out_valid(rx_valid), .out_last(rx_last)
    );

    wire [7:0]  mac_out_data;
    wire        mac_out_valid, mac_out_sof, mac_out_eof, mac_out_err;
    wire [31:0] stat_good, stat_bad_fcs, stat_filtered;

    eth_mac_rx #(.MAC_ADDR(MAC_ADDR)) u_mac_rx (
        .clk(clk_sys), .rst_n(rst_n),
        .rx_data(rx_byte), .rx_valid(rx_valid), .rx_last(rx_last),
        .out_data(mac_out_data), .out_valid(mac_out_valid),
        .out_sof(mac_out_sof), .out_eof(mac_out_eof), .out_err(mac_out_err),
        .stat_good(stat_good), .stat_bad_fcs(stat_bad_fcs),
        .stat_filtered(stat_filtered)
    );

    // ---- command engine: parses frames, DMAs payloads into SDRAM ----
    wire [7:0]  cmd_tx_data;
    wire        cmd_tx_valid, cmd_tx_last, cmd_tx_ready;

    wire [31:0] eth_avm_address, eth_avm_writedata;
    wire        eth_avm_write;
    wire [3:0]  eth_avm_byteenable;
    wire [6:0]  eth_avm_burstcount;
    wire        eth_avm_waitrequest;

    wire [31:0] stat_frames, stat_bytes, stat_wdrop;
    wire [63:0] rx_bitmap;

    eth_cmd_engine #(.MAC_ADDR(MAC_ADDR)) u_cmd (
        .clk(clk_sys), .rst_n(rst_n),
        .rx_data(mac_out_data), .rx_valid(mac_out_valid),
        .rx_sof(mac_out_sof), .rx_eof(mac_out_eof), .rx_err(mac_out_err),
        .tx_data(cmd_tx_data), .tx_valid(cmd_tx_valid),
        .tx_last(cmd_tx_last), .tx_ready(cmd_tx_ready),
        .avm_address(eth_avm_address), .avm_write(eth_avm_write),
        .avm_writedata(eth_avm_writedata),
        .avm_byteenable(eth_avm_byteenable),
        .avm_burstcount(eth_avm_burstcount),
        .avm_waitrequest(eth_avm_waitrequest),
        .stat_frames(stat_frames), .stat_bytes(stat_bytes),
        .stat_wdrop(stat_wdrop),
        .rx_bitmap(rx_bitmap)
    );

    wire [7:0] tx_byte;
    wire       tx_en, tx_busy;

    eth_mac_tx u_mac_tx (
        .clk(clk_sys), .rst_n(rst_n),
        .tx_data(cmd_tx_data), .tx_valid(cmd_tx_valid),
        .tx_last(cmd_tx_last), .tx_ready(cmd_tx_ready),
        .phy_data(tx_byte), .phy_en(tx_en), .busy(tx_busy)
    );

    mii_tx_adapter u_mii_tx (
        .clk(clk_sys), .rst_n(rst_n),
        .in_data(tx_byte), .in_valid(tx_en), .in_ready(),
        .tx_clk(ENET1_TX_CLK),
        .mii_txd(ENET1_TX_DATA), .mii_tx_en(ENET1_TX_EN)
    );

    // Unused in MII: the PHY drives TXD off its own ENET1_TX_CLK. Held low
    // rather than left floating, and rather than driven with a 25 MHz clock
    // the PHY ignores in this mode.
    assign ENET1_GTX_CLK = 1'b0;

    // PHY out of reset. MDIO is left idle: the 88E1111 powers up in
    // auto-negotiation with its strap defaults, which is what is wanted here --
    // MDIO would only be needed to force a mode or read link status.
    assign ENET1_RST_N   = rst_n;
    assign ENET1_MDC     = 1'b0;
    assign ENET1_MDIO    = 1'bz;

    // =========================================================================
    // Control registers
    //
    // eth_cmd_engine will drive this Avalon slave. Until it exists the port is
    // held idle -- no reads, no writes -- so the accelerator sits in reset with
    // its registers at their defaults.
    // =========================================================================
    wire [11:0] rg_awid;  wire [63:0] rg_awaddr; wire [7:0] rg_awlen;
    wire [2:0]  rg_awsize; wire [1:0] rg_awburst; wire rg_awlock;
    wire [3:0]  rg_awcache, rg_awqos; wire [2:0] rg_awprot;
    wire        rg_awvalid; wire rg_awready;
    wire [63:0] rg_wdata; wire [7:0] rg_wstrb; wire rg_wlast, rg_wvalid;
    wire        rg_wready;
    wire        rg_bready, rg_bvalid;
    wire [11:0] rg_arid; wire [63:0] rg_araddr; wire [7:0] rg_arlen;
    wire [2:0]  rg_arsize; wire [1:0] rg_arburst; wire rg_arlock;
    wire [3:0]  rg_arcache, rg_arqos; wire [2:0] rg_arprot;
    wire        rg_arvalid; wire rg_arready;
    wire        rg_rready, rg_rvalid; wire [63:0] rg_rdata;

    wire [31:0] regs_readdata;
    wire        regs_waitrequest;

    // The host drives this register block over JTAG. jtag_ctrl replaced a boot
    // sequencer that hardcoded four addresses and a START after reset; that
    // existed only so the accelerator had a reason to run and would not be
    // deleted by constant propagation.
    wire [5:0]  jt_address;
    wire        jt_read, jt_write;
    wire [31:0] jt_writedata;

    wire [31:0] jt_mem_address;
    wire        jt_mem_read;
    wire        jt_mem_write;
    wire [31:0] jt_mem_writedata;
    wire [3:0]  jt_mem_byteenable;
    wire [31:0] jt_mem_readdata;
    wire        jt_mem_readdatavalid, jt_mem_waitrequest;

    // Driven by the accelerator (instantiated below), read by jtag_ctrl
    // here -- so the declaration has to precede the first consumer.
    wire [31:0] ac_dbg_ce_src, ac_dbg_ce_dst;
    wire [3:0]  dbg_desc_sel;
    wire [1:0]  sdram_cap_sel;
    wire [31:0] ac_dbg_conv0, ac_dbg_conv1, ac_dbg_elem, ac_dbg_arb;
    wire [31:0] ac_dbg_desc_val;

    jtag_ctrl u_jtag (
        .clk(clk_sys), .rst_n(rst_n),
        .avm_address(jt_address), .avm_read(jt_read), .avm_write(jt_write),
        .avm_writedata(jt_writedata),
        .avm_readdata(regs_readdata), .avm_waitrequest(regs_waitrequest),
        .mem_address(jt_mem_address), .mem_read(jt_mem_read),
        .mem_write(jt_mem_write), .mem_writedata(jt_mem_writedata),
        .mem_byteenable(jt_mem_byteenable),
        .mem_readdata(jt_mem_readdata),
        .mem_readdatavalid(jt_mem_readdatavalid),
        .mem_waitrequest(jt_mem_waitrequest),
        // Link diagnostics, readable over JTAG at 0x3A..0x3D.
        .eth_good(stat_good), .eth_bad(stat_bad_fcs),
        .eth_filtered(stat_filtered),
        .dbg_ce_src(ac_dbg_ce_src), .dbg_ce_dst(ac_dbg_ce_dst),
        .eth_frames(stat_frames), .eth_bitmap(rx_bitmap),
        .eth_wdrop(stat_wdrop),
        .dbg_desc_sel(dbg_desc_sel), .dbg_desc_val(ac_dbg_desc_val),
        .dbg_conv0(ac_dbg_conv0), .dbg_conv1(ac_dbg_conv1),
        .dbg_elem(ac_dbg_elem), .dbg_arb(ac_dbg_arb),
        .sdram_cap_sel(sdram_cap_sel)
    );

    dvcon_regs #(.ARRAY_SIZE(ARRAY_SIZE)) u_regs (
        .clk(clk_sys), .rst_n(rst_n),
        .avs_address(jt_address), .avs_read(jt_read), .avs_write(jt_write),
        .avs_writedata(jt_writedata),
        .avs_readdata(regs_readdata), .avs_waitrequest(regs_waitrequest),
        .m_awid(rg_awid), .m_awaddr(rg_awaddr), .m_awlen(rg_awlen),
        .m_awsize(rg_awsize), .m_awburst(rg_awburst), .m_awlock(rg_awlock),
        .m_awcache(rg_awcache), .m_awprot(rg_awprot), .m_awqos(rg_awqos),
        .m_awvalid(rg_awvalid), .m_awready(rg_awready),
        .m_wdata(rg_wdata), .m_wstrb(rg_wstrb), .m_wlast(rg_wlast),
        .m_wvalid(rg_wvalid), .m_wready(rg_wready),
        .m_bready(rg_bready), .m_bvalid(rg_bvalid),
        .m_arid(rg_arid), .m_araddr(rg_araddr), .m_arlen(rg_arlen),
        .m_arsize(rg_arsize), .m_arburst(rg_arburst), .m_arlock(rg_arlock),
        .m_arcache(rg_arcache), .m_arprot(rg_arprot), .m_arqos(rg_arqos),
        .m_arvalid(rg_arvalid), .m_arready(rg_arready),
        .m_rready(rg_rready), .m_rvalid(rg_rvalid), .m_rdata(rg_rdata)
    );

    // =========================================================================
    // The accelerator, unchanged
    // =========================================================================
    wire        ac_awvalid; wire [11:0] ac_awid; wire [7:0] ac_awlen;
    wire [2:0]  ac_awsize; wire [1:0] ac_awburst; wire [0:0] ac_awlock;
    wire [3:0]  ac_awcache, ac_awqos; wire [63:0] ac_awaddr;
    wire [2:0]  ac_awprot; wire ac_awready;

    wire        ac_wvalid, ac_wlast; wire [63:0] ac_wdata; wire [7:0] ac_wstrb;
    wire        ac_wready;

    wire        ac_bready, ac_bvalid; wire [11:0] ac_bid; wire [1:0] ac_bresp;

    wire        ac_arvalid; wire [11:0] ac_arid; wire [7:0] ac_arlen;
    wire [2:0]  ac_arsize; wire [1:0] ac_arburst; wire [0:0] ac_arlock;
    wire [3:0]  ac_arcache, ac_arqos; wire [63:0] ac_araddr;
    wire [2:0]  ac_arprot; wire ac_arready;

    wire        ac_rready, ac_rvalid; wire [11:0] ac_rid;
    wire [63:0] ac_rdata; wire [1:0] ac_rresp; wire ac_rlast;

    wire [7:0]  ac_dbg_engine;
    wire [15:0] ac_dbg_layer;
    wire        ac_dbg_busy;

    Accelerator_Top #(.ARRAY_SIZE(ARRAY_SIZE)) u_accel (
        .s_axi_aclk(clk_sys), .s_axi_aresetn(rst_n),
        .dbg_engine(ac_dbg_engine), .dbg_layer(ac_dbg_layer),
        .dbg_busy(ac_dbg_busy),
        .dbg_ce_src(ac_dbg_ce_src), .dbg_ce_dst(ac_dbg_ce_dst),
        .dbg_desc_sel(dbg_desc_sel), .dbg_desc_val(ac_dbg_desc_val),
        .dbg_conv0(ac_dbg_conv0), .dbg_conv1(ac_dbg_conv1),
        .dbg_elem(ac_dbg_elem), .dbg_arb(ac_dbg_arb),

        .m_axi_awvalid(ac_awvalid), .m_axi_awid(ac_awid),
        .m_axi_awlen(ac_awlen), .m_axi_awsize(ac_awsize),
        .m_axi_awburst(ac_awburst), .m_axi_awlock(ac_awlock),
        .m_axi_awcache(ac_awcache), .m_axi_awqos(ac_awqos),
        .m_axi_awaddr(ac_awaddr), .m_axi_awprot(ac_awprot),
        .m_axi_awready(ac_awready),
        .m_axi_wvalid(ac_wvalid), .m_axi_wlast(ac_wlast),
        .m_axi_wdata(ac_wdata), .m_axi_wstrb(ac_wstrb),
        .m_axi_wready(ac_wready),
        .m_axi_bready(ac_bready), .m_axi_bvalid(ac_bvalid),
        .m_axi_bid(ac_bid), .m_axi_bresp(ac_bresp),
        .m_axi_arvalid(ac_arvalid), .m_axi_arid(ac_arid),
        .m_axi_arlen(ac_arlen), .m_axi_arsize(ac_arsize),
        .m_axi_arburst(ac_arburst), .m_axi_arlock(ac_arlock),
        .m_axi_arcache(ac_arcache), .m_axi_arqos(ac_arqos),
        .m_axi_araddr(ac_araddr), .m_axi_arprot(ac_arprot),
        .m_axi_arready(ac_arready),
        .m_axi_rready(ac_rready), .m_axi_rvalid(ac_rvalid),
        .m_axi_rid(ac_rid), .m_axi_rdata(ac_rdata),
        .m_axi_rresp(ac_rresp), .m_axi_rlast(ac_rlast),

        .s_axi_awid(rg_awid), .s_axi_awaddr(rg_awaddr), .s_axi_awlen(rg_awlen),
        .s_axi_awsize(rg_awsize), .s_axi_awburst(rg_awburst),
        .s_axi_awlock(rg_awlock), .s_axi_awcache(rg_awcache),
        .s_axi_awprot(rg_awprot), .s_axi_awqos(rg_awqos),
        .s_axi_awvalid(rg_awvalid), .s_axi_awready(rg_awready),
        .s_axi_wdata(rg_wdata), .s_axi_wstrb(rg_wstrb), .s_axi_wlast(rg_wlast),
        .s_axi_wvalid(rg_wvalid), .s_axi_wready(rg_wready),
        .s_axi_bready(rg_bready), .s_axi_bid(), .s_axi_bresp(),
        .s_axi_bvalid(rg_bvalid),
        .s_axi_arid(rg_arid), .s_axi_araddr(rg_araddr), .s_axi_arlen(rg_arlen),
        .s_axi_arsize(rg_arsize), .s_axi_arburst(rg_arburst),
        .s_axi_arlock(rg_arlock), .s_axi_arcache(rg_arcache),
        .s_axi_arprot(rg_arprot), .s_axi_arqos(rg_arqos),
        .s_axi_arvalid(rg_arvalid), .s_axi_arready(rg_arready),
        .s_axi_rready(rg_rready), .s_axi_rid(), .s_axi_rdata(rg_rdata),
        .s_axi_rresp(), .s_axi_rlast(), .s_axi_rvalid(rg_rvalid)
    );

    // =========================================================================
    // Memory port: AXI -> Avalon
    // =========================================================================
    wire [31:0] avm_address;
    wire        avm_read, avm_write;
    wire [31:0] avm_writedata;
    wire [3:0]  avm_byteenable;
    wire [6:0]  avm_burstcount;

    wire [31:0] avm_readdata;
    wire        avm_readdatavalid;
    wire        avm_waitrequest;

    // The real 128 MB SDRAM. This replaced avalon_onchip_ram, which was a
    // 16 KB stand-in and never inferred as M9K: synthesis attributed 103,920
    // combinational functions and 131,145 registers to it (4096 x 32 = 131,072
    // bits built out of flip-flops), which is what pushed dvcon_top to 357,518
    // logic elements against this part's 114,480.
    //
    // avm_address is a BYTE address; sdram_ctrl addresses 32-bit WORDS, so the
    // low two bits are dropped here rather than inside the controller.
    wire [31:0] dram_dq_out;
    wire        dram_dq_oe;

    // Two masters, one SDRAM. The accelerator (master 0) has priority: it is the
    // only one with a deadline, since a stall mid-convolution idles the whole
    // systolic array. eth_cmd_engine has a retransmit window and can wait.
    wire [31:0] arb_address, arb_writedata, arb_readdata;
    wire        arb_read, arb_write, arb_readdatavalid, arb_waitrequest;
    wire [3:0]  arb_byteenable;
    wire [6:0]  arb_burstcount;

    avalon_arbiter #(.BURST_W(7)) u_arb (
        .clk(clk_sys), .rst_n(rst_n),

        .m0_address(avm_address), .m0_read(avm_read), .m0_write(avm_write),
        .m0_writedata(avm_writedata), .m0_byteenable(avm_byteenable),
        .m0_burstcount(avm_burstcount), .m0_readdata(avm_readdata),
        .m0_readdatavalid(avm_readdatavalid), .m0_waitrequest(avm_waitrequest),

        // The command engine only ever writes; its read channel is tied off.
        .m1_address(eth_avm_address), .m1_read(1'b0), .m1_write(eth_avm_write),
        .m1_writedata(eth_avm_writedata),
        .m1_byteenable(eth_avm_byteenable),
        .m1_burstcount(eth_avm_burstcount), .m1_readdata(),
        .m1_readdatavalid(), .m1_waitrequest(eth_avm_waitrequest),

        // The JTAG read window: how the box list gets back to the host.
        .m2_address(jt_mem_address), .m2_read(jt_mem_read),
        .m2_write(jt_mem_write), .m2_writedata(jt_mem_writedata),
        .m2_byteenable(jt_mem_byteenable),
        .m2_readdata(jt_mem_readdata),
        .m2_readdatavalid(jt_mem_readdatavalid),
        .m2_waitrequest(jt_mem_waitrequest),

        .s_address(arb_address), .s_read(arb_read), .s_write(arb_write),
        .s_writedata(arb_writedata), .s_byteenable(arb_byteenable),
        .s_burstcount(arb_burstcount), .s_readdata(arb_readdata),
        .s_readdatavalid(arb_readdatavalid), .s_waitrequest(arb_waitrequest)
    );

    sdram_ctrl #(
        .DATA_W(32), .MAX_BURST(64)
    ) u_sdram (
        .clk(clk_sys), .rst_n(rst_n),
        .cap_sel(sdram_cap_sel),
        .avs_address(arb_address[26:2]), .avs_read(arb_read),
        .avs_write(arb_write), .avs_writedata(arb_writedata),
        .avs_byteenable(arb_byteenable), .avs_burstcount(arb_burstcount),
        .avs_readdata(arb_readdata), .avs_readdatavalid(arb_readdatavalid),
        .avs_waitrequest(arb_waitrequest),
        .dram_addr(DRAM_ADDR), .dram_ba(DRAM_BA),
        .dram_cas_n(DRAM_CAS_N), .dram_ras_n(DRAM_RAS_N),
        .dram_we_n(DRAM_WE_N), .dram_cs_n(DRAM_CS_N), .dram_cke(DRAM_CKE),
        .dram_dqm(DRAM_DQM),
        .dram_dq_in(DRAM_DQ), .dram_dq_out(dram_dq_out),
        .dram_dq_oe(dram_dq_oe)
    );

    // The controller keeps DQ as separate in/out/oe so it stays synthesisable
    // and testable without a tristate inside it; the pin is driven here.
    assign DRAM_DQ = dram_dq_oe ? dram_dq_out : 32'bz;

    // SDRAM clock. The device samples on the rising edge of DRAM_CLK, so it is
    // driven inverted: that gives the data half a period of setup relative to
    // the controller's own edge, which is the standard arrangement on this
    // board. A PLL with an explicit phase shift is the better answer once the
    // timing report says what shift is needed.
    assign DRAM_CLK = ~clk_sys;

    axi_to_avalon #(
        .AXI_DATA_W(64), .AVM_DATA_W(32), .AVM_ADDR_W(32), .MAX_BURST(64)
    ) u_bridge (
        .clk(clk_sys), .rst_n(rst_n),
        .s_awvalid(ac_awvalid), .s_awid(ac_awid), .s_awlen(ac_awlen),
        .s_awsize(ac_awsize), .s_awburst(ac_awburst), .s_awaddr(ac_awaddr),
        .s_awready(ac_awready),
        .s_wvalid(ac_wvalid), .s_wlast(ac_wlast), .s_wdata(ac_wdata),
        .s_wstrb(ac_wstrb), .s_wready(ac_wready),
        .s_bready(ac_bready), .s_bvalid(ac_bvalid), .s_bid(ac_bid),
        .s_bresp(ac_bresp),
        .s_arvalid(ac_arvalid), .s_arid(ac_arid), .s_arlen(ac_arlen),
        .s_arsize(ac_arsize), .s_arburst(ac_arburst), .s_araddr(ac_araddr),
        .s_arready(ac_arready),
        .s_rready(ac_rready), .s_rvalid(ac_rvalid), .s_rid(ac_rid),
        .s_rdata(ac_rdata), .s_rresp(ac_rresp), .s_rlast(ac_rlast),
        .avm_address(avm_address), .avm_read(avm_read), .avm_write(avm_write),
        .avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable),
        .avm_burstcount(avm_burstcount),
        .avm_readdata(avm_readdata), .avm_readdatavalid(avm_readdatavalid),
        .avm_waitrequest(avm_waitrequest)
    );

    // The SDRAM pins were held at a safe idle here (CS high, CKE low) while no
    // controller existed. sdram_ctrl drives them now -- see u_sdram above.

    // =========================================================================
    // Status LEDs
    //
    // These also stop the fitter from stripping the whole design: with nothing
    // driving the register block and nothing consuming the memory port, every
    // engine is unobservable and synthesis is entitled to delete it.
    //
    // That is no longer what holds the netlist up, so these no longer have to
    // be raw bus taps. jtag_ctrl drives dvcon_regs into the accelerator's slave
    // port, and the accelerator's master port runs through the arbiter and
    // sdram_ctrl to real SDRAM pins -- every signal the old taps exposed now
    // has a genuine consumer. Check the logic-element count after changing
    // this: a collapse means something did become unobservable.
    //
    // The raw taps were unreadable as a status display. avm_byteenable is
    // combinational off wr_strb and is NOT gated by avm_write, so at idle it
    // sat at 4'hF and four LEDs were lit for no reason -- with rst_n that made
    // "five lights on" the healthy state, which looks exactly like a fault.
    //
    // Bus activity is nanoseconds wide and invisible on an LED, so each event
    // is stretched to about 1/12 s. What you should see:
    //
    //   LEDG0  heartbeat, ~1.5 Hz. Solid or dark = clock or reset is dead.
    //   LEDG1  accelerator fetching or writing memory -- IT IS RUNNING
    //   LEDG2  SDRAM traffic (either master)
    //   LEDG3  Ethernet frame received
    //   LEDG4  Ethernet frame transmitting
    //   LEDG5  Ethernet receive error (bad FCS)
    //   LEDG6  JTAG register access
    //   LEDG7  JTAG memory-window read
    //   LEDG8  rst_n -- steady on in normal operation
    //
    //   LEDR   idle: dark except LEDR0 (rst_n).
    //          running: a sweep across LEDR17..LEDR1, so "busy" is obvious
    //          from across the room and distinguishable from a hung design,
    //          which freezes the sweep while the heartbeat keeps going.
    // =========================================================================

    // ~1.5 Hz at 50 MHz: bit 24 toggles every 2**24 cycles (0.336 s).
    reg [24:0] hb_cnt;
    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) hb_cnt <= 25'd0;
        else        hb_cnt <= hb_cnt + 25'd1;
    end
    wire heartbeat = hb_cnt[24];

    wire accel_act = ac_awvalid | ac_arvalid | ac_rvalid | ac_wvalid;
    wire mem_act   = avm_read | avm_write | arb_read | arb_write;
    wire jtag_act  = jt_read | jt_write;

    // One stretcher per indicator: an event reloads the counter, which then
    // runs down. 2**22 cycles is 84 ms -- long enough to see, short enough
    // that a burst of activity still reads as flicker rather than solid.
    localparam integer STRETCH_W = 22;

    reg [STRETCH_W-1:0] str_accel, str_mem, str_rx, str_tx, str_err,
                        str_jtag, str_jmem;

    // verilator lint_off WIDTH
    `define STRETCH(reg_name, event_sig)                                   \
        always @(posedge clk_sys or negedge rst_n) begin                   \
            if (!rst_n)            reg_name <= '0;                         \
            else if (event_sig)    reg_name <= {STRETCH_W{1'b1}};          \
            else if (|reg_name)    reg_name <= reg_name - 1'b1;            \
        end

    `STRETCH(str_accel, accel_act)
    `STRETCH(str_mem,   mem_act)
    `STRETCH(str_rx,    mac_out_eof)
    `STRETCH(str_tx,    tx_busy)
    `STRETCH(str_err,   mac_out_err)
    `STRETCH(str_jtag,  jtag_act)
    `STRETCH(str_jmem,  jt_mem_read)
    `undef STRETCH
    // verilator lint_on WIDTH

    wire running = ac_dbg_busy | |str_accel;

    // =========================================================================
    // LEDR: what the accelerator is doing, and how far through it is
    //
    // LEDR0..7 are the engines, one bit each, straight from Accelerator_Top's
    // dbg_engine tap. Engine activity is bursty at 50 MHz -- a conv tile is
    // microseconds -- so each is stretched to ~84 ms or the LED never appears
    // to leave its idle state.
    //
    // LEDR8..17 are a ten-segment progress bar over the descriptor list. The
    // network is 181 descriptors, so each segment is ~18 layers. This is the
    // single most useful thing on the board: a run that wedges freezes the bar
    // at the layer it died on, which is exactly the number you need, and it
    // reads from across the room without a JTAG poll.
    // =========================================================================
    localparam integer N_DESC = 181;

    reg [STRETCH_W-1:0] str_eng [0:7];
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : g_eng
            always @(posedge clk_sys or negedge rst_n) begin
                if (!rst_n)                    str_eng[gi] <= '0;
                else if (ac_dbg_engine[gi])    str_eng[gi] <= {STRETCH_W{1'b1}};
                else if (|str_eng[gi])         str_eng[gi] <= str_eng[gi] - 1'b1;
            end
        end
    endgenerate

    wire [7:0] eng_led = {|str_eng[7], |str_eng[6], |str_eng[5], |str_eng[4],
                          |str_eng[3], |str_eng[2], |str_eng[1], |str_eng[0]};

    // Thermometer bar: segment k lights once the run has passed k/10 of the
    // descriptor list. Multiply rather than divide -- there is no divider here
    // and a compare against a scaled constant is the same answer.
    wire [9:0] prog_bar;
    generate
        for (gi = 0; gi < 10; gi = gi + 1) begin : g_bar
            assign prog_bar[gi] =
                (ac_dbg_layer * 10) >= (N_DESC * (gi + 1)) ? 1'b1 : 1'b0;
        end
    endgenerate

    assign LEDR = {prog_bar, eng_led};

    // =========================================================================
    // LEDG: the host-facing view -- link, transfers, and whether it is alive
    // =========================================================================
    assign LEDG = {rst_n,          // 8  powered and out of reset
                   ac_dbg_busy,    // 7  INFERENCE RUNNING
                   |str_jmem,      // 6  FILE TRANSFER (JTAG memory window)
                   |str_jtag,      // 5  JTAG register access
                   |str_err,       // 4  Ethernet receive error
                   |str_tx,        // 3  Ethernet transmitting
                   |str_rx,        // 2  Ethernet frame received
                   |str_mem,       // 1  SDRAM traffic
                   heartbeat};     // 0  ~1.5 Hz: clock and reset are alive

endmodule
