// =============================================================================
// dvcon_top.sv - DE2-115 top level: YOLO26n NPU, SDRAM, Ethernet, JTAG
//
//     ENET1 --- mii_rx/tx --- eth_mac_rx/tx --- eth_cmd_engine ---+ (m1)
//                                                                  |
//     JTAG --- jtag_ctrl --- registers --> npu_top --Avalon--------+ (m0)
//                  |                                               |
//                  +--- SDRAM memory window ------------------------+ (m2)
//                                                                  |
//                                                   avalon_arbiter -> sdram_ctrl -> 128 MB
//
// Control is over JTAG; bulk data (the model blob and each frame) over
// Ethernet, or over the JTAG memory window when Ethernet is not available.
// The NPU is Avalon master 0 and has priority: it is the only master with a
// deadline.
//
// Everything runs on the 50 MHz oscillator. The SDRAM controller's timing
// constants were written for 100 MHz and are conservative at 50 MHz, EXCEPT
// the refresh interval, which is a count of cycles and so halves in rate when
// the clock halves: 780 cycles is 15.6 us at 50 MHz against the 7.8 us the
// IS42S16320D requires. It is overridden to 380 here.
// =============================================================================
`timescale 1ns/1ps

module dvcon_top #(
    parameter [47:0]  MAC_ADDR   = 48'h02_00_00_C0_FF_EE
)(
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,

    // SDRAM (IS42S16320D x2, 128 MB, 32-bit)
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

    // Ethernet PHY 1 (88E1111, connector J5), MII: JP2 on pins 2-3
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

    // status
    output wire [17:0] LEDR,
    output wire [8:0]  LEDG,
    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,
    output wire [6:0]  HEX4,
    output wire [6:0]  HEX5,
    output wire [6:0]  HEX6,
    output wire [6:0]  HEX7
);
    wire clk_sys = CLOCK_50;

    // KEY0: asynchronous assert, synchronous release
    reg [3:0] rst_sync;
    always @(posedge clk_sys or negedge KEY[0]) begin
        if (!KEY[0]) rst_sync <= 4'b0000;
        else         rst_sync <= {rst_sync[2:0], 1'b1};
    end
    wire rst_n = rst_sync[3];

    // KEY3: run one inference. Synchronised, then a new level must hold for
    // 2^20 cycles (21 ms) before it counts, so contact bounce gives one press.
    reg [2:0]  k3_s;
    reg [19:0] k3_c;
    reg        k3_db, k3_q;
    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) begin
            k3_s <= 3'b111; k3_c <= 20'd0; k3_db <= 1'b1; k3_q <= 1'b1;
        end else begin
            k3_s <= {k3_s[1:0], KEY[3]};
            k3_q <= k3_db;
            if (k3_s[2] == k3_db)  k3_c <= 20'd0;
            else if (&k3_c)        begin k3_db <= k3_s[2]; k3_c <= 20'd0; end
            else                   k3_c <= k3_c + 20'd1;
        end
    end
    wire key_start = k3_q && !k3_db;     // press: debounced level falls

    // =========================================================================
    // Ethernet: MII front end, MAC, command engine (unchanged, proven on board)
    // =========================================================================
    wire [7:0] rx_byte;
    wire       rx_valid, rx_last;

    mii_rx_adapter u_mii_rx (
        .rx_clk(ENET1_RX_CLK), .rst_n(rst_n),
        .mii_rxd(ENET1_RX_DATA), .mii_rx_dv(ENET1_RX_DV),
        .clk(clk_sys),
        .out_data(rx_byte), .out_valid(rx_valid), .out_last(rx_last));

    wire [7:0]  mac_out_data;
    wire        mac_out_valid, mac_out_sof, mac_out_eof, mac_out_err;
    wire [31:0] stat_good, stat_bad_fcs, stat_filtered;

    eth_mac_rx #(.MAC_ADDR(MAC_ADDR)) u_mac_rx (
        .clk(clk_sys), .rst_n(rst_n),
        .rx_data(rx_byte), .rx_valid(rx_valid), .rx_last(rx_last),
        .out_data(mac_out_data), .out_valid(mac_out_valid),
        .out_sof(mac_out_sof), .out_eof(mac_out_eof), .out_err(mac_out_err),
        .stat_good(stat_good), .stat_bad_fcs(stat_bad_fcs),
        .stat_filtered(stat_filtered));

    wire [7:0]  cmd_tx_data;
    wire        cmd_tx_valid, cmd_tx_last, cmd_tx_ready;
    wire [31:0] eth_avm_address, eth_avm_writedata;
    wire        eth_avm_write, eth_avm_waitrequest;
    wire [3:0]  eth_avm_byteenable;
    wire [6:0]  eth_avm_burstcount;
    wire [31:0] stat_frames, stat_bytes, stat_wdrop;
    wire [63:0] rx_bitmap;

    eth_cmd_engine #(.MAC_ADDR(MAC_ADDR)) u_cmd (
        .clk(clk_sys), .rst_n(rst_n),
        .rx_data(mac_out_data), .rx_valid(mac_out_valid),
        .rx_sof(mac_out_sof), .rx_eof(mac_out_eof), .rx_err(mac_out_err),
        .tx_data(cmd_tx_data), .tx_valid(cmd_tx_valid),
        .tx_last(cmd_tx_last), .tx_ready(cmd_tx_ready),
        .avm_address(eth_avm_address), .avm_write(eth_avm_write),
        .avm_writedata(eth_avm_writedata), .avm_byteenable(eth_avm_byteenable),
        .avm_burstcount(eth_avm_burstcount), .avm_waitrequest(eth_avm_waitrequest),
        .stat_frames(stat_frames), .stat_bytes(stat_bytes),
        .stat_wdrop(stat_wdrop), .rx_bitmap(rx_bitmap));

    wire [7:0] tx_byte;
    wire       tx_en, tx_busy;

    eth_mac_tx u_mac_tx (
        .clk(clk_sys), .rst_n(rst_n),
        .tx_data(cmd_tx_data), .tx_valid(cmd_tx_valid),
        .tx_last(cmd_tx_last), .tx_ready(cmd_tx_ready),
        .phy_data(tx_byte), .phy_en(tx_en), .busy(tx_busy));

    mii_tx_adapter u_mii_tx (
        .clk(clk_sys), .rst_n(rst_n),
        .in_data(tx_byte), .in_valid(tx_en), .in_ready(),
        .tx_clk(ENET1_TX_CLK),
        .mii_txd(ENET1_TX_DATA), .mii_tx_en(ENET1_TX_EN));

    assign ENET1_GTX_CLK = 1'b0;       // unused in MII
    assign ENET1_RST_N   = rst_n;
    assign ENET1_MDC     = 1'b0;
    assign ENET1_MDIO    = 1'bz;

    // =========================================================================
    // JTAG control and the NPU
    // =========================================================================
    wire [5:0]  jt_address;
    wire        jt_read, jt_write;
    wire [31:0] jt_writedata, npu_rdata;
    wire        npu_wait;

    wire [31:0] jt_mem_address, jt_mem_writedata, jt_mem_readdata;
    wire        jt_mem_read, jt_mem_write, jt_mem_readdatavalid, jt_mem_waitrequest;
    wire [3:0]  jt_mem_byteenable;
    wire [1:0]  sdram_cap_sel;

    wire        n_busy, n_array, n_drd, n_dwr;
    wire [3:0]  n_state;
    wire [7:0]  n_op;
    wire [2:0]  n_cph, n_eph;
    wire [15:0] n_tag, n_idx, n_boxes;
    wire [1:0]  n_flags;

    jtag_ctrl u_jtag (
        .clk(clk_sys), .rst_n(rst_n),
        .avm_address(jt_address), .avm_read(jt_read), .avm_write(jt_write),
        .avm_writedata(jt_writedata),
        .avm_readdata(npu_rdata), .avm_waitrequest(npu_wait),
        .mem_address(jt_mem_address), .mem_read(jt_mem_read),
        .mem_write(jt_mem_write), .mem_writedata(jt_mem_writedata),
        .mem_byteenable(jt_mem_byteenable),
        .mem_readdata(jt_mem_readdata),
        .mem_readdatavalid(jt_mem_readdatavalid),
        .mem_waitrequest(jt_mem_waitrequest),
        .eth_good(stat_good), .eth_bad(stat_bad_fcs),
        .eth_filtered(stat_filtered),
        .eth_frames(stat_frames), .eth_bitmap(rx_bitmap),
        .eth_wdrop(stat_wdrop),
        // jtag_ctrl's diagnostic words (0x2F..0x38) carry NPU status now
        .dbg_ce_src({16'd0, n_tag}), .dbg_ce_dst({16'd0, n_idx}),
        .dbg_desc_sel(), .dbg_desc_val(32'd0),
        .dbg_conv0({10'd0, n_eph, n_cph, n_op, n_state, n_busy, n_array, n_drd, n_dwr}),
        .dbg_conv1({16'd0, n_boxes}),
        .dbg_elem(32'd0), .dbg_arb(32'd0),
        .sdram_cap_sel(sdram_cap_sel));

    wire [31:0] npu_address, npu_writedata, npu_readdata;
    wire        npu_read, npu_write, npu_readdatavalid, npu_waitrequest;
    wire [3:0]  npu_byteenable;
    wire [6:0]  npu_burstcount;

    npu_top u_npu (
        .clk(clk_sys), .rst_n(rst_n),
        .reg_addr(jt_address), .reg_read(jt_read), .reg_write(jt_write),
        .reg_wdata(jt_writedata), .reg_rdata(npu_rdata),
        .reg_waitrequest(npu_wait),
        .avm_address(npu_address), .avm_read(npu_read), .avm_write(npu_write),
        .avm_writedata(npu_writedata), .avm_byteenable(npu_byteenable),
        .avm_burstcount(npu_burstcount), .avm_readdata(npu_readdata),
        .avm_readdatavalid(npu_readdatavalid), .avm_waitrequest(npu_waitrequest),
        .ext_start(key_start), .o_flags(n_flags),
        .o_busy(n_busy), .o_state(n_state), .o_op(n_op),
        .o_conv_phase(n_cph), .o_elem_phase(n_eph),
        .o_tag(n_tag), .o_idx(n_idx), .o_boxes(n_boxes),
        .o_array(n_array), .o_dma_rd(n_drd), .o_dma_wr(n_dwr));

    // =========================================================================
    // SDRAM: three masters, one controller
    // =========================================================================
    wire [31:0] arb_address, arb_writedata, arb_readdata;
    wire        arb_read, arb_write, arb_readdatavalid, arb_waitrequest;
    wire [3:0]  arb_byteenable;
    wire [6:0]  arb_burstcount;

    avalon_arbiter #(.BURST_W(7)) u_arb (
        .clk(clk_sys), .rst_n(rst_n),
        .m0_address(npu_address), .m0_read(npu_read), .m0_write(npu_write),
        .m0_writedata(npu_writedata), .m0_byteenable(npu_byteenable),
        .m0_burstcount(npu_burstcount), .m0_readdata(npu_readdata),
        .m0_readdatavalid(npu_readdatavalid), .m0_waitrequest(npu_waitrequest),
        .m1_address(eth_avm_address), .m1_read(1'b0), .m1_write(eth_avm_write),
        .m1_writedata(eth_avm_writedata), .m1_byteenable(eth_avm_byteenable),
        .m1_burstcount(eth_avm_burstcount), .m1_readdata(),
        .m1_readdatavalid(), .m1_waitrequest(eth_avm_waitrequest),
        .m2_address(jt_mem_address), .m2_read(jt_mem_read),
        .m2_write(jt_mem_write), .m2_writedata(jt_mem_writedata),
        .m2_byteenable(jt_mem_byteenable), .m2_readdata(jt_mem_readdata),
        .m2_readdatavalid(jt_mem_readdatavalid), .m2_waitrequest(jt_mem_waitrequest),
        .s_address(arb_address), .s_read(arb_read), .s_write(arb_write),
        .s_writedata(arb_writedata), .s_byteenable(arb_byteenable),
        .s_burstcount(arb_burstcount), .s_readdata(arb_readdata),
        .s_readdatavalid(arb_readdatavalid), .s_waitrequest(arb_waitrequest));

    wire [31:0] dram_dq_out;
    wire        dram_dq_oe;

    sdram_ctrl #(
        .DATA_W(32), .MAX_BURST(64),
        .T_INIT(10000),   // 200 us at 50 MHz
        // 6.0 us at 50 MHz. The request is only serviced once the controller
        // is idle, and a 64-beat burst with its activate/precharge holds it
        // off for up to ~75 cycles, so the interval must leave that much
        // headroom under the 7.8 us (390 cycle) limit. 380 did not: the
        // sdram_model bench saw 391-cycle gaps with the NPU streaming.
        .T_REFI(300)
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
        .dram_dq_oe(dram_dq_oe));

    assign DRAM_DQ  = dram_dq_oe ? dram_dq_out : 32'bz;
    assign DRAM_CLK = ~clk_sys;        // memory samples half a period later

    // =========================================================================
    // Board display
    //
    // LEDR is the NPU state machine, one lamp per state (see dvcon_status).
    // LEDG is the host-facing view: link, transfers, heartbeat.
    // HEX7..HEX0: model layer, operation letter, descriptor index while
    // running; the box count once a frame is done.
    // =========================================================================
    dvcon_status u_status (
        .clk(clk_sys), .rst_n(rst_n),
        .n_busy(n_busy), .n_state(n_state), .n_op(n_op),
        .n_cph(n_cph), .n_eph(n_eph), .n_tag(n_tag), .n_idx(n_idx),
        .n_boxes(n_boxes), .n_array(n_array), .n_drd(n_drd), .n_dwr(n_dwr),
        .ld_flags(n_flags), .ev_rx(mac_out_eof), .ev_tx(tx_busy),
        .ev_err(mac_out_err), .ev_jmem(jt_mem_read | jt_mem_write),
        .LEDR(LEDR), .LEDG(LEDG),
        .HEX0(HEX0), .HEX1(HEX1), .HEX2(HEX2), .HEX3(HEX3),
        .HEX4(HEX4), .HEX5(HEX5), .HEX6(HEX6), .HEX7(HEX7));
endmodule
