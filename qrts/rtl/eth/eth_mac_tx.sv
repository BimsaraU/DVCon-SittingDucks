// =============================================================================
// eth_mac_tx.sv — transmit MAC: preamble, payload, pad, FCS
//
// Takes a frame body (destination MAC onward) and puts a legal Ethernet frame
// on the wire:
//
//   [55 x7][D5][ ...body... ][pad to 60][FCS x4][IPG]
//
// Three things here are easy to skip and each breaks the link silently:
//
//   * The minimum frame is 64 bytes INCLUDING the FCS, so a body shorter than
//     60 bytes must be zero padded. A short frame is a runt: the receiving NIC
//     drops it in hardware, and nothing on either side reports why.
//   * The FCS covers the body AND the padding, not the body alone.
//   * The inter-packet gap is 12 byte times minimum. Without it the next
//     frame's preamble runs into this one's FCS and both are lost.
//
// The FCS goes out least-significant byte first. Getting that backwards
// produces a frame that looks perfect on a scope and is discarded by every
// receiver on the segment.
// =============================================================================
`timescale 1ns/1ps

module eth_mac_tx (
    input  wire        clk,
    input  wire        rst_n,

    // ---- frame body in (destination MAC onward, no preamble, no FCS) ----
    input  wire [7:0]  tx_data,
    input  wire        tx_valid,
    input  wire        tx_last,
    output wire        tx_ready,     // body byte accepted this cycle

    // ---- to the RGMII front end ----
    output reg  [7:0]  phy_data,
    output reg         phy_en,

    output reg         busy
);

    localparam integer MIN_BODY = 60;   // 64-byte frame less the 4-byte FCS
    localparam integer IPG      = 12;

    localparam [2:0] T_IDLE  = 3'd0,
                     T_PRE   = 3'd1,
                     T_BODY  = 3'd2,
                     T_PAD   = 3'd3,
                     T_FCS   = 3'd4,
                     T_GAP   = 3'd5;

    reg [2:0]  state;
    reg [3:0]  pre_cnt;
    reg [15:0] byte_cnt;     // body + pad bytes emitted
    reg [2:0]  fcs_cnt;
    reg [3:0]  gap_cnt;

    reg        crc_init;
    wire [31:0] crc_out;
    reg [31:0] fcs_lat;

    // The CRC is fed COMBINATIONALLY from the byte being emitted this cycle,
    // not from a registered copy. With a registered feed the last body byte
    // only reaches the CRC after the FSM has left T_BODY, so crc_out lags by
    // one byte and the frame carries the CRC of all but its last byte -- which
    // every receiver on the segment then rejects.
    //
    // Feeding it here means crc_out is final on the same edge that leaves the
    // body, so the FCS can go out immediately with no settling cycle. A
    // settling cycle is not a workable alternative: holding the carrier
    // repeats a byte into the receiver's CRC, and dropping it looks like
    // end-of-carrier.
    wire       crc_feed_en = (state == T_BODY && tx_valid) || (state == T_PAD);
    wire [7:0] crc_feed_d  = (state == T_PAD) ? 8'h00 : tx_data;

    crc32_eth u_crc (
        .clk(clk), .rst_n(rst_n),
        .init(crc_init), .en(crc_feed_en), .data_in(crc_feed_d),
        .crc(), .crc_ok(), .crc_out(crc_out)
    );

    // The body is consumed only while streaming it; everything else the MAC
    // generates itself.
    assign tx_ready = (state == T_BODY) && tx_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= T_IDLE;
            pre_cnt  <= 4'd0;
            byte_cnt <= 16'd0;
            fcs_cnt  <= 3'd0;
            gap_cnt  <= 4'd0;
            phy_data <= 8'h0;
            phy_en   <= 1'b0;
            crc_init <= 1'b0;
            fcs_lat  <= 32'h0;
            busy     <= 1'b0;
        end else begin
            crc_init <= 1'b0;
            phy_en   <= 1'b0;

            case (state)
            T_IDLE: begin
                busy <= 1'b0;
                if (tx_valid) begin
                    busy     <= 1'b1;
                    crc_init <= 1'b1;
                    pre_cnt  <= 4'd0;
                    byte_cnt <= 16'd0;
                    state    <= T_PRE;
                end
            end

            // Seven 0x55 then the 0xD5 start-of-frame delimiter. Not covered
            // by the CRC.
            T_PRE: begin
                phy_en   <= 1'b1;
                phy_data <= (pre_cnt == 4'd7) ? 8'hD5 : 8'h55;
                if (pre_cnt == 4'd7) state <= T_BODY;
                else pre_cnt <= pre_cnt + 4'd1;
            end

            T_BODY: begin
                if (tx_valid) begin
                    phy_en   <= 1'b1;
                    phy_data <= tx_data;
                    byte_cnt <= byte_cnt + 16'd1;
                    if (tx_last) begin
                        // Pad only if the body is short of the minimum.
                        state <= (byte_cnt + 16'd1 < MIN_BODY) ? T_PAD : T_FCS;
                        fcs_cnt <= 3'd0;
                    end
                end
            end

            // Zero padding, covered by the CRC like any other body byte.
            T_PAD: begin
                phy_en   <= 1'b1;
                phy_data <= 8'h00;
                byte_cnt <= byte_cnt + 16'd1;
                if (byte_cnt + 16'd1 >= MIN_BODY) begin
                    fcs_cnt <= 3'd0;
                    state   <= T_FCS;
                end
            end

            // Four FCS bytes, least significant first. Reversing them produces
            // a frame that looks correct on a scope and is dropped by every
            // receiver.
            T_FCS: begin
                phy_en <= 1'b1;
                if (fcs_cnt == 3'd0) begin
                    phy_data <= crc_out[7:0];
                    fcs_lat  <= {8'h0, crc_out[31:8]};
                end else begin
                    phy_data <= fcs_lat[7:0];
                    fcs_lat  <= {8'h0, fcs_lat[31:8]};
                end

                if (fcs_cnt == 3'd3) begin
                    gap_cnt <= 4'd0;
                    state   <= T_GAP;
                end else begin
                    fcs_cnt <= fcs_cnt + 3'd1;
                end
            end

            // Inter-packet gap: carrier down for at least 12 byte times.
            T_GAP: begin
                if (gap_cnt == IPG[3:0]) begin
                    busy  <= 1'b0;
                    state <= T_IDLE;
                end else begin
                    gap_cnt <= gap_cnt + 4'd1;
                end
            end

            default: state <= T_IDLE;
            endcase
        end
    end

endmodule
