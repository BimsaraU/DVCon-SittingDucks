// =============================================================================
// eth_mac_rx.sv — receive MAC: preamble strip, address filter, FCS check
//
// Consumes the byte stream the RGMII front end produces and hands whole,
// validated frames to eth_cmd_engine. Intel's Triple-Speed Ethernet IP is
// licensed and unusable in Quartus Lite, so this is ours.
//
// What it does, in order:
//   1. Strip the preamble (0x55 repeated) and the SFD (0xD5). The preamble
//      length is not checked -- a repeater can eat some of it, and a receiver
//      that insists on exactly seven bytes drops legal frames.
//   2. Filter on destination MAC: our unicast address, or broadcast. Anything
//      else is discarded without disturbing the consumer. Multicast is not
//      accepted; nothing in this protocol uses it.
//   3. Run every byte from the destination MAC onward through crc32_eth,
//      INCLUDING the trailing FCS, and require the residue at end of frame.
//      A frame with a bad FCS is dropped, and `err` pulses so the host can see
//      a cable problem rather than silently losing commands.
//
// The output stream carries the frame from the destination MAC through to the
// last payload byte, with the four FCS bytes removed -- which means the last
// four bytes must be held back, since the FCS is only recognisable once the
// carrier drops. That is what the four-deep delay line below is for.
// =============================================================================
`timescale 1ns/1ps

module eth_mac_rx #(
    // Locally-administered unicast (bit 1 of the first octet set, bit 0
    // clear). Overridden by the top level; this default only has to be legal.
    parameter [47:0] MAC_ADDR = 48'h02_00_00_C0_FF_EE
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- from the RGMII front end ----
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,     // byte on the wire this cycle
    input  wire        rx_last,      // final byte of the carrier burst

    // ---- validated frame out ----
    output reg  [7:0]  out_data,
    output reg         out_valid,
    output reg         out_sof,      // first byte of a frame
    output reg         out_eof,      // last payload byte (FCS already removed)
    output reg         out_err,      // pulses instead of out_eof on a bad frame

    // ---- observability ----
    output reg  [31:0] stat_good,
    output reg  [31:0] stat_bad_fcs,
    output reg  [31:0] stat_filtered
);

    localparam [2:0] R_IDLE  = 3'd0,
                     R_PRE   = 3'd1,  // in the preamble, waiting for the SFD
                     R_DA    = 3'd2,  // collecting the destination MAC
                     R_BODY  = 3'd3,  // streaming out
                     R_CHECK = 3'd4,  // frame ended; decide on the FCS
                     R_DROP  = 3'd5;  // filtered; swallow the rest

    reg [2:0]  state;
    reg [2:0]  da_cnt;
    reg [47:0] da;

    // ---- CRC ----
    //
    // Fed COMBINATIONALLY from the byte on the wire this cycle, not from a
    // registered copy. With a registered feed the final FCS byte only reaches
    // the CRC after the FSM has already moved to the verdict state, so the
    // residue is evaluated one byte early and every frame -- including
    // perfectly good ones -- is rejected.
    wire [31:0] crc_val;
    wire        crc_ok;

    wire crc_en = rx_valid && (state == R_DA || state == R_BODY);

    // Init is combinational on the SFD cycle, for the same reason the data
    // feed is: a registered init would still be asserted during the first
    // destination-MAC byte, and init takes priority over en inside
    // crc32_eth, so that byte would be swallowed.
    wire crc_init = rx_valid && (rx_data == 8'hD5) &&
                    (state == R_IDLE || state == R_PRE);

    crc32_eth u_crc (
        .clk(clk), .rst_n(rst_n),
        .init(crc_init), .en(crc_en), .data_in(rx_data),
        .crc(crc_val), .crc_ok(crc_ok), .crc_out()
    );

    // ---- four-deep delay line -------------------------------------------------
    // The FCS is the last four bytes, but nothing marks them: they are only
    // identifiable once the carrier drops. Holding four bytes back means the
    // consumer never sees them, and the frame ends exactly at the last payload
    // byte. Emitting first and retracting later is not an option -- the command
    // engine has already written those bytes into SDRAM by then.
    reg [7:0] d0, d1, d2, d3;
    reg [2:0] fill;

    reg first_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= R_IDLE;
            da_cnt     <= 3'd0;
            da         <= 48'h0;
            out_data   <= 8'h0;
            out_valid  <= 1'b0;
            out_sof    <= 1'b0;
            out_eof    <= 1'b0;
            out_err    <= 1'b0;
            d0 <= 8'h0; d1 <= 8'h0; d2 <= 8'h0; d3 <= 8'h0;
            fill       <= 3'd0;
            first_out  <= 1'b0;
            stat_good  <= 32'd0;
            stat_bad_fcs <= 32'd0;
            stat_filtered <= 32'd0;
        end else begin
            out_valid <= 1'b0;
            out_sof   <= 1'b0;
            out_eof   <= 1'b0;
            out_err   <= 1'b0;

            case (state)
            // ----------------------------------------------------------------
            R_IDLE: begin
                fill      <= 3'd0;
                first_out <= 1'b1;
                if (rx_valid) begin
                    if (rx_data == 8'h55)      state <= R_PRE;
                    else if (rx_data == 8'hD5) begin
                        // SFD with no preamble seen: still a legal start.
                        da_cnt <= 3'd0;
                        state  <= R_DA;
                    end
                end
            end

            // Preamble bytes are not covered by the CRC.
            R_PRE: begin
                if (rx_valid) begin
                    if (rx_data == 8'hD5) begin
                        da_cnt <= 3'd0;
                        state  <= R_DA;
                    end else if (rx_data != 8'h55) begin
                        state <= R_DROP;      // junk between preamble and SFD
                    end
                end
                if (rx_valid && rx_last) state <= R_IDLE;
            end

            // ----------------------------------------------------------------
            // Destination MAC. The CRC covers it, and so does the output
            // stream -- the command engine's header parser counts from the
            // start of the frame.
            R_DA: begin
                if (rx_valid) begin
                    da <= {da[39:0], rx_data};

                    // The destination MAC is part of the frame the consumer
                    // sees, so it goes through the same delay line -- and must
                    // START DRAINING here, not only once R_BODY is reached.
                    // Filling for all six address bytes and draining only
                    // afterwards leaves two bytes permanently stuck in the
                    // line, and the frame arrives two bytes short.
                    if (fill == 3'd4) begin
                        out_data  <= d3;
                        out_valid <= 1'b1;
                        out_sof   <= first_out;
                        first_out <= 1'b0;
                    end else begin
                        fill <= fill + 3'd1;
                    end
                    d3 <= d2; d2 <= d1; d1 <= d0; d0 <= rx_data;

                    if (da_cnt == 3'd5) begin
                        // Whole destination address seen. `da` holds it now
                        // because the shift above is non-blocking and this
                        // compares the value including the current byte.
                        if ({da[39:0], rx_data} == MAC_ADDR ||
                            {da[39:0], rx_data} == 48'hFFFFFFFFFFFF) begin
                            state <= R_BODY;
                        end else begin
                            stat_filtered <= stat_filtered + 32'd1;
                            state <= R_DROP;
                        end
                    end else begin
                        da_cnt <= da_cnt + 3'd1;
                    end
                end
                if (rx_valid && rx_last) state <= R_IDLE;
            end

            // ----------------------------------------------------------------
            R_BODY: begin
                if (rx_valid) begin
                    // Emit the byte that is falling out of the delay line, so
                    // the four still inside it (the FCS, once the frame ends)
                    // are never handed on.
                    if (fill == 3'd4) begin
                        out_data  <= d3;
                        out_valid <= 1'b1;
                        out_sof   <= first_out;
                        first_out <= 1'b0;
                    end else begin
                        fill <= fill + 3'd1;
                    end
                    d3 <= d2; d2 <= d1; d1 <= d0; d0 <= rx_data;

                    if (rx_last) state <= R_CHECK;
                end
            end

            // ----------------------------------------------------------------
            // The CRC is fed combinationally, so the final FCS byte was
            // absorbed on the edge that entered this state and the residue is
            // valid now.
            R_CHECK: begin
                if (crc_ok) begin
                    out_eof   <= 1'b1;
                    stat_good <= stat_good + 32'd1;
                end else begin
                    out_err      <= 1'b1;
                    stat_bad_fcs <= stat_bad_fcs + 32'd1;
                end
                state <= R_IDLE;
            end

            // Filtered by address: consume to the end of the carrier without
            // emitting anything or counting an FCS error.
            R_DROP: begin
                if (rx_valid && rx_last) state <= R_IDLE;
            end

            default: state <= R_IDLE;
            endcase
        end
    end

endmodule
