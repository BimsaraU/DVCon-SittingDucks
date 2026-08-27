// =============================================================================
// crc32_eth.sv — byte-serial Ethernet FCS (CRC-32/ISO-HDLC)
//
// Shared by eth_mac_rx (to check the received FCS) and eth_mac_tx (to append
// one). Ethernet's CRC is the standard reflected CRC-32:
//
//   polynomial  0x04C11DB7, reflected to 0xEDB88320
//   init        0xFFFFFFFF
//   reflect     input and output both reflected
//   final xor   0xFFFFFFFF
//
// Reflected form means the LSB of each byte is shifted in first, which is also
// the order bits travel on the wire, so no bit reversal is needed anywhere.
//
// Checking a frame: run every byte INCLUDING the four FCS bytes through it.
// A frame with a good FCS leaves the register at the "magic" residue
// 0xDEBB20E3 (the complement of 0x2144DF1C). Comparing against that residue is
// simpler and cheaper than holding the CRC back four bytes to compare with the
// received value, and it cannot get the byte order wrong.
//
// Eight bits per clock: at 125 MHz that is one byte per cycle, which keeps up
// with gigabit RGMII (8 bits per clock at 125 MHz) with no pipelining.
// =============================================================================
`timescale 1ns/1ps

module crc32_eth (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        init,       // load 0xFFFFFFFF (start of a frame)
    input  wire        en,         // consume data_in this cycle
    input  wire [7:0]  data_in,

    output reg  [31:0] crc,        // running register, reflected domain
    // Frame check: true when the bytes fed so far, including a trailing FCS,
    // form a valid frame.
    output wire        crc_ok,
    // Value to transmit as the FCS, MSB-first byte order handled by the caller.
    output wire [31:0] crc_out
);

    localparam [31:0] CRC_INIT     = 32'hFFFF_FFFF;
    localparam [31:0] CRC_RESIDUE  = 32'hDEBB_20E3;

    // One byte of reflected CRC-32, unrolled. Written as a function so the
    // same expression serves both engines and cannot drift between them.
    function automatic [31:0] crc32_byte(input [31:0] c, input [7:0] d);
        reg [31:0] v;
        integer i;
        begin
            v = c ^ {24'h0, d};
            for (i = 0; i < 8; i = i + 1)
                v = v[0] ? ((v >> 1) ^ 32'hEDB8_8320) : (v >> 1);
            crc32_byte = v;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      crc <= CRC_INIT;
        else if (init)   crc <= CRC_INIT;
        else if (en)     crc <= crc32_byte(crc, data_in);
    end

    assign crc_ok  = (crc == CRC_RESIDUE);
    assign crc_out = ~crc;

endmodule
