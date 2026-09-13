// =============================================================================
// npu_ram.sv - simple dual-port RAM, one write port and one registered read
//
// Written in the one shape Quartus maps onto M9K without argument: a single
// clocked process, write and registered read, no reset, no second read port.
// Every buffer in the NPU is built from this.
// =============================================================================
`timescale 1ns/1ps

module npu_ram #(
    parameter integer W  = 32,
    parameter integer D  = 1024,
    parameter integer AW = (D > 1) ? $clog2(D) : 1
)(
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [W-1:0]  wdata,
    input  wire [AW-1:0] raddr,
    output reg  [W-1:0]  q
);
    (* ramstyle = "M9K" *) reg [W-1:0] mem [0:D-1];

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        q <= mem[raddr];
    end
endmodule
