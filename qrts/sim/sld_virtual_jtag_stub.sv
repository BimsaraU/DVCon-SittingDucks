// =============================================================================
// sld_virtual_jtag_stub.sv — drivable stand-in for Altera's sld_virtual_jtag
//
// The real primitive resolves inside Quartus and has no portable simulation
// model that a bench can DRIVE -- the shipped sld_virtual_jtag_sim.v expects a
// scan file. This stub exposes the same port list plus a `scan` task, so
// tb_jtag_ctrl can shift a real IR/DR sequence through jtag_ctrl and check
// what comes out on the Avalon side.
//
// It is a bench component only: dvcon_top instantiates the real primitive, and
// this file is never in the Quartus project.
// =============================================================================
`timescale 1ns/1ps

module sld_virtual_jtag #(
    parameter sld_auto_instance_index = "YES",
    parameter integer sld_instance_index = 0,
    parameter integer sld_ir_width = 2,
    parameter sld_sim_action = "",
    parameter integer sld_sim_n_scan = 0,
    parameter integer sld_sim_total_length = 0
)(
    output reg                     tck,
    output reg                     tdi,
    input  wire                    tdo,
    output reg  [sld_ir_width-1:0] ir_in,
    input  wire [sld_ir_width-1:0] ir_out,
    output reg                     virtual_state_cdr,
    output reg                     virtual_state_sdr,
    output reg                     virtual_state_udr,
    output reg                     virtual_state_uir,
    output wire                    virtual_state_e1dr,
    output wire                    virtual_state_pdr,
    output wire                    virtual_state_e2dr,
    output wire                    virtual_state_cir,
    output wire                    tms,
    output wire                    jtag_state_tlr,
    output wire                    jtag_state_rti,
    output wire                    jtag_state_sdrs,
    output wire                    jtag_state_cdr,
    output wire                    jtag_state_sdr,
    output wire                    jtag_state_e1dr,
    output wire                    jtag_state_pdr,
    output wire                    jtag_state_e2dr,
    output wire                    jtag_state_udr,
    output wire                    jtag_state_sirs,
    output wire                    jtag_state_cir,
    output wire                    jtag_state_uir,
    output wire                    jtag_state_e1ir,
    output wire                    jtag_state_pir,
    output wire                    jtag_state_e2ir
);

    assign virtual_state_e1dr = 1'b0;
    assign virtual_state_pdr  = 1'b0;
    assign virtual_state_e2dr = 1'b0;
    assign virtual_state_cir  = 1'b0;
    assign tms = 1'b0;
    assign {jtag_state_tlr, jtag_state_rti, jtag_state_sdrs, jtag_state_cdr,
            jtag_state_sdr, jtag_state_e1dr, jtag_state_pdr, jtag_state_e2dr,
            jtag_state_udr, jtag_state_sirs, jtag_state_cir, jtag_state_uir,
            jtag_state_e1ir, jtag_state_pir, jtag_state_e2ir} = 15'h0;

    // tck is deliberately unrelated to the system clock, and slower, so the
    // bench exercises the clock-domain crossing rather than stepping around it.
    localparam realtime TCK_HALF = 35ns;

    initial begin
        tck   = 1'b0;
        tdi   = 1'b0;
        ir_in = '0;
        virtual_state_cdr = 1'b0;
        virtual_state_sdr = 1'b0;
        virtual_state_udr = 1'b0;
        virtual_state_uir = 1'b0;
    end

    task automatic tick;
        begin
            #TCK_HALF tck = 1'b1;
            #TCK_HALF tck = 1'b0;
        end
    endtask

    // Select a virtual instruction.
    task automatic scan_ir(input [sld_ir_width-1:0] ir);
        begin
            ir_in = ir;
            virtual_state_uir = 1'b1;
            tick();
            virtual_state_uir = 1'b0;
        end
    endtask

    // Shift `n` bits of `din` LSB first, returning what shifts out.
    // Mirrors the real node's CDR -> SDR* -> UDR sequence.
    task automatic scan_dr(input integer n, input [31:0] din,
                           output [31:0] dout);
        integer i;
        reg [31:0] acc;
        begin
            acc = 32'h0;

            virtual_state_cdr = 1'b1;
            tick();
            virtual_state_cdr = 1'b0;

            // tdo is sampled before the edge that shifts the DUT's register --
            // the bit on tdo is the one about to move out, as on a real TAP.
            //
            // The samples are right-aligned afterwards rather than during the
            // loop: acc shifts in from the top, so after n < 32 iterations the
            // captured bits sit in the HIGH end of acc and the untouched low
            // bits are still X. Shifting down by (32-n) at the end puts an
            // n-bit result where the caller expects it. Without this an 8-bit
            // scan returns X in every bit the scan did not fill, and a 32-bit
            // scan that follows one looks like it returned garbage.
            virtual_state_sdr = 1'b1;
            for (i = 0; i < n; i = i + 1) begin
                tdi = din[i];
                acc = {tdo, acc[31:1]};
                tick();
            end
            virtual_state_sdr = 1'b0;
            if (n < 32) acc = acc >> (32 - n);

            virtual_state_udr = 1'b1;
            tick();
            virtual_state_udr = 1'b0;

            // Idle tcks with every virtual state low. A real scan always
            // returns to Run_Test/Idle between operations, and jtag_ctrl uses
            // one of these cycles to raise its request toggle once the address
            // and data registers have settled.
            repeat (4) tick();

            dout = acc;
        end
    endtask

endmodule
