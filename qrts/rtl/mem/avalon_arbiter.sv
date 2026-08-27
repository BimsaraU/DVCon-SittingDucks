// =============================================================================
// avalon_arbiter.sv — two Avalon-MM masters onto one SDRAM slave
//
// The accelerator streams weights and feature maps; eth_cmd_engine writes the
// model blob and each incoming image. Both want the same 128 MB, and the SDRAM
// has one port.
//
// ---------------------------------------------------------------------------
// PRIORITY, AND WHY IT IS NOT ROUND ROBIN
// ---------------------------------------------------------------------------
// Master 0 (the accelerator) wins. It is the only master with a deadline: it
// is mid-convolution with a systolic array waiting on the next weight tile, and
// a stall there idles 256 multipliers. eth_cmd_engine has a 64-frame window and
// a retransmit path -- being made to wait costs it nothing, and the host is
// four orders of magnitude slower than the memory anyway.
//
// The starvation that fixed priority normally invites does not apply here: the
// accelerator's traffic is bursty (it reads a tile, then computes for many
// cycles), so gaps are frequent and the Ethernet writer always gets in.
//
// ---------------------------------------------------------------------------
// THE RULE THAT MATTERS
// ---------------------------------------------------------------------------
// Once a burst is granted it is HELD to completion. Avalon presents address and
// burstcount with the first beat only; if the grant moved mid-burst the slave
// would attribute the remaining beats to the wrong master's address, and the
// symptom is two interleaved transfers each corrupting the other -- which looks
// like a memory fault, not an arbitration bug.
//
// Read data returns unsolicited (readdatavalid, no address), so it is steered
// by which master owns the outstanding read, tracked separately from the
// command grant.
// =============================================================================
`timescale 1ns/1ps

module avalon_arbiter #(
    parameter integer ADDR_W = 32,
    parameter integer DATA_W = 32,
    parameter integer BURST_W = 7
)(
    input  wire                clk,
    input  wire                rst_n,

    // ---- master 0: the accelerator (priority) ----
    input  wire [ADDR_W-1:0]   m0_address,
    input  wire                m0_read,
    input  wire                m0_write,
    input  wire [DATA_W-1:0]   m0_writedata,
    input  wire [DATA_W/8-1:0] m0_byteenable,
    input  wire [BURST_W-1:0]  m0_burstcount,
    output wire [DATA_W-1:0]   m0_readdata,
    output reg                 m0_readdatavalid,
    output wire                m0_waitrequest,

    // ---- master 1: eth_cmd_engine ----
    input  wire [ADDR_W-1:0]   m1_address,
    input  wire                m1_read,
    input  wire                m1_write,
    input  wire [DATA_W-1:0]   m1_writedata,
    input  wire [DATA_W/8-1:0] m1_byteenable,
    input  wire [BURST_W-1:0]  m1_burstcount,
    output wire [DATA_W-1:0]   m1_readdata,
    output reg                 m1_readdatavalid,
    output wire                m1_waitrequest,

    // ---- master 2: jtag_ctrl's SDRAM read window (lowest priority) ----
    // Reads only, one word at a time, driven by a human at a JTAG cable. It
    // gets whatever is left after the other two.
    input  wire [ADDR_W-1:0]   m2_address,
    input  wire                m2_read,
    output wire [DATA_W-1:0]   m2_readdata,
    output reg                 m2_readdatavalid,
    output wire                m2_waitrequest,

    // ---- slave: the SDRAM controller ----
    output wire [ADDR_W-1:0]   s_address,
    output wire                s_read,
    output wire                s_write,
    output wire [DATA_W-1:0]   s_writedata,
    output wire [DATA_W/8-1:0] s_byteenable,
    output wire [BURST_W-1:0]  s_burstcount,
    input  wire [DATA_W-1:0]   s_readdata,
    input  wire                s_readdatavalid,
    input  wire                s_waitrequest
);

    reg        busy;        // a burst is in flight
    reg        owner;       // which master owns it (0 or 1)
    reg [BURST_W-1:0] beats_left;

    wire m0_req = m0_read || m0_write;
    wire m1_req = m1_read || m1_write;
    wire m2_req = m2_read;

    // Who is being served this cycle: the burst owner if one is in flight,
    // otherwise strict priority 0 > 1 > 2.
    wire [1:0] sel = busy ? {1'b0, owner}
                          : (m0_req ? 2'd0 : (m1_req ? 2'd1 : 2'd2));

    // One dead cycle between bursts.
    //
    // Without it, the cycle in which the outgoing burst's LAST beat is accepted
    // is also the cycle in which `sel` has already swung to the other master --
    // so the slave, seeing waitrequest low, accepts the new master's first beat
    // while the arbiter is still retiring the old burst. That beat is written to
    // memory but never counted, and the new master's burst effectively starts at
    // its SECOND word: every value it wrote came back shifted by one.
    //
    // A dead cycle costs one clock per burst handover and makes the boundary
    // unambiguous.
    reg gap;
    wire grant_ok = !gap;

    assign s_address    = (sel == 2'd0) ? m0_address :
                          (sel == 2'd1) ? m1_address : m2_address;
    assign s_read       = grant_ok && ((sel == 2'd0) ? m0_read :
                                       (sel == 2'd1) ? m1_read : m2_read);
    assign s_write      = grant_ok && ((sel == 2'd0) ? m0_write :
                                       (sel == 2'd1) ? m1_write : 1'b0);
    assign s_writedata  = (sel == 2'd1) ? m1_writedata : m0_writedata;
    assign s_byteenable = (sel == 2'd1) ? m1_byteenable : m0_byteenable;
    assign s_burstcount = (sel == 2'd0) ? m0_burstcount :
                          (sel == 2'd1) ? m1_burstcount : 7'd1;

    // A master that is not selected is held off. Note this is combinational
    // from the slave's waitrequest, as Avalon requires.
    assign m0_waitrequest = (grant_ok && sel == 2'd0) ? s_waitrequest : 1'b1;
    assign m1_waitrequest = (grant_ok && sel == 2'd1) ? s_waitrequest : 1'b1;
    assign m2_waitrequest = (grant_ok && sel == 2'd2) ? s_waitrequest : 1'b1;

    // Read data is broadcast; only the flag is steered.
    assign m0_readdata = s_readdata;
    assign m1_readdata = s_readdata;
    assign m2_readdata = s_readdata;

    // Which master has a read outstanding. Tracked separately from `owner`
    // because a read's data can still be arriving after its command burst has
    // finished and the grant has moved on. It simply stays pointing at the last
    // read's requester until another read is granted, which is sufficient
    // because the SDRAM controller serves one read burst at a time.
    reg [1:0] rd_owner;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            owner      <= 1'b0;
            beats_left <= '0;
            rd_owner   <= 2'd0;
            gap        <= 1'b0;
        end else begin
            gap <= 1'b0;
            if (!busy) begin
                // Grant on the first accepted beat of a new burst.
                if (grant_ok && (m0_req || m1_req || m2_req)
                    && !s_waitrequest) begin
                    // `owner` only ever tracks a WRITE burst, and only masters
                    // 0 and 1 write -- master 2's reads are single beats that
                    // never set `busy`, so one bit is sufficient here.
                    owner <= sel[0];
                    if (s_read) begin
                        // A read burst is one command; the data follows.
                        rd_owner <= sel;
                    end else begin
                        // A write burst holds the grant for every beat.
                        if (s_burstcount > 1) begin
                            busy       <= 1'b1;
                            beats_left <= s_burstcount - 1'b1;
                        end
                    end
                end
            end else begin
                // Mid-burst: hold the grant until the last beat is accepted.
                if (!s_waitrequest && (owner ? m1_write : m0_write)) begin
                    if (beats_left <= 1) begin
                        busy <= 1'b0;
                        gap  <= 1'b1;   // no new grant in the next cycle
                    end else begin
                        beats_left <= beats_left - 1'b1;
                    end
                end
            end
        end
    end

    // Steer returning read data to whichever master asked for it.
    always @(*) begin
        m0_readdatavalid = s_readdatavalid && (rd_owner == 2'd0);
        m1_readdatavalid = s_readdatavalid && (rd_owner == 2'd1);
        m2_readdatavalid = s_readdatavalid && (rd_owner == 2'd2);
    end

endmodule
