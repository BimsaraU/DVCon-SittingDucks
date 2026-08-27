// =============================================================================
// jtag_ctrl.sv — host control of the accelerator over JTAG
//
// Replaces the boot sequencer that used to sit in dvcon_top hardcoding four
// addresses and a START. The host drives registers, starts a frame, polls
// status and reads the box list back, all through the USB-Blaster already on
// the DE2-115 -- no extra cable, no Nios, no UART pins.
//
// altera_virtual_jtag rather than altera_avalon_jtag_uart: the JTAG UART is a
// character stream that wants a CPU on the other end. This is a register
// interface, so a raw shift register is both smaller and a better fit, and the
// host side is a quartus_stp / System Console script.
//
// ---------------------------------------------------------------------------
// WIRE FORMAT
// ---------------------------------------------------------------------------
// Two virtual instructions, selected through ir_in:
//
//   IR = 0  ADDR   shift in  { write:1, addr:7 }
//   IR = 1  DATA   shift in  32 bits of write data
//                  shift out 32 bits of read data (captured at CDR)
//
// A write is: scan ADDR with write=1, then scan DATA. The register write is
// issued on the DATA instruction's Update_DR.
// A read is:  scan ADDR with write=0 -- which issues the Avalon read -- then
// scan DATA and take what shifts out.
//
// The 32-bit shift register is used for BOTH directions, which is why a read
// needs the address scanned first: the data captured at Capture_DR has to
// already be there.
//
// ---------------------------------------------------------------------------
// CLOCK DOMAINS
// ---------------------------------------------------------------------------
// tck is supplied by the USB-Blaster and is unrelated to clk_sys, typically
// far slower (~10 MHz against 100). Everything that crosses is handshaked:
// a toggle bit set in the tck domain, synchronised into clk_sys with two
// flops, and edge-detected there. Nothing assumes a frequency ratio.
//
// The captured read data crosses the other way. It is only sampled at
// Capture_DR, long after the Avalon read retired, so a two-flop synchroniser
// on the toggle plus a plain register on the data is sufficient -- the data is
// stable for many tck periods before it is looked at.
// =============================================================================
`timescale 1ns/1ps

module jtag_ctrl (
    input  wire        clk,          // system clock
    input  wire        rst_n,

    // Avalon-MM master, driving the dvcon_regs slave
    output reg  [5:0]  avm_address,
    output reg         avm_read,
    output reg         avm_write,
    output reg  [31:0] avm_writedata,
    input  wire [31:0] avm_readdata,
    input  wire        avm_waitrequest,

    // ---- SDRAM read window ----
    // Registers alone cannot return the box list: it lives in SDRAM, and the
    // Ethernet engine implements WRITE_MEM only, so without this there is no
    // path for results to reach the host at all.
    //
    // Two pseudo-registers do it: write MEMADDR (0x3E) to set a byte address,
    // then read MEMDATA (0x3F) to get the 32-bit word there. MEMADDR
    // post-increments by 4 on each read, so the host walks the box list by
    // setting the base once and then reading repeatedly.
    output reg  [31:0] mem_address,
    output reg         mem_read,
    input  wire [31:0] mem_readdata,
    input  wire        mem_readdatavalid,
    input  wire        mem_waitrequest,

    // ---- Ethernet statistics, readable at 0x3A..0x3D ----
    input  wire [31:0] eth_good,
    input  wire [31:0] eth_bad,
    input  wire [31:0] eth_frames,
    input  wire [63:0] eth_bitmap
);

    localparam [5:0] REG_MEMADDR = 6'h3E,
                     REG_MEMDATA = 6'h3F,
                     // Link diagnostics. During bring-up the first question is
                     // always "is the board receiving anything at all", and
                     // four LED bits cannot answer it. These are read-only.
                     REG_ETH_GOOD = 6'h3A,   // frames with a good FCS
                     REG_ETH_BAD  = 6'h3B,   // frames that failed FCS
                     REG_ETH_CMD  = 6'h3C,   // command frames accepted
                     REG_ETH_BM   = 6'h3D;   // low 32 bits of the ACK bitmap

    // ---- virtual JTAG node --------------------------------------------------
    wire        tck, tdi;
    wire [1:0]  ir_in;
    wire        v_cdr, v_sdr, v_udr, v_uir;
    wire        tdo;

    sld_virtual_jtag #(
        .sld_auto_instance_index("YES"),
        .sld_instance_index(0),
        .sld_ir_width(2),
        .sld_sim_action(""),
        .sld_sim_n_scan(0),
        .sld_sim_total_length(0)
    ) u_vjtag (
        .tck(tck), .tdi(tdi), .tdo(tdo),
        .ir_in(ir_in), .ir_out(2'b00),
        .virtual_state_cdr(v_cdr),
        .virtual_state_sdr(v_sdr),
        .virtual_state_udr(v_udr),
        .virtual_state_uir(v_uir),
        .virtual_state_e1dr(), .virtual_state_pdr(),
        .virtual_state_e2dr(), .virtual_state_cir(),
        .tms(), .jtag_state_tlr(), .jtag_state_rti(),
        .jtag_state_sdrs(), .jtag_state_cdr(), .jtag_state_sdr(),
        .jtag_state_e1dr(), .jtag_state_pdr(), .jtag_state_e2dr(),
        .jtag_state_udr(), .jtag_state_sirs(), .jtag_state_cir(),
        .jtag_state_uir(), .jtag_state_e1ir(), .jtag_state_pir(),
        .jtag_state_e2ir()
    );

    localparam [1:0] IR_ADDR = 2'd0,
                     IR_DATA = 2'd1;

    // ---- tck domain: the shift register -------------------------------------
    reg [31:0] shift = 32'h0;
    // Read data, written in the clk domain below. Initialised as well as reset
    // there: the tck domain samples it through a synchroniser that runs before
    // any read has completed, and an uninitialised value shifts straight out
    // as X on the very first read.
    reg [31:0] capture = 32'h0;
    // Flipped to ask the clk domain for a transfer. It MUST start at a known
    // value: the clk side detects an edge by comparing two synchroniser
    // stages, and an X here propagates into that comparison so no edge is ever
    // seen -- every register write is decoded correctly and then silently
    // dropped. There is no reset on tck, so it is initialised instead.
    reg        req_toggle = 1'b0;
    reg        req_write  = 1'b0;
    reg [5:0]  req_addr   = 6'h0;
    reg [31:0] req_data   = 32'h0;
    // Set at Update_DR, consumed one tck later. The delay is what makes the
    // request fields stable before the clk domain is told to look at them.
    reg        arm        = 1'b0;

    // Read data crossing clk -> tck. capture only changes long before it is
    // looked at (the Avalon read retires while the host is still moving the
    // TAP towards the DATA scan), so two flops are enough; there is no need
    // for a full handshake in this direction.
    reg [31:0] capture_s1 = 32'h0;
    reg [31:0] capture_s  = 32'h0;
    always @(posedge tck) begin
        capture_s1 <= capture;
        capture_s  <= capture_s1;
    end

    always @(posedge tck) begin
        if (v_cdr) begin
            // Capture_DR: preload the shifter. Only the DATA instruction has
            // anything to return.
            //
            // capture_s, not capture: capture is written in the clk domain and
            // read here in the tck domain, so it crosses through the two flops
            // above rather than being sampled raw.
            shift <= (ir_in == IR_DATA) ? capture_s : 32'h0;
        end else if (v_sdr) begin
            shift <= {tdi, shift[31:1]};
        end else if (v_udr) begin
            case (ir_in)
            IR_ADDR: begin
                // The scan is 8 bits, so after eight shifts of a 32-bit
                // register the payload sits in the TOP byte: shift[31] is the
                // write flag and shift[30:25] the register index.
                req_write <= shift[31];
                req_addr  <= shift[30:25];
                // A read is issued as soon as the address is known, so the
                // data is waiting by the time DATA is scanned. The toggle is
                // armed rather than flipped here: req_write is assigned in
                // this same cycle, so flipping now would let the clk domain
                // sample the PREVIOUS req_write -- a read directly after a
                // write ran as another write, and the read returned nothing.
                arm <= !shift[31];
            end
            IR_DATA: begin
                req_data <= shift;
                arm      <= req_write;
            end
            default: ;
            endcase
        end else if (arm) begin
            // One tck after Update_DR: req_write / req_addr / req_data have
            // settled, so it is now safe to tell the clk domain to act.
            arm        <= 1'b0;
            req_toggle <= ~req_toggle;
        end
    end

    // A continuous assignment, not always @(*): tdo follows shift with no
    // event-ordering of its own, so a host sampling it in the same time step
    // as the Capture_DR edge sees the loaded value rather than the stale one.
    assign tdo = shift[0];

    // ---- clk domain: perform the Avalon transfer ----------------------------
    reg [2:0] req_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) req_sync <= 3'b000;
        else        req_sync <= {req_sync[1:0], req_toggle};
    end
    wire req_edge = req_sync[2] ^ req_sync[1];

    localparam [2:0] C_IDLE = 3'd0, C_WRITE = 3'd1, C_READ = 3'd2,
                     C_RDCAP = 3'd3, C_MEM = 3'd4, C_MEMW = 3'd5;
    reg [2:0] cstate;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cstate        <= C_IDLE;
            avm_address   <= '0;
            avm_read      <= 1'b0;
            avm_write     <= 1'b0;
            avm_writedata <= '0;
            capture       <= '0;
            mem_address   <= '0;
            mem_read      <= 1'b0;
        end else begin
            case (cstate)
            C_IDLE: begin
                avm_read  <= 1'b0;
                avm_write <= 1'b0;
                mem_read  <= 1'b0;
                if (req_edge) begin
                    if (!req_write && (req_addr == REG_ETH_GOOD ||
                                       req_addr == REG_ETH_BAD  ||
                                       req_addr == REG_ETH_CMD  ||
                                       req_addr == REG_ETH_BM)) begin
                        // Answered directly, without an Avalon cycle: these
                        // are wires from the Ethernet blocks, not registers in
                        // the accelerator.
                        case (req_addr)
                        REG_ETH_GOOD: capture <= eth_good;
                        REG_ETH_BAD:  capture <= eth_bad;
                        REG_ETH_CMD:  capture <= eth_frames;
                        default:      capture <= eth_bitmap[31:0];
                        endcase
                        cstate <= C_IDLE;
                    end else if (req_addr == REG_MEMADDR) begin
                        // Set the SDRAM cursor. A write only; reading it back
                        // is not useful enough to spend a state on.
                        if (req_write) mem_address <= req_data;
                        cstate <= C_IDLE;
                    end else if (req_addr == REG_MEMDATA && !req_write) begin
                        // Fetch the word at the cursor.
                        mem_read <= 1'b1;
                        cstate   <= C_MEM;
                    end else begin
                        avm_address <= req_addr;
                        if (req_write) begin
                            avm_writedata <= req_data;
                            avm_write     <= 1'b1;
                            cstate        <= C_WRITE;
                        end else begin
                            avm_read <= 1'b1;
                            cstate   <= C_READ;
                        end
                    end
                end
            end

            // SDRAM returns read data as a separate readdatavalid beat rather
            // than by holding waitrequest, so the command and the data are two
            // distinct waits.
            C_MEM: if (!mem_waitrequest) begin
                mem_read <= 1'b0;
                cstate   <= C_MEMW;
            end
            C_MEMW: if (mem_readdatavalid) begin
                capture     <= mem_readdata;
                // Post-increment: the host sets the base once and then reads
                // the box list by repeating the MEMDATA read.
                mem_address <= mem_address + 32'd4;
                cstate      <= C_IDLE;
            end
            C_WRITE: if (!avm_waitrequest) begin
                avm_write <= 1'b0;
                cstate    <= C_IDLE;
            end
            C_READ: if (!avm_waitrequest) begin
                avm_read <= 1'b0;
                cstate   <= C_RDCAP;
            end
            // Capture a cycle after the command is accepted. Slaves differ on
            // whether readdata is valid in the accepting cycle or the one
            // after -- dvcon_regs holds waitrequest until the data is genuinely
            // there, so a late capture is correct for it and for a slave that
            // registers its output. Capturing in the accepting cycle is only
            // correct for the first kind, and returns zero for the second.
            C_RDCAP: begin
                capture <= avm_readdata;
                cstate  <= C_IDLE;
            end
            default: cstate <= C_IDLE;
            endcase
        end
    end

endmodule
