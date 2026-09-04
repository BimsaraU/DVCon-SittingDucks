# =============================================================================
# dvcon_jtag.tcl — control the accelerator over the USB-Blaster
#
#     quartus_stp -t tools/dvcon_jtag.tcl ident
#     quartus_stp -t tools/dvcon_jtag.tcl link
#     quartus_stp -t tools/dvcon_jtag.tcl run
#     quartus_stp -t tools/dvcon_jtag.tcl boxes
#     quartus_stp -t tools/dvcon_jtag.tcl peek 0x01
#     quartus_stp -t tools/dvcon_jtag.tcl poke 0x09 0x20
#
# This is the control half of the split: registers, START, status and the box
# list come through here, while the model blob and each image go over Ethernet
# (sw/dvcon_host.c). The JTAG cable is already attached for programming, so
# control costs no extra hardware, and a dropped Ethernet frame can never start
# a frame or corrupt a register.
#
# The wire format is jtag_ctrl's, two virtual instructions:
#
#   IR 0  ADDR   8 bits: { write:1, addr:6, pad:1 }
#   IR 1  DATA   32 bits, shifted in for a write, out for a read
#
# A read scans ADDR first (which issues the Avalon read), then DATA.
# =============================================================================

set REG_CTRL        0x00
set REG_STATUS      0x01
set REG_DESC_ADDR   0x06
set REG_IMG_ADDR    0x07
set REG_BOX_ADDR    0x08
set REG_CONF_THRESH 0x09
set REG_NUM_BOXES   0x0A
# Word address, not byte: dvcon_regs decodes byte_off = avs_address*4, and
# IDENT is at byte 0x30 -> word 0x0C. This said 0x0F (byte 0x3C), which is
# not IDENT, so the read fell through to the accelerator and returned 0 --
# reported as "is the right bitstream loaded?" on a correctly loaded board.
set REG_IDENT       0x0C
set REG_MEMADDR     0x3E
set REG_MEMDATA     0x3F

# Link diagnostics, answered from wires inside jtag_ctrl rather than through
# the accelerator's register block.
set REG_ETH_GOOD    0x3A
set REG_ETH_BAD     0x3B
set REG_ETH_CMD     0x3C
set REG_ETH_BM      0x3D
set REG_ETH_FILT    0x39

# Kept in step with tools/gen_memmap.py. Only the values this script actually
# writes are duplicated here; quartus_stp's Tcl cannot include the C header.
set MODEL_BASE  0x00000000
set MODEL_HDR_BYTES 32
set FRAME_BASE  0x00400000
set BOXES_BASE  0x00600000

proc dvcon_open {} {
    set hw ""
    # get_hardware_names THROWS when no cable is attached -- it does not return
    # an empty list -- and the raw throw reads as a Tcl failure rather than as
    # "plug the board in".
    if {[catch {get_hardware_names} all]} {
        error "no JTAG hardware found: is the USB-Blaster plugged in and its driver installed? ($all)"
    }
    foreach h $all {
        if {[string match "*USB-Blaster*" $h]} { set hw $h; break }
    }
    if {$hw eq ""} {
        error "no USB-Blaster found -- check the cable and that no other\
               Quartus tool holds it open"
    }

    # Match the IDCODE, not the part name. Quartus never calls this device
    # "EP4CE115": the EP4CE115, EP3C120 and 10CL120 share IDCODE 0x020F70DD, and
    # the name reported is the alias list --
    #
    #     @1: 10CL120(Y|Z)/EP3C120/.. (0x020F70DD)
    #
    # so a "*EP4CE115*" match failed on a correctly connected, powered board and
    # reported it as "is the board powered and configured?", which sends you
    # looking at the cable instead of at this line.
    set dev ""
    foreach d [get_device_names -hardware_name $hw] {
        if {[string match "*0x020F70DD*" $d]} { set dev $d; break }
    }
    if {$dev eq ""} {
        error "no device with IDCODE 0x020F70DD (EP4CE115) on $hw --               devices seen: [get_device_names -hardware_name $hw]"
    }

    open_device -hardware_name $hw -device_name $dev

    # open_device alone is not enough. Every virtual IR/DR shift below needs
    # exclusive access, and without this the first one fails with
    # "A device has not been locked; exclusive communication must be obtained
    # first" -- which reads like a JTAG fault rather than a missing call.
    device_lock -timeout 10000

    return [list $hw $dev]
}


# Always paired with dvcon_open. The lock must be released, or the next
# invocation -- and jtagconfig, and the programmer -- find the cable held.
proc dvcon_close {} {
    catch {device_unlock}
    close_device
}

# jtag_ctrl reads shift[31] as the write flag and shift[30:25] as the register,
# so an 8-bit scan of { write, addr, 0 } lands in the top byte.
proc _scan_addr {addr is_write} {
    device_virtual_ir_shift -instance_index 0 -ir_value 0 -no_captured_ir_value
    set v [expr {($is_write << 7) | (($addr & 0x3F) << 1)}]
    device_virtual_dr_shift -instance_index 0 -length 8 \
        -dr_value [format %02X $v] -value_in_hex -no_captured_dr_value
}

proc dvcon_poke {addr data} {
    _scan_addr $addr 1
    device_virtual_ir_shift -instance_index 0 -ir_value 1 -no_captured_ir_value
    device_virtual_dr_shift -instance_index 0 -length 32 \
        -dr_value [format %08X $data] -value_in_hex -no_captured_dr_value
}

proc dvcon_peek {addr} {
    _scan_addr $addr 0

    # KNOWN BUG, not fixed here: every register except IDENT comes back one
    # transaction LATE.
    #
    # The read is issued at the ADDR scan's Update_DR and has to cross to the
    # system clock, run on Avalon and cross back. jtag_ctrl's C_READ treats
    # !avm_waitrequest as "accepted, data ready" and captures one cycle later --
    # but dvcon_regs drives `avs_waitrequest = (state != S_IDLE)`, and in the
    # cycle a request is accepted the state is still S_IDLE. So waitrequest
    # reads LOW exactly when the transaction is being launched, and jtag_ctrl
    # captures whatever the PREVIOUS access left in avs_readdata.
    #
    # Measured on the board: CTRL read 0xDC100001 straight after an IDENT read,
    # then 0x00000000 when read again. IDENT is the one register that works,
    # because dvcon_regs answers it in S_IDLE and never touches the AXI path --
    # which is why bring-up looked healthy.
    #
    # A host-side delay does NOT help: Quartus queues the scans, so a Tcl
    # `after` between them does not separate them on the wire (tried, no
    # change). Scanning DATA twice does not help either -- the read is only
    # issued once. The fix belongs in RTL and needs a rebuild: either jtag_ctrl
    # must not treat the accepting cycle as completion, or dvcon_regs must
    # assert waitrequest combinationally when a request will stall.
    #
    # Note neither module's own bench catches this: tb_dvcon_regs drives a
    # master that waits for waitrequest to rise and then fall, and tb_jtag_ctrl
    # uses a slave that answers immediately. Nothing connects the two.
    device_virtual_ir_shift -instance_index 0 -ir_value 1 -no_captured_ir_value
    set r [device_virtual_dr_shift -instance_index 0 -length 32 \
              -dr_value 00000000 -value_in_hex]
    return [expr {"0x$r"}]
}

# ----------------------------------------------------------------------------
# Load a binary into SDRAM over JTAG.
#
# This exists because Ethernet is the only bulk path the design was designed
# around, and it is unusable until the board's JP1 jumper selects MII. The
# memory window's write side (REG_MEMDATA) post-increments the cursor, so the
# base is set once and words are streamed.
#
# Measured 0.37 ms/word on a USB-Blaster, so a 2.46 MB model is about four
# minutes. Slow next to 100 Mbit, but it needs no jumper and no Linux host.
# ----------------------------------------------------------------------------
# Stream words to the CURRENT cursor without re-selecting the register.
#
# jtag_ctrl latches req_write/req_addr on the ADDR scan and reuses them on every
# later DATA scan (see its IR_DATA case), and REG_MEMDATA post-increments the
# cursor. So a load needs ONE address scan and then nothing but data scans --
# dvcon_poke was doing an IR+DR address scan per word, which is half the JTAG
# traffic in a transfer that is entirely JTAG-traffic-bound.
proc dvcon_stream_begin {} {
    _scan_addr $::REG_MEMDATA 1
}

proc dvcon_stream_word {w} {
    device_virtual_ir_shift -instance_index 0 -ir_value 1 -no_captured_ir_value
    device_virtual_dr_shift -instance_index 0 -length 32 \
        -dr_value [format %08X [expr {$w & 0xFFFFFFFF}]] \
        -value_in_hex -no_captured_dr_value
}

proc dvcon_load {path base} {
    set fh [open $path rb]
    fconfigure $fh -translation binary
    set data [read $fh]
    close $fh

    set nbytes [string length $data]
    # Pad to a whole number of 32-bit words; the memory window is word-wide.
    set pad [expr {(4 - ($nbytes % 4)) % 4}]
    if {$pad} { append data [string repeat \x00 $pad] }
    set nwords [expr {([string length $data]) / 4}]

    puts [format "  %s: %d bytes (%d words) -> 0x%08X" \
            [file tail $path] $nbytes $nwords $base]

    # binary scan once, little-endian, rather than per-word string indexing --
    # the latter is O(n^2) on a multi-megabyte string in Tcl.
    binary scan $data iu* words

    dvcon_poke $::REG_MEMADDR $base
    dvcon_stream_begin
    set t0 [clock milliseconds]
    set i 0
    foreach w $words {
        dvcon_stream_word $w
        incr i
        if {$i % 20000 == 0} {
            set el [expr {([clock milliseconds]-$t0)/1000.0}]
            puts [format "    %d/%d words  %.0f%%  %.0f words/s" \
                    $i $nwords [expr {100.0*$i/$nwords}] [expr {$i/($el+0.001)}]]
        }
    }
    set el [expr {([clock milliseconds]-$t0)/1000.0}]
    puts [format "  done: %d words in %.1f s" $nwords $el]
    return $nwords
}

# Read back a few words and compare against the file, so a load is never
# assumed to have worked. A silently short or shifted load produces plausible
# garbage detections rather than an error.
proc dvcon_verify {path base nsample} {
    set fh [open $path rb]
    fconfigure $fh -translation binary
    set data [read $fh]
    close $fh
    set pad [expr {(4 - ([string length $data] % 4)) % 4}]
    if {$pad} { append data [string repeat \x00 $pad] }
    binary scan $data iu* words
    set nwords [llength $words]

    set bad 0
    for {set k 0} {$k < $nsample} {incr k} {
        set idx [expr {int(double($k)*($nwords-1)/($nsample-1))}]
        dvcon_poke $::REG_MEMADDR [expr {$base + $idx*4}]
        set got [dvcon_peek $::REG_MEMDATA]
        set exp [expr {[lindex $words $idx] & 0xFFFFFFFF}]
        if {$got != $exp} {
            incr bad
            puts [format "    MISMATCH word %d (0x%08X): got 0x%08X want 0x%08X" \
                    $idx [expr {$base+$idx*4}] $got $exp]
        }
    }
    puts [format "  verify: %d/%d sampled words match" \
            [expr {$nsample-$bad}] $nsample]
    return $bad
}

proc cmd_ident {} {
    dvcon_open
    set id [dvcon_peek $::REG_IDENT]
    dvcon_close
    set magic [expr {($id >> 24) & 0xFF}]
    set asize [expr {($id >> 16) & 0xFF}]
    set build [expr {$id & 0xFFFF}]

    puts [format "IDENT      0x%08X" $id]
    if {$magic != 0xDC} {
        puts "  magic is 0x[format %02X $magic], expected 0xDC --\
              is the right bitstream loaded?"
        exit 1
    }
    puts "  ARRAY_SIZE $asize"
    puts "  build id   $build"
    puts ""
    puts "  The model blob MUST have been exported for ARRAY_SIZE=$asize --"
    puts "  export_yolo26n.py emits weight tiles in the engine's loop order,"
    puts "  so a blob built for another value reads the wrong tile silently."
}

# The first question during bring-up is always whether the board is receiving
# anything at all. Four LED bits cannot answer it; these counters can.
proc cmd_link {} {
    dvcon_open
    set good [dvcon_peek $::REG_ETH_GOOD]
    set bad  [dvcon_peek $::REG_ETH_BAD]
    set cmd  [dvcon_peek $::REG_ETH_CMD]
    set bm   [dvcon_peek $::REG_ETH_BM]
    set filt [dvcon_peek $::REG_ETH_FILT]
    dvcon_close
    puts [format "frames with good FCS : %d" $good]
    puts [format "frames failing FCS   : %d" $bad]
    puts [format "command frames taken : %d" $cmd]
    puts [format "not addressed to us  : %d" $filt]
    puts [format "ACK bitmap (low 32)  : 0x%08X" $bm]
    puts ""
    if {$good == 0 && $bad == 0} {
        puts "Nothing has reached the MAC -- not one frame decoded, not even"
        puts "one addressed elsewhere. A PC on the far end broadcasts ARP and"
        puts "mDNS constantly, so with the link up and the PHY in MII the"
        puts "'not addressed to us' count would be climbing on its own."
        puts ""
        puts "That points at the PHY mode, not at this design: JP1 selects"
        puts "RGMII (pins 1-2, the factory default) or MII (pins 2-3), and only"
        puts "MII matches this RTL. Moving it needs a hardware reset to take."
        puts "Check the link LED on the RJ45 too."
        puts "and that the host is sending to the board's MAC address."
    } elseif {$bad > 0 && $good == 0} {
        puts "Frames are arriving but every one fails its FCS. That is a"
        puts "physical-layer problem -- MII timing or the PHY's mode straps --"
        puts "not a protocol one."
    } elseif {$good > 0 && $cmd == 0} {
        puts "Frames pass FCS but none is being accepted as a command."
        puts "Check the ethertype (0x88B5) and the destination MAC."
    }
}

proc cmd_run {} {
    dvcon_open

    # The blob starts with a 32-byte file header (magic, version, count, ...);
    # the descriptor table follows it. yolo_layer_sequencer does `desc_ptr <=
    # desc_base` and fetches immediately, with no header skip -- so pointing
    # this at MODEL_BASE would execute the header as descriptor 0.
    dvcon_poke $::REG_DESC_ADDR   [expr {$::MODEL_BASE + $::MODEL_HDR_BYTES}]
    dvcon_poke $::REG_IMG_ADDR    $::FRAME_BASE
    dvcon_poke $::REG_BOX_ADDR    $::BOXES_BASE
    dvcon_poke $::REG_CONF_THRESH 0x00000020

    # CTRL bit0 START, bit1 YOLO mode (descriptor walk, not microcode).
    dvcon_poke $::REG_CTRL 0x00000003

    puts "started; polling STATUS"
    for {set i 0} {$i < 2000} {incr i} {
        set st [dvcon_peek $::REG_STATUS]
        if {$st & 0x4} {
            puts "ERROR bit set (STATUS = 0x[format %08X $st])"
            dvcon_close
            exit 1
        }
        if {$st & 0x2} {
            set n [dvcon_peek $::REG_NUM_BOXES]
            puts "done: $n box(es)"
            dvcon_close
            return
        }
    }
    puts "timed out waiting for done -- STATUS = 0x[format %08X [dvcon_peek $::REG_STATUS]]"
    dvcon_close
    exit 1
}

# Boxes are Q12.4 -- 1/16 of a pixel -- so the engine never has to emit a float.
proc q4 {v} {
    # 16-bit signed
    if {$v >= 0x8000} { set v [expr {$v - 0x10000}] }
    return [format %.2f [expr {$v / 16.0}]]
}

proc cmd_boxes {} {
    dvcon_open
    set n [dvcon_peek $::REG_NUM_BOXES]

    if {$n == 0} {
        puts "no boxes"
        dvcon_close
        return
    }
    if {$n > 300} {
        puts "NUM_BOXES reads $n, which exceeds the 300-record buffer --\
              treating as 300"
        set n 300
    }

    # MEMADDR post-increments by 4 on each MEMDATA read, so the base is set
    # once and the list is walked by reading repeatedly.
    dvcon_poke $::REG_MEMADDR $::BOXES_BASE

    puts [format "%-4s %-9s %-9s %-9s %-9s %-6s %s" \
          "#" "x1" "y1" "x2" "y2" "score" "class"]
    for {set i 0} {$i < $n} {incr i} {
        set w0 [dvcon_peek $::REG_MEMDATA]   ;# y1:x1
        set w1 [dvcon_peek $::REG_MEMDATA]   ;# y2:x2
        set w2 [dvcon_peek $::REG_MEMDATA]   ;# pad:score
        set w3 [dvcon_peek $::REG_MEMDATA]   ;# stride:class

        set x1 [expr {$w0 & 0xFFFF}]
        set y1 [expr {($w0 >> 16) & 0xFFFF}]
        set x2 [expr {$w1 & 0xFFFF}]
        set y2 [expr {($w1 >> 16) & 0xFFFF}]
        set sc [expr {($w2 >> 16) & 0xFFFF}]
        if {$sc >= 0x8000} { set sc [expr {$sc - 0x10000}] }
        set cl [expr {$w3 & 0xFFFF}]

        puts [format "%-4d %-9s %-9s %-9s %-9s %-6d %d" \
              $i [q4 $x1] [q4 $y1] [q4 $x2] [q4 $y2] $sc $cl]
    }
    dvcon_close
}

proc main {} {
    set argv $::quartus(args)
    if {[llength $argv] < 1} {
        puts "usage: quartus_stp -t dvcon_jtag.tcl\
              {ident|link|run|boxes|load <file> <addr>|verify <file> <addr>|peek <reg>|poke <reg> <val>}"
        exit 2
    }

    switch -- [lindex $argv 0] {
        ident { cmd_ident }
        link  { cmd_link }
        run   { cmd_run }
        boxes { cmd_boxes }
        load  {
            dvcon_open
            dvcon_load [lindex $argv 1] [expr {[lindex $argv 2]}]
            dvcon_close
        }
        verify {
            dvcon_open
            set bad [dvcon_verify [lindex $argv 1] [expr {[lindex $argv 2]}] 64]
            dvcon_close
            if {$bad} { exit 1 }
        }
        peek  {
            dvcon_open
            set v [dvcon_peek [expr {[lindex $argv 1]}]]
            dvcon_close
            puts [format "0x%08X" $v]
        }
        poke  {
            dvcon_open
            dvcon_poke [expr {[lindex $argv 1]}] [expr {[lindex $argv 2]}]
            dvcon_close
            puts "ok"
        }
        default {
            puts "unknown command: [lindex $argv 0]"
            exit 2
        }
    }
}

# Run main only when this file IS the script. infprog/dvcon_link.py sources
# it as a library to reuse dvcon_open/peek/poke/load, and without this guard
# the source itself executed a command -- which on a machine with no cable
# attached failed before the caller's own code ever ran.
if {![info exists ::DVCON_NO_MAIN]} {
    main
}
