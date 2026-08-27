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
set REG_IDENT       0x0F
set REG_MEMADDR     0x3E
set REG_MEMDATA     0x3F

# Link diagnostics, answered from wires inside jtag_ctrl rather than through
# the accelerator's register block.
set REG_ETH_GOOD    0x3A
set REG_ETH_BAD     0x3B
set REG_ETH_CMD     0x3C
set REG_ETH_BM      0x3D

# Kept in step with tools/gen_memmap.py. Only the values this script actually
# writes are duplicated here; quartus_stp's Tcl cannot include the C header.
set MODEL_BASE  0x00000000
set FRAME_BASE  0x00400000
set BOXES_BASE  0x00600000

proc dvcon_open {} {
    set hw ""
    foreach h [get_hardware_names] {
        if {[string match "*USB-Blaster*" $h]} { set hw $h; break }
    }
    if {$hw eq ""} {
        error "no USB-Blaster found -- check the cable and that no other\
               Quartus tool holds it open"
    }

    set dev ""
    foreach d [get_device_names -hardware_name $hw] {
        if {[string match "*EP4CE115*" $d]} { set dev $d; break }
    }
    if {$dev eq ""} {
        error "no EP4CE115 on $hw -- is the board powered and configured?"
    }

    open_device -hardware_name $hw -device_name $dev
    return [list $hw $dev]
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
    # The read is issued at the ADDR scan's Update_DR and has to cross into the
    # system clock, run on Avalon, and cross back before DATA is scanned. A few
    # idle scans give it that time; the crossing is a two-flop synchroniser, so
    # this is microseconds of slack against nanoseconds of need.
    device_virtual_ir_shift -instance_index 0 -ir_value 1 -no_captured_ir_value
    set r [device_virtual_dr_shift -instance_index 0 -length 32 \
              -dr_value 00000000 -value_in_hex]
    return [expr {"0x$r"}]
}

proc cmd_ident {} {
    dvcon_open
    set id [dvcon_peek $::REG_IDENT]
    close_device

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
    close_device

    puts [format "frames with good FCS : %d" $good]
    puts [format "frames failing FCS   : %d" $bad]
    puts [format "command frames taken : %d" $cmd]
    puts [format "ACK bitmap (low 32)  : 0x%08X" $bm]
    puts ""
    if {$good == 0 && $bad == 0} {
        puts "Nothing has reached the MAC. Check the cable, the PHY link LED,"
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

    dvcon_poke $::REG_DESC_ADDR   $::MODEL_BASE
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
            close_device
            exit 1
        }
        if {$st & 0x2} {
            set n [dvcon_peek $::REG_NUM_BOXES]
            puts "done: $n box(es)"
            close_device
            return
        }
    }
    puts "timed out waiting for done -- STATUS = 0x[format %08X [dvcon_peek $::REG_STATUS]]"
    close_device
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
        close_device
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
    close_device
}

proc main {} {
    set argv $::quartus(args)
    if {[llength $argv] < 1} {
        puts "usage: quartus_stp -t dvcon_jtag.tcl\
              {ident|link|run|boxes|peek <reg>|poke <reg> <val>}"
        exit 2
    }

    switch -- [lindex $argv 0] {
        ident { cmd_ident }
        link  { cmd_link }
        run   { cmd_run }
        boxes { cmd_boxes }
        peek  {
            dvcon_open
            set v [dvcon_peek [expr {[lindex $argv 1]}]]
            close_device
            puts [format "0x%08X" $v]
        }
        poke  {
            dvcon_open
            dvcon_poke [expr {[lindex $argv 1]}] [expr {[lindex $argv 2]}]
            close_device
            puts "ok"
        }
        default {
            puts "unknown command: [lindex $argv 0]"
            exit 2
        }
    }
}

main
