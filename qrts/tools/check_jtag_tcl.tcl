# =============================================================================
# check_jtag_tcl.tcl — parse dvcon_jtag.tcl without a board attached
#
#     tclsh tools/check_jtag_tcl.tcl
#
# dvcon_jtag.tcl only ever runs under quartus_stp with a USB-Blaster plugged
# in, which means a typo in it -- a misspelled variable, an unbalanced brace, a
# proc that does not exist -- is discovered at the bench, with hardware powered
# and a board in front of you. That is the most expensive place to find it.
#
# This stubs the handful of Quartus TAP commands and runs each subcommand, so
# the script is at least exercised end to end. It cannot check that the JTAG
# transactions are CORRECT -- only tb_jtag_ctrl and a real board can do that --
# but it does check that the script runs.
# =============================================================================

set stub_reads 0

proc get_hardware_names {} { return [list {USB-Blaster [USB-0]}] }
proc get_device_names {args} { return [list {@1: EP4CE115F29 (0x020F70DD)}] }
proc open_device {args} {}
proc close_device {args} {}
# dvcon_open takes exclusive access before any virtual shift; without
# these stubs the script dies here with "invalid command name".
proc device_lock {args} {}
proc device_unlock {args} {}
proc device_virtual_ir_shift {args} {}

# Track which register the ADDR scan selected, so the DATA scan can return
# something the script will accept. The ADDR scan is 8 bits and the DATA scan
# 32, which is how they are told apart.
set stub_reg -1

proc device_virtual_dr_shift {args} {
    global stub_reg
    set len 32
    set val ""
    for {set i 0} {$i < [llength $args]} {incr i} {
        switch -- [lindex $args $i] {
            -length   { set len [lindex $args [expr {$i+1}]] }
            -dr_value { set val [lindex $args [expr {$i+1}]] }
        }
    }

    if {$len == 8} {
        # {write:1, addr:6, pad:1} in the top byte of the scanned value.
        set v [expr {"0x$val"}]
        set stub_reg [expr {($v >> 1) & 0x3F}]
        return "00"
    }

    # DATA scan. IDENT must carry the 0xDC magic or cmd_ident exits; STATUS
    # must have the done bit set or cmd_run polls forever.
    # These indices must track the REG_* constants in dvcon_jtag.tcl. IDENT is
    # word 0x0C (byte 0x30); it was 15 here, matching the 0x0F the script used
    # to send, so this check kept passing while the real read went to the wrong
    # register on hardware.
    switch -- $stub_reg {
        12      { return "DC100001" }
        57      { return "00000000" }
        1       { return "00000002" }
        10      { return "00000003" }
        default { return "00000004" }
    }
}

set here [file dirname [file normalize [info script]]]
set script [file join $here dvcon_jtag.tcl]

set failures 0
foreach cmd {ident link run boxes} {
    set ::quartus(args) [list $cmd]
    set ::stub_reads 0
    # `run` polls STATUS until done; the stub returns 2, which has the done bit
    # set, so it terminates on the first pass.
    if {[catch {source $script} err]} {
        # exit inside the sourced script raises this; treat a clean exit as a
        # pass, anything else as a failure.
        if {[string match "*invalid command*" $err] ||
            [string match "*no such variable*" $err] ||
            [string match "*syntax*" $err] ||
            [string match "*wrong # args*" $err]} {
            puts "  FAIL $cmd : $err"
            incr failures
        } else {
            puts "  ok   $cmd (exited: $err)"
        }
    } else {
        puts "  ok   $cmd"
    }
}

if {$failures} {
    puts "=== FAILED: $failures error(s) ==="
    exit 1
}
puts "=== PASSED: dvcon_jtag.tcl parses and runs ==="
