#!/usr/bin/env bash
# =============================================================================
# qrts/quartus/build.sh — synthesise, fit, time, and assemble a bitstream
#
#     bash build.sh              full flow -> output_files/dvcon.sof
#     bash build.sh map          Analysis & Synthesis only (fast, catches
#                                syntax and elaboration errors)
#     bash build.sh report       print the summaries from the last run
#
# Produces output_files/dvcon.sof (JTAG) and dvcon.rbf (raw binary, for the
# EPCS/AS configuration device).
#
# ---------------------------------------------------------------------------
# WHAT THE BITSTREAM DOES, AND DOES NOT DO
# ---------------------------------------------------------------------------
# It configures, it fits, and its timing report is real. It does NOT run the
# network: the Qsys SDRAM system and eth_cmd_engine do not exist yet, so the
# accelerator is wired to a 16 KB on-chip RAM and started by a small boot
# sequencer instead of by a host over Ethernet.
#
# That is deliberate rather than a shortcut. A design whose memory port is
# tied off gets deleted by constant propagation -- an earlier run reported
# 5000 registers removed and ZERO multipliers, i.e. an empty chip whose
# resource and timing numbers described nothing. Giving it a real (if small)
# memory and a real reason to run keeps the measured design and the intended
# design the same shape.
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

QUARTUS_BIN="${QUARTUS_BIN:-/d/qrtus/quartus/bin64}"
if ! command -v quartus_sh >/dev/null 2>&1; then
    if [ -x "$QUARTUS_BIN/quartus_sh.exe" ]; then
        export PATH="$QUARTUS_BIN:$PATH"
    else
        echo "quartus_sh not on PATH and not at $QUARTUS_BIN"
        echo "set QUARTUS_BIN to the directory holding quartus_sh.exe"
        exit 1
    fi
fi

# The activation tables are read by $readmemh at elaboration. They are
# generated, not checked in, because Quartus will not fold $exp at elaboration
# the way Vivado does -- a self-initialising ROM comes up undefined on this
# device.
regen_tables() {
    python "$HERE/../../DVCFinal/tools/gen_silu_mif.py"    --outdir "$HERE" >/dev/null
    python "$HERE/../../DVCFinal/tools/gen_softmax_mem.py" --outdir "$HERE" >/dev/null
}

# Analysis & Synthesis says "successful, 0 errors" for a design four times too
# big for the part -- an unfittable netlist is not a synthesis error. The run
# that first survived constant propagation reported 462,600 logic elements
# against 114,480, because Accelerator_Top fixed ARRAY_SIZE at 24 in a
# localparam and dvcon_top's 16 never reached it. Reading the error count
# would have called that a pass.
check_size() {
    local s="output_files/dvcon.map.summary"
    [ -f "$s" ] || return 0

    # "Total logic elements : 462,600" -> 462600
    local le mult
    le=$(sed -n 's/.*Total logic elements *: *\([0-9,]*\).*/\1/p' "$s" | head -1 | tr -d ,)
    mult=$(sed -n 's/.*Embedded Multiplier 9-bit elements *: *\([0-9,]*\).*/\1/p' "$s" | head -1 | tr -d ,)

    # EP4CE115F29C7.
    local LE_AVAIL=114480 MULT_AVAIL=532 bad=0

    if [ -n "$le" ]; then
        echo "  logic elements : $le / $LE_AVAIL ($((le * 100 / LE_AVAIL))%)"
        [ "$le" -gt "$LE_AVAIL" ] && bad=1
    fi
    if [ -n "$mult" ]; then
        echo "  9-bit mults    : $mult / $MULT_AVAIL ($((mult * 100 / MULT_AVAIL))%)"
        [ "$mult" -gt "$MULT_AVAIL" ] && bad=1
    fi

    if [ "$bad" = 1 ]; then
        echo
        echo "  DOES NOT FIT EP4CE115F29C7 -- the fitter will fail."
        echo "  Check that ARRAY_SIZE reached Accelerator_Top: dvcon_top passes"
        echo "  it at the instantiation, and nothing downstream may redeclare it."
        return 1
    fi
    return 0
}

report() {
    for f in map fit sta asm; do
        s="output_files/dvcon.$f.summary"
        [ -f "$s" ] || continue
        echo
        echo "=== $f ==="
        cat "$s"
    done

    # Timing is the number that decides whether this can ever run at speed.
    # The Xilinx build missed 50 MHz by 21.5 ns on a FASTER fabric, so a
    # negative slack here is expected until the conv engine's long paths are
    # dealt with -- but it has to be read, not assumed.
    if [ -f output_files/dvcon.sta.rpt ]; then
        echo
        echo "=== worst-case slack ==="
        grep -A4 -iE "Slow.*Model.*Setup Summary|Worst-case Setup Slack" \
            output_files/dvcon.sta.rpt 2>/dev/null | head -12 || true
    fi
}

case "${1:-all}" in
    map)
        regen_tables
        echo "== Analysis & Synthesis =="
        quartus_map dvcon
        sed -n '1,14p' output_files/dvcon.map.summary
        check_size
        ;;

    report)
        report
        ;;

    all)
        regen_tables

        # Pin assignments are checked BEFORE the fitter spends twenty minutes
        # discovering the same thing. Quartus treats an assignment to a
        # nonexistent port as a warning and then leaves the real port unplaced,
        # and it has nothing at all to say about a pin that is legal but wired
        # to the wrong peripheral. check_pins.py compares every location to the
        # Terasic pin table in resources/, so both classes fail here.
        #
        # There is no override. The DVCON_ALLOW_UNPINNED escape hatch existed
        # while DRAM_DQ[28] and [31] were unassigned; it produced a .sof that
        # was safe to measure and unsafe to program, which is a distinction the
        # file itself does not carry. Those pins are resolved, so the hatch is
        # gone -- if this check fails again, the pinout is wrong and the
        # bitstream is not worth measuring either.
        echo "== pin check =="
        python "$HERE/../tools/check_pins.py" || exit 1
        echo

        echo "== full compile (map -> fit -> sta -> asm) =="
        quartus_sh --flow compile dvcon

        report

        echo
        if [ -f output_files/dvcon.sof ]; then
            echo "== BITSTREAM: $(cd output_files && pwd)/dvcon.sof =="
            ls -la output_files/dvcon.sof output_files/dvcon.rbf 2>/dev/null || true
            echo
            echo "   Program over JTAG with:"
            echo "     quartus_pgm -m jtag -o \"p;output_files/dvcon.sof\""
            echo
            echo "   It will configure and sit idle: no SDRAM controller and no"
            echo "   command engine yet, so it cannot run a frame."
        else
            echo "== NO BITSTREAM PRODUCED -- see the summaries above =="
            exit 1
        fi
        ;;

    *)
        echo "usage: build.sh [all|map|report]"
        exit 2
        ;;
esac
