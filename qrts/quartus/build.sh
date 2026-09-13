#!/usr/bin/env bash
# =============================================================================
# qrts/quartus/build.sh - synthesise, fit, time and assemble the bitstream
#
#     bash build.sh          full flow -> output_files/dvcon.sof (+ .rbf)
#     bash build.sh map      Analysis & Synthesis only, then the size check
#     bash build.sh report   summaries and worst slack from the last run
#
# The pin check runs first: Quartus treats an assignment to a missing port as
# a warning and places the real port anywhere, and it cannot tell a legal pin
# from the right one. check_pins.py compares against the Terasic table.
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

QUARTUS_BIN="${QUARTUS_BIN:-/d/qrtus/quartus/bin64}"
if ! command -v quartus_sh >/dev/null 2>&1; then
    [ -x "$QUARTUS_BIN/quartus_sh.exe" ] || { echo "quartus_sh not found; set QUARTUS_BIN"; exit 1; }
    export PATH="$QUARTUS_BIN:$PATH"
fi
PY="${PY:-python}"

# "successful, 0 errors" from synthesis says nothing about fitting the part
check_size() {
    local s="output_files/dvcon.map.summary"
    [ -f "$s" ] || return 0
    local le mult
    le=$(sed -n 's/.*Total logic elements *: *\([0-9,]*\).*/\1/p' "$s" | head -1 | tr -d ,)
    mult=$(sed -n 's/.*Embedded Multiplier 9-bit elements *: *\([0-9,]*\).*/\1/p' "$s" | head -1 | tr -d ,)
    [ -n "$le" ]   && echo "  logic elements : $le / 114480 ($((le * 100 / 114480))%)"
    [ -n "$mult" ] && echo "  9-bit mults    : $mult / 532 ($((mult * 100 / 532))%)"
    if [ -n "$le" ] && [ "$le" -gt 114480 ]; then echo "  DOES NOT FIT"; return 1; fi
    if [ -n "$mult" ] && [ "$mult" -gt 532 ]; then echo "  DOES NOT FIT"; return 1; fi
}

report() {
    for f in map fit sta asm; do
        s="output_files/dvcon.$f.summary"
        [ -f "$s" ] && { echo; echo "=== $f ==="; cat "$s"; }
    done
    # An unconstrained clock looks exactly like a clean one: grep for it.
    grep -hE "332049|332174|332060" output_files/*.rpt 2>/dev/null | head -5 || true
}

case "${1:-all}" in
    map)
        "$PY" ../tools/check_pins.py
        quartus_map dvcon
        check_size
        ;;
    report)
        report
        ;;
    all)
        "$PY" ../tools/check_pins.py
        quartus_sh --flow compile dvcon
        check_size
        report
        if [ -f output_files/dvcon.sof ]; then
            echo
            echo "== BITSTREAM: $HERE/output_files/dvcon.sof =="
            echo "   quartus_pgm -c 1 -m jtag -o \"p;output_files/dvcon.sof\""
        else
            echo "== NO BITSTREAM PRODUCED =="; exit 1
        fi
        ;;
    *) echo "usage: build.sh [all|map|report]"; exit 2 ;;
esac
