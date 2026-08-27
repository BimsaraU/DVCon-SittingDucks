#!/usr/bin/env bash
# =============================================================================
# qrts/sim/run_all.sh — every platform bench, in one command
#
#     bash sim/run_all.sh
#
# Compiles and runs each testbench from scratch. Building from scratch is the
# point: more than one wrong conclusion in this project came from reading the
# output of a stale snapshot after an edit that failed to apply.
#
# The accelerator core has its own suite -- `bash DVCFinal/build.sh sim` --
# which covers the conv numerics and the end-to-end descriptor walk.
# =============================================================================
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
QRTS="$(cd "$HERE/.." && pwd)"
cd "$HERE"

RTL="$QRTS/rtl"

# name : work dir : extra sources beyond the testbench itself
BENCHES=(
  "tb_sdram|wsdram|$HERE/sdram_model.sv $RTL/mem/sdram_ctrl.sv"
  "tb_arbiter|warb|$HERE/sdram_model.sv $RTL/mem/sdram_ctrl.sv $RTL/mem/avalon_arbiter.sv"
  "tb_jtag_ctrl|wjtag|$HERE/sld_virtual_jtag_stub.sv $RTL/platform/jtag_ctrl.sv"
  "tb_mii_adapters|wmii|$RTL/eth/mii_rx_adapter.sv $RTL/eth/mii_tx_adapter.sv"
  "tb_eth_cmd|wcmd|$RTL/eth/eth_cmd_engine.sv"
  "tb_eth_path|wpath|$RTL/eth/crc32_eth.sv $RTL/eth/eth_mac_rx.sv $RTL/eth/eth_mac_tx.sv $RTL/eth/mii_rx_adapter.sv $RTL/eth/mii_tx_adapter.sv $RTL/eth/eth_cmd_engine.sv"
  "tb_avalon_master|wavm|$RTL/platform/avalon_mm_master.sv $RTL/core/axi4_master.sv"
  "tb_dvcon_regs|wregs|$RTL/platform/dvcon_regs.sv"
  "tb_crc32_eth|wcrc|$RTL/eth/crc32_eth.sv"
  "tb_eth_loopback|weth|$RTL/eth/crc32_eth.sv $RTL/eth/eth_mac_rx.sv $RTL/eth/eth_mac_tx.sv"
)

fail=0
pass=0
skipped=""

# The host's frame layout, checked against the offsets the RTL actually reads.
# dvcon_host.c itself cannot be built on every machine -- AF_PACKET is Linux
# only -- but its byte layout is pure arithmetic, and a mismatch there means
# the FPGA parses a valid frame as garbage and drops it with no error anywhere.
if command -v cc >/dev/null 2>&1 || command -v gcc >/dev/null 2>&1; then
    CC=$(command -v cc || command -v gcc)
    if "$CC" -O2 -Wall -Wextra -o "$QRTS/sw/test_frame"              "$QRTS/sw/test_frame.c" > "$HERE/test_frame.log" 2>&1; then
        if out=$("$QRTS/sw/test_frame" 2>&1); then
            printf "  %-20s %s
" "test_frame"                    "$(echo "$out" | grep -E '^=== ' | tail -1)"
            pass=$((pass + 1))
        else
            printf "  %-20s FAILED
" "test_frame"
            echo "$out" | grep "  FAIL" | head -5
            fail=$((fail + 1))
        fi
    else
        printf "  %-20s BUILD FAILED  see sim/test_frame.log
" "test_frame"
        fail=$((fail + 1))
    fi
else
    skipped="$skipped test_frame(no-compiler)"
fi

# The JTAG control script only ever runs under quartus_stp with a board
# attached, so a typo in it is normally found at the bench. This runs all four
# subcommands against stubbed TAP commands.
if command -v tclsh >/dev/null 2>&1; then
    if out=$(tclsh "$QRTS/tools/check_jtag_tcl.tcl" 2>&1); then
        printf "  %-20s %s
" "dvcon_jtag.tcl"                "$(echo "$out" | grep -E '^=== ' | tail -1)"
        pass=$((pass + 1))
    else
        printf "  %-20s FAILED
" "dvcon_jtag.tcl"
        echo "$out" | grep "  FAIL" | head -5
        fail=$((fail + 1))
    fi
else
    skipped="$skipped dvcon_jtag(no-tclsh)"
fi

for entry in "${BENCHES[@]}"; do
    IFS='|' read -r name dir srcs <<< "$entry"

    if [ ! -f "$HERE/$name.sv" ]; then
        skipped="$skipped $name"
        continue
    fi

    mkdir -p "$dir"
    (
        cd "$dir"
        # shellcheck disable=SC2086
        xvlog -sv $srcs "$HERE/$name.sv" > compile.log 2>&1 &&
        xelab "work.$name" -s "snap_$name" > elab.log 2>&1 &&
        xsim "snap_$name" -R > run.log 2>&1
    )

    result=$(grep -hE "^=== (PASSED|FAILED)" "$dir/run.log" 2>/dev/null | head -1)

    if [ -z "$result" ]; then
        # No verdict line: a compile or elaboration failure, or a timeout.
        err=$(grep -hE "^ERROR" "$dir"/compile.log "$dir"/elab.log 2>/dev/null | head -1)
        printf "  %-20s BUILD FAILED  %s\n" "$name" "${err:-see $dir/}"
        fail=$((fail + 1))
    elif echo "$result" | grep -q PASSED; then
        printf "  %-20s %s\n" "$name" "$result"
        pass=$((pass + 1))
    else
        printf "  %-20s %s\n" "$name" "$result"
        grep -h "  FAIL" "$dir/run.log" 2>/dev/null | head -5
        fail=$((fail + 1))
    fi
done

echo
[ -n "$skipped" ] && echo "  not present:$skipped"
echo "  $pass passed, $fail failed"
[ "$fail" -eq 0 ] || exit 1
