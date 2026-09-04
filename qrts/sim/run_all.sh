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
  "tb_desc_fetch|wdf|$HERE/sdram_model.sv $RTL/mem/sdram_ctrl.sv $RTL/mem/avalon_arbiter.sv $RTL/platform/avalon_mm_master.sv $RTL/platform/axi_to_avalon.sv"
  "tb_jtag_ctrl|wjtag|$HERE/sld_virtual_jtag_stub.sv $RTL/platform/jtag_ctrl.sv"
  "tb_mii_adapters|wmii|$RTL/eth/mii_rx_adapter.sv $RTL/eth/mii_tx_adapter.sv"
  "tb_eth_cmd|wcmd|$RTL/eth/eth_cmd_engine.sv"
  "tb_eth_path|wpath|$RTL/eth/crc32_eth.sv $RTL/eth/eth_mac_rx.sv $RTL/eth/eth_mac_tx.sv $RTL/eth/mii_rx_adapter.sv $RTL/eth/mii_tx_adapter.sv $RTL/eth/eth_cmd_engine.sv"
  "tb_fast_slave|wfast|$RTL/platform/avalon_mm_master.sv"
  "tb_elem_write|welem|$HERE/sdram_model.sv $RTL/mem/sdram_ctrl.sv $RTL/mem/avalon_arbiter.sv $RTL/platform/avalon_mm_master.sv $RTL/platform/axi_to_avalon.sv $RTL/core/axi4_master.sv"
  "tb_avalon_master|wavm|$RTL/platform/avalon_mm_master.sv $RTL/core/axi4_master.sv"
  "tb_dvcon_regs|wregs|$RTL/platform/dvcon_regs.sv"
  "tb_crc32_eth|wcrc|$RTL/eth/crc32_eth.sv"
  "tb_yolo_arbiter|wyarb|$RTL/core/yolo_axi_arbiter.sv"
  "tb_eth_loopback|weth|$RTL/eth/crc32_eth.sv $RTL/eth/eth_mac_rx.sv $RTL/eth/eth_mac_tx.sv"
  # Writes under backpressure. tb_eth_cmd and tb_eth_path both hold
  # avm_waitrequest LOW, so neither could ever see the engine drop a word when
  # the SDRAM is busy -- 212 of 518 payload bytes went missing before the fix.
  "tb_eth_stall|wstall|$RTL/eth/eth_cmd_engine.sv"
  # The descriptor path end to end, sequencer to SDRAM. tb_desc_fetch drives
  # the AXI side from the bench and so skips both AXI translations and the
  # accelerator-internal arbiter; this covers them.
  "tb_desc_path|wdpath|$HERE/sdram_model.sv $RTL/mem/sdram_ctrl.sv $RTL/mem/avalon_arbiter.sv $RTL/platform/avalon_mm_master.sv $RTL/platform/axi_to_avalon.sv $RTL/core/*.sv"
)

fail=0
pass=0
skipped=""

# Files under rtl/core are GENERATED from DVCFinal/rtl by tools/port_rtl.py.
# Editing the generated copy is silently undone the next time the porter runs,
# which has already cost two fixes: an arbiter change and the accelerator's
# debug ports both vanished mid-session and surfaced later as a regression and
# a build failure. --check reports any generated file that no longer matches
# what the source would produce, so the drift is caught while the edit still
# exists rather than after it is lost.
if drift=$(python "$QRTS/tools/port_rtl.py" --check 2>&1 | grep "would update"); then
    printf "  %-20s FAILED  generated files edited by hand:
" "port_rtl drift"
    echo "$drift" | sed 's/^/      /'
    echo "      -> move the change into DVCFinal/rtl and re-run tools/port_rtl.py"
    fail=1
else
    printf "  %-20s === PASSED: rtl/core matches DVCFinal/rtl ===
" "port_rtl drift"
fi

# The memory map and register offsets, cross-checked against the RTL that
# implements them. IDENT was wrong here for a while -- 0x0F when dvcon_regs
# answers it at word 0x0C -- and nothing caught it, because every other copy of
# the number agreed with the wrong one.
if out=$(python "$QRTS/tools/gen_memmap.py" --check 2>&1); then
    printf "  %-20s === PASSED: %s ===
" "memmap"            "$(echo "$out" | tail -1 | sed 's/^ *//')"
    pass=$((pass + 1))
else
    printf "  %-20s FAILED
" "memmap"
    echo "$out" | sed 's/^/      /' | head -6
    fail=$((fail + 1))
fi

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
        # -i rtl/core: the core files `include "ucode_pkg.svh" by bare name.
        xvlog -sv -i "$RTL/core" $srcs "$HERE/$name.sv" > compile.log 2>&1 &&
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
