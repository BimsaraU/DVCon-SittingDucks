#!/usr/bin/env bash
# =============================================================================
# qrts/sim/run.sh -- platform-shim testbenches
#
# Runs in xsim, not Questa. Vivado 2025.2 is what is installed, Phase A already
# happens there, and it sidesteps the line limit in the Questa Starter edition
# bundled with Quartus Lite. Quartus is for synthesis, fitting and timing.
#
# The CORE testbenches (conv numerics, sequencer, ucode, elem) live in
# DVCFinal/ and are run by 'DVCFinal/build.sh sim' -- that is the upstream the
# porter pulls from, so the core is proven there and copied here, not re-proven.
# This script covers what is NEW on this platform: the shims that replace the
# Xilinx bus layer.
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
QRTS="$(dirname "$HERE")"
# xsim can keep a handle on the previous work directory on Windows, so each
# run gets its own and old ones are swept up best-effort.
WORK="$HERE/work.$$"

command -v xvlog >/dev/null || {
	echo "xvlog not on PATH; add Vivado's bin (e.g. /d/AMD/2025.2/Vivado/bin)"
	exit 1
}

rm -rf "$HERE"/work.* 2>/dev/null || true
mkdir -p "$WORK"; cd "$WORK"
trap 'cd "$HERE"; rm -rf "$WORK" 2>/dev/null || true' EXIT

fail=0

# ---- pin assignments vs the top-level port list ---------------------------
# Quartus treats an assignment to a nonexistent port as a WARNING and then
# leaves the real port unplaced, so a typo becomes a pin the fitter chooses.
# On an SDRAM data line that is a dead interface found on the bench.
echo "== pin assignments =="
python "$QRTS/tools/check_pins.py" || fail=1
echo

# ---- the DE2-115 top level elaborates -------------------------------------
# Not a functional test -- dvcon_top is still a skeleton -- but it proves the
# thing the .qsf names as TOP_LEVEL_ENTITY actually builds, which is what the
# first quartus_map would otherwise discover.
echo "== dvcon_top elaborates =="
# dvcon_top instantiates Accelerator_Top whole, so the whole core is needed --
# it is not a leaf any more.
# rtl/mem holds avalon_arbiter and sdram_ctrl, and sld_virtual_jtag is an
# Altera megafunction xsim does not have -- sim/ carries a stub for it. Both
# were missing here, so xelab failed, and because this step redirects to
# /dev/null under `set -e` the whole suite aborted with no message. Every
# testbench below it had stopped running.
xvlog -sv -i "$QRTS/rtl/core" "$QRTS"/rtl/core/*.sv "$QRTS"/rtl/platform/*.sv 	"$QRTS"/rtl/eth/*.sv "$QRTS"/rtl/mem/*.sv "$HERE/sld_virtual_jtag_stub.sv"
xelab work.dvcon_top -s dtop >/dev/null
echo "   ok"
echo

# ---- avalon_mm_master vs axi4_master ---------------------------------------
# The port's central claim is that no engine and no line of yolo_axi_arbiter
# changes, because the new master reproduces the old one's internal handshake.
# This is that claim as a test: identical stimulus into both masters, compare
# the beats returned and the bytes committed.
echo "== avalon_mm_master equivalence =="
xvlog -sv -i "$QRTS/rtl/core" \
	"$QRTS/rtl/core/axi4_master.sv" \
	"$QRTS/rtl/platform/avalon_mm_master.sv" \
	"$HERE/tb_avalon_master.sv" >/dev/null
xelab work.tb_avalon_master -s tbav >/dev/null
xsim tbav -R | tee tb_avalon_master.log
grep -q "PASSED" tb_avalon_master.log || { echo "  FAILED"; fail=1; }

# ---- register block -------------------------------------------------------
# dvcon_regs against the REAL yolo_axi4_slave_regs, checking the values that
# land in the datapath rather than the AXI waveform. The 64-bit slave selects
# the 32-bit half with WSTRB, so a write to an odd word arrives at the ALIGNED
# address with the upper strobe -- decoding that naively drops half the
# register map, which is the bug that cost hours in Stage 3A.
echo "== dvcon_regs =="
xvlog -sv -i "$QRTS/rtl/core"     "$QRTS/rtl/core/yolo_axi4_slave_regs.sv"     "$QRTS/rtl/platform/dvcon_regs.sv"     "$HERE/tb_dvcon_regs.sv" >/dev/null
xelab work.tb_dvcon_regs -s tbregs >/dev/null
xsim tbregs -R | tee tb_dvcon_regs.log
grep -q "PASSED" tb_dvcon_regs.log || { echo "  FAILED"; fail=1; }

echo
# ---- Ethernet -------------------------------------------------------------
# crc32_eth is checked against EXTERNAL known-answer vectors, not against
# another copy of the same algorithm: a wrong FCS is the worst kind of link
# bug, because frames leave the FPGA and the host NIC drops every one of them
# in hardware with nothing to see at either end.
echo "== ethernet CRC (known-answer vectors) =="
xvlog -sv "$QRTS/rtl/eth/crc32_eth.sv" "$HERE/tb_crc32_eth.sv" >/dev/null
xelab work.tb_crc32_eth -s tbcrc >/dev/null
xsim tbcrc -R | tee tb_crc32_eth.log
grep -q "PASSED" tb_crc32_eth.log || { echo "  FAILED"; fail=1; }

echo
# ---- ethernet writes under backpressure ------------------------------------
# tb_eth_cmd and tb_eth_path both hold avm_waitrequest LOW for the whole run,
# so neither ever exercised what happens to a packed word when the SDRAM is
# busy -- which on the board it often is. The engine used to overwrite the
# pending write, losing the word silently; this bench measured 212 of 518
# payload bytes never reaching memory. It is the regression for the write queue.
echo "== ethernet writes under backpressure =="
xvlog -sv "$QRTS"/rtl/eth/*.sv "$HERE/tb_eth_stall.sv" >/dev/null
xelab work.tb_eth_stall -s tbstall >/dev/null
xsim tbstall -R | tee tb_eth_stall.log
grep -q "PASSED" tb_eth_stall.log || { echo "  FAILED"; fail=1; }

echo
# ---- the whole descriptor read path ----------------------------------------
# tb_desc_fetch drives the AXI side from the bench, which leaves the half of the
# path inside the accelerator untested: two AXI translations and two arbiters.
# This walks a known ramp from yolo_layer_sequencer all the way to sdram_ctrl.
# It is what establishes that a wrong descriptor on the board is NOT a logic bug.
echo "== descriptor path, sequencer to SDRAM =="
xvlog -sv -i "$QRTS/rtl/core" "$QRTS"/rtl/core/*.sv 	"$QRTS/rtl/platform/avalon_mm_master.sv" 	"$QRTS/rtl/platform/axi_to_avalon.sv" 	"$QRTS"/rtl/mem/*.sv "$HERE/sdram_model.sv" 	"$HERE/tb_desc_path.sv" >/dev/null
xelab work.tb_desc_path -s tbdpath >/dev/null
xsim tbdpath -R | tee tb_desc_path.log
grep -q "PASSED" tb_desc_path.log || { echo "  FAILED"; fail=1; }

echo
echo "== ethernet MAC loopback =="
xvlog -sv "$QRTS/rtl/eth/crc32_eth.sv"           "$QRTS/rtl/eth/eth_mac_rx.sv"           "$QRTS/rtl/eth/eth_mac_tx.sv"           "$HERE/tb_eth_loopback.sv" >/dev/null
xelab work.tb_eth_loopback -s tbeth >/dev/null
xsim tbeth -R | tee tb_eth_loopback.log
grep -q "PASSED" tb_eth_loopback.log || { echo "  FAILED"; fail=1; }

echo
if [ "$fail" -ne 0 ]; then
	echo "== platform sim FAILED =="
	exit 1
fi
echo "== platform sim passed =="
