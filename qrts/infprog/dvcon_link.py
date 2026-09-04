#!/usr/bin/env python3
"""dvcon_link.py -- the board-facing half of the inference tooling.

Everything here runs one Tcl script under quartus_stp, because the USB-Blaster
is only reachable through Quartus's own TAP driver. There is no socket and no
daemon: each operation opens the cable, does its work and closes it, so a crash
here never leaves the cable locked against the programmer.

Why not sw/dvcon_host.c: that talks raw L2 Ethernet and needs Linux plus a PHY
in MII mode. This path needs neither -- it uses the JTAG memory window, which
works on Windows with nothing but Quartus installed. It is slower (measured
~2440 words/s, so ~4 minutes for the 2.46 MB model) and that is the whole cost.

The register numbers are WORD addresses. dvcon_regs decodes byte_off =
avs_address*4, so IDENT at byte 0x30 is word 0x0C. Getting that wrong reads a
different register and reports "wrong bitstream loaded", which is what it did.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
QRTS = HERE.parent
JTAG_TCL = QRTS / "tools" / "dvcon_jtag.tcl"

# Word addresses into dvcon_regs (byte offset / 4).
REG_CTRL, REG_STATUS = 0x00, 0x01
REG_DESC_ADDR, REG_IMG_ADDR, REG_BOX_ADDR = 0x06, 0x07, 0x08
REG_CONF, REG_NUM_BOXES, REG_LAYER_IDX, REG_IDENT = 0x09, 0x0A, 0x0B, 0x0C
REG_ETH_FILT, REG_ETH_GOOD, REG_ETH_BAD = 0x39, 0x3A, 0x3B
REG_DBG_SRC, REG_DBG_DST = 0x37, 0x38
# Descriptor readback window: poke DSEL with an index 0..15, peek DVAL to get
# the word the sequencer actually LATCHED there. REG_ETH_DROP counts words the
# Ethernet write queue could not accept -- nonzero means SDRAM has holes.
REG_DBG_DSEL, REG_DBG_DVAL, REG_ETH_DROP = 0x34, 0x35, 0x36
# SDRAM read capture tap. 1 is the nominal CAS latency and is what reset picks;
# 0..3 walk the capture one cycle either side. Whether the nominal value is
# right is a property of the FITTED design -- quartus/dvcon.sdc works out that
# the budget is only 4.6 ns at the pin before routing -- so it is measured, not
# assumed. See sdram_tap_sweep().
REG_SD_TAP = 0x33
# Conv engine internals: CV0 = {beat_cnt[15:0], 3'b0, state[4:0]},
# CV1 = {oc_tile[15:0], ic_tile[15:0]}. Reading these twice a second apart
# separates "waiting on memory" (nothing moves) from "looping" (beat_cnt or
# the tile counters advance).
REG_DBG_CV0, REG_DBG_CV1 = 0x32, 0x31
# Elem engine state, and the accelerator-internal arbiter's grant + lock.
REG_DBG_EE, REG_DBG_ARB = 0x30, 0x2F
ELEM_STATE = {0: "IDLE"}
CONV_STATE = {0:"IDLE",1:"WLOAD",2:"BLOAD",3:"CFG",4:"CFG_W",5:"ACT",6:"ACT_W",
              7:"RUN",8:"ACCUM",9:"REQ",10:"STORE",11:"STORE_W",12:"NEXT",
              13:"DONE",14:"ERR",15:"SILU",16:"REQ_M"}
REG_ETH_CMD, REG_ETH_BM = 0x3C, 0x3D
REG_MEMADDR, REG_MEMDATA = 0x3E, 0x3F

# Mirrors sw/dvcon_memmap.h. Kept here rather than parsed so the dashboard can
# run without the C header, but it is the same map the RTL and host agree on.
MODEL_BASE, MODEL_SIZE = 0x00000000, 4 * 1024 * 1024
MODEL_HDR_BYTES = 32          # magic/version/count/... then the descriptor table
FRAME_BASE, FRAME_SIZE = 0x00400000, 2 * 1024 * 1024
BOXES_BASE = 0x00600000
ARENA_BASE = 0x00800000

EXPECTED_MAGIC = 0xDC
EXPECTED_ARRAY_SIZE = 16


class LinkError(RuntimeError):
    """A cable/tool failure, as opposed to the board answering with bad data."""


def find_quartus_stp() -> str:
    """Locate quartus_stp without assuming it is on PATH.

    Quartus installs are rarely on PATH on Windows, and the failure mode when it
    is missing is a bare FileNotFoundError from subprocess that says nothing
    useful about what to install.
    """
    env = os.environ.get("QUARTUS_STP")
    if env and Path(env).exists():
        return env

    found = shutil.which("quartus_stp") or shutil.which("quartus_stp.exe")
    if found:
        return found

    for base in (r"D:\qrtus", r"C:\intelFPGA_lite", r"C:\altera", r"C:\intelFPGA"):
        root = Path(base)
        if not root.exists():
            continue
        for exe in root.glob("**/quartus/bin64/quartus_stp.exe"):
            return str(exe)

    raise LinkError(
        "quartus_stp not found. Set the QUARTUS_STP environment variable to its "
        "full path, or add Quartus's bin64 directory to PATH."
    )


@dataclass
class Result:
    ok: bool
    stdout: str = ""
    error: str = ""
    data: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {"ok": self.ok, "stdout": self.stdout, "error": self.error,
                **self.data}


def run_tcl(body: str, timeout: int = 1800, env_extra: dict | None = None) -> Result:
    """Run a Tcl fragment with dvcon_jtag.tcl's procs already defined.

    The fragment is written to a temp file rather than passed on the command
    line: quartus_stp's argument handling mangles braces, and every useful
    fragment is full of them.
    """
    stp = find_quartus_stp()

    # DVCON_NO_MAIN keeps the sourced script from running a command of its own.
    # Without it, sourcing executed `ident`, which on a machine with no cable
    # attached failed before this fragment ever ran -- so every operation
    # reported the same opaque Tcl error regardless of what it was asked to do.
    script = (
        "set ::DVCON_NO_MAIN 1\n"
        "set ::quartus(args) [list]\n"
        f"source {{{JTAG_TCL.as_posix()}}}\n"
        f"{body}\n"
    )

    with tempfile.NamedTemporaryFile("w", suffix=".tcl", delete=False,
                                     encoding="utf-8") as fh:
        fh.write(script)
        path = fh.name

    env = dict(os.environ)
    if env_extra:
        env.update(env_extra)

    try:
        proc = subprocess.run([stp, "-t", path], capture_output=True, text=True,
                              timeout=timeout, cwd=str(QRTS), env=env)
    except subprocess.TimeoutExpired:
        return Result(False, error=f"quartus_stp timed out after {timeout}s")
    except OSError as exc:
        return Result(False, error=f"could not run quartus_stp: {exc}")
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass

    # Quartus prints the framed error block on stderr and the useful output on
    # stdout, so both have to be considered: parsing stdout alone left every
    # failure reading as the bare "evaluation unsuccessful" wrapper.
    out = (proc.stdout or "") + "\n" + (proc.stderr or "")
    # Quartus prints a 20-line licence banner before anything useful.
    lines = [ln for ln in out.splitlines()
             if not ln.startswith(("Info:", "    Info:", "Info (", "Warning ("))]
    trimmed = "\n".join(lines).strip()

    if "Error (23031)" in out or "Error (23018)" in out:
        # The 23031 wrapper says only "evaluation unsuccessful". The line that
        # explains why is the bare "ERROR: ..." Quartus prints above it.
        # Quartus frames the real message in a block of dashes, and it does NOT
        # necessarily start with "ERROR" -- a Tcl `error` from our own procs is
        # just the bare message. Take the block, stop at the stack trace.
        real, in_block = [], False
        for ln in out.splitlines():
            s = ln.strip()
            if set(s) == {"-"} and s:
                if in_block:
                    break
                in_block = True
                continue
            if in_block:
                if s.startswith(("while executing", "invoked from within",
                                 "(procedure", "(file")):
                    break
                if s:
                    real.append(s)
        detail = " ".join(real) if real else "\n".join(
            ln for ln in out.splitlines() if "Error (" in ln)
        return Result(False, stdout=trimmed, error=detail or "Tcl script failed")

    if proc.returncode != 0 and "DVCON_OK" not in out:
        return Result(False, stdout=trimmed,
                      error=(proc.stderr or "").strip() or
                            f"quartus_stp exited {proc.returncode}")

    return Result(True, stdout=trimmed)


def _kv(stdout: str, key: str) -> str | None:
    m = re.search(rf"^\s*{re.escape(key)}\s*=\s*(.+?)\s*$", stdout, re.M)
    return m.group(1) if m else None


# ---------------------------------------------------------------------------
# Operations
# ---------------------------------------------------------------------------

def cable_status() -> Result:
    """Is a USB-Blaster present, and is our device on it?

    Matches the IDCODE, not the part name: Quartus reports the EP4CE115 by the
    alias list it shares with EP3C120/10CL120 and never as "EP4CE115".
    """
    r = run_tcl(
        # get_hardware_names THROWS when nothing is attached rather than
        # returning an empty list, so "no cable" has to be caught here or it
        # surfaces as a script failure instead of a plain "plug the board in".
        'if {[catch {get_hardware_names} all]} { set all {} }\n'
        'foreach h $all {\n'
        '    puts "HW=$h"\n'
        '    if {[string match "*USB-Blaster*" $h]} {\n'
        '        if {[catch {get_device_names -hardware_name $h} ds]} { set ds {} }\n'
        '        foreach d $ds { puts "DEV=$d" }\n'
        '    }\n'
        '}\n', timeout=120)
    if not r.ok:
        return r
    hw = re.findall(r"^HW=(.+)$", r.stdout, re.M)
    dev = re.findall(r"^DEV=(.+)$", r.stdout, re.M)
    r.data = {
        "cables": hw,
        "devices": dev,
        "device_present": any("0x020F70DD" in d for d in dev),
    }
    return r


def ident() -> Result:
    r = run_tcl(
        "dvcon_open\n"
        f"set v [dvcon_peek {REG_IDENT}]\n"
        "dvcon_close\n"
        'puts "IDENT=[format 0x%08X $v]"\n', timeout=180)
    if not r.ok:
        return r
    raw = _kv(r.stdout, "IDENT")
    if raw is None:
        r.ok = False
        r.error = "no IDENT in output"
        return r
    v = int(raw, 16)
    magic = (v >> 24) & 0xFF
    r.data = {
        "ident": raw,
        "magic": f"0x{magic:02X}",
        "array_size": (v >> 16) & 0xFF,
        "build_id": v & 0xFFFF,
        "magic_ok": magic == EXPECTED_MAGIC,
    }
    return r


def registers() -> Result:
    names = [("CTRL", REG_CTRL), ("STATUS", REG_STATUS),
             ("DESC_ADDR", REG_DESC_ADDR), ("IMG_ADDR", REG_IMG_ADDR),
             ("BOX_ADDR", REG_BOX_ADDR), ("CONF", REG_CONF),
             ("DBG_CE_SRC", REG_DBG_SRC), ("DBG_CE_DST", REG_DBG_DST),
             ("NUM_BOXES", REG_NUM_BOXES), ("LAYER_IDX", REG_LAYER_IDX),
             ("IDENT", REG_IDENT)]
    body = ["dvcon_open"]
    for n, a in names:
        body.append(f'puts "{n}=[format 0x%08X [dvcon_peek {a}]]"')
    body.append("dvcon_close")
    r = run_tcl("\n".join(body), timeout=300)
    if not r.ok:
        return r
    regs = {n: _kv(r.stdout, n) for n, _ in names}
    st = int(regs.get("STATUS") or "0", 16)
    r.data = {
        "registers": regs,
        "status": {"busy": st & 1, "done": (st >> 1) & 1,
                   "error": (st >> 2) & 1, "fsm": (st >> 4) & 0xF},
    }
    return r


def eth_counters() -> Result:
    names = [("GOOD", REG_ETH_GOOD), ("BAD_FCS", REG_ETH_BAD),
             ("CMD", REG_ETH_CMD), ("FILTERED", REG_ETH_FILT),
             ("BITMAP", REG_ETH_BM)]
    body = ["dvcon_open"]
    for n, a in names:
        body.append(f'puts "{n}=[dvcon_peek {a}]"')
    body.append("dvcon_close")
    r = run_tcl("\n".join(body), timeout=300)
    if not r.ok:
        return r
    c = {n: int(_kv(r.stdout, n) or 0) for n, _ in names}
    # FILTERED is the discriminator: a connected PC broadcasts ARP/mDNS
    # constantly, so with the link up and the PHY in MII this climbs on its own.
    # All-zero means nothing is decoding at the MII pins at all -- which points
    # at the PHY mode jumper (RGMII by default) rather than at this design.
    if sum(c.values()) == 0:
        verdict = ("nothing decoded at all -- not even a frame addressed "
                   "elsewhere. Check the PHY mode jumper is on pins 2-3 (MII) "
                   "for the connector the cable is in -- JP1 for ENET0 (J4), "
                   "JP2 for ENET1 (J5) -- and hard-reset the board; both ship "
                   "in RGMII. Confirm the bitstream is pinned to that same PHY.")
    elif c["GOOD"] == 0 and c["FILTERED"] > 0:
        # Measured on this board: 100 BROADCAST frames were sent and 99 landed
        # in FILTERED. Broadcast cannot be filtered -- eth_mac_rx accepts
        # 48'hFFFFFFFFFFFF explicitly -- so the six bytes reaching the compare
        # are not the six bytes that were sent. A frame whose first 20 bytes
        # were all 0xFF DID pass the filter (and then failed FCS), which says
        # the byte stream is corrupt rather than merely offset: any window of
        # all-0xFF matches, a placed 0xFF*6 at any offset 0..15 does not.
        #
        # That is the signature of sampling an RGMII stream with an MII
        # adapter. At 1 Gbit RGMII clocks data on BOTH edges at 125 MHz;
        # mii_rx_adapter samples the rising edge only, so it captures half the
        # nibbles. Preamble (all 0x5) and all-0xFF survive decimation intact,
        # which is exactly the pattern observed.
        verdict = ("frames are decoded but every one is rejected at the "
                   "destination-MAC filter -- even broadcast. The RX byte "
                   "stream is corrupt, not merely misaddressed. Almost "
                   "certainly the PHY is in RGMII at 1 Gbit while the MAC is "
                   "MII: move the PHY's mode jumper to pins 2-3 (JP1 for "
                   "ENET0, JP2 for ENET1), hard-reset the board, and force the "
                   "PC's NIC to 100 Mbps Full Duplex.")
    elif c["BAD_FCS"] > 0 and c["BAD_FCS"] >= c["GOOD"]:
        verdict = "frames arrive but most fail FCS -- physical layer, not protocol."
    elif c["GOOD"] > 0 and c["CMD"] == 0:
        verdict = "frames pass FCS but none accepted -- check ethertype 0x88B5 and dest MAC."
    elif c["CMD"] > 0:
        verdict = "link carrying command frames."
    else:
        verdict = ("counters moved but no command frame was accepted; "
                   "compare GOOD against FILTERED to see where they stop.")
    r.data = {"counters": c, "verdict": verdict}
    return r


def desc_tap_sweep(desc_addr: int = 0x00001000) -> Result:
    """Sweep the read capture tap against a BURST read, not a single word.

    sdram_tap_sweep() below walks the same knob using the JTAG memory window,
    and that is the weaker measurement: the JTAG window reads ONE word per
    transaction, and quartus/dvcon.sdc records that single JTAG reads were
    always correct even in the runs where burst reads were not -- an isolated
    read leaves data on DQ far longer than a back-to-back burst beat does. A
    JTAG sweep can therefore show every tap clean while the descriptor fetch is
    still skewed, which would be a confident and wrong answer.

    The descriptor fetch is a 16-beat burst through the accelerator's own path,
    so it is the thing that actually fails. This programs a known ramp, then for
    each tap starts the sequencer and reads back what it latched.

    Timing context that makes this worth doing: in the fitted design the
    capture path DRAM_DQ[*] -> sdram_ctrl|avs_readdata[*] has only +0.468 ns of
    setup slack at the slow 85C corner. It passes, but a single clk_sys cycle
    either way is 20 ns, so if the board is effectively capturing a cycle off,
    a neighbouring tap is the fix and this finds which.
    """
    n = 16
    ramp = [0xA0000000 + i for i in range(n)]
    out = {}
    for tap in range(4):
        body = ("dvcon_open\n"
                f"dvcon_poke {REG_SD_TAP} {tap}\n"
                f"dvcon_poke {REG_DESC_ADDR} {desc_addr}\n"
                f"dvcon_poke {REG_IMG_ADDR} {FRAME_BASE}\n"
                f"dvcon_poke {REG_BOX_ADDR} {BOXES_BASE}\n"
                f"dvcon_poke {REG_CONF} 32\n"
                f"dvcon_poke {REG_CTRL} 3\n"
                "for {set i 0} {$i < 200} {incr i} { dvcon_peek 1 }\n")
        for i in range(n):
            body += (f"dvcon_poke {REG_DBG_DSEL} {i}\n"
                     f'puts "D=[format 0x%08X [dvcon_peek {REG_DBG_DVAL}]]"\n')
        body += f"dvcon_poke {REG_SD_TAP} 1\n" + "dvcon_close\n"
        r = run_tcl(body, timeout=900)
        if not r.ok:
            return r
        got = [int(x, 16) for x in
               re.findall(r"^D=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)]
        out[tap] = {
            "words": got,
            "match": sum(1 for i, v in enumerate(got) if i < n and v == ramp[i]),
        }
    best = max(out, key=lambda t: out[t]["match"])
    res = Result(True)
    res.data = {"taps": out, "n": n, "best": best,
                "clean": [t for t in out if out[t]["match"] == n]}
    return res


def sdram_tap_sweep(addr: int = 0x00700000) -> Result:
    """Which read capture tap does this fitted bitstream actually need?

    Writes a ramp through the JTAG window, then for each tap 0..3 reads it back
    and counts the words that match. The JTAG window and the accelerator share
    one sdram_ctrl, so the tap that reads back cleanly here is the tap the
    accelerator's descriptor fetch needs too.

    A sweep where only tap 1 works means the timing is fine and the descriptor
    fault is elsewhere. A sweep where 1 fails and another tap is clean is the
    answer: set it before START. A sweep where NOTHING is clean means the
    capture window is missed at every tap, and the fix is the PLL phase shift
    the SDC describes rather than a tap.
    """
    n = 32
    pat = [0xA0000000 + i for i in range(n)]
    body = "dvcon_open\n" + f"dvcon_poke {REG_MEMADDR} {addr}\n"
    body += "".join(f"dvcon_poke {REG_MEMDATA} {w}\n" for w in pat)
    for tap in range(4):
        body += (f"dvcon_poke {REG_SD_TAP} {tap}\n"
                 f"dvcon_poke {REG_MEMADDR} {addr}\n")
        for _ in range(n):
            body += f'puts "T{tap}=[format 0x%08X [dvcon_peek {REG_MEMDATA}]]"\n'
    body += f"dvcon_poke {REG_SD_TAP} 1\n" + "dvcon_close\n"

    r = run_tcl(body, timeout=1800)
    if not r.ok:
        return r
    res = {}
    for tap in range(4):
        got = [int(x, 16) for x in
               re.findall(rf"^T{tap}=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)]
        res[tap] = sum(1 for i, v in enumerate(got) if i < n and v == pat[i])
    best = max(res, key=lambda t: res[t])
    r.data = {"matches": res, "n": n, "best": best,
              "clean": [t for t in res if res[t] == n]}
    return r


def conv_state() -> Result:
    """The conv engine's FSM state and loop counters, as one sample."""
    r = run_tcl("dvcon_open\n"
                f'puts "CV0=[format 0x%08X [dvcon_peek {REG_DBG_CV0}]]"\n'
                f'puts "CV1=[format 0x%08X [dvcon_peek {REG_DBG_CV1}]]"\n'
                "dvcon_close\n", timeout=300)
    if not r.ok:
        return r
    m0 = re.search(r"^CV0=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)
    m1 = re.search(r"^CV1=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)
    if not (m0 and m1):
        r.ok = False
        r.error = "conv debug registers did not answer"
        return r
    v0, v1 = int(m0.group(1), 16), int(m1.group(1), 16)
    r.data = {"state": v0 & 0x1F, "state_name": CONV_STATE.get(v0 & 0x1F, "?"),
              "beat_cnt": (v0 >> 16) & 0xFFFF,
              "ic_tile": v1 & 0xFFFF, "oc_tile": (v1 >> 16) & 0xFFFF}
    m2 = re.search(r"^EE=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)
    m3 = re.search(r"^ARB=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)
    if m2:
        r.data["elem_state"] = int(m2.group(1), 16) & 0x1F
    if m3:
        a = int(m3.group(1), 16)
        # {.., wr_locked, wr_grant[1:0], 0, rd_locked, rd_grant[1:0], 0}
        r.data["rd_grant"] = (a >> 1) & 0x3
        r.data["rd_locked"] = (a >> 3) & 1
        r.data["wr_grant"] = (a >> 5) & 0x3
        r.data["wr_locked"] = (a >> 7) & 1
    return r


def desc_latched() -> Result:
    """The 16 descriptor words as the sequencer latched them.

    This is the measurement the board needed and did not have. The descriptor
    path is clean in simulation -- sim/tb_desc_path.sv walks a known ramp all
    the way from yolo_layer_sequencer through both AXI translations and both
    arbiters into sdram_ctrl and reproduces it exactly -- so a wrong descriptor
    on hardware is not a logic bug and reading desc[0] alone cannot say what it
    is instead. The whole array can:

      * every word shifted up by one, with desc[15] holding the word AFTER the
        descriptor -> the first beat of the burst is being dropped
      * words wrong but not shifted, and different between runs -> the SDRAM
        read capture is missing its timing window (see quartus/dvcon.sdc: the
        budget at the pin is 4.6 ns before routing)
    """
    body = "dvcon_open\n"
    for i in range(16):
        body += (f"dvcon_poke {REG_DBG_DSEL} {i}\n"
                 f'puts "D=[format 0x%08X [dvcon_peek {REG_DBG_DVAL}]]"\n')
    body += "dvcon_close\n"
    r = run_tcl(body, timeout=600)
    if not r.ok:
        return r
    r.data = {"desc": re.findall(r"^D=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)}
    return r


def eth_drops() -> Result:
    """Words the Ethernet write queue had to throw away. Must be 0."""
    r = run_tcl("dvcon_open\n"
                f'puts "DROP=[format 0x%08X [dvcon_peek {REG_ETH_DROP}]]"\n'
                "dvcon_close\n", timeout=300)
    if not r.ok:
        return r
    m = re.search(r"^DROP=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)
    r.data = {"drops": int(m.group(1), 16) if m else None}
    return r


def mem_read(addr: int, nwords: int) -> Result:
    r = run_tcl(
        "dvcon_open\n"
        f"dvcon_poke {REG_MEMADDR} {addr}\n"
        f"for {{set i 0}} {{$i < {nwords}}} {{incr i}} {{\n"
        f'    puts "W=[format 0x%08X [dvcon_peek {REG_MEMDATA}]]"\n'
        "}\n"
        "dvcon_close\n", timeout=600)
    if not r.ok:
        return r
    r.data = {"addr": f"0x{addr:08X}",
              "words": re.findall(r"^W=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)}
    return r


def mem_selftest(addr: int = 0x00700000) -> Result:
    """Write a pattern and read it back through the JTAG window.

    All-zeros and all-ones are in the pattern deliberately: a stuck data line
    passes a walking-ones test that never drives the opposite state.
    """
    pat = ["0xDEADBEEF", "0x00000000", "0xFFFFFFFF", "0x12345678",
           "0xA5A5A5A5", "0x5A5A5A5A"]
    r = run_tcl(
        "dvcon_open\n"
        f"set pat {{{' '.join(pat)}}}\n"
        f"dvcon_poke {REG_MEMADDR} {addr}\n"
        f"foreach w $pat {{ dvcon_poke {REG_MEMDATA} [expr {{$w}}] }}\n"
        f"dvcon_poke {REG_MEMADDR} {addr}\n"
        "set bad 0\n"
        "foreach w $pat {\n"
        f"    set got [dvcon_peek {REG_MEMDATA}]\n"
        "    set exp [expr {$w & 0xFFFFFFFF}]\n"
        "    if {$got != $exp} { incr bad }\n"
        "    puts [format {W=0x%08X E=0x%08X} $got $exp]\n"
        "}\n"
        "dvcon_close\n"
        'puts "BAD=$bad"\n', timeout=300)
    if not r.ok:
        return r
    got = re.findall(r"^W=(0x[0-9A-Fa-f]+) E=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)
    bad = int(_kv(r.stdout, "BAD") or "-1")
    r.data = {"pairs": got, "bad": bad, "passed": bad == 0}
    return r


def load_file(path: Path, base: int, verify: bool = True) -> Result:
    """Stream a file into SDRAM, then sample it back against the file.

    The verify is not optional in spirit: a short or shifted load produces
    plausible wrong detections rather than an error, which is far more
    expensive to find later than the seconds this costs.
    """
    # Absolute, always. quartus_stp is a separate process with its own working
    # directory, so a path relative to ours resolves to nothing on its side --
    # which surfaces as 'couldn't open ...: no such file or directory' from Tcl
    # long after the transfer itself has already succeeded.
    path = Path(path).resolve()
    if not path.exists():
        return Result(False, error=f"no such file: {path}")

    body = [f"dvcon_load {{{path.as_posix()}}} {base}"]
    if verify:
        body.append(f"set bad [dvcon_verify {{{path.as_posix()}}} {base} 64]")
        body.append('puts "VERIFY_BAD=$bad"')
    r = run_tcl("dvcon_open\n" + "\n".join(body) + "\ndvcon_close\n",
                timeout=3600)
    if not r.ok:
        return r
    m = re.search(r"done:\s+(\d+)\s+words in ([\d.]+) s", r.stdout)
    r.data = {
        "file": str(path),
        "base": f"0x{base:08X}",
        "words": int(m.group(1)) if m else None,
        "seconds": float(m.group(2)) if m else None,
        "verify_bad": int(_kv(r.stdout, "VERIFY_BAD") or -1) if verify else None,
    }
    if verify and r.data["verify_bad"] not in (0, None):
        r.ok = False
        r.error = f"verify failed: {r.data['verify_bad']} sampled word(s) differ"
    return r


def load_verify_only(path: Path, base: int, nsample: int = 64) -> Result:
    """Sample the loaded bytes against the file without writing anything.

    Split out from load_file so the Ethernet path can reuse it: that sends the
    data by a completely different route and still needs the same proof it
    landed correctly.
    """
    # Absolute, always. quartus_stp is a separate process with its own working
    # directory, so a path relative to ours resolves to nothing on its side --
    # which surfaces as 'couldn't open ...: no such file or directory' from Tcl
    # long after the transfer itself has already succeeded.
    path = Path(path).resolve()
    if not path.exists():
        return Result(False, error=f"no such file: {path}")
    r = run_tcl(
        "dvcon_open\n"
        f"set bad [dvcon_verify {{{path.as_posix()}}} {base} {nsample}]\n"
        "dvcon_close\n"
        'puts "VERIFY_BAD=$bad"\n', timeout=1200)
    if not r.ok:
        return r
    r.data = {"bad": int(_kv(r.stdout, "VERIFY_BAD") or -1), "sampled": nsample}
    return r


def run_inference(conf: int = 0x20, desc_base: int | None = None,
                  poll_limit: int = 200000) -> Result:
    """Set up the control registers, pulse START, and poll to completion.

    desc_base defaults to MODEL_BASE + 32: the blob begins with a 32-byte file
    header and yolo_layer_sequencer fetches from desc_base immediately with no
    header skip, so pointing it at the blob start executes the header as a
    descriptor.
    """
    if desc_base is None:
        desc_base = MODEL_BASE + MODEL_HDR_BYTES

    r = run_tcl(
        "dvcon_open\n"
        # SDRAM read capture tap. Bitstreams built before this was measured
        # reset to 1, which captures a cycle late: every beat of a burst picks
        # up the NEXT 32-bit word, so desc[0] arrives holding `flags`, every
        # layer dispatches to the elementwise engine and the walk hangs.
        # desc_tap_sweep reads 16/16 descriptor words at tap 0 against 1/16 at
        # tap 1. Newer bitstreams reset to 0 already and this poke is a no-op.
        f"dvcon_poke {REG_SD_TAP} 0\n"
        f"dvcon_poke {REG_DESC_ADDR} {desc_base}\n"
        f"dvcon_poke {REG_IMG_ADDR} {FRAME_BASE}\n"
        f"dvcon_poke {REG_BOX_ADDR} {BOXES_BASE}\n"
        f"dvcon_poke {REG_CONF} {conf}\n"
        "set t0 [clock milliseconds]\n"
        f"dvcon_poke {REG_CTRL} 0x00000003\n"
        "set st 0\n"
        f"for {{set i 0}} {{$i < {poll_limit}}} {{incr i}} {{\n"
        f"    set st [dvcon_peek {REG_STATUS}]\n"
        "    if {$st & 0x6} break\n"
        "}\n"
        "set dt [expr {[clock milliseconds]-$t0}]\n"
        'puts "STATUS=[format 0x%08X $st]"\n'
        'puts "POLLS=$i"\n'
        'puts "MS=$dt"\n'
        f'puts "NUM_BOXES=[dvcon_peek {REG_NUM_BOXES}]"\n'
        f'puts "LAYER_IDX=[dvcon_peek {REG_LAYER_IDX}]"\n'
        "dvcon_close\n", timeout=3600)
    if not r.ok:
        return r

    st = int(_kv(r.stdout, "STATUS") or "0", 16)
    nb = int(_kv(r.stdout, "NUM_BOXES") or 0)
    layer = int(_kv(r.stdout, "LAYER_IDX") or 0)
    ms = int(_kv(r.stdout, "MS") or 0)

    notes = []
    if st & 0x4:
        notes.append("ERROR bit set. Since build 0006 a descriptor fetch that "
                     "returns fewer than 8 beats raises this instead of looking "
                     "like a clean OP_END -- check the fetch path first.")
    if not (st & 0x4) and nb == 0 and layer == 0:
        notes.append("done at layer 0 with no error and nothing written. On "
                     "hardware this meant the descriptor fetch returned only "
                     "its first beat, so DESC_NEXT read as 0. Treat 0 boxes "
                     "here as UNPROVEN, not as a real negative.")
    if ms < 100 and nb == 0:
        notes.append(f"finished in {ms} ms -- far too fast for a full network; "
                     "conv 0 alone is >3 ms at 50 MHz.")

    r.data = {
        "status": f"0x{st:08X}",
        "busy": st & 1, "done": (st >> 1) & 1, "error": (st >> 2) & 1,
        "fsm": (st >> 4) & 0xF,
        "num_boxes": nb, "layer_idx": layer,
        "polls": int(_kv(r.stdout, "POLLS") or 0),
        "ms": ms,
        "notes": notes,
    }
    return r


def read_boxes(limit: int = 300) -> Result:
    """Read the detection list. 4 words per box, coordinates in Q4."""
    r = run_tcl(
        "dvcon_open\n"
        f"set n [dvcon_peek {REG_NUM_BOXES}]\n"
        'puts "N=$n"\n'
        f"if {{$n > {limit}}} {{ set n {limit} }}\n"
        f"dvcon_poke {REG_MEMADDR} {BOXES_BASE}\n"
        "for {set i 0} {$i < $n} {incr i} {\n"
        f"    set w0 [dvcon_peek {REG_MEMDATA}]\n"
        f"    set w1 [dvcon_peek {REG_MEMDATA}]\n"
        f"    set w2 [dvcon_peek {REG_MEMDATA}]\n"
        f"    set w3 [dvcon_peek {REG_MEMDATA}]\n"
        '    puts "B=$w0 $w1 $w2 $w3"\n'
        "}\n"
        "dvcon_close\n", timeout=1200)
    if not r.ok:
        return r

    boxes = []
    for line in re.findall(r"^B=(\d+) (\d+) (\d+) (\d+)$", r.stdout, re.M):
        w0, w1, w2, w3 = (int(x) for x in line)
        score = (w2 >> 16) & 0xFFFF
        if score >= 0x8000:
            score -= 0x10000
        boxes.append({
            "x1": ((w0 & 0xFFFF) / 16.0), "y1": (((w0 >> 16) & 0xFFFF) / 16.0),
            "x2": ((w1 & 0xFFFF) / 16.0), "y2": (((w1 >> 16) & 0xFFFF) / 16.0),
            "score": score, "cls": w3 & 0xFFFF,
        })
    r.data = {"count": int(_kv(r.stdout, "N") or 0), "boxes": boxes}
    return r


# ---------------------------------------------------------------------------
# CLI -- the dashboard is optional; everything is reachable from a terminal.
# ---------------------------------------------------------------------------

def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        print("commands: cable ident regs eth memtest memread <addr> <n>")
        print("          load <file> <addr> | model <file> | frame <file>")
        print("          run [conf] | boxes")
        return 2

    cmd = argv[1]
    try:
        if cmd == "cable":
            r = cable_status()
        elif cmd == "ident":
            r = ident()
        elif cmd == "regs":
            r = registers()
        elif cmd == "eth":
            r = eth_counters()
        elif cmd == "memtest":
            r = mem_selftest()
        elif cmd == "memread":
            r = mem_read(int(argv[2], 0), int(argv[3]))
        elif cmd == "load":
            r = load_file(Path(argv[2]), int(argv[3], 0))
        elif cmd == "model":
            r = load_file(Path(argv[2]), MODEL_BASE)
        elif cmd == "frame":
            r = load_file(Path(argv[2]), FRAME_BASE)
        elif cmd == "run":
            r = run_inference(int(argv[2], 0) if len(argv) > 2 else 0x20)
        elif cmd == "boxes":
            r = read_boxes()
        else:
            print(f"unknown command: {cmd}")
            return 2
    except LinkError as exc:
        print(f"error: {exc}")
        return 1

    print(json.dumps(r.as_dict(), indent=2))
    return 0 if r.ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
