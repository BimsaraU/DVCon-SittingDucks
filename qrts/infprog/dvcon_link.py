#!/usr/bin/env python3
"""dvcon_link.py -- the board-facing half of the inference tooling.

Every operation runs one Tcl script under quartus_stp, because the USB-Blaster
is only reachable through Quartus's TAP driver. Each call opens the cable, does
its work and closes it, so a crash here never leaves the cable locked.

Register numbers are WORD addresses in npu_top (see rtl/npu/npu_top.sv). The
ones the old accelerator also had (CTRL, STATUS, DESC/IMG/BOX_ADDR, NUM_BOXES,
LAYER_IDX, IDENT) keep their old numbers.
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
sys.path.insert(0, str(QRTS / "compiler"))
import npu_isa as isa                                     # noqa: E402

# npu_top registers
REG_CTRL, REG_STATUS = 0x00, 0x01
REG_DESC_ADDR, REG_IMG_ADDR, REG_BOX_ADDR = 0x06, 0x07, 0x08
REG_CONF, REG_NUM_BOXES, REG_LAYER_IDX, REG_IDENT = 0x09, 0x0A, 0x0B, 0x0C
REG_CYCLES, REG_SLOTS, REG_DMA_WORDS, REG_TAG, REG_OP = 0x0D, 0x0E, 0x0F, 0x10, 0x11
REG_FLAGS = 0x12    # [0] model loaded [1] frame loaded; drives LEDG1 / LEDG2
# jtag_ctrl's own registers
REG_SD_TAP = 0x33
REG_ETH_DROP = 0x36
REG_ETH_FILT, REG_ETH_GOOD, REG_ETH_BAD = 0x39, 0x3A, 0x3B
REG_ETH_CMD, REG_ETH_BM = 0x3C, 0x3D
REG_MEMADDR, REG_MEMDATA = 0x3E, 0x3F

# SDRAM map, shared with compiler/npu_compile.py
MODEL_BASE, MODEL_SIZE = 0x00000000, 4 * 1024 * 1024
FRAME_BASE, FRAME_SIZE = 0x00400000, 2 * 1024 * 1024
BOXES_BASE = 0x00600000
ARENA_BASE = 0x00800000

EXPECTED_MAGIC = 0xDC
EXPECTED_ARRAY_SIZE = 16
CLOCK_HZ = 50_000_000
DEFAULT_MODEL = QRTS / "model" / "yolo26n_npu.bin"

SEQ_STATE = {0: "IDLE", 1: "FETCH", 2: "CONV", 3: "ELEM", 4: "DONE",
             5: "ERROR", 6: "DECODE"}
ERR_CODE = {0: "", 1: "unknown opcode in the descriptor table",
            2: "aborted by the host"}


class LinkError(RuntimeError):
    """A cable/tool failure, as opposed to the board answering with bad data."""


def find_quartus_stp() -> str:
    env = os.environ.get("QUARTUS_STP")
    if env and Path(env).exists():
        return env
    found = shutil.which("quartus_stp") or shutil.which("quartus_stp.exe")
    if found:
        return found
    for base in (r"D:\qrtus", r"C:\intelFPGA_lite", r"C:\altera", r"C:\intelFPGA"):
        root = Path(base)
        if root.exists():
            for exe in root.glob("**/quartus/bin64/quartus_stp.exe"):
                return str(exe)
    raise LinkError("quartus_stp not found. Set QUARTUS_STP to its full path, "
                    "or add Quartus's bin64 directory to PATH.")


@dataclass
class Result:
    ok: bool
    stdout: str = ""
    error: str = ""
    data: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {"ok": self.ok, "stdout": self.stdout, "error": self.error,
                **self.data}


def run_tcl(body: str, timeout: int = 1800) -> Result:
    """Run a Tcl fragment with dvcon_jtag.tcl's procs already defined.

    Written to a temp file: quartus_stp's argument handling mangles braces.
    """
    stp = find_quartus_stp()
    script = ("set ::DVCON_NO_MAIN 1\n"
              "set ::quartus(args) [list]\n"
              f"source {{{JTAG_TCL.as_posix()}}}\n{body}\n")
    with tempfile.NamedTemporaryFile("w", suffix=".tcl", delete=False,
                                     encoding="utf-8") as fh:
        fh.write(script)
        path = fh.name
    try:
        proc = subprocess.run([stp, "-t", path], capture_output=True, text=True,
                              timeout=timeout, cwd=str(QRTS))
    except subprocess.TimeoutExpired:
        return Result(False, error=f"quartus_stp timed out after {timeout}s")
    except OSError as exc:
        return Result(False, error=f"could not run quartus_stp: {exc}")
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass

    out = (proc.stdout or "") + "\n" + (proc.stderr or "")
    lines = [ln for ln in out.splitlines()
             if not ln.startswith(("Info:", "    Info:", "Info (", "Warning ("))]
    trimmed = "\n".join(lines).strip()
    if "Error (23031)" in out or "Error (23018)" in out:
        # Quartus frames the real message in a block of dashes on stderr.
        real, in_block = [], False
        for ln in out.splitlines():
            s = ln.strip()
            if s and set(s) == {"-"}:
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
        return Result(False, stdout=trimmed,
                      error=" ".join(real) or "Tcl script failed")
    if proc.returncode != 0:
        return Result(False, stdout=trimmed,
                      error=(proc.stderr or "").strip() or
                      f"quartus_stp exited {proc.returncode}")
    return Result(True, stdout=trimmed)


def _kv(stdout: str, key: str) -> str | None:
    m = re.search(rf"^\s*{re.escape(key)}\s*=\s*(.+?)\s*$", stdout, re.M)
    return m.group(1) if m else None


def _peeks(names: list[tuple[str, int]], timeout: int = 300) -> Result:
    body = ["dvcon_open"]
    for n, a in names:
        body.append(f'puts "{n}=[format 0x%08X [dvcon_peek {a}]]"')
    body.append("dvcon_close")
    r = run_tcl("\n".join(body), timeout=timeout)
    if r.ok:
        r.data = {"raw": {n: int(_kv(r.stdout, n) or "0", 16) for n, _ in names}}
    return r


# ---------------------------------------------------------------------------
# Model blob (read on the host, never over the cable)
# ---------------------------------------------------------------------------
def blob_info(path: Path | str = DEFAULT_MODEL) -> dict:
    """Header of a compiled model: where the descriptors are, input size, and
    the per-level scales the host uses to explain the box records."""
    blob = Path(path).read_bytes()
    h = isa.Header.unpack(blob)
    return {"path": str(path), "bytes": len(blob), "n_desc": h.n_desc,
            "desc_addr": MODEL_BASE + h.desc_off, "imgsz": h.imgsz,
            "ncls": h.ncls, "arena_base": h.arena_base,
            "arena_size": h.arena_size, "in_scale": h.in_scale,
            "levels": [{"stride": s, "h": hh, "w": ww} for s, hh, ww, _, _ in h.levels]}


# ---------------------------------------------------------------------------
# Operations
# ---------------------------------------------------------------------------
def cable_status() -> Result:
    """Is a USB-Blaster present, and is our device (IDCODE 0x020F70DD) on it?"""
    r = run_tcl(
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
    r.data = {"cables": hw, "devices": dev,
              "device_present": any("0x020F70DD" in d for d in dev)}
    return r


def ident() -> Result:
    r = _peeks([("IDENT", REG_IDENT)], timeout=180)
    if not r.ok:
        return r
    v = r.data["raw"]["IDENT"]
    r.data = {"ident": f"0x{v:08X}", "magic": f"0x{v >> 24:02X}",
              "array_size": (v >> 16) & 0xFF, "build_id": v & 0xFFFF,
              "magic_ok": (v >> 24) == EXPECTED_MAGIC,
              "is_npu": (v & 0xFF00) >= 0x0200}
    return r


def decode_status(st: int) -> dict:
    return {"busy": st & 1, "done": (st >> 1) & 1, "error": (st >> 2) & 1,
            "fsm": (st >> 4) & 0xF, "state": SEQ_STATE.get((st >> 4) & 0xF, "?"),
            "conv_phase": (st >> 8) & 7, "elem_phase": (st >> 11) & 7,
            "err_code": (st >> 16) & 0xFF,
            "err_text": ERR_CODE.get((st >> 16) & 0xFF, "")}


def registers() -> Result:
    names = [("CTRL", REG_CTRL), ("STATUS", REG_STATUS),
             ("DESC_ADDR", REG_DESC_ADDR), ("IMG_ADDR", REG_IMG_ADDR),
             ("BOX_ADDR", REG_BOX_ADDR), ("CONF", REG_CONF),
             ("NUM_BOXES", REG_NUM_BOXES), ("LAYER_IDX", REG_LAYER_IDX),
             ("TAG", REG_TAG), ("OP", REG_OP), ("CYCLES", REG_CYCLES),
             ("SLOTS", REG_SLOTS), ("DMA_WORDS", REG_DMA_WORDS),
             ("FLAGS", REG_FLAGS), ("IDENT", REG_IDENT)]
    r = _peeks(names)
    if not r.ok:
        return r
    raw = r.data["raw"]
    r.data = {"registers": {k: f"0x{v:08X}" for k, v in raw.items()},
              "status": decode_status(raw["STATUS"])}
    return r


def eth_counters() -> Result:
    names = [("GOOD", REG_ETH_GOOD), ("BAD_FCS", REG_ETH_BAD),
             ("CMD", REG_ETH_CMD), ("FILTERED", REG_ETH_FILT),
             ("BITMAP", REG_ETH_BM), ("DROPS", REG_ETH_DROP)]
    r = _peeks(names)
    if not r.ok:
        return r
    c = r.data["raw"]
    if sum(c.values()) == 0:
        verdict = ("nothing decoded at all, not even a frame addressed "
                   "elsewhere. Check the PHY mode jumper is on pins 2-3 (MII): "
                   "JP2 for ENET1 (J5). Hard-reset the board after moving it.")
    elif c["GOOD"] == 0 and c["FILTERED"] > 0:
        verdict = ("frames are decoded but every one is rejected at the MAC "
                   "filter, even broadcast: the byte stream is corrupt. The "
                   "PHY is almost certainly in RGMII at 1 Gbit. Move JP2 to "
                   "pins 2-3, hard-reset, and force the NIC to 100 Mbps Full.")
    elif c["BAD_FCS"] > 0 and c["BAD_FCS"] >= c["GOOD"]:
        verdict = "frames arrive but most fail FCS: physical layer, not protocol."
    elif c["GOOD"] > 0 and c["CMD"] == 0:
        verdict = "frames pass FCS but none accepted: check ethertype 0x88B5 and the MAC."
    elif c["DROPS"]:
        verdict = (f"{c['DROPS']} words dropped by the write queue: SDRAM has "
                   f"holes. Re-send with a larger inter-frame gap.")
    elif c["CMD"] > 0:
        verdict = "link carrying command frames."
    else:
        verdict = "counters moved but no command frame was accepted."
    r.data = {"counters": c, "verdict": verdict}
    return r


def eth_drops() -> Result:
    r = _peeks([("DROP", REG_ETH_DROP)])
    if r.ok:
        r.data = {"drops": r.data["raw"]["DROP"]}
    return r


def sdram_tap_sweep(addr: int = 0x00700000) -> Result:
    """Which SDRAM read capture tap reads a ramp back cleanly (JTAG window)."""
    n = 32
    pat = [0xA0000000 + i for i in range(n)]
    body = "dvcon_open\n" + f"dvcon_poke {REG_MEMADDR} {addr}\n"
    body += "".join(f"dvcon_poke {REG_MEMDATA} {w}\n" for w in pat)
    for tap in range(4):
        body += f"dvcon_poke {REG_SD_TAP} {tap}\ndvcon_poke {REG_MEMADDR} {addr}\n"
        body += "".join(f'puts "T{tap}=[format 0x%08X [dvcon_peek {REG_MEMDATA}]]"\n'
                        for _ in range(n))
    body += f"dvcon_poke {REG_SD_TAP} 0\ndvcon_close\n"
    r = run_tcl(body, timeout=1800)
    if not r.ok:
        return r
    res = {}
    for tap in range(4):
        got = [int(x, 16) for x in re.findall(rf"^T{tap}=(0x[0-9A-Fa-f]+)$",
                                              r.stdout, re.M)]
        res[tap] = sum(1 for i, v in enumerate(got) if i < n and v == pat[i])
    r.data = {"matches": res, "n": n, "best": max(res, key=res.get),
              "clean": [t for t in res if res[t] == n]}
    return r


def mem_read(addr: int, nwords: int) -> Result:
    r = run_tcl(
        "dvcon_open\n"
        f"dvcon_poke {REG_MEMADDR} {addr}\n"
        f"for {{set i 0}} {{$i < {nwords}}} {{incr i}} {{\n"
        f'    puts "W=[format 0x%08X [dvcon_peek {REG_MEMDATA}]]"\n'
        "}\n"
        "dvcon_close\n", timeout=600)
    if r.ok:
        r.data = {"addr": f"0x{addr:08X}",
                  "words": re.findall(r"^W=(0x[0-9A-Fa-f]+)$", r.stdout, re.M)}
    return r


def mem_selftest(addr: int = 0x00700000) -> Result:
    """Write a pattern and read it back; all-zero and all-one words included."""
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


def _flag_bit(base: int) -> int:
    """FLAGS bit for a load address: 1 model, 2 frame, 0 anything else."""
    return {MODEL_BASE: 1, FRAME_BASE: 2}.get(base, 0)


def _flag_tcl(bit: int, on: bool) -> str:
    op = f"| {bit}" if on else f"& {~bit & 3}"
    return f"dvcon_poke {REG_FLAGS} [expr {{[dvcon_peek {REG_FLAGS}] {op}}}]"


def set_loaded(base: int, on: bool) -> Result:
    """Light or clear the board's 'model loaded' / 'image loaded' LED."""
    bit = _flag_bit(base)
    if not bit:
        return Result(True)
    return run_tcl(f"dvcon_open\n{_flag_tcl(bit, on)}\ndvcon_close\n", timeout=120)


def load_file(path: Path, base: int, verify: bool = True) -> Result:
    """Stream a file into SDRAM over JTAG, then sample it back.

    The model / image LED goes dark for the transfer and lights again only
    when every sampled word verified, so a lit lamp means usable contents.
    """
    # Absolute: quartus_stp runs with its own working directory.
    path = Path(path).resolve()
    if not path.exists():
        return Result(False, error=f"no such file: {path}")
    bit = _flag_bit(base)
    body = [_flag_tcl(bit, False)] if bit else []
    body.append(f"dvcon_load {{{path.as_posix()}}} {base}")
    if verify:
        body.append(f"set bad [dvcon_verify {{{path.as_posix()}}} {base} 64]")
        body.append('puts "VERIFY_BAD=$bad"')
        if bit:
            body.append(f"if {{$bad == 0}} {{ {_flag_tcl(bit, True)} }}")
    elif bit:
        body.append(_flag_tcl(bit, True))
    r = run_tcl("dvcon_open\n" + "\n".join(body) + "\ndvcon_close\n", timeout=3600)
    if not r.ok:
        return r
    m = re.search(r"done:\s+(\d+)\s+words in ([\d.]+) s", r.stdout)
    r.data = {"file": str(path), "base": f"0x{base:08X}",
              "words": int(m.group(1)) if m else None,
              "seconds": float(m.group(2)) if m else None,
              "verify_bad": int(_kv(r.stdout, "VERIFY_BAD") or -1) if verify else None}
    if verify and r.data["verify_bad"] not in (0, None):
        r.ok = False
        r.error = f"verify failed: {r.data['verify_bad']} sampled word(s) differ"
    return r


def load_verify_only(path: Path, base: int, nsample: int = 64) -> Result:
    path = Path(path).resolve()
    if not path.exists():
        return Result(False, error=f"no such file: {path}")
    r = run_tcl("dvcon_open\n"
                f"set bad [dvcon_verify {{{path.as_posix()}}} {base} {nsample}]\n"
                "dvcon_close\n"
                'puts "VERIFY_BAD=$bad"\n', timeout=1200)
    if r.ok:
        r.data = {"bad": int(_kv(r.stdout, "VERIFY_BAD") or -1), "sampled": nsample}
    return r


def run_inference(conf: float = 0.25, model: Path | str = DEFAULT_MODEL,
                  poll_limit: int = 400000) -> Result:
    """Program the pointers and threshold, pulse START, poll to completion.

    The descriptor address comes from the model blob's own header, so a blob
    compiled with a different layout still starts at its first descriptor.
    conf is a probability; the NPU compares logits, so it is sent as a signed
    Q8.8 logit.
    """
    try:
        info = blob_info(model)
    except (OSError, ValueError) as exc:
        return Result(False, error=f"model blob: {exc}")
    q88 = isa.conf_to_logit_q88(float(conf)) & 0xFFFF
    # Programming the FPGA wipes SDRAM; without this the NPU reads power-up
    # noise and reports "unknown opcode at descriptor 0".
    m = mem_read(MODEL_BASE, 1)
    if not m.ok:
        return m
    if not m.data["words"] or int(m.data["words"][0], 16) != 0x504E5644:
        got = m.data["words"][0] if m.data["words"] else "nothing"
        return Result(False, error=f"no model in SDRAM (word 0 at 0x{MODEL_BASE:08X} "
                      f"is {got}, not 'DVNP'). Send the model and the frame again: "
                      f"programming the FPGA clears SDRAM.")
    r = run_tcl(
        "dvcon_open\n"
        f"dvcon_poke {REG_SD_TAP} 0\n"
        f"dvcon_poke {REG_DESC_ADDR} {info['desc_addr']}\n"
        f"dvcon_poke {REG_IMG_ADDR} {FRAME_BASE}\n"
        f"dvcon_poke {REG_BOX_ADDR} {BOXES_BASE}\n"
        f"dvcon_poke {REG_CONF} {q88}\n"
        "set t0 [clock milliseconds]\n"
        f"dvcon_poke {REG_CTRL} 1\n"
        "set st 0\n"
        f"for {{set i 0}} {{$i < {poll_limit}}} {{incr i}} {{\n"
        f"    set st [dvcon_peek {REG_STATUS}]\n"
        "    if {$st & 0x2} break\n"
        "}\n"
        "set dt [expr {[clock milliseconds]-$t0}]\n"
        'puts "STATUS=[format 0x%08X $st]"\n'
        'puts "MS=$dt"\n'
        f'puts "NUM_BOXES=[dvcon_peek {REG_NUM_BOXES}]"\n'
        f'puts "LAYER_IDX=[dvcon_peek {REG_LAYER_IDX}]"\n'
        f'puts "CYCLES=[dvcon_peek {REG_CYCLES}]"\n'
        f'puts "SLOTS=[dvcon_peek {REG_SLOTS}]"\n'
        f'puts "DMA_WORDS=[dvcon_peek {REG_DMA_WORDS}]"\n'
        "dvcon_close\n", timeout=3600)
    if not r.ok:
        return r
    st = int(_kv(r.stdout, "STATUS") or "0", 16)
    cyc = int(_kv(r.stdout, "CYCLES") or 0)
    slots = int(_kv(r.stdout, "SLOTS") or 0)
    s = decode_status(st)
    notes = []
    if s["error"]:
        notes.append(f"ERROR: {s['err_text'] or 'code ' + str(s['err_code'])} "
                     f"(at descriptor {_kv(r.stdout, 'LAYER_IDX')})")
    if not s["done"]:
        notes.append("did not finish within the poll limit; STATUS shows where "
                     "it is. Read the registers again, or abort with CTRL bit 1.")
    r.data = {"status": f"0x{st:08X}", **s,
              "num_boxes": int(_kv(r.stdout, "NUM_BOXES") or 0),
              "layer_idx": int(_kv(r.stdout, "LAYER_IDX") or 0),
              "n_desc": info["n_desc"],
              "cycles": cyc, "npu_ms": round(cyc / CLOCK_HZ * 1000, 1),
              "host_ms": int(_kv(r.stdout, "MS") or 0),
              "array_util": round(slots / cyc, 3) if cyc else 0,
              "dma_mb": round(int(_kv(r.stdout, "DMA_WORDS") or 0) * 4 / 1e6, 2),
              "conf": conf, "conf_logit_q88": q88, "notes": notes}
    return r


def read_boxes(limit: int = isa.MAX_BOXES) -> Result:
    """The detection list: 16-byte records, decoded by npu_isa.decode_boxes."""
    r = run_tcl(
        "dvcon_open\n"
        f"set n [dvcon_peek {REG_NUM_BOXES}]\n"
        'puts "N=$n"\n'
        f"if {{$n > {limit}}} {{ set n {limit} }}\n"
        f"dvcon_poke {REG_MEMADDR} {BOXES_BASE}\n"
        "for {set i 0} {$i < 4*$n} {incr i} {\n"
        f'    puts "W=[dvcon_peek {REG_MEMDATA}]"\n'
        "}\n"
        "dvcon_close\n", timeout=1200)
    if not r.ok:
        return r
    n = int(_kv(r.stdout, "N") or 0)
    words = [int(x) for x in re.findall(r"^W=(\d+)$", r.stdout, re.M)]
    boxes = isa.decode_boxes(words, min(n, len(words) // 4))
    for b in boxes:
        b["name"] = isa.COCO_NAMES[b["cls"]] if b["cls"] < len(isa.COCO_NAMES) else f"#{b['cls']}"
    boxes.sort(key=lambda b: -b["conf"])
    r.data = {"count": n, "boxes": boxes}
    return r


DEFAULT_SOF = QRTS / "quartus" / "output_files" / "dvcon.sof"


def program_fpga(sof: Path | str = DEFAULT_SOF) -> Result:
    """Configure the FPGA over the USB-Blaster with quartus_pgm.

    Configuration resets the design (SDRAM contents are lost, the NPU starts
    idle) and pulses the Ethernet PHY's reset, so the PC's link drops for a
    few seconds. Load the model and frame AFTER programming.
    """
    sof = Path(sof).resolve()
    if not sof.exists():
        return Result(False, error=f"no bitstream at {sof}; build it with quartus/build.sh")
    pgm = str(Path(find_quartus_stp()).with_name(
        "quartus_pgm.exe" if os.name == "nt" else "quartus_pgm"))
    try:
        proc = subprocess.run([pgm, "-c", "1", "-m", "JTAG", "-o", f"p;{sof.as_posix()}"],
                              capture_output=True, text=True, timeout=300)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return Result(False, error=f"quartus_pgm: {exc}")
    out = (proc.stdout or "") + (proc.stderr or "")
    ok = proc.returncode == 0 and "successful" in out.lower()
    lines = [l for l in out.splitlines() if "Error" in l or "successful" in l.lower()]
    r = Result(ok, stdout="\n".join(lines),
               error="" if ok else ("\n".join(l for l in lines if "Error" in l)
                                    or f"quartus_pgm exited {proc.returncode}"))
    r.data = {"sof": str(sof), "sof_time": os.path.getmtime(sof)}
    return r


def abort() -> Result:
    return run_tcl(f"dvcon_open\ndvcon_poke {REG_CTRL} 2\ndvcon_close\n", timeout=120)


# ---------------------------------------------------------------------------
def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        print("commands: cable ident regs eth memtest memread <addr> <n> taps")
        print("          load <file> <addr> | model [file] | frame <file>")
        print("          run [conf] [model] | boxes | abort | blob [file]")
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
        elif cmd == "taps":
            r = sdram_tap_sweep()
        elif cmd == "memread":
            r = mem_read(int(argv[2], 0), int(argv[3]))
        elif cmd == "load":
            r = load_file(Path(argv[2]), int(argv[3], 0))
        elif cmd == "model":
            r = load_file(Path(argv[2]) if len(argv) > 2 else DEFAULT_MODEL, MODEL_BASE)
        elif cmd == "frame":
            r = load_file(Path(argv[2]), FRAME_BASE)
        elif cmd == "run":
            r = run_inference(float(argv[2]) if len(argv) > 2 else 0.25,
                              argv[3] if len(argv) > 3 else DEFAULT_MODEL)
        elif cmd == "boxes":
            r = read_boxes()
        elif cmd == "abort":
            r = abort()
        elif cmd == "program":
            r = program_fpga(argv[2] if len(argv) > 2 else DEFAULT_SOF)
        elif cmd == "blob":
            print(json.dumps(blob_info(argv[2] if len(argv) > 2 else DEFAULT_MODEL),
                             indent=2))
            return 0
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
