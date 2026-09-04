#!/usr/bin/env python3
"""Is the accelerator's descriptor fetch shifted, and if so is it the ADDRESS or
the returned DATA?

probe_align.py's second data point was not informative: it read a base whose
contents were unknown, and BOTH "garbage" and the deliberately-invalid opcode
decode as an error, so `err=1` there distinguished nothing. This replaces it
with a test whose two outcomes are different, non-error states.

The descriptor is a single LAYER_OP_ADD (op=2) followed by END, written at P.
The word immediately BELOW it is set to an invalid opcode on purpose, so the
contents of every address the fetch might land on are known.

    DESC_ADDR = P     aligned -> desc[0]=2   -> ADD runs, bytes appear at dst
                      shifted -> desc[0]=P+4 -> that word is 0 = LAYER_OP_END
                                                -> done=1, err=0, nothing written
    DESC_ADDR = P-4   aligned -> desc[0]=invalid -> err=1
                      shifted -> desc[0]=2       -> ADD runs, bytes appear at dst

Whichever run actually writes also answers the second question. The elementwise
engine writes single bytes at dst+idx and reads single beats (rd_len=1), so a
burst-reassembly fault cannot reach it:

    first written byte at dst    -> address path clean
    first written byte at dst+4  -> address offset of one 32-bit word
"""
from __future__ import annotations

import struct
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dvcon_link as link          # noqa: E402

QRTS = Path(__file__).resolve().parent.parent
SOF = QRTS / "quartus" / "output_files" / "dvcon.sof"
PGM = "D:/qrtus/quartus/bin64/quartus_pgm.exe"

P     = 0x00700000      # descriptor
SRC0  = 0x00710000
SRC1  = 0x00720000
DST   = 0x00730000

N       = 16            # output elements
SENT    = 0xDEADBEEF
GUARD   = 0x0000BAD0    # invalid opcode, sits at P-4
A_BYTE  = 0x11
B_BYTE  = 0x22
EXPECT  = (A_BYTE + B_BYTE) & 0xFF     # 0x33

FSM = {0: "IDLE", 1: "FETCH", 2: "FETCH_W", 3: "DECODE", 4: "CONV", 5: "CONV_W",
       6: "ELEM", 7: "ELEM_W", 8: "DETECT", 9: "DETECT_W", 10: "ADVANCE",
       11: "DONE", 12: "ERROR"}

tmp = Path("_probe.bin")


def put(data: bytes, base: int):
    tmp.write_bytes(data)
    r = link.load_file(tmp, base, verify=False)
    if not r.ok:
        print(f"  load to 0x{base:08X} FAILED: {r.error}")
    tmp.unlink(missing_ok=True)


def words(addr, n):
    r = link.mem_read(addr, n)
    if not r.ok:
        print(f"  read 0x{addr:08X} FAILED: {r.error}")
        return []
    return [int(str(w), 16) if isinstance(w, str) else int(w)
            for w in r.data["words"]]


def arm():
    """Re-lay the operands, the sentinel and the descriptor."""
    put(bytes([A_BYTE]) * 64, SRC0)
    put(bytes([B_BYTE]) * 64, SRC1)
    put(struct.pack("<I", SENT) * 48, DST - 64)     # DST-64 .. DST+128

    d = [0] * 16
    d[0]  = 2                       # LAYER_OP_ADD
    d[2]  = SRC0
    d[3]  = SRC1
    d[4]  = DST
    d[7]  = N | (1 << 16)           # in_w=N, in_h=1
    d[8]  = 1                       # in_c=1
    d[9]  = N | (1 << 16)           # out_w=N, out_h=1
    d[10] = 1                       # out_c=1
    d[15] = 0                       # next_off=0 -> END
    # The guard goes down with the descriptor in one write so the word below P
    # is never whatever the last run happened to leave there.
    put(struct.pack("<I", GUARD) + b"".join(struct.pack("<I", x) for x in d),
        P)


def run(base, tag):
    arm()
    print(f"\n--- {tag}: DESC_ADDR = 0x{base:08X} ---")
    print("  mem[P-4..P+16]:", [hex(x) for x in words(P - 4, 6)])
    link.run_tcl(f"dvcon_open\n"
                 f"dvcon_poke {link.REG_DESC_ADDR} {base}\n"
                 f"dvcon_poke {link.REG_IMG_ADDR} {link.FRAME_BASE}\n"
                 f"dvcon_poke {link.REG_BOX_ADDR} {link.BOXES_BASE}\n"
                 f"dvcon_poke {link.REG_CONF} 32\n"
                 f"dvcon_poke {link.REG_CTRL} 3\n"
                 f"dvcon_close\n", timeout=600)

    st = None
    for i in range(8):
        time.sleep(2)
        rg = link.registers()
        if not rg.ok:
            continue
        st, R = rg.data["status"], rg.data["registers"]
        print(f"  t+{2*(i+1):2d}s fsm={st['fsm']:2d} "
              f"({FSM.get(st['fsm'],'?'):8s}) busy={st['busy']} "
              f"done={st['done']} err={st['error']} "
              f"layer={int(R['LAYER_IDX'],16)}")
        if not st["busy"]:
            break

    lo = DST - 16
    w = words(lo, 12)
    raw = b"".join(struct.pack("<I", x) for x in w)
    hits = [i for i, b in enumerate(raw) if b == EXPECT]
    if not hits:
        print(f"  dst area unchanged -- engine wrote nothing")
        return None
    first = lo + hits[0]
    print(f"  dst area after: {[hex(x) for x in w]}")
    print(f"  first 0x{EXPECT:02X} at 0x{first:08X}, dst=0x{DST:08X}, "
          f"delta={first - DST:+d}")
    return first - DST


def main():
    print("reprogramming...")
    subprocess.run([PGM, "-c", "1", "-m", "JTAG", "-o", f"p;{SOF}"],
                   capture_output=True)
    time.sleep(3)

    # P stays 8-byte aligned in both runs. A 64-bit master reading at a
    # 4-byte-aligned address has its low address bits truncated, so the
    # earlier P-4 run could not distinguish the two hypotheses -- both of
    # them landed on an invalid opcode. Shift the CONTENT instead: the
    # guard sits at P and the real descriptor starts at P+4.
    a = run(P, "descriptor content at P+4, base = P")
    b = None

    print("\n================ verdict ================")
    wrote, delta = (("A (base=P)", a) if a is not None else
                    ("B (base=P-4)", b) if b is not None else (None, None))
    if wrote is None:
        print("neither run wrote anything -- the ADD never dispatched in either")
        print("case, so the descriptor decode is failing for a third reason.")
        return 0

    if wrote.startswith("A"):
        print("fetch ALIGNED: reading at P decoded op=2 correctly.")
        print("The layer-2 hang is therefore NOT a uniform descriptor skew.")
    else:
        print("fetch SHIFTED one 32-bit word: only base=P-4 decoded op=2.")

    if delta == 0:
        print("write landed exactly at dst -> ADDRESS PATH CLEAN.")
        print("  => any fetch shift is in the returned read DATA, not the address.")
    elif delta == 4:
        print("write landed at dst+4 -> ADDRESS OFFSET of one 32-bit word,")
        print("  which explains the descriptor shift as the same single fault.")
    else:
        print(f"write landed at dst{delta:+d} -- neither signature.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
