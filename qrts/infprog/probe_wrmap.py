#!/usr/bin/env python3
"""Which output element lands at which byte?

probe_wraddr.py showed the elementwise engine's writes start at exactly dst --
the address path is clean -- but land in every other 32-bit word, so the bytes
are being placed with gaps. A constant operand cannot say which element went
where. A ramp can.

src0[i] = 0x40+i, src1 = 0, so output element i is the byte 0x40+i and every
element is distinguishable. None of those values collide with the 0xDEADBEEF
sentinel, so an untouched byte is obvious.

The engine writes one byte per element: wr_addr = dst+idx, wr_data shifted by
addr[2:0], wr_strb = 1<<addr[2:0]. Those are 8-byte-lane strobes on a 64-bit
core bus that avalon_mm_master splits into two 32-bit Avalon beats. If the
upper beat's byteenables are being dropped or mis-halved, exactly the elements
whose addr[2] is 1 go missing -- which the map below makes visible directly.
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

P, SRC0, SRC1, DST = 0x00700000, 0x00710000, 0x00720000, 0x00730000
N     = 32
SENT  = 0xDEADBEEF
GUARD = 0x0000BAD0
tmp   = Path("_probe.bin")


def put(data, base):
    tmp.write_bytes(data)
    r = link.load_file(tmp, base, verify=False)
    if not r.ok:
        print(f"  load 0x{base:08X} FAILED: {r.error}")
    tmp.unlink(missing_ok=True)


def words(addr, n):
    r = link.mem_read(addr, n)
    if not r.ok:
        print(f"  read 0x{addr:08X} FAILED: {r.error}")
        return []
    return [int(str(w), 16) if isinstance(w, str) else int(w)
            for w in r.data["words"]]


def main():
    print("reprogramming...")
    subprocess.run([PGM, "-c", "1", "-m", "JTAG", "-o", f"p;{SOF}"],
                   capture_output=True)
    time.sleep(3)

    # The board captures SDRAM reads one cycle late at the reset tap of 1,
    # which shifts every burst by one 32-bit word. Tap 0 reads back 16/16
    # descriptor words exactly; see desc_tap_sweep.
    link.run_tcl("dvcon_open\n"
                 f"dvcon_poke {link.REG_SD_TAP} 0\n"
                 "dvcon_close\n", timeout=120)

    put(bytes((0x40 + i) & 0xFF for i in range(N)), SRC0)
    put(bytes(N), SRC1)
    put(struct.pack("<I", SENT) * 40, DST)          # dst .. dst+160

    d = [0] * 16
    d[0], d[2], d[3], d[4] = 2, SRC0, SRC1, DST
    d[7]  = N | (1 << 16)
    d[8]  = 1
    d[9]  = N | (1 << 16)
    d[10] = 1
    d[15] = 0
    # Descriptor at P, plainly. The one-word offset this used to compensate for
    # was the SDRAM read capture tap, and REG_SD_TAP=0 above removes it at the
    # source -- desc_tap_sweep reads back 16/16 words there against 1/16 at the
    # reset tap of 1.
    put(b"".join(struct.pack("<I", x) for x in d), P)

    link.run_tcl(f"dvcon_open\n"
                 f"dvcon_poke {link.REG_DESC_ADDR} {P}\n"
                 f"dvcon_poke {link.REG_IMG_ADDR} {link.FRAME_BASE}\n"
                 f"dvcon_poke {link.REG_BOX_ADDR} {link.BOXES_BASE}\n"
                 f"dvcon_poke {link.REG_CONF} 32\n"
                 f"dvcon_poke {link.REG_CTRL} 3\n"
                 f"dvcon_close\n", timeout=600)

    for i in range(8):
        time.sleep(2)
        rg = link.registers()
        if rg.ok:
            s = rg.data["status"]
            print(f"  t+{2*(i+1):2d}s fsm={s['fsm']} busy={s['busy']} "
                  f"done={s['done']} err={s['error']}")
            if not s["busy"]:
                break

    w = words(DST, 24)
    raw = b"".join(struct.pack("<I", x) for x in w)

    print("\noffset  byte  element")
    landed = {}
    for off, b in enumerate(raw):
        if 0x40 <= b < 0x40 + N:
            landed[off] = b - 0x40
    for off in sorted(landed):
        print(f"  +{off:3d}   0x{0x40+landed[off]:02X}   elem {landed[off]}")

    print("\nelement -> byte offset")
    inv = {v: k for k, v in landed.items()}
    missing = [e for e in range(N) if e not in inv]
    for e in range(N):
        if e in inv:
            print(f"  elem {e:2d} -> +{inv[e]:3d}   (delta {inv[e]-e:+d})")
    print(f"\nwritten {len(landed)}/{N};  missing elements: {missing}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
