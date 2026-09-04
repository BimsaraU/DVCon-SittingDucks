#!/usr/bin/env python3
"""Sweep the SDRAM read capture tap against the descriptor BURST read."""
import struct, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import dvcon_link as link

SOF = Path(__file__).resolve().parent.parent / "quartus" / "output_files" / "dvcon.sof"
PGM = "D:/qrtus/quartus/bin64/quartus_pgm.exe"
D = 0x00001000

print("reprogramming...")
subprocess.run([PGM, "-c", "1", "-m", "JTAG", "-o", f"p;{SOF}"], capture_output=True)
time.sleep(3)

# A 16-word ramp at the descriptor address. Word i is 0xA0000000+i, so the
# latched desc[i] says directly which memory word reached slot i.
ramp = b"".join(struct.pack("<I", 0xA0000000 + i) for i in range(16))
tmp = Path("_ramp.bin"); tmp.write_bytes(ramp)
r = link.load_file(tmp, D, verify=True)
tmp.unlink(missing_ok=True)
print("ramp written, verify ok =", r.ok)
print("readback :", [hex(x) for x in
      [int(str(w), 16) for w in link.mem_read(D, 8).data["words"]]])

res = link.desc_tap_sweep(D)
if not res.ok:
    print("sweep failed:", res.error); sys.exit(1)

for tap, d in sorted(res.data["taps"].items()):
    w = d["words"]
    shown = [hex(x) for x in w[:6]]
    # desc[0] should be 0xA0000000. Anything else names the offset directly.
    off = (w[0] - 0xA0000000) if w and 0xA0000000 <= w[0] <= 0xA000000F else None
    tag = f"desc[0] is memory word {off}" if off is not None else "desc[0] not a ramp value"
    print(f"tap {tap}: match {d['match']}/16  {shown}  -> {tag}")

print("\nclean taps:", res.data["clean"], " best:", res.data["best"])
