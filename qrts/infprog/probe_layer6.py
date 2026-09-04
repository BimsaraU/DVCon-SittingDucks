#!/usr/bin/env python3
"""Run yolo26n layer 6 alone, at shrinking heights, to find what hangs it.

Layer 6 (CONV 160x160x8 -> 160x160x16, k3 s1 p1) hangs the conv engine
indefinitely: fsm stays CONV_W, no error, no timeout, and the arbiter starves
every other master so even a JTAG memory read comes back as a constant. Layer 2
is LARGER (160x160x32, reduction 288 over 18 tiles against layer 6's 72 over 5)
and completes, so it is not simply work volume -- and tb_conv_engine passes for
this exact shape class at 6x6, so it is not the tiling either.

This walks the height down. Where it starts completing says whether the trigger
is a total-size threshold or something about the full-height geometry.
"""
import struct, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import dvcon_link as link
import dvcon_eth as eth

QRTS = Path(__file__).resolve().parent.parent
SOF = QRTS / "quartus" / "output_files" / "dvcon.sof"
PGM = "D:/qrtus/quartus/bin64/quartus_pgm.exe"
MODEL = QRTS / "model" / "yolo26n_a16.bin"
IFACE = "5B994A7C"
DESC6 = 0x1A0
FSM = {0: "IDLE", 5: "CONV_W", 11: "DONE", 12: "ERROR"}
tmp = Path("_l6.bin")

subprocess.run([PGM, "-c", "1", "-m", "JTAG", "-o", f"p;{SOF}"], capture_output=True)
for _ in range(30):
    q = subprocess.run(["powershell", "-NoProfile", "-Command",
                        "(Get-NetAdapter -Name Ethernet).Status"],
                       capture_output=True, text=True)
    if "Up" in q.stdout:
        break
    time.sleep(2)
time.sleep(2)
eth.send_file(MODEL, link.MODEL_BASE, IFACE, gap_us=0, verify=True)

base = struct.unpack("<16I", MODEL.read_bytes()[DESC6:DESC6 + 64])

for h in (4, 16, 64, 160):
    d = list(base)
    d[7] = (h << 16) | (d[7] & 0xFFFF)          # in_h
    d[9] = (h << 16) | (d[9] & 0xFFFF)          # out_h
    d[15] = 0                                    # stop after this layer
    tmp.write_bytes(b"".join(struct.pack("<I", x) for x in d))
    link.load_file(tmp, DESC6, verify=False)

    link.run_tcl("dvcon_open\n"
                 f"dvcon_poke {link.REG_SD_TAP} 0\n"
                 f"dvcon_poke {link.REG_DESC_ADDR} {DESC6}\n"
                 f"dvcon_poke {link.REG_IMG_ADDR} {link.FRAME_BASE}\n"
                 f"dvcon_poke {link.REG_BOX_ADDR} {link.BOXES_BASE}\n"
                 f"dvcon_poke {link.REG_CONF} 32\n"
                 f"dvcon_poke {link.REG_CTRL} 3\n"
                 "dvcon_close\n", timeout=600)

    t0 = time.time()
    verdict = "HUNG"
    while time.time() - t0 < 45:
        time.sleep(3)
        rg = link.registers()
        if not rg.ok:
            continue
        s = rg.data["status"]
        if not s["busy"]:
            verdict = f"done in {time.time()-t0:.0f}s err={s['error']}"
            break
    print(f"  in_h=out_h={h:3d}  outputs={h*160*16:7d}  -> {verdict}")

tmp.unlink(missing_ok=True)
