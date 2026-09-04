#!/usr/bin/env python3
"""Run descriptor 0 alone and prove the conv engine actually wrote its output.

The arena already held 0x7F7F7F7F from an earlier run, so reading 0x7F back
after a conv proves nothing -- a saturated write and no write at all look
identical. Zero the destination first, then run.
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
FRAME = QRTS / "infprog" / "uploads" / "bus.bin"
IFACE = "5B994A7C"
FSM = {0: "IDLE", 5: "CONV_W", 7: "ELEM_W", 11: "DONE", 12: "ERROR"}

subprocess.run([PGM, "-c", "1", "-m", "JTAG", "-o", f"p;{SOF}"], capture_output=True)
for _ in range(30):
    q = subprocess.run(["powershell", "-NoProfile", "-Command",
                        "(Get-NetAdapter -Name Ethernet).Status"],
                       capture_output=True, text=True)
    if "Up" in q.stdout:
        break
    time.sleep(2)
time.sleep(2)

print("model:", eth.send_file(MODEL, link.MODEL_BASE, IFACE, gap_us=0, verify=True)["ok"])
print("frame:", eth.send_file(FRAME, link.FRAME_BASE, IFACE, gap_us=0, verify=False)["ok"])

# Zero 64 KB of the arena so anything non-zero afterwards came from the conv.
zeros = Path("_zero.bin"); zeros.write_bytes(bytes(65536))
eth.send_file(zeros, 0x00800000, IFACE, gap_us=0, verify=False)
zeros.unlink(missing_ok=True)

tmp = Path("_next0.bin"); tmp.write_bytes(struct.pack("<I", 0))
link.load_file(tmp, 0x20 + 0x3C, verify=False)
tmp.unlink(missing_ok=True)

print("desc0 :", link.mem_read(0x20, 12).data["words"])
print("arena before:", link.mem_read(0x800000, 8).data["words"])

link.run_tcl("dvcon_open\n"
             f"dvcon_poke {link.REG_SD_TAP} 0\n"
             f"dvcon_poke {link.REG_DESC_ADDR} 32\n"
             f"dvcon_poke {link.REG_IMG_ADDR} {link.FRAME_BASE}\n"
             f"dvcon_poke {link.REG_BOX_ADDR} {link.BOXES_BASE}\n"
             f"dvcon_poke {link.REG_CONF} 32\n"
             f"dvcon_poke {link.REG_CTRL} 3\n"
             "dvcon_close\n", timeout=600)

for i in range(15):
    time.sleep(3)
    rg = link.registers()
    if not rg.ok:
        continue
    s = rg.data["status"]
    print(f"  t+{3*(i+1):3d}s fsm={s['fsm']} ({FSM.get(s['fsm'],'?')}) "
          f"busy={s['busy']} done={s['done']} err={s['error']}")
    if not s["busy"]:
        break

for off in (0x000, 0x100, 0x1000, 0x8000):
    w = link.mem_read(0x800000 + off, 8).data["words"]
    nz = sum(1 for x in w if int(str(x), 16) != 0)
    print(f"arena+0x{off:04X}: {w}   ({nz}/8 non-zero)")
