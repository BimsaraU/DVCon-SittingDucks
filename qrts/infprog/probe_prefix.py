#!/usr/bin/env python3
"""Which earlier layer has to run before layer 6 hangs?

Layer 6 alone completes in 5s at full size. In the full walk it hangs forever.
So the trigger is state carried across layers, not layer 6's own parameters.

next_off is an offset from the TABLE BASE, not from the current descriptor, so
the chain can be re-pointed without moving anything: patching descriptor 0's
next_off to 0x40*N makes the walk go layer 0 -> layer N -> N+1 -> ... and
descriptor 6's next_off is zeroed to stop there.
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
tmp = Path("_pfx.bin")

subprocess.run([PGM, "-c", "1", "-m", "JTAG", "-o", f"p;{SOF}"], capture_output=True)
for _ in range(30):
    q = subprocess.run(["powershell", "-NoProfile", "-Command",
                        "(Get-NetAdapter -Name Ethernet).Status"],
                       capture_output=True, text=True)
    if "Up" in q.stdout:
        break
    time.sleep(2)
time.sleep(2)

blob = MODEL.read_bytes()
eth.send_file(MODEL, link.MODEL_BASE, IFACE, gap_us=0, verify=True)
eth.send_file(FRAME, link.FRAME_BASE, IFACE, gap_us=0, verify=False)

def put(words, addr):
    tmp.write_bytes(b"".join(struct.pack("<I", x) for x in words))
    link.load_file(tmp, addr, verify=False)

d0 = list(struct.unpack("<16I", blob[0x20:0x20 + 64]))

# (start, stop): layer 0 jumps to `start`, and `stop` terminates the chain.
for (n, stop) in ((4, 4), (4, 5), (4, 6), (3, 4), (3, 6)):
    for k in range(1, 8):                       # restore every descriptor
        put(list(struct.unpack("<16I", blob[0x20+0x40*k:0x20+0x40*k+64])),
            0x20 + 0x40*k)
    ds = list(struct.unpack("<16I", blob[0x20+0x40*stop:0x20+0x40*stop+64]))
    ds[15] = 0
    put(ds, 0x20 + 0x40*stop)
    d0[15] = 0x40 * n
    put(d0, 0x20)

    link.run_tcl("dvcon_open\n"
                 f"dvcon_poke {link.REG_SD_TAP} 0\n"
                 f"dvcon_poke {link.REG_DESC_ADDR} 32\n"
                 f"dvcon_poke {link.REG_IMG_ADDR} {link.FRAME_BASE}\n"
                 f"dvcon_poke {link.REG_BOX_ADDR} {link.BOXES_BASE}\n"
                 f"dvcon_poke {link.REG_CONF} 32\n"
                 f"dvcon_poke {link.REG_CTRL} 3\n"
                 "dvcon_close\n", timeout=600)

    t0, verdict = time.time(), "HUNG"
    while time.time() - t0 < 60:
        time.sleep(3)
        rg = link.registers()
        if not rg.ok:
            continue
        s, R = rg.data["status"], rg.data["registers"]
        if not s["busy"]:
            verdict = f"done in {time.time()-t0:4.0f}s err={s['error']}"
            break
        verdict = (f"HUNG fsm={s['fsm']} layer={int(R['LAYER_IDX'],16)} "
                   f"ce_src={R.get('DBG_CE_SRC')}")
    print(f"  chain 0 -> {n}..{stop} : {verdict}")

tmp.unlink(missing_ok=True)
