#!/usr/bin/env python3
"""run_board.py -- flash, load over Ethernet, run the network, report.

Kept in the repo rather than a scratch directory: this is the sequence used to
qualify a build on hardware, and it has been rewritten from memory several
times already.

    python run_board.py            # full 181-layer walk
    python run_board.py --one      # truncate after descriptor 0 (one conv)

Reads REG_DBG_SRC / REG_DBG_DST, which expose the source and destination the
sequencer decoded and handed to the conv engine -- the pair that says whether a
misplaced write is a misread descriptor field or an engine ignoring it.
"""

from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dvcon_link as link          # noqa: E402
import dvcon_eth as eth            # noqa: E402

QRTS = Path(__file__).resolve().parent.parent
SOF = QRTS / "quartus" / "output_files" / "dvcon.sof"
PGM = "D:/qrtus/quartus/bin64/quartus_pgm.exe"
MODEL = QRTS / "model" / "yolo26n_a16.bin"
FRAME = QRTS / "infprog" / "uploads" / "scene.bin"
IFACE = "5B994A7C"

FSM = {0: "IDLE", 1: "FETCH", 2: "FETCH_W", 3: "DECODE", 4: "CONV", 5: "CONV_W",
       6: "ELEM", 7: "ELEM_W", 8: "DETECT", 9: "DETECT_W", 10: "ADVANCE",
       11: "DONE", 12: "ERROR"}


def flash():
    print("reprogramming...")
    subprocess.run([PGM, "-c", "1", "-m", "JTAG", "-o", f"p;{SOF}"],
                   capture_output=True)
    # Configuring the FPGA drives ENET1_RST_N low, resetting the PHY; the link
    # takes a few seconds to renegotiate and sending before then fails with
    # "network media is disconnected".
    for _ in range(30):
        q = subprocess.run(["powershell", "-NoProfile", "-Command",
                            "(Get-NetAdapter -Name Ethernet).Status"],
                           capture_output=True, text=True)
        if "Up" in q.stdout:
            print("link up")
            time.sleep(2)
            return True
        time.sleep(2)
    print("link did NOT come up")
    return False


def sample(tag):
    rg = link.registers()
    if not rg.ok:
        print(f"  {tag}: regs error {rg.error}")
        return None
    s, R = rg.data["status"], rg.data["registers"]
    line = (f"  {tag} fsm={s['fsm']:2d} ({FSM.get(s['fsm'],'?'):8s}) "
            f"busy={s['busy']} done={s['done']} err={s['error']} "
            f"layer={int(R['LAYER_IDX'],16):3d} boxes={int(R['NUM_BOXES'],16)}")
    # When the sequencer sits in CONV/CONV_W the interesting state is inside the
    # conv engine, not in these registers: CONV_W only means "an engine was
    # started and has not reported done", which reads the same whether it is
    # blocked on memory or looping without ever finishing.
    if s["fsm"] in (4, 5):
        c = link.conv_state()
        if c.ok:
            line += (f" | conv={c.data['state_name']:7s} "
                     f"beat={c.data['beat_cnt']:5d} "
                     f"oc={c.data['oc_tile']:3d} ic={c.data['ic_tile']:3d}"
                     f" ee={c.data.get('elem_state','?')}"
                     f" arb rd={c.data.get('rd_grant','?')}"
                     f"/lk{c.data.get('rd_locked','?')}")
    print(line)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--one", action="store_true",
                    help="zero descriptor 0's next_off so only one conv runs")
    ap.add_argument("--secs", type=int, default=90)
    args = ap.parse_args()

    if not flash():
        return 1

    r = eth.send_file(MODEL, link.MODEL_BASE, IFACE, gap_us=0, verify=True)
    print(f"model: ok={r['ok']} {r['seconds']}s verify_bad={r.get('verify_bad')}")
    r = eth.send_file(FRAME, link.FRAME_BASE, IFACE, gap_us=0, verify=False)
    print(f"frame: ok={r['ok']} {r['seconds']}s")

    # Words the Ethernet write queue could not accept. This has to be zero:
    # a dropped word is a silent hole in the model, and nothing else reports
    # it -- the frame's FCS was good so the host never retransmits, and
    # stat_bytes counts bytes received rather than words committed.
    d = link.eth_drops()
    if d.ok:
        n = d.data.get("drops")
        print(f"eth write-queue drops: {n}")
        if n:
            print("  WARNING: SDRAM has holes. Detections below are not "
                  "trustworthy; the queue in eth_cmd_engine is too shallow "
                  "for this traffic.")

    if args.one:
        tmp = Path("_next0.bin")
        tmp.write_bytes(struct.pack("<I", 0))
        link.load_file(tmp, 0x20 + 0x3C, verify=False)
        tmp.unlink(missing_ok=True)
        print("descriptor 0 next_off zeroed")

    print("desc0    :", link.mem_read(0x20, 6).data["words"])
    print("arena    :", link.mem_read(0x800000, 4).data["words"])

    link.run_tcl(f"dvcon_open\n"
                 f"dvcon_poke {link.REG_DESC_ADDR} 32\n"
                 f"dvcon_poke {link.REG_IMG_ADDR} {link.FRAME_BASE}\n"
                 f"dvcon_poke {link.REG_BOX_ADDR} {link.BOXES_BASE}\n"
                 f"dvcon_poke {link.REG_CONF} 32\n"
                 f"dvcon_poke {link.REG_CTRL} 3\n"
                 f"dvcon_close\n", timeout=600)
    print("started\n")

    t0 = time.time()
    while time.time() - t0 < args.secs:
        s = sample(f"t+{int(time.time()-t0):3d}s")
        if s and not s["busy"]:
            break
        time.sleep(4)

    print("\ndesc0 after:", link.mem_read(0x20, 6).data["words"])
    print("arena after:", link.mem_read(0x800000, 4).data["words"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
