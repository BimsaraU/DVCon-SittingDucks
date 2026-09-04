#!/usr/bin/env python3
"""Is the descriptor fetch shifted by one 32-bit word on hardware?

desc[0] should be the descriptor's word 0 (the opcode). If the 64-bit read is
delivered a sub-beat out of step, desc[0] instead picks up word 1 -- which in
the real table is `flags`, and flags==2 decodes as LAYER_OP_ADD, i.e. the
elementwise engine. That is exactly what the board does on every layer.

Write a descriptor whose word0 and word1 differ in a way that tells the two
apart, and see which one the sequencer acted on:

    word0 = 0x0000000F   -> invalid opcode, would raise error
    word1 = 0x00000001   -> LAYER_OP_CONV, would dispatch to the conv engine

  fsm=ERROR / err=1  -> desc[0] took word0. Alignment is correct.
  fsm=CONV_W         -> desc[0] took word1. Fetch is shifted one word.
"""
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
FSM = {0: "IDLE", 2: "FETCH_W", 5: "CONV_W", 7: "ELEM_W", 11: "DONE", 12: "ERROR"}

subprocess.run([PGM, "-c", "1", "-m", "JTAG", "-o", f"p;{SOF}"], capture_output=True)
time.sleep(3)

w = [0] * 16
w[0] = 0x0000000F      # invalid opcode
w[1] = 0x00000001      # LAYER_OP_CONV
w[4] = 0x00800000      # dst, so a CONV dispatch is visible in ce_dst
tmp = Path("_align.bin")
tmp.write_bytes(b"".join(struct.pack("<I", x) for x in w))
link.load_file(tmp, 0x20, verify=False)
BASE = int(__import__("os").environ.get("DBASE", "32"))
tmp.unlink(missing_ok=True)

print("descriptor written:", link.mem_read(0x20, 6).data["words"])
print("DESC_ADDR =", hex(BASE))

link.run_tcl(f"dvcon_open\n"
             f"dvcon_poke {link.REG_DESC_ADDR} {BASE}\n"
             f"dvcon_poke {link.REG_IMG_ADDR} {link.FRAME_BASE}\n"
             f"dvcon_poke {link.REG_BOX_ADDR} {link.BOXES_BASE}\n"
             f"dvcon_poke {link.REG_CONF} 32\n"
             f"dvcon_poke {link.REG_CTRL} 3\n"
             f"dvcon_close\n", timeout=600)
time.sleep(3)

rg = link.registers()
s, R = rg.data["status"], rg.data["registers"]
print(f"fsm={s['fsm']} ({FSM.get(s['fsm'],'?')}) busy={s['busy']} done={s['done']} "
      f"err={s['error']} layer={int(R['LAYER_IDX'],16)}")
print(f"ce_src={R.get('DBG_CE_SRC')}  ce_dst={R.get('DBG_CE_DST')}")

if s["error"]:
    print("\n-> desc[0] took WORD 0. Descriptor alignment is CORRECT.")
elif s["fsm"] == 5 or R.get("DBG_CE_DST") not in (None, "0x00000000"):
    print("\n-> desc[0] took WORD 1. The fetch is SHIFTED ONE 32-BIT WORD.")
else:
    print("\n-> inconclusive; neither signature seen.")
