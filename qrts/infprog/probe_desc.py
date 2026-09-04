#!/usr/bin/env python3
"""What does the sequencer actually latch, word by word?

probe_align.py could only ask one bit of the question: did desc[0] come from
word 0 or word 1. That is enough to know something is wrong and not enough to
know what -- three different faults all present as "desc[0] holds word 1".

Meanwhile sim/tb_desc_path.sv now walks a known ramp through the WHOLE path --
yolo_layer_sequencer -> yolo_axi_arbiter -> axi4_master -> AXI -> axi_to_avalon
-> avalon_mm_master -> avalon_arbiter -> sdram_ctrl -> sdram_model -- and
reproduces it exactly. So the logic is not the problem, and the remaining
candidates are hardware-only. They are distinguishable, but only if you can see
all sixteen words:

  ramp intact (A0..AF)
      The descriptor path is fine. Whatever is wrong is elsewhere.

  every word shifted up by one, desc[15] = the guard word AFTER the descriptor
      The first beat of the burst is being dropped. The burst is 16 Avalon
      beats; 15 arrive where 16 are expected, so each engine beat is assembled
      from the wrong pair.

  words wrong but NOT shifted, and different from run to run
      SDRAM read capture is missing its window. quartus/dvcon.sdc spells out the
      budget: DRAM_CLK rises at clk_sys + 10 ns, the memory drives tAC = 5.4 ns
      later, and the controller captures at 20 ns -- 4.6 ns at the pin before
      any routing. This is the failure that bench already documents.

  words wrong, NOT shifted, IDENTICAL every run
      Not timing. Look at the address decode instead.

Run it several times before concluding: the difference between the last two is
whether the answer is stable.
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

DESC = 0x00001000
GUARD_BEFORE = 0xDEADBEEF
GUARD_AFTER = 0xFEEDFACE

print(f"programming {SOF.name} ...")
subprocess.run([PGM, "-c", "1", "-m", "JTAG", "-o", f"p;{SOF}"],
               capture_output=True)
time.sleep(3)

# ---------------------------------------------------------------------------
# Step 1: single-word reads through the JTAG window.
#
# This is the WEAKER of the two sweeps and is here only as a control. The SDC
# records that single JTAG reads were always correct even in the runs where
# burst reads were not: an isolated read leaves data on DQ far longer than a
# back-to-back burst beat does. So a clean result here does NOT mean the memory
# path is healthy -- it only tells us whether single reads are ALSO broken,
# which would point at something cruder than burst timing.
# ---------------------------------------------------------------------------
sw = link.sdram_tap_sweep()
if sw.ok:
    print(f"\n[control] single-word JTAG reads, out of {sw.data['n']}:")
    for tap, m in sorted(sw.data["matches"].items()):
        print(f"   tap {tap}: {m:3d}" + ("  <-- nominal" if tap == 1 else ""))
    if sw.data["clean"]:
        print("   single reads are fine at tap(s)", sw.data["clean"],
              "-- expected, and not evidence that bursts are fine.")
    else:
        print("   even SINGLE reads fail at every tap. The fault is cruder than "
              "burst timing -- check the address decode and the DQ pinout "
              "before reading anything into the burst sweep below.")
else:
    print("control sweep failed:", sw.error[:300])

# A ramp, with a guard word on each side. The guards are what make a dropped
# beat legible: if the fetch runs one word long, desc[15] holds GUARD_AFTER,
# which cannot be confused with any descriptor word.
words = [GUARD_BEFORE] + [0xA0000000 + i for i in range(16)] + [GUARD_AFTER]
tmp = Path("_desc.bin")
tmp.write_bytes(b"".join(struct.pack("<I", w) for w in words))
link.load_file(tmp, DESC - 4, verify=False)
tmp.unlink(missing_ok=True)

rb = link.mem_read(DESC, 16)
print("\nin SDRAM, read back over JTAG:")
print("  " + " ".join(w[2:] for w in rb.data["words"]))

# ---------------------------------------------------------------------------
# Step 2: the real measurement -- the tap swept against a 16-beat BURST.
#
# This is the transaction that actually fails, so this is the sweep whose answer
# counts. In the fitted design the capture path DRAM_DQ[*] ->
# sdram_ctrl|avs_readdata[*] carries just +0.468 ns of setup slack at the slow
# 85C corner, and one clk_sys cycle is 20 ns -- so if the board is capturing a
# cycle off, a neighbouring tap reads clean and names the fix outright.
# ---------------------------------------------------------------------------
bs = link.desc_tap_sweep(DESC)
if bs.ok:
    print(f"\n[burst] descriptor fetch, words matching out of {bs.data['n']}:")
    for tap in sorted(bs.data["taps"]):
        info = bs.data["taps"][tap]
        head = " ".join(f"{w:08X}" for w in info["words"][:4])
        print(f"   tap {tap}: {info['match']:3d}/16   first four: {head}"
              + ("   <-- nominal" if tap == 1 else ""))
    clean = bs.data["clean"]
    if 1 in clean:
        print("\n-> the nominal tap reads the descriptor CORRECTLY. The skew is "
              "not the capture point; look elsewhere.")
    elif clean:
        print(f"\n-> tap(s) {clean} read the descriptor correctly and the "
              "nominal tap does not. THIS IS THE FIX: poke REG_SD_TAP "
              f"({hex(link.REG_SD_TAP)}) = {clean[0]} before START, and make "
              "that the reset default in jtag_ctrl.sv.")
    else:
        print("\n-> NO tap reads the descriptor correctly. The capture window is "
              "missed at every offset, so this is not a one-cycle alignment "
              "problem: it needs the PLL phase shift quartus/dvcon.sdc "
              "describes (a real -3 ns shifted DRAM_CLK, not the fabric "
              "inverter currently used).")
else:
    print("burst sweep failed:", bs.error[:300])

link.run_tcl(f"dvcon_open\n"
             f"dvcon_poke {link.REG_DESC_ADDR} {DESC}\n"
             f"dvcon_poke {link.REG_IMG_ADDR} {link.FRAME_BASE}\n"
             f"dvcon_poke {link.REG_BOX_ADDR} {link.BOXES_BASE}\n"
             f"dvcon_poke {link.REG_CONF} 32\n"
             f"dvcon_poke {link.REG_CTRL} 3\n"
             f"dvcon_close\n", timeout=600)
time.sleep(3)

got = link.desc_latched()
if not got.ok:
    print("could not read the descriptor window:", got.stderr[:400])
    sys.exit(1)

vals = [int(w, 16) for w in got.data["desc"]]
print("\n  idx  expected    latched")
shifted = 0
intact = 0
for i, v in enumerate(vals):
    exp = 0xA0000000 + i
    mark = ""
    if v == exp:
        intact += 1
    else:
        mark = "  <-- differs"
        if i + 1 < 16 and v == 0xA0000000 + i + 1:
            shifted += 1
        elif i == 15 and v == GUARD_AFTER:
            shifted += 1
    print(f"  {i:3d}  {exp:08X}    {v:08X}{mark}")

print()
if intact == 16:
    print("-> the ramp came back intact. The descriptor path is CLEAN.")
elif shifted >= 12:
    print("-> every word is its successor: the FIRST BEAT OF THE BURST IS "
          "DROPPED.")
    if vals[15] == GUARD_AFTER:
        print("   desc[15] holds the guard word from after the descriptor, "
              "which confirms it: the fetch ran one word past the end.")
else:
    print("-> words are wrong but not shifted. Run this several times: if the "
          "values move between runs it is the SDRAM read capture window "
          "(quartus/dvcon.sdc), and if they are identical every run it is the "
          "address decode.")

d = link.eth_drops()
if d.ok and d.data.get("drops"):
    print(f"\nNOTE: the Ethernet write queue dropped {d.data['drops']} words. "
          "Anything in SDRAM loaded over Ethernet has holes in it.")
