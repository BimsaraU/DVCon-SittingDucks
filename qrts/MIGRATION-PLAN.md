# Port the DVCFinal accelerator to Cyclone IV / DE2-115, host-driven over Ethernet

## Context

`DVCFinal` is a whole-network YOLO26n INT8 accelerator — 181 descriptors,
102 convs, a no-NMS detect head — designed to run from a single START with no
CPU orchestration. It lives inside a Xilinx Kintex-7 (`xc7k325t`) SoC where a
C-DAC RISC-V soft core writes its AXI4 registers and a MIG DDR3 controller
backs its AXI4 master.

We are moving it to a **DE2-115 (Cyclone IV E, EP4CE115F29C7)** where:

- there is **no soft CPU** — the FPGA is a pure accelerator;
- the **host PC drives it over Ethernet** (raw L2, with a UDP fallback);
- the FPGA uses **its own 128 MB SDRAM** for weights, frame and feature maps;
- the FPGA returns **bounding-box coordinates only**;
- the host app runs on **Windows and Raspberry Pi 5** from one source tree.

Target: `qrts/` (currently empty) becomes a self-contained Quartus project plus
a portable C99 host application.

Four decisions confirmed with the user: raw L2 **plus** UDP fallback,
**gigabit RGMII**, **whole network on the FPGA**, **one C99 app with two
network backends**.

---

## The finding that reorders everything

**DVCFinal is not a working design awaiting a port. It is an unfinished design,
and its own documentation overstates its state.** Reconnaissance of the RTL and
the post-implementation reports turned up six defects, none of which are
Xilinx-specific and all of which travel with the code:

| # | Defect | Evidence |
|---|---|---|
| 1 | **Routed design fails timing by 21.5 ns at 50 MHz**, 7,136–8,963 failing setup endpoints. Worst path is *inside* our conv engine: `u_conv/ic_tile_reg → u_conv/kk_x_reg`, 41.4 ns, **148 logic levels, 92 CARRY4** | `GEN_BIT_OUT/report_timing_summary.rpt`. That is `seed_reduction()`, `yolo_conv_engine.sv:258-271` — the divides/modulos `base % k_sq`, `base / k_sq`, `kk_seed / g_k`, `kk_seed % g_k`. Docs still quote the *pre-accelerator* baseline `WNS −0.034` |
| 2 | **`yolo_layer_sequencer` is never instantiated.** `ucode_engine` replaced it; arbiter slot 0 is explicitly parked | `Accelerator_Top.sv:394-397, 467`. `tb_layer_sequencer` PASSES — it tests dead code |
| 3 | **The microcode program that would run YOLO26n does not exist**, and the instruction RAM holds only 1024 words | `docs/MICROCODE.md:109-112`. The only `.ucode` artifacts are 16 and 32 bytes (`gemm; halt`). Writing `CTRL=START\|MODE_YOLO` today runs whatever was last loaded |
| 4 | **`DESC_ADDR`, `IMG_ADDR`, `BOX_ADDR` are write-only dead ends** — they appear at their declaration and the regfile port and nowhere else. Only `conf_thresh` reaches an engine | `Accelerator_Top.sv:138, 180, 533` |
| 5 | **SiLU is not implemented.** All 102 CONV descriptors carry `act=2` (`ACT_SILU`); `requant()` handles only `4'd1` (RELU) and falls through `default: requant = r`. `silu_lut.sv` exists and is **not referenced by `yolo_conv_engine.sv`**. Every activation in the network is linear | `yolo_conv_engine.sv:302-318` |
| 6 | **Weight/bias pointers in `yolo26n.bin` are file offsets, not addresses.** Descriptors carry `wgt=0x3260`, `bias=0x4D60` while `src0`/`dst` carry absolute `0xAC000000+`. Nothing adds the model base — not the exporter, not the app, not the RTL | dumped from `out/yolo26n.bin` |

Add the already-documented gaps: SOFTMAX is a pass-through copy
(`yolo_elem_engine.sv:294-299`), so the two C2PSA attention blocks are wrong;
activation scales were exported without `--calib`; and **no hardware result of
any kind exists**, because the board never booted Linux at all.

Two of these — #5 and #6 — mean that even with perfect hardware the network
would produce wrong numbers. **Debugging them on a brand-new platform, with a
new memory controller, a new bus and a hand-written Ethernet MAC underneath,
would be the expensive way to find them.** They are cheap to fix now, in a
simulator that already works.

So this plan is ordered: **fix and prove the core first, on the existing Vivado
flow; port second.** Phase A is platform-independent work that would be needed
whether or not we moved to Cyclone IV.

### The good news the same survey turned up

- **The move to Cyclone IV deletes a whole class of problems.** With no Linux,
  no device tree and no driver, the five mutually-contradicting memory maps,
  the reserved-window/`FDC.dts` coupling that "nothing checks at build time"
  (`docs/HANDOFF.md:173-175`), the cache-coherency theory, the `/dev/` node
  mismatch, and the unbootable kernel all become irrelevant. We define one map
  and own all 128 MB.
- **The unfixed 4 KB burst-crossing defect** (`docs/HANDOFF.md:103-122` — AXI4
  forbids it, bursts reach 1152 B, no logic anywhere, no error flag)
  **disappears on Avalon-MM**, which has no 4 KB rule. Porting fixes it.
- **The 64-bit WSTRB word+lane decode** (`docs/MEMORY_MAP.md` §2), which cost
  hours in Stage 3A, is unnecessary on a 32-bit bus. Do not carry it over.
- **A golden reference already exists.** `yoloe_compiler/build/out/` holds
  **422 per-tensor `.bin` dumps that match ONNXRuntime**, needing no torch.
  DVCFinal never had golden vectors; we do.

---

## What carries over, and what does not

The RTL is more portable than its origins suggest: **no SystemVerilog packages,
no `interface`/`modport`, no `typedef`, no `xpm_*` macros, no `.bd` block
design, no instantiated Xilinx primitives** anywhere in the accelerator IP.
`ucode_pkg.svh` and `yolo_descriptor_pkg.svh` are `` `include ``-ed localparam
headers.

| Layer | Verdict |
|---|---|
| `yolo_conv_engine`, `yolo_elem_engine`, `yolo_detect_engine`, `yolo_layer_sequencer`, `ucode_engine`, `yolo_axi_arbiter`, `systolic_array`, `pe`, `vector_unit`, `silu_lut`, `accelerator`, `system_top` | Carry over after Phase A; attribute rewrite + possible unpacked-port flattening |
| `axi4_master.sv` | **Replace** with an Avalon-MM master behind the *identical* internal handshake |
| `yolo_axi4_slave_regs.sv` | **Replace** with a 32-bit Avalon-MM slave driving the *identical* datapath bundle |
| `Accelerator_Top.sv` | **Replace** with `dvcon_top.sv` — no AXI socket, no CPU, sequencer wired in |
| `accel_dma_seq.sv`, `axi4_lite_slave.sv`, legacy GEMM path | **Drop** after bring-up — worth 13,695 LUT / 24,250 FF |
| `AS1061_SYSTEM_TOP.edn` (175 MB RISC-V SoC), MIG DDR3, `rom_32KB_axi`, UNISIM, all `.xdc`, `FDC.dts`, the kernel driver, `ether` | **Gone.** Nothing here ports |

The two seams that make the port cheap are already clean interfaces:

`axi4_master.sv:63-78` — the engines never see AXI. They see
`rd_start/rd_addr/rd_len → rd_data/rd_data_valid/rd_done` and
`wr_start/wr_addr/wr_len/wr_data → wr_data_ready/wr_done`. Avalon-MM burst
read/write maps onto that almost 1:1, so **the new master is ~200 lines and no
engine file is touched.**

`yolo_axi4_slave_regs.sv:86-113` — the datapath side is a flat signal bundle.
Reproducing it from a 32-bit Avalon slave is mechanical.

Note there are **two incompatible register maps** in the tree: DVCFinal's flat
map (`0x08` SRC … `0x2C` LAYER_IDX) and DVCon_2026F's published-PDF map
(`0x10/0x14` Matrix A … `0x44` DESC_ADDR), which is what the built bitstream
actually contains. **Mirror DVCFinal's**, because `sw/accel.h` and
`sw/yolo_model.h` mirror it and those are the files the host app reuses.

---

## Resource budget

EP4CE115: **114,480 LE**, **432 M9K** (486 KB), **266** 18×18 multipliers
(= 532 9×9), 4 PLLs.

Measured routed cost of `u_Accelerator_Top` today at `ARRAY_SIZE=24`:
**43,112 LUT6, 94,473 FF, 2 BRAM36, 610 DSP.**

### Multipliers — the one hard constraint

Each PE is one signed 8×8 → fits a 9×9 → two PEs per 18×18 block.

| `ARRAY_SIZE` | MACs | 18×18 blocks | Fits in 266? |
|---|---|---|---|
| 24 (current) | 576 | 288 | **No** |
| 20 | 400 | 200 | Yes, 66 spare |
| **16 (recommended)** | **256** | **128** | **Yes, 138 spare** |

Pick **`ARRAY_SIZE = 16`**. It also attacks defect #1: 16 is a power of two, so
the address arithmetic that produced the 148-level divider collapses into
shifts. `system_top.sv:31` already documents `ARRAY_SIZE(16)` as supported, and
`K` follows as `4×16 = 64` (`docs/DESIGN.md` §3).

`ARRAY_SIZE` is **not** a hardware-only knob: `export_yolo26n.py` emits weight
tiles in the engine's exact loop order
(`g_wgt + ((oc_tile*n_ic_tiles + ic_tile) * ARRAY_SIZE²)`), with reduction
flattening `(c,ky,kx) → c*k*k + ky*k + kx` on both sides. Re-export for 16 or
every conv reads the wrong tile — silently, with no error flag.

**Do not enable `pe_pair.sv`'s DSP packing.** It forms
`act * ((w1 << 18) + w0)`, a 9 × 27 asymmetric multiply — a DSP48E1 trick. On
Cyclone IV's symmetric 18×18 blocks that decomposes into two blocks, so the
"optimisation" *doubles* multiplier cost here. It is currently uninstantiated;
leave it that way.

### Logic

| Step | LUT6 | FF |
|---|---|---|
| As routed, `ARRAY_SIZE=24` | 43,112 | 94,473 |
| − drop `accel_dma_seq` | −13,695 | −24,250 |
| − array 24→16 (9,501 → ~4,200) | −5,300 | −15,500 |
| Remaining | **~24,100** | **~54,700** |

At a 1.6–2.0× LUT6→LE ratio: **~39–48 k LE**, plus ~4 k Ethernet MAC and ~2 k
SDRAM/Qsys glue → **~45–55 k of 114,480**. Comfortable. M9K is a non-issue
(2 BRAM36 today), with room for the line buffers Phase D needs.

Overflow levers in order: `ACCUM_WIDTH` 32→24 (INT8×INT8 over a 64-deep
reduction peaks at 22 bits — lossless); move the detect head to the host (it is
8,396 LUT / 26,960 FF and its decode is pure elementwise math); then
`ARRAY_SIZE`→12.

---

## Target architecture

```
   HOST (Windows / Pi 5)                 DE2-115
   ┌───────────────────┐                 ┌──────────────────────────────────┐
   │ dvcon_host        │   RGMII 1000    │  rgmii_rx/tx ── eth_mac ── dcfifo│
   │  stb_image load   │  ◄───────────►  │        │                         │
   │  letterbox→INT8   │  EtherType      │  eth_cmd_engine (protocol FSM)   │
   │  raw L2 | UDP     │  0x88B5         │        ├── Avalon master ──┐     │
   │  m2v rerank       │  (or UDP:5001)  │        └── reg writes ──┐  │     │
   │  annotate → PNG   │                 │                         ▼  ▼     │
   └───────────────────┘                 │   dvcon_regs ─► yolo_layer_seq   │
                                         │                 (ucode_engine)   │
                                         │       conv / elem / detect       │
                                         │            ▼                     │
                                         │      avalon_mm_master            │
                                         │            ▼                     │
                                         │   Qsys arbiter ──► SDRAM 128 MB  │
                                         └──────────────────────────────────┘
```

Two Avalon-MM masters (accelerator, ethernet) share the SDRAM controller;
Platform Designer generates the arbiter. Set the accelerator's data width to
**32** to match the SDRAM slave natively — `system_top.sv:35` already carries
`AXI_DATA_W = 32` — so Qsys inserts no width adapter.

### Memory map (accelerator's Avalon address space, 128 MB SDRAM)

| Base | Size | Contents |
|---|---|---|
| `0x0000_0000` | 4 MB | model blob `yolo26n.bin` (32-byte header + 181×64 B descriptors + weights) |
| `0x0040_0000` | 2 MB | input frame, INT8 CHW 3×640×640 = 1,228,800 B |
| `0x0060_0000` | 64 KB | box list — 300 × 16-byte `yolo_box`, Q12.4 |
| `0x0080_0000` | 56 MB | feature-map arena (exporter `--fmap-base`) |
| `0x0400_0000` | 64 MB | free — second frame buffer / double buffering |

Register block is a separate Avalon slave. Keep DVCFinal's offsets verbatim
(`0x00` CTRL … `0x30` UCODE_PC, plus the `0x1000` microcode aperture) so
`sw/accel.h` and `sw/yolo_model.h` stay valid for the host app.

`descriptor.next_off` is a byte offset from the table base, so the descriptor
table is position independent and needs no relocation. `src0 == 0` is the
sentinel for "the input bitmap" and `dst == 0` for "the box list"; those two
sentinels are how the table stays independent of where frames live.

### Wire protocol

One 16-byte header, carried identically inside a raw L2 frame or a UDP
datagram, so the FPGA parses one payload format either way:

```
raw L2: [dst 6][src 6][0x88B5]      [hdr 16][payload ≤1408][FCS]
UDP   : [eth][IPv4][UDP :5001]      [hdr 16][payload ≤1408]

hdr = ver:1  opcode:1  flags:1  rsv:1  seq:4  addr:4  len:2  rsv2:2
```

| op | meaning |
|---|---|
| `0x01 WRITE_MEM` | payload → SDRAM at `addr` |
| `0x02 READ_MEM` | reply `DATA` with `len` bytes from `addr` |
| `0x03 WRITE_REG` / `0x04 READ_REG` | `addr` = register offset |
| `0x05 START` | pulse CTRL START |
| `0x06 ACK_REQ` | reply with the 64-bit received-bitmap for the window |
| `0x10 ACK` `0x11 DATA` `0x12 DONE(num_boxes)` `0x20 IDENT` | FPGA→host |

Payload is 1408 B (a multiple of 128) so every `WRITE_MEM` lands as whole SDRAM
bursts. Reliability is a 64-frame window: host blasts 64 `WRITE_MEM`, sends
`ACK_REQ`, retransmits whatever the returned bitmap is missing. `IDENT` returns
MAC, build ID and `ARRAY_SIZE`; the host refuses to run against a blob compiled
for a different array size.

---

## Plan

### Phase A — Fix and prove the core, in Vivado, before any Quartus work

Platform-independent. Every item here is needed regardless of target, and each
is far cheaper to find in xsim than on a new board.

**A1 — Timing (defect #1).** Rewrite `seed_reduction()`
(`yolo_conv_engine.sv:258-271`) to eliminate the variable divides/modulos. It
runs once per `(oc_tile, ic_tile)`, so serializing it over tens of cycles costs
nothing measurable. Sweep the rest: `grep -n '[/%]' rtl/*.sv`. Then re-implement
and read the reports — DVCFinal's own hard-won lesson is
`grep -A7 "Multipliers :" synth.log` rather than guessing, after a wrong guess
cost a full synthesis run.
**Gate: WNS positive at 50 MHz with margin**, because Cyclone IV C7 is slower.

**A2 — Control path (defects #2, #3, #4).** Instantiate
`yolo_layer_sequencer` and wire `DESC_ADDR`/`IMG_ADDR`/`BOX_ADDR` to it.

Choose the sequencer over writing a ~1,800-instruction microcode program:
it walks a descriptor table **in DRAM** so there is no 1024-instruction ceiling;
`tb_layer_sequencer` already PASSES 6/6; `export_yolo26n.py` already emits its
table; `sw/yolo_model.h` already mirrors the 64-byte record; and the arbiter
already reserves slot 0. Keep `ucode_engine` instantiated as the experimental
path, selected by CTRL bit1 exactly as GEMM/YOLO is selected today.
Also fix `REG_LAYER_IDX`, currently aliased to the microcode PC
(`Accelerator_Top.sv:426`), so the host's progress indicator means what it says.

**A3 — SiLU (defect #5).** Wire `silu_lut.sv` into `requant()` and handle
`act == 4'd2`. Without this every one of the 102 convs is linear and the
network cannot detect anything. Note for Phase B: `silu_lut.sv:24-36` builds
its ROM in an `initial` block using `$exp`/`$rtoi` real math — Vivado evaluates
that at elaboration, Quartus will not, so it needs a `.mif`.

**A4 — Weight pointer base (defect #6).** Decide where the model base is added
and do it in exactly one place. Simplest: have `export_yolo26n.py` take
`--model-base` and bake absolute addresses, matching how `src0`/`dst` already
work. Add a `_Static_assert`-style range check in the exporter — "zero
out-of-range weight pointers" is already a checked invariant, and it passed
only because the check compared against the file, not against DRAM.

**A5 — SOFTMAX.** Implement `LAYER_OP_SOFTMAX` in `yolo_elem_engine`, or the
two C2PSA attention blocks stay pass-throughs and the backbone is wrong.

**A6 — Golden-vector bench.** The highest-value item, and the piece DVCFinal
never got to (`README.md` §4 item 1; `tb_layer_sequencer.sv`'s own header says
it does not check conv numerics). Build it against
`yoloe_compiler/build/out/`'s 422 ORT-matched per-tensor dumps. Start with one
conv, then the C3k2 block, then the full graph.

**A7 — Re-export with calibration.** Run `export_yolo26n.py` with
`--calib <~100 images>`. Without it `pick_act_scale` assumes `|act| ≤ 8.0`
post-SiLU, which is an assumption, not a bound — and the shipped blob was built
that way.

### Phase B — Quartus platform port

Install **Quartus Prime Lite 20.1 or 22.1std** (later versions drop Cyclone IV
E). Device `EP4CE115F29C7`. Only Vivado 2025.2 is installed today.

Create `qrts/{rtl/{core,platform,eth},qsys,quartus,pin,sim,host,tools}`.

**Do not hand-copy the core RTL.** Write `qrts/tools/port_rtl.py`: copies
`DVCon_2026/DVCon_SoC_SRC/ACCELERATOR_IP/*.sv,*.svh` → `qrts/rtl/core/` and
rewrites attributes, idempotently, so Phase A fixes can be re-pulled with one
command.

| Xilinx | Quartus | Where |
|---|---|---|
| `(* ram_style = "block" *)` | `(* ramstyle = "M9K" *)` | `ucode_engine.sv:78`, `yolo_detect_engine.sv:120`, `bram_{act,weight,out}_buffer.sv`, `line_buffer.sv:45` |
| `(* use_dsp = "yes" *)` | `(* multstyle = "dsp" *)` | `pe.sv:66`, `pe_pair.sv:74,82,83` |
| `(* rom_style = "distributed" *)` | `(* romstyle = "logic" *)` | `silu_lut.sv:21` |

The `ram_style` rewrite is not cosmetic — Quartus ignores the Xilinx spelling
and those arrays would silently become LE register files. Add `SEARCH_PATH` for
the `.svh` includes in the `.qsf`.

Three items `port_rtl.py` cannot fix:

- **`silu_lut.sv`'s `initial`-block ROM** — generate a `.mif` offline (one
  Python script) and load it with `altsyncram`-style initialisation.
- **`accel_dma_seq.sv:112-118`'s `$fatal` elaboration guard** — Quartus ignores
  it. Moot if the legacy path is dropped; otherwise re-express as a
  `generate`-if that instantiates a nonexistent module on failure.
- **Unpacked array module ports** — `input wire signed [7:0] act_in [0:ROWS-1]`
  in `systolic_array.sv`, `accelerator.sv`, `system_top.sv`,
  `yolo_conv_engine.sv`. Quartus Lite's cross-boundary support is weaker than
  Vivado's. Try as-is; if Analysis & Synthesis complains, flatten to packed
  vectors (`[ROWS*8-1:0]`) with a `genvar` slice at each end. Budget time — it
  touches the array's hot interface.

Then the shims:

- `qrts/rtl/platform/avalon_mm_master.sv` — exposes the exact port names and
  semantics of `axi4_master.sv:63-78`. Note the write-data contract: `wr_data`
  is forwarded combinationally, caller advances on `wr_data_ready`
  (`axi4_master.sv:10-12`); Avalon `waitrequest` maps straight onto it.
- `qrts/rtl/platform/dvcon_regs.sv` — 32-bit Avalon-MM slave, DVCFinal offsets,
  same datapath bundle, plus the microcode RAM write port. On a 32-bit bus an
  instruction word is two writes; keep DVCFinal's rule that **the high half
  commits** so START cannot race a half-written instruction.
- `qrts/rtl/platform/dvcon_top.sv` — replaces `Accelerator_Top.sv`: PLL, reset
  synchroniser, two Avalon masters, register slave, engines, sequencer.

Set `ARRAY_SIZE = 16`, `K = 64`, data width 32; re-export the model for 16 with
`--fmap-base 0x00800000` and the Phase-A4 `--model-base`.

### Phase C — SDRAM, and the burst problem

Platform Designer: `altera_avalon_new_sdram_controller` for the DE2-115 part
(128 MB, 32-bit, 4 banks, 13 row / 10 col, CAS 3), two masters, the register
slave. PLL from the 50 MHz board clock → 100 MHz system, 100 MHz −3 ns to the
SDRAM pins, 125 MHz + 125 MHz@90° for RGMII TX.

**This is the largest single work item and the dominant performance risk.**
`docs/DESIGN.md` §6 records that the conv engine issues *one AXI read per
activation element* (`rd_len = 1`) and *one write per output byte*
(`wr_len = 1`), deliberately, correct-first. On DDR3 that wasted 7/8 of each
beat. On DE2-115 SDRAM a non-sequential single read costs ~8–10 clocks, and the
activation stream is re-read once per output-channel tile — minutes per frame,
not seconds. **Coalescing is mandatory here, not an optimisation.**

Follow §6's own order. The design already ships the parts: `im2col_engine.sv`
and `line_buffer.sv` implement exactly the line-buffer pattern §6 names as the
starting point, and both are complete, documented and currently never
instantiated.

1. Line-buffer in front of the activation address generator. A 3-row window at
   640 wide × 16 channels is ~30 KB against 486 KB of M9K.
2. Batch the 16 output bytes for a position into one burst.
3. Only then overlap the elementwise engine with conv weight loads.

None of these change numerics, so they land after A6 is green.

### Phase D — Ethernet

Intel's Triple-Speed Ethernet IP is licensed and unusable in Lite, so the MAC
is ours:

- `rgmii_rx.sv` / `rgmii_tx.sv` — `altddio_in` / `altddio_out`.
- `eth_mac_rx.sv` — preamble/SFD strip, destination-MAC filter (unicast +
  broadcast), `crc32_eth.sv` check, drop on bad FCS.
- `eth_mac_tx.sv` — preamble/SFD, payload, CRC32 append, min-frame pad.
- `mdio_master.sv` — 88E1111 bring-up. **Program the PHY's own TX clock delay
  (register 20)** rather than phase-shifting in the FPGA; far easier than
  closing 125 MHz source-synchronous timing on a C7 part.
- `dcfifo` both directions to cross 125 MHz PHY ↔ 100 MHz core.

RGMII is the only gigabit option on this board — DE2-115 exposes 4-bit
`ENET_TX_DATA`/`ENET_RX_DATA` plus `ENET_GTX_CLK`.

**Fallback decided in advance:** if 125 MHz will not close, the same MAC runs
at 100 Mbit MII by swapping only the `rgmii_*` front-end; the 8-bit internal
datapath is unchanged. ~100 ms per frame on the wire, small next to inference.

`eth_cmd_engine.sv` parses the 16-byte header from either EtherType 0x88B5 or
UDP:5001 (static IP plus a minimal ARP responder), drives an Avalon master into
SDRAM for `WRITE_MEM`/`READ_MEM`, drives `dvcon_regs` for
`WRITE_REG`/`READ_REG`/`START`, maintains the received-bitmap, and emits an
unsolicited `DONE` when the sequencer's `F_LAST|F_IRQ` descriptor completes.
The IRQ flag is decoded in the ISA today but was never wired to anything
(`docs/MICROCODE.md:114-116`) — this is where it finally gets a consumer, and
it replaces the CPU's 200-million-iteration poll loop.

### Phase E — Host application

One tree, `qrts/host/`, C99, CMake, no external libraries beyond an optional
libpcap:

```
src/main.c           argument parse, orchestration
src/dvcon_proto.c/h  frame build/parse, windowed ACK, retransmit
src/net.h            backend API: open/send/recv/close
src/net_pcap.c       raw L2 — Npcap on Windows, libpcap on Linux (one file, both)
src/net_afpacket.c   raw L2 — Linux AF_PACKET, dependency-free on the Pi
src/net_udp.c        fallback — Winsock / BSD sockets, no admin, no driver
src/preprocess.c     load → letterbox 640 grey(114) → CHW → INT8 (in_scale 1/127)
src/annotate.c       reuse DVCFinal/sw/annotate.c verbatim
src/m2v.c            reuse DVCFinal/sw/m2v.c verbatim (prompt reranking)
thirdparty/          stb_image, stb_image_write from yoloe_compiler/runtime
```

Reuse is verified safe: the `yoloe_compiler` runtime is **hosted ISO C11 with
zero POSIX includes**, its `thirdparty/` (cJSON, stb) already handles `_WIN32`,
and Windows `.exe` binaries built from that exact tree already sit in
`yoloe_compiler/runtime/`. `sw/m2v.c` and `sw/annotate.c` are plain C with no
device dependency — only `sw/accel.c` is board-specific, and `dvcon_proto.c`
replaces it. This also retires the whole `/dev/accelerator_*` node-mismatch
problem and the driver's byte-vs-word ABI trap.

Runtime `--transport raw|udp` with automatic fallback: try raw L2, and if the
socket cannot be opened (no Npcap, no `CAP_NET_RAW`), fall back to UDP with a
one-line warning rather than failing.

Per frame: upload INT8 CHW image → `WRITE_REG` the four addresses + `CONF` →
`START` → wait for `DONE` → `READ_MEM` the box list → `x >> 4` to convert Q12.4
to pixels → rerank `0.75*semantic + 0.25*(score/127)` against the m2v prompt
embedding → annotate → write PNG.

### Phase F — Bring-up order

One new variable at a time:

1. `IDENT` round-trip — proves RGMII, MAC, CRC, both directions.
2. `WRITE_MEM` → `READ_MEM` loopback over 4 MB — proves SDRAM and the command
   engine's Avalon master.
3. 16×16 identity GEMM (CTRL bit1 = 0), if the legacy path is still in.
   DVCFinal kept the old register offsets specifically so this survives
   (`README.md` §5); it separates "the array works" from "the sequencer works".
   Drop the legacy path only after this passes.
4. One conv layer against the Phase-A6 golden vector.
5. Full frame.

---

## Verification

**Simulate the core in xsim, not Questa.** Vivado 2025.2 is installed, Phase A
happens there anyway, and this sidesteps the line limit in the Questa Starter
edition bundled with Quartus Lite. Use Quartus for synthesis, fitting and
timing; Questa only for the new platform and Ethernet modules, which are small.

Fix `build.sh sim` first: it currently only *runs* `tb_layer_sequencer`.
`tb_ucode_engine` and `tb_elem_engine` compile but are never launched — and the
sequencer is about to become load-bearing.

| Gate | How | Pass criterion |
|---|---|---|
| **A1 timing** | Vivado implementation | WNS positive at 50 MHz |
| **A2 control path** | `tb_layer_sequencer` against the real top | 181 descriptors dispatched from one START; `LAYER_IDX` advances |
| **A3/A5 activations** | unit bench per op | SiLU matches the reference LUT; softmax is not a copy |
| **A4 pointers** | exporter self-check | every `wgt`/`bias` inside the model region in DRAM terms |
| **A6 conv numerics** | golden vector vs `yoloe_compiler/build/out/` | bit-exact for one conv, then the C3k2 block, then the graph |
| Attribute port | Quartus Analysis & Synthesis | RAMs report M9K, multipliers embedded; **≤266 18×18, ≤114,480 LE, ≤432 M9K** |
| Avalon master equivalent | `tb_avalon_master.sv` with an Avalon BFM | same `rd_data`/`wr` sequence as the AXI master for identical stimulus |
| Register file equivalent | `tb_dvcon_regs.sv` | every offset reads/writes; microcode word commits only on the high half |
| Ethernet | `tb_eth_loopback.sv` | good FCS accepted, bad FCS dropped, wrong MAC dropped, TX CRC verifies |
| Host app | build + `IDENT` on both platforms | MSVC/MinGW on Windows, gcc on Pi 5, one tree |
| End to end | real image | boxes match the `DVCNA` CPU reference within quantisation error |

`DVCNA` is the ready-made end-to-end oracle: the same model running as pure C
on CPU, documented at 422/422 tensors matching ONNXRuntime.

---

## Risks, ranked

1. **The core has never produced a correct number anywhere.** No hardware
   result of any kind exists — the board never booted Linux. Phase A is the
   whole mitigation; treat A6 as the real milestone, not the port.
2. **Timing.** The design is 21.5 ns short on a *faster* fabric than the one we
   are moving to. A1 is a gate, not a task.
3. **SDRAM bandwidth against the element-at-a-time conv engine.** Without Phase
   C the port is correct and unusable.
4. **Unpacked-array ports in Quartus Lite.** Mechanical if it bites, but it
   touches the array's hot interface.
5. **LE budget.** ~45–55 k estimated against 114 k. Measure with
   synthesis-only at the end of Phase B, before writing any Ethernet.
6. **125 MHz RGMII on a C7 part.** Mitigated by PHY-side delay and the
   pre-decided MII fallback.
7. **`ARRAY_SIZE` coupling between RTL and exporter.** Silent wrongness, not a
   crash. The `IDENT` array-size check is the guard.
8. **Documentation drift.** Five conflicting memory maps, two register maps,
   `DESIGN.md` §9 stale against `MEMORY_MAP.md`, and both handoffs contradicting
   each other on whether `0xBF000000` is valid. Trust the RTL and the reports
   over the prose; this plan supersedes all of the address discussion.
