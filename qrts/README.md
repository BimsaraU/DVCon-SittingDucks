# qrts — YOLO26n accelerator on Cyclone IV / DE2-115

Quartus project tree for porting the DVCFinal accelerator to an
**EP4CE115F29C7**, driven by a host PC over Ethernet. The design target and the
reasoning behind it are in [MIGRATION-PLAN.md](MIGRATION-PLAN.md); this file
records **what is actually built and what is not**.

## State

This is a partial tree. It does not synthesise, and there is no bitstream.

| Piece | State |
|---|---|
| `tools/port_rtl.py` | **works** — pulls core RTL, rewrites attributes, idempotent |
| `rtl/core/` | **populated** — 30 files, 24 in the build |
| `rtl/platform/avalon_mm_master.sv` | **written and proven equivalent to `axi4_master`** |
| `sim/tb_avalon_master.sv` | **passes**, 14 checks |
| `sim/tb_crc32_eth.sv`, `tb_eth_loopback.sv` | **pass** |
| `rtl/platform/dvcon_top.sv` | **instantiates the real accelerator**; builds a bitstream, cannot run a frame |
| `rtl/platform/dvcon_regs.sv` | **written and verified** (`tb_dvcon_regs`, 16/16) |
| `rtl/platform/axi_to_avalon.sv` | bridges the accelerator's AXI master onto Avalon |
| `rtl/platform/avalon_onchip_ram.sv` | 16 KB M9K standing in for SDRAM until Qsys exists |
| `quartus/build.sh` | `bash build.sh` → `output_files/dvcon.sof` |
| `quartus/dvcon.qpf` / `.qsf` / `.sdc` | **synthesises clean** (Quartus 25.1std) |
| `quartus/budget/` | resource probe — **core measured, ARRAY_SIZE=16 fits** |
| `pin/de2_115_pins.tcl` | **written**, checked against the top level; **not verified against hardware** |
| `qsys/` | empty — no Platform Designer system, no SDRAM controller |
| `rtl/eth/crc32_eth.sv` | **passes** known-answer vectors |
| `rtl/eth/eth_mac_rx.sv` / `eth_mac_tx.sv` | **pass** loopback, 5 cases |
| `rtl/eth/` RGMII, MDIO, cmd engine | not written |
| `host/` | empty — no host application |

Nothing here has been near hardware.

## What is proven, and by what

**`avalon_mm_master` is a drop-in replacement for `axi4_master`.**
`sim/tb_avalon_master.sv` drives both masters with identical stimulus against
equivalent memory models and compares the beats returned and the bytes
committed. 14/14 checks agree, covering:

- `rd_len = 1` (the conv engine's per-element activation fetch) through 96 beats
  (forces an Avalon burst split, which the caller must not be able to observe);
- `rd_done` landing the cycle **after** the last `rd_data_valid` — the engines
  depend on that gap, and a master that asserts both together silently drops the
  last beat of every burst;
- single-byte writes in **every one of the eight lanes**, which is the case the
  all-ones-byteenable defect destroyed.

That is the port's central claim tested directly: because the new master
reproduces the old internal handshake, no engine file and no line of
`yolo_axi_arbiter` changes.

Three bugs were found and fixed writing it — a read FSM that desynchronised its
engine-beat and Avalon-beat counters at a burst split, and registered
`writedata`/`byteenable` that lagged the sub-beat selector by a cycle. All three
would have looked like memory corruption on hardware.

**The Ethernet MAC works in simulation.** `tb_crc32_eth` checks `crc32_eth`
against zlib-computed vectors (`0xCBF43926` for `"123456789"` and two others),
because a wrong FCS is the worst kind of link bug: frames leave the FPGA and
the host NIC drops every one in hardware, with nothing to see at either end.
`tb_eth_loopback` then runs the transmitter into the receiver and checks all
four properties the plan names — good FCS accepted with the payload intact,
bad FCS dropped, wrong MAC dropped, TX CRC verifies — plus short-body padding.

Four bugs surfaced building it, every one of which would have looked like a
dead link:

- **TX sampled `crc_out` a cycle early**, so every frame carried the CRC of all
  but its last byte. Fixed by feeding the CRC combinationally rather than from
  a registered copy; a settling cycle is not a workable alternative, because
  holding the carrier repeats a byte into the receiver's CRC and dropping it
  reads as end-of-carrier.
- **RX had the mirror-image bug**, evaluating the residue before the final FCS
  byte had been absorbed, so it rejected perfectly good frames.
- **RX's `crc_init` was registered**, so it was still asserted during the first
  destination-MAC byte and swallowed it (`init` outranks `en` in `crc32_eth`).
- **RX's four-deep delay line filled during the address but only drained in the
  body**, leaving two bytes permanently stuck and every frame two bytes short.

## The Quartus project

The board part is marked **EP4CE115F29C7N**, but the `.qsf` says
`EP4CE115F29C7`. Quartus rejects the N form outright — `Error (125095): Part
name EP4CE115F29C7N is invalid`. The N is an ordering suffix for the RoHS
lead-free package, not a separate fitting target: same die, same C7 speed
grade, identical timing. `get_part_list -family "Cyclone IV E"` returns C7,
I7, C8, C8L, I8L, C9L and no N variants at all.

**Quartus Prime Lite 25.1std**, installed at `D:\qrtus\quartus` (not on PATH,
not under the usual `intelFPGA_lite` path). It still supports Cyclone IV E, so
the migration plan's "use 20.1 or 22.1std" caution does not apply.

```sh
export PATH="/d/qrtus/quartus/bin64:$PATH"
cd quartus
quartus_map dvcon && quartus_fit dvcon && quartus_sta dvcon
```

**`quartus_map dvcon` succeeds: 0 errors.** It reports only 4 logic elements,
which is correct and not encouraging — `dvcon_top` is a skeleton with the
engines not yet wired in, so synthesis strips everything unconnected. The 104
pins are real.

`quartus/budget/` is a separate probe that synthesises `system_top` directly at
ARRAY_SIZE=16, which is how the resource question gets a real answer:

| Resource | Used | Available | % |
|---|---|---|---|
| Logic elements | **25,509** | 114,480 | 22% |
| 9-bit multiplier elements | **256** | 532 | 48% |
| Memory bits | 2,176 | 3,981,312 | <1% |

**ARRAY_SIZE=16 fits.** The multiplier count matches the plan's arithmetic
exactly — 16x16 = 256 MACs = 128 of the 266 18x18 blocks. The plan estimated
~39–48 k LE for the whole design; the core alone is 25.5 k, so that estimate
looks sound with the MAC and SDRAM glue still to come.

What else has been checked:

- `dvcon_top` elaborates clean (xsim), so `TOP_LEVEL_ENTITY` names something
  that builds;
- `tools/check_pins.py` cross-checks every assignment against the top-level
  port list — 23 ports, 104 pins, no orphans, no duplicates, every port with an
  explicit I/O standard. The checker was verified by injecting a typo'd port
  name, a duplicated pin and a missing bus bit, and confirming it caught all
  three. Quartus reports an assignment to a nonexistent port as a *warning* and
  then leaves the real port unplaced, which on an SDRAM data line is a dead
  interface found on the bench.

The core file list in `quartus/core_files.tcl` is **generated** by the porter
from the same manifest that populates `rtl/core/`, and sourced by the `.qsf`.
Do not add core files to the `.qsf` by hand — two lists drift.

### Pins are transcribed, not confirmed

`pin/de2_115_pins.tcl` comes from the Terasic DE2-115 User Manual. No board
file existed in this tree to check against, and the DE2-115 has several
revisions. **Cross-check `DRAM_*` and `ENET0_*` against the manual for your
board before the first programming run** — a wrong pin on a bidirectional line
driven against an external driver can damage the part.

### What the SDC does and does not claim

`quartus/dvcon.sdc` constrains `CLOCK_50` and the PHY's `ENET0_RX_CLK`,
declares those two domains asynchronous (a promise about the RTL: every
crossing must go through a dcfifo or synchroniser), and false-paths the button,
LEDs and MDIO.

The SDRAM and RGMII I/O constraints are **written but commented out**, because
constraining a generated clock that does not exist yet is an *error*, and a
`.sdc` that errors leaves the whole design unconstrained — the exact failure
this file exists to prevent. They come back with the PLL.

The multicycle section is deliberately empty. Relaxing the conv engine's long
arithmetic there would make a report go green and lie to whoever reads it next;
the A1 rewrite removed four variable dividers from the RTL for that reason.

## Building a bitstream

```sh
cd quartus
bash build.sh            # map -> fit -> sta -> asm, into output_files/dvcon.sof
bash build.sh map        # Analysis & Synthesis only, much faster
bash build.sh report     # summaries from the last run
```

`build.sh` regenerates the activation tables, runs `check_pins.py` **before**
the fitter spends twenty minutes rediscovering a bad assignment, then runs the
full flow.

### What the bitstream does and does not do

It configures and its resource and timing numbers are real. It **cannot run a
frame**: the Qsys SDRAM system and `eth_cmd_engine` do not exist, so the
accelerator is attached to a 16 KB on-chip RAM and started by a small boot
sequencer in `dvcon_top` instead of by a host over Ethernet.

That stand-in is not a shortcut — it is what makes the build measurable. With
the memory port tied off (`readdata=0`, `readdatavalid=0`), constant
propagation ran backwards through the whole design: synthesis reported **5000
registers removed, zero multipliers, 208 logic elements**. A bitstream built
that way configures perfectly and describes an empty chip.

### Unresolved: two SDRAM pins

`DRAM_DQ[28]` and `DRAM_DQ[31]` were transcribed onto PIN_T2 and PIN_T1, and
Quartus rejects both (`Error 171016: illegal location assignment`). Every other
`DRAM_DQ` pin is accepted, so these are transcription errors.

They are left **unassigned** rather than guessed. An unassigned pin gets placed
by the fitter and reported in `output_files/dvcon.pin`, which is recoverable; a
*wrong* pin on an SDRAM data line drives a real signal against an external
driver, which is not. `check_pins.py` reports them as a warning so they stay
visible. Read the correct locations from the DE2-115 manual for your board
revision before programming — nothing is blocked meanwhile, since there is no
SDRAM controller yet.

### Four defects real synthesis found that simulation did not

Every one of these passes xsim, and the error messages are misleading:

- **`vector_unit.sv` used `localparam int` in its parameter port list** — an
  SV-2009 construct, and `SystemVerilog_2005` is the only SystemVerilog value
  Quartus accepts. Changed to `parameter` (they size ports, so they cannot move
  into the body).
- **`systolic_array.sv` had a bare `assign` and an unnamed `for` inside
  `generate`.** One construct produced **eleven cascading syntax errors**
  pointing at innocent lines.
- **`generic_mux` collided with a Quartus built-in primitive**, so instances
  resolved to the built-in: `Error (12004): Port "in" does not exist in
  primitive "generic_mux"` — reads as a port problem, is a name collision.
  Renamed to `dvc_generic_mux`.
- **`dvcon_regs` selected the 32-bit half from the LIVE `avs_address`** in its
  read-data state. Avalon does not require the requester to hold the address for
  the whole transaction, so by the time AXI read data came back the address
  could already be the next one and the read returned the other register of the
  64-bit pair. Caught by `tb_dvcon_regs`; fixed with a latched `upper_lat`.
- **`avalon_onchip_ram` was inferred as an `altshift_taps` delay line**, not a
  RAM, because the array was touched in several branches. Quartus then reported
  436 errors of the form `Port "avs_read" does not exist in macrofunction
  "u_ram"` — none of which mention memory inference. `Error (12002) ... does not
  exist in macrofunction` almost never means the port list is wrong; it means
  the module was replaced by an inferred megafunction.
- **`avalon_mm_master` drove `avm_address`/`avm_burstcount` from both the read
  and write always blocks.** Simulation is happy (the channels never overlap);
  synthesis gives `Error (10028): Can't resolve multiple constant drivers` per
  bit of both buses. Split into per-channel registers muxed at the port — and
  the equivalence bench still passes, because it never could have caught this.

Also note there is **no `SYSTEMVERILOG_INPUT_VERSION`** assignment in Quartus;
setting it makes the tool reject the entire `.qsf`.

## The core RTL is generated — do not edit `rtl/core/`

`rtl/core/` is written by `tools/port_rtl.py`. Every file carries a header
saying so. To change core RTL, change it in `DVCFinal/` and re-run:

```sh
python tools/port_rtl.py            # pull + rewrite, idempotent
python tools/port_rtl.py --check    # report what would change
```

The porter also resolves a trap worth knowing about. Two directories hold
accelerator sources and they overlap:

- `DVCFinal/rtl/` — the files Phase A fixes
- `DVCon_2026/DVCon_SoC_SRC/ACCELERATOR_IP/` — everything else, **plus stale
  copies** of the `rtl/` files, put there by `DVCFinal/build.sh install`

`DVCFinal/rtl/` wins for every name it provides (10 files today). This is the
same trap that made `build.sh sim` test the wrong code for who knows how long:
`xvlog` only *warns* when a later file redefines a module, so the stale copy
quietly replaced the fixed one.

`axi4_master.sv` is a special case — it lives only in the IP directory but
carries a Phase-A fix (the `wr_strb` byte enable), so it is **not** pristine
upstream. It is copied for reference (the Avalon master must match its
handshake) and excluded from the Quartus build.

### Attribute rewriting

Xilinx synthesis attributes mean nothing to Quartus. 13 rewrites are applied:

| Xilinx | Quartus | count |
|---|---|---|
| `ram_style = "block"` | `ramstyle = "M9K"` | 8 |
| `use_dsp = "yes"` | `multstyle = "dsp"` | 4 |
| `rom_style = "distributed"` | `romstyle = "logic"` | 1 |

The `ram_style` rewrite is not cosmetic: Quartus ignores the Xilinx spelling and
those arrays silently become LE register files, which on a 114 k-LE part is the
difference between fitting and not.

Attribute text also appears in prose comments explaining what the attribute
does. Those are left alone — rewriting them would make the comments describe the
wrong tool.

## Excluded from the build

| File | Why |
|---|---|
| `Accelerator_Top.sv` | Xilinx SoC socket; AXI4 slave for a RISC-V core that does not exist on DE2-115. Replaced by `dvcon_top.sv` (unwritten) |
| `yolo_axi4_slave_regs.sv`, `axi4_lite_slave.sv` | AXI register files. Replaced by `dvcon_regs.sv` (unwritten) |
| `axi4_master.sv` | replaced by `avalon_mm_master.sv`; kept for reference |
| `soc_axi4_slave_regs.sv` | belongs to the old SoC |

## Three things the porter cannot fix

Reported on every run, and still open:

- **`silu_lut.sv`** builds its ROM in an `initial` block with `$exp`/`$rtoi`
  real math. Vivado folds that at elaboration; **Quartus will not.** Needs a
  `.mif` generated offline.
- **`accel_dma_seq.sv`**'s `$fatal` elaboration guard is ignored by Quartus.
  Moot if the legacy GEMM path is dropped.
- **Unpacked array module ports** in `systolic_array`, `accelerator`,
  `system_top`, `yolo_conv_engine`. Quartus Lite's cross-boundary support is
  weaker than Vivado's. Try as-is; flatten to packed vectors with a `genvar`
  slice at each end if Analysis & Synthesis complains. Budget time — it touches
  the array's hot interface.

## Running the tests

```sh
bash sim/run.sh          # pins, dvcon_top elaboration, shims, ethernet
cd ../DVCFinal && ./build.sh sim   # core: conv numerics, top-level walk, sequencer, ucode, elem
```

Core testbenches live in `DVCFinal/` deliberately: that is the upstream the
porter pulls from, so the core is proven there and copied here rather than
re-proven in two places. `DVCFinal/build.sh sim` currently passes all five,
including `tb_conv_engine` at **1560/1560 outputs bit-exact** against an
independent reference and `tb_top_sequencer` running a descriptor chain from a
single START.

## Before this can target hardware

Blocking, roughly in order:

1. **Phase A is unfinished.** A2 is now **done** — `yolo_layer_sequencer` is
   instantiated, `DESC_ADDR`/`IMG_ADDR`/`BOX_ADDR` reach it, and
   `DVCFinal/tb/tb_top_sequencer.sv` proves one START walks a descriptor chain
   through the real engines with bit-exact conv output. Still open: A3 (SiLU
   unimplemented — all 102 convs are linear), A4 (weight pointers are file
   offsets, not addresses), A5 (SOFTMAX is a pass-through, so both C2PSA blocks
   are wrong), A7 (activation scales uncalibrated).
2. **A1's timing gate is open.** The dividers are gone from
   `yolo_conv_engine`, but implementation has not been re-run, so "WNS positive
   at 50 MHz" is unproven — and Cyclone IV C7 is slower fabric than the
   Kintex-7 the design already missed timing on by 21.5 ns.
3. **Quartus is not installed.** Needs Prime Lite 20.1 or 22.1std; later
   versions drop Cyclone IV E.
4. `dvcon_regs.sv`, `dvcon_top.sv`, the Qsys system, SDRAM, Ethernet, host app.
5. **Phase C is now more load-bearing than planned.** Fixing the conv engine's
   accumulator defect required swapping the loop nest to
   `oc_tile -> pos -> ic_tile`, which re-reads the weight tile once per output
   position. Correct, but it *increases* memory traffic — and the plan already
   called SDRAM bandwidth the dominant performance risk.
