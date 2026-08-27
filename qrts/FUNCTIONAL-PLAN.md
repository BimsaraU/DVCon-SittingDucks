# Functional DE2-115 build — no stand-in components

Split, as specified:

* **JTAG** drives the accelerator: registers, START, status, box readback.
* **Ethernet** carries bulk data into SDRAM: the model blob AND image bitmaps.
* **SDRAM** is real, and its map is a contract generated for both sides.

## Blocks, and the bench each one passed

| block | file | bench | result |
|---|---|---|---|
| SDRAM controller | `rtl/mem/sdram_ctrl.sv` | `tb_sdram` | 19/19 |
| 3-master arbiter | `rtl/mem/avalon_arbiter.sv` | `tb_arbiter` | 34/34 |
| JTAG control + SDRAM read window | `rtl/platform/jtag_ctrl.sv` | `tb_jtag_ctrl` | 8/8 |
| MII nibble/byte adapters | `rtl/eth/mii_*_adapter.sv` | `tb_mii_adapters` | 25/25 |
| command engine + DMA | `rtl/eth/eth_cmd_engine.sv` | `tb_eth_cmd` | 30/30 |
| accelerator | `rtl/core/` (ported) | `tb_top_sequencer` | end to end |

`tb_arbiter` runs against the REAL `sdram_ctrl`, not a stub: the failure it
exists to catch (a grant moving mid-burst) still "works" against a recorder.

## Data paths

```
ENET0 --mii_rx--> eth_mac_rx --> eth_cmd_engine --+
                                                  |
JTAG --> jtag_ctrl --> dvcon_regs --AXI--> Accelerator_Top
             |                                 |          |
             |  (SDRAM read window)      axi_to_avalon     |
             |                                 |          |
             +---------> avalon_arbiter <------+----------+
                                |
                           sdram_ctrl --> 128 MB SDRAM
```

Arbitration is strict priority: accelerator > ethernet > JTAG. Only the
accelerator has a deadline -- a stall mid-convolution idles 256 multipliers.
Ethernet has a retransmit window; JTAG is a human at a cable.

## Host side

* `sw/dvcon_host.c` -- raw L2, 64-frame ACK window with selective retransmit.
  `load` / `model` / `frame` / `ident`.
* `tools/dvcon_jtag.tcl` -- `quartus_stp` script: `ident`, `run`, `boxes`,
  `peek`, `poke`.
* `tools/gen_memmap.py` -- emits `sw/dvcon_memmap.h` and `rtl/dvcon_memmap.vh`
  from one definition, with an overlap check. Neither side writes the numbers
  down.

## Why the box list needed an SDRAM read window

Boxes live in SDRAM, and `eth_cmd_engine` implements WRITE_MEM only, so with
registers alone there was **no path for results to reach the host at all**.
Two pseudo-registers fix it: write MEMADDR (0x3E), then read MEMDATA (0x3F),
which post-increments. That is why the arbiter has a third master port.

## Timing

Read, and closed in three of four attempts. Each critical path was ONE wide
arithmetic structure in a per-element path, found by noticing that all ten
worst paths shared an endpoint.

| fix | slow 85C | slow 0C | fast |
|---|---:|---:|---:|
| (none) | -53.466 | -46.075 | -16.071 |
| softmax divide -> 24-cycle serial | -4.936 | -2.665 | +7.522 |
| requantize split into two states | -2.858 | -0.613 | +8.801 |
| weight address -> per-channel base register | -0.502 | +1.523 | +10.028 |
| iy/ix bases -> per-position registers | measuring | | |

All four clocks are genuinely analysed. `eth_tx_clk` was silently unconstrained
until its `create_generated_clock` target was fixed -- **grep every compile log
for Quartus 332049 and 332174**, because an unanalysed domain looks exactly
like a clean one.

One attempt FAILED and was reverted: registering the whole row address
`cc*plane_sz + iy*g_in_w` gave 2784 wrong outputs out of 3560, because that sum
depends on `cc` and `kk_y` and so changes every element, not every position.
The comment in `yolo_conv_engine.sv` records it so nobody retries it the same
way.

`tb_conv_engine` gained cases 3b and 3c: case 3 is named "multi-oc-tile" but
uses oc_n=16, which at ARRAY_SIZE=16 is ONE tile, so `oc_tile` never advanced
and the weight-address change was untested by a passing suite. Now 3560
checked outputs, up from 1560.

## Not done

- ~~`DRAM_DQ[28]`/`[31]` have no pin~~ **Fixed.** The Terasic pin table is now
  in `resources/DE2-115 Pin Assignments.csv`, and it showed the problem was
  larger than two missing pins: `DRAM_DQ[26:30]` were each shifted one position
  against the real table, so `DRAM_DQ[26]` sat on `PIN_T7` (`SRAM_ADDR[9]`),
  and three `ENET0` pins pointed at `ENETCLK_25` and at PHY 1. All nine are
  corrected. `check_pins.py` now diffs every location against that table --
  which is what catches a pin that is legal but wired to the wrong peripheral
  -- and the `DVCON_ALLOW_UNPINNED` override is gone. The pinout lives only in
  `pin/de2_115_pins.tcl`; the duplicate copy Quartus had written into
  `dvcon.qsf` (and which was the copy actually winning) is deleted, and the
  check fails if it comes back.
  **The existing `output_files/dvcon.sof` was built with the wrong pins and has
  been deleted. Rebuild before programming.**
- No PLL: the design runs off the 50 MHz oscillator, so `sdram_ctrl`'s 100 MHz
  timing constants are conservative rather than wrong.
- `sw/dvcon_host.c` compiles clean under `-Wall -Wextra` against stubbed
  headers and its frame layout matches the RTL offsets, but it has never been
  linked or run -- it needs a Linux host.
- Nothing has been on hardware.
