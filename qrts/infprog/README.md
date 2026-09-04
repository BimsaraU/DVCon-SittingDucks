# infprog — transfer and inference tooling for the DE2-115 accelerator

Everything here talks to the board over **JTAG**, using the memory window in
`jtag_ctrl`. That is a deliberate choice, not a fallback:

- `sw/dvcon_host.c` moves bulk data over raw L2 Ethernet. It needs a Linux host
  (`AF_PACKET`) and the board's PHY in MII mode, which is a jumper on pins 2-3
  — **JP1** for ENET0 (J4), **JP2** for ENET1 (J5). Both ship in RGMII, so out
  of the box that path moves nothing.
- This path needs neither. It runs on Windows with nothing but Quartus
  installed, and works with the jumpers untouched.

Measured **4,890 words/s**, so the 2.46 MB model takes ~125 s and a 640×640×3
frame ~63 s. That is twice what it was: `dvcon_poke` re-selected the register on
every word, but `jtag_ctrl` reuses the latched `req_write`/`req_addr` on later
DATA scans and `REG_MEMDATA` post-increments, so a transfer needs ONE address
scan and then nothing but data scans (`dvcon_stream_word`).

## Ethernet: working, at 12 MB/s

`dvcon_eth.py` speaks the same wire protocol as `sw/dvcon_host.c` but injects
frames through **Npcap** (`wpcap.dll` via ctypes), so it runs on Windows with no
Linux host and no WSL bridging. It sends over Ethernet and confirms over
**JTAG** — the received-frame bitmap is a register (`REG_ETH_BM`), so the
board's transmit path, the least-proven part of the design, is not needed.

### Measured

| transfer | over JTAG | over Ethernet |
|---|---|---|
| frame, 1.23 MB | 67 s | **0.1 s** (12.03 MB/s) |
| model, 2.46 MB | 123 s | **0.2 s** (12.13 MB/s) |

`BAD_FCS` was 0 across 2,645 frames, so `gap_us 0` is correct on a direct
cable. Both transfers verified against the file over JTAG.

This needed the PHY in MII mode. On this board JP1 was unreachable, so **JP2
was moved to pins 2-3 and the cable went into ENET1 (J5)**, and the design was
repointed to PHY 1 — pin locations and top-level port names only, no Ethernet
RTL change. See `ETHERNET_SETUP.md` in the repo root.

### How the wrong PHY mode presented, before that

The NIC is cabled and up, and the MAC **is** decoding frames — `FILTERED`
climbs on its own, which it never did before. But `GOOD` stays at 0 and no
command frame is ever accepted. Three measurements pin it down:

| test | sent | result |
|---|---|---|
| unicast to the FPGA MAC | 3 | all `FILTERED` |
| **broadcast** `FF:FF:FF:FF:FF:FF` | 100 | 99 `FILTERED` |
| first 20 bytes all `0xFF` | 100 | 99 `BAD_FCS`, 0 filtered |
| `FF*6` placed at offsets 0–15 | 50 each | all `FILTERED`, every offset |

Broadcast **cannot** be filtered — `eth_mac_rx` accepts `48'hFFFFFFFFFFFF`
explicitly. So the six bytes reaching the comparator are not the six that were
sent. And it is not a fixed byte offset either, because no placement of `FF*6`
at any offset 0–15 passes, yet a frame that is *entirely* `0xFF` does. The byte
stream is **corrupt**, not merely shifted.

That is the signature of sampling an **RGMII** stream with an **MII** adapter.
Windows reports the link at **1 Gbps**, and MII stops at 100 Mbit — so the PHY
negotiated gigabit and is in RGMII: 125 MHz with data on *both* clock edges.
`mii_rx_adapter` samples the rising edge only, capturing half the nibbles.
Preamble (all `0x5`) and all-`0xFF` survive that decimation intact; nothing
else does. Every observation above follows from it.

### The two fixes, neither of them software

1. **Mode jumper to pins 2-3** selects MII — JP1 for ENET0, JP2 for ENET1.
   Both ship in RGMII. Power-cycle afterwards; a warm reset does not re-strap
   the PHY. **The bitstream must be pinned to whichever PHY the cable is in** —
   this design is now on **ENET1 (J5)**, because JP1 could not be reached on
   this board. See `ETHERNET_SETUP.md` in the repo root.
2. **Force the PC's NIC to 100 Mbps Full Duplex.** In an **Administrator**
   PowerShell:

   ```powershell
   Set-NetAdapterAdvancedProperty -Name Ethernet -RegistryKeyword '*SpeedDuplex' -RegistryValue 4
   Restart-NetAdapter -Name Ethernet
   ```

   Without elevation this fails with `Access is denied`.

The dashboard's **Ethernet → Check this PC's link** button reports the link
speed and says this directly; a wired link at 1 Gbps is on its own enough to
predict the failure.

```
python dvcon_eth.py list
python dvcon_eth.py model ../model/yolo26n_a16.bin Realtek
python dvcon_eth.py frame uploads/<name>.bin Realtek
python dvcon_eth.py send <file> <addr> [iface] [gap_us]
```

There is no flow control in this protocol — the FPGA drops what it cannot
absorb. On a direct cable start with `gap_us` 0; if the verify reports dropped
words, raise it.

## Files

| file | what it is |
|---|---|
| `dvcon_link.py` | board operations over JTAG. Importable, and a CLI on its own. |
| `dvcon_eth.py` | bulk transfer over raw Ethernet via Npcap. |
| `dvcon_image.py` | any image file -> the INT8 CHW frame the board reads, plus an RGB copy to draw on. |
| `dvcon_draw.py` | draws the box list onto the frame as a PNG, on the host CPU. |
| `server.py` | localhost HTTP server for the dashboard. Stdlib only. |
| `dashboard.html/.css/.js` | the UI. |

## Dashboard

```
python server.py
# open http://127.0.0.1:8750/
```

Binds `127.0.0.1` only, and has no authentication — "write 2.5 MB into SDRAM" is
one HTTP call, so it is not something to put on a network.

Five panels, in the order you actually use them:

- **Health** — the bring-up chain: cable → bitstream identity → SDRAM
  write/read → control registers. *Run all checks* stops at the first failure,
  because a failed cable check makes every later result meaningless.
- **Transfer** — load an image, then push it and the model into SDRAM with a
  read-back verify. Long transfers run as a background job the page polls, so
  the browser never holds a request open for minutes. Pick **JTAG** (always
  works, ~70 s for a frame) or **Ethernet** (seconds, once the link is in MII).
- **Inference** — sets the pointers, pulses START, polls STATUS, reads boxes.
- **Memory** — inspect any address through the same window the loader writes
  through, with the blob header and descriptors decoded by name.
- **Ethernet** — the MAC counters and what they mean.
- **Board LEDs** — what every lamp on the DE2-115 means, served from the same
  table the RTL is written against so the two cannot drift apart.

## Board LEDs

At rest: one blinking green (heartbeat) and one steady green (reset released).
Everything else dark. Bus activity is nanoseconds wide, so every indicator is
stretched to ~1/12 s or it would never appear to leave idle.

| LED | Meaning |
|---|---|
| LEDG0 | Heartbeat ~1.5 Hz. Solid or dark = clock or reset dead |
| LEDG1 | SDRAM traffic |
| LEDG2/3/4 | Ethernet RX frame / TX / RX error |
| LEDG5 | JTAG register access |
| **LEDG6** | **FILE TRANSFER** — the JTAG memory window is moving data |
| **LEDG7** | **INFERENCE RUNNING** |
| LEDG8 | Reset released |
| LEDR0 | Systolic array busy |
| LEDR1 | Weight load into the array |
| LEDR2 | Activation streaming |
| LEDR3 | Vector unit (requantise + activation) produced a result |
| LEDR4 | Conv engine |
| LEDR5 | Elementwise engine (add/concat/split/upsample) |
| LEDR6 | Detect engine |
| LEDR7 | Engine error |
| LEDR8-17 | Ten-segment progress bar over the 181 descriptors |

The progress bar is the useful one: a wedged run **freezes the bar at the layer
it died on** while the heartbeat keeps blinking, which separates "hung" from
"dead clock" without a single JTAG poll.

These come from a `dbg_engine`/`dbg_layer`/`dbg_busy` tap added to
`Accelerator_Top`. It is output-only — nothing inside reads it back — so it
cannot change behaviour.

## Loading an image

**Transfer → Load an image** takes any photo the browser can read and turns it
into the two files that matter:

| file | what it is | who reads it |
|---|---|---|
| `uploads/<name>.bin` | 3×640×640 **INT8 CHW** | the accelerator |
| `uploads/<name>.rgb` | 640×640×3 **uint8 HWC** | the annotator |

The image is letterboxed, not stretched — aspect preserved, remainder padded
with grey 114, exactly as ultralytics does it. Stretching shifts every box the
detector produces and reads as an accuracy bug in the accelerator rather than a
preprocessing one. Quantisation is `int8 = clip(round((pixel/255) / (1/127)))`,
which keeps the input in the positive half of the INT8 range.

The output is **byte-identical** to `DVCFinal/tools/prepare_image.py`, verified
by hashing both.

### Why the `.rgb` companion exists

The `.bin` is not a picture. It is signed, quantised and plane-ordered, so
rendering it as RGB888 gives a dark, tinted, banded mess — which is what
"the bounding box was just noise" actually was. `dvcon_draw` now prefers the
`.rgb` beside the frame and only falls back to dequantising the tensor when no
companion exists.

Note the `bus.jpg` files in this repo are **noise placeholders, not
photographs**, as is `DVCFinal/out/frame.bin`. They exercise the plumbing; no
model will detect anything in them. Load a real photo through the dashboard.

The response carries the letterbox transform so a detection can be put back on
the original image:

```
x_orig = (x_640 - pad_x) / scale
```

The CLI does the same thing:

```
python dvcon_image.py photo.jpg frame.bin
```

## Drawing detections

```
python dvcon_draw.py <frame.bin> <out.png>              # boxes read from the board
python dvcon_draw.py <frame.bin> <out.png> boxes.json   # or from a file
```

Runs on the host CPU with no Pillow and no numpy — the frame is raw INT8 CHW and
the PNG is written with `zlib` and `struct`. The dashboard's *Draw boxes on the
frame* button does the same thing and shows the result inline.

Note `DVCFinal/out/frame.bin` is a **noise test pattern, not a photograph**. No
model will produce meaningful detections from it; it exercises the plumbing.

## CLI

```
python dvcon_link.py cable          # is a USB-Blaster there, and our device on it
python dvcon_link.py ident          # magic 0xDC, ARRAY_SIZE, build id
python dvcon_link.py memtest        # SDRAM write/read-back through JTAG
python dvcon_link.py regs           # control registers + decoded STATUS bits
python dvcon_link.py eth            # MAC counters + a verdict
python dvcon_link.py model ../model/yolo26n_a16.bin
python dvcon_link.py frame ../../DVCFinal/out/frame.bin
python dvcon_link.py run 0x20       # conf threshold
python dvcon_link.py boxes
python dvcon_link.py memread 0x20 16
```

Every command prints JSON, so it composes with `jq` and is easy to log.

If `quartus_stp` is not on `PATH`, set `QUARTUS_STP` to its full path. The tool
also probes the usual install roots, including `D:\qrtus`.

## Things that will bite you

**ARRAY_SIZE must match.** `export_yolo26n.py` emits weight tiles in the
engine's exact loop order. A blob built for 24 and run on a 16 bitstream reads
the wrong tile for every convolution — silently, with no error flag, producing
plausible wrong detections. The IDENT register carries the value the bitstream
was built with, and the Health panel refuses to go on if it disagrees. Export
with `--array-size 16`.

**The descriptor base is not the model base.** The blob starts with a 32-byte
header (magic/version/count/stride) and `yolo_layer_sequencer` fetches from
`desc_base` immediately with no header skip. Point it at `MODEL_BASE + 32`
(`0x00000020`), or descriptor 0 is the file header.

**"0 boxes" is not automatically a real negative.** On hardware the descriptor
fetch has returned only its first beat, leaving `desc[2..15]` at their power-up
zeros. `DESC_NEXT` then reads 0 and the walk ends after layer 0 — reporting
`done=1, error=0, layer_idx=0` with nothing written, which looks exactly like a
clean run that found nothing. The Inference panel flags this combination
explicitly. Since build **0006** the sequencer counts fetched beats and raises
the error bit instead, so a short fetch is loud.

Two independent signs it happened: the run finishes far too fast (conv 0 alone
is >3 ms at 50 MHz, and a full network is far longer), and the feature-map
arena at `0x00800000` is unchanged.

**Only one cable operation at a time.** Two `quartus_stp` processes fighting
over the USB-Blaster produce JTAG errors that look unrelated to either. The
server serialises this; if you also run `quartus_pgm` by hand, stop the server
first.

## Memory map

Mirrors `sw/dvcon_memmap.h`:

| region | base | size |
|---|---|---|
| model blob | `0x00000000` | 4 MB |
| descriptor table | `0x00000020` | (blob + 32) |
| input frame | `0x00400000` | 2 MB |
| box list | `0x00600000` | 64 KB |
| feature-map arena | `0x00800000` | 56 MB |
