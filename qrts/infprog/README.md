# infprog: host tools and dashboard for the DE2-115 NPU

Control goes over **JTAG** (the USB-Blaster: registers, START, status, the box
list). Bulk data goes over **Ethernet** (model blob, frames) or, slower, over the
JTAG memory window. Everything runs on Windows with Quartus installed; Ethernet
also needs Npcap.

## Dashboard

```
python server.py            # then open http://127.0.0.1:8321/  (DVCON_PORT to change)
```

Binds 127.0.0.1 only, with no authentication: every button drives the real board.

| Panel | What it does |
|---|---|
| Health | cable, bitstream IDENT (`0xDC10....`), SDRAM write/read, registers. Run top-down. |
| Transfer | load any photo (letterboxed to 640 x 640, grey 114, quantised to the INT8 frame), then send the model and the frame over JTAG or Ethernet with read-back verify |
| Inference | threshold (0 to 1), Start, Abort, and **Simulate on this PC**: the bit-exact golden model of the NPU on the prepared frame, which shows the exact box list the board must return, with no board attached |
| Detections | read the box list (class name, confidence, box, pyramid level) and draw it on the picture |
| Memory | read any SDRAM address; the blob header and descriptors are decoded |
| Ethernet | MAC counters with a verdict, and the PC's link speed |
| Board LEDs | what every LED and 7-segment digit means |

A run reports the NPU's own cycle count (so the time excludes JTAG polling),
the systolic array utilisation and the SDRAM traffic.

## CLI

```
python dvcon_link.py cable | ident | regs | eth | memtest | taps
python dvcon_link.py model [blob]            # JTAG load at 0x0
python dvcon_link.py frame uploads/<x>.bin   # JTAG load at 0x00400000
python dvcon_link.py run [conf] [blob]       # START, poll, report
python dvcon_link.py boxes                   # decoded, sorted by confidence
python dvcon_link.py abort | blob [file]

python dvcon_eth.py model ../model/yolo26n_npu.bin Realtek
python dvcon_eth.py frame uploads/<x>.bin Realtek

python dvcon_image.py photo.jpg frame.bin    # frame + .rgb companion
python dvcon_draw.py frame.bin out.png [boxes.json]
```

Every command prints JSON.

## Files

| File | Role |
|---|---|
| `dvcon_link.py` | board operations over JTAG; imports `../compiler/npu_isa.py` for the blob header and the box records |
| `dvcon_eth.py` | bulk transfer over raw Ethernet (Npcap), confirmed over JTAG |
| `dvcon_image.py` | image -> `<name>.bin` (3 x 640 x 640 INT8 CHW, what the NPU reads) and `<name>.rgb` (what gets drawn on) |
| `dvcon_draw.py` | draws the boxes on the frame as a PNG, no Pillow needed |
| `server.py`, `dashboard.*` | the UI |

## What the threshold means

The NPU compares the best class **logit** of each cell with the CONF register,
a signed Q8.8 number. The tools convert: `conf 0.25 -> logit -1.10 -> -281`.
Box records carry the logit, and the host shows `sigmoid(logit)`.

## Setup that has to be right

- PHY 1 in MII: **JP2 on pins 2-3**, cable in **J5**, board power-cycled after
  moving the jumper.
- PC NIC at 100 Mbps full duplex (Administrator PowerShell):
  `Set-NetAdapterAdvancedProperty -Name Ethernet -RegistryKeyword '*SpeedDuplex' -RegistryValue 4`
- One cable operation at a time: stop the server before running `quartus_pgm` by hand.
- The model blob must be the one compiled for this bitstream
  (`model/yolo26n_npu.bin`); IDENT reports array edge 16 and version 0x0200.
