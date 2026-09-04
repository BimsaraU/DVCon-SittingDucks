#!/usr/bin/env python3
"""dvcon_draw.py -- draw the board's detections onto the input frame.

Runs on the HOST CPU, not the FPGA. The accelerator writes a box list to SDRAM
and that is all it owes anyone; turning boxes into a picture is presentation,
and putting a rasteriser in the fabric to do it would spend logic on something
a laptop does in milliseconds.

No Pillow, no numpy. The frame is a raw 640x640x3 INT8 bitmap and the output is
a PNG written with zlib and struct, so this works on a bare Python install --
the same constraint the rest of infprog is built to.

The coordinates are Q4 fixed point (see cmd_boxes in tools/dvcon_jtag.tcl): the
register value is sixteenths of a pixel.
"""

from __future__ import annotations

import struct
import sys
import zlib
from pathlib import Path

IMG_W = IMG_H = 640
CHANNELS = 3

# COCO class names, index order as the exporter emits them. Only used for
# labelling; an out-of-range index is shown as its number rather than guessed.
COCO = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]

# Distinct enough to tell apart on a busy frame, and readable on both light and
# dark image content.
PALETTE = [
    (255, 64, 64), (64, 200, 255), (120, 255, 120), (255, 200, 60),
    (220, 120, 255), (255, 140, 40), (80, 255, 220), (255, 90, 170),
]


def load_frame(path: Path) -> list[bytearray]:
    """Frame file -> rows of RGB bytes, ready to draw on.

    Two formats are accepted, and which one arrives matters more than it looks:

      <name>.rgb  640x640x3 uint8 HWC -- the actual picture. Preferred.
      <name>.bin  3x640x640 INT8 CHW  -- what the accelerator eats.

    The .bin is NOT an image. It is signed, quantised and plane-ordered, so
    rendering it directly gives a dark, tinted, banded mess that looks like
    noise -- which is exactly what a detection drawn on it appeared to be.
    dvcon_image.py writes the .rgb companion next to every .bin for this
    reason, so if one exists it is used and the quantised tensor is not
    touched.

    When only the .bin exists (a frame prepared by some other tool), it is
    dequantised back to 8-bit rather than shown raw: INT8 0..127 covers the
    [0,1] pixel range, so the value is doubled. The result is approximate and
    a bit posterised, but it is a picture rather than a puzzle.
    """
    rgb = path.with_suffix(".rgb")
    if rgb.exists() and rgb.stat().st_size == IMG_W * IMG_H * 3:
        data = rgb.read_bytes()
        return [bytearray(data[y * IMG_W * 3:(y + 1) * IMG_W * 3])
                for y in range(IMG_H)]

    data = path.read_bytes()
    want = IMG_W * IMG_H * CHANNELS
    if len(data) != want:
        raise ValueError(f"{path.name}: {len(data)} bytes, expected {want} "
                         f"({IMG_W}x{IMG_H}x{CHANNELS} INT8 CHW). Load an "
                         f"image through the dashboard to build one.")

    # INT8 -> visible 8-bit. Negatives clamp to black; the exporter keeps the
    # input in the positive half, so anything below zero is out-of-range data.
    lut = bytes(min(255, max(0, (v - 256 if v > 127 else v) * 2))
                for v in range(256))
    plane = IMG_W * IMG_H
    rows = []
    for y in range(IMG_H):
        row = bytearray(IMG_W * 3)
        base = y * IMG_W
        for x in range(IMG_W):
            i = base + x
            row[x * 3 + 0] = lut[data[i]]
            row[x * 3 + 1] = lut[data[plane + i]]
            row[x * 3 + 2] = lut[data[2 * plane + i]]
        rows.append(row)
    return rows


def put(rows, x: int, y: int, rgb: tuple[int, int, int]):
    if 0 <= x < IMG_W and 0 <= y < IMG_H:
        o = x * 3
        rows[y][o:o + 3] = bytes(rgb)


def rect(rows, x1: int, y1: int, x2: int, y2: int, rgb, width: int = 2):
    x1, x2 = sorted((max(0, x1), min(IMG_W - 1, x2)))
    y1, y2 = sorted((max(0, y1), min(IMG_H - 1, y2)))
    for w in range(width):
        for x in range(x1, x2 + 1):
            put(rows, x, y1 + w, rgb)
            put(rows, x, y2 - w, rgb)
        for y in range(y1, y2 + 1):
            put(rows, x1 + w, y, rgb)
            put(rows, x2 - w, y, rgb)


# A 5x7 bitmap font, enough for the label text. Drawing text without a font
# library means carrying the glyphs; this is the smallest set that covers
# class names and scores.
GLYPHS = {
    "0": ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "2": ("01110", "10001", "00001", "00010", "00100", "01000", "11111"),
    "3": ("11111", "00010", "00100", "00010", "00001", "10001", "01110"),
    "4": ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
    "5": ("11111", "10000", "11110", "00001", "00001", "10001", "01110"),
    "6": ("00110", "01000", "10000", "11110", "10001", "10001", "01110"),
    "7": ("11111", "00001", "00010", "00100", "01000", "01000", "01000"),
    "8": ("01110", "10001", "10001", "01110", "10001", "10001", "01110"),
    "9": ("01110", "10001", "10001", "01111", "00001", "00010", "01100"),
    ".": ("00000", "00000", "00000", "00000", "00000", "01100", "01100"),
    "%": ("11001", "11010", "00010", "00100", "01000", "01011", "10011"),
    " ": ("00000",) * 7,
    "?": ("01110", "10001", "00001", "00010", "00100", "00000", "00100"),
    "#": ("01010", "01010", "11111", "01010", "11111", "01010", "01010"),
}
# a-z as real 5x7 glyphs. A fallback block for every letter was tried
# first and produced labels like 'AAAAA 92' -- present and positioned, but
# unreadable, which is worse than a number because it looks like text.
GLYPHS.update({
    "a": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "b": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "c": ("01110", "10001", "10000", "10000", "10000", "10001", "01110"),
    "d": ("11100", "10010", "10001", "10001", "10001", "10010", "11100"),
    "e": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "f": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "g": ("01110", "10001", "10000", "10111", "10001", "10001", "01111"),
    "h": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "i": ("01110", "00100", "00100", "00100", "00100", "00100", "01110"),
    "j": ("00111", "00010", "00010", "00010", "00010", "10010", "01100"),
    "k": ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    "l": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "m": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "n": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "o": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "p": ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    "q": ("01110", "10001", "10001", "10001", "10101", "10010", "01101"),
    "r": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "s": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "t": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "u": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "v": ("10001", "10001", "10001", "10001", "10001", "01010", "00100"),
    "w": ("10001", "10001", "10001", "10101", "10101", "11011", "10001"),
    "x": ("10001", "10001", "01010", "00100", "01010", "10001", "10001"),
    "y": ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
    "z": ("11111", "00001", "00010", "00100", "01000", "10000", "11111"),
})

def text(rows, x: int, y: int, s: str, rgb):
    cx = x
    for ch in s.lower():
        g = GLYPHS.get(ch, GLYPHS["?"])
        for dy, line in enumerate(g):
            for dx, bit in enumerate(line):
                if bit == "1":
                    put(rows, cx + dx, y + dy, rgb)
        cx += 6
        if cx > IMG_W:
            break


def write_png(path: Path, rows: list[bytearray]):
    """Minimal PNG writer: filter type 0 on each row, one IDAT, no interlace."""
    raw = b"".join(b"\x00" + bytes(r) for r in rows)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", IMG_W, IMG_H, 8, 2, 0, 0, 0)  # 8-bit RGB
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) +
           chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    path.write_bytes(png)


def draw(frame_path: Path, boxes: list[dict], out_path: Path,
         q4: bool = True) -> dict:
    """Render boxes onto the frame and write a PNG.

    boxes are dicts as read_boxes() returns them: x1/y1/x2/y2 already divided
    by 16 (Q4 -> pixels), plus score and cls.
    """
    rows = load_frame(frame_path)
    drawn = 0
    for i, b in enumerate(boxes):
        x1, y1 = int(round(b["x1"])), int(round(b["y1"]))
        x2, y2 = int(round(b["x2"])), int(round(b["y2"]))
        if x2 <= x1 or y2 <= y1:
            continue                      # degenerate, nothing to draw
        colour = PALETTE[b.get("cls", 0) % len(PALETTE)]
        rect(rows, x1, y1, x2, y2, colour, width=2)

        cls = b.get("cls", -1)
        name = COCO[cls] if 0 <= cls < len(COCO) else f"#{cls}"
        label = f"{name} {b.get('score', 0)}"
        ly = y1 - 9 if y1 >= 9 else y2 + 2
        for dx in range(len(label) * 6 + 2):      # dark plate behind the text
            for dy in range(9):
                put(rows, x1 + dx, ly + dy - 1, (0, 0, 0))
        text(rows, x1 + 1, ly, label, colour)
        drawn += 1

    write_png(out_path, rows)
    return {"ok": True, "out": str(out_path), "boxes": len(boxes),
            "drawn": drawn}


def main(argv: list[str]) -> int:
    import json
    if len(argv) < 3:
        print(__doc__)
        print("usage: dvcon_draw.py <frame.bin> <out.png> [boxes.json]")
        print("  with no boxes.json, the box list is read from the board.")
        return 2

    frame, out = Path(argv[1]), Path(argv[2])
    if len(argv) > 3:
        boxes = json.loads(Path(argv[3]).read_text())
        if isinstance(boxes, dict):
            boxes = boxes.get("boxes", [])
    else:
        import dvcon_link as link
        r = link.read_boxes()
        if not r.ok:
            print(f"error reading boxes: {r.error}")
            return 1
        boxes = r.data.get("boxes", [])

    try:
        print(json.dumps(draw(frame, boxes, out), indent=2))
    except ValueError as exc:
        print(f"error: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
