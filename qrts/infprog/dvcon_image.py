#!/usr/bin/env python3
"""dvcon_image.py -- any image file -> the two files the board and the UI need.

The accelerator eats a raw INT8 CHW plane and nothing else; there is no image
codec in the fabric and there should not be one. So every picture the user
loads is converted here, on the host, into:

    <stem>.bin   3 x 640 x 640 INT8, CHW      -> transferred into SDRAM
    <stem>.rgb   640 x 640 x 3 uint8, HWC     -> what the annotator draws on

The .rgb companion exists because the .bin is NOT a picture. It is signed,
quantised, plane-ordered data; reading it back as RGB888 produces three tinted
vertical bands of noise. That is exactly the "bounding box was just noise"
symptom -- the drawing code was rendering the quantised tensor, not the image.

This mirrors DVCFinal/tools/prepare_image.py so a frame prepared by either is
byte-identical. Pillow does the decoding (it handles JPEG/PNG/BMP/GIF/WEBP);
if it is missing, a pure-Python PNG and BMP reader covers the common cases so
the dashboard still works on a bare Python install.

LETTERBOX, NOT STRETCH
----------------------
Aspect ratio is preserved and the remainder padded with grey 114, matching
ultralytics. Stretching instead shifts every box the detector produces, which
reads as an accuracy bug in the accelerator rather than a preprocessing one.
The letterbox scale and padding are returned so a box in 640-space can be
mapped back onto the original photograph.
"""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

IMGSZ = 640
PAD = 114                      # ultralytics' grey
IN_SCALE = 1.0 / 127.0         # must match the export's input quantisation


# ---------------------------------------------------------------------------
# decoding
# ---------------------------------------------------------------------------

def _decode_pillow(path: Path):
    from PIL import Image
    im = Image.open(path).convert("RGB")
    return im.size, list(im.tobytes())


def _decode_bmp(data: bytes):
    """24/32-bit uncompressed BMP. Rows are bottom-up and padded to 4 bytes."""
    if data[:2] != b"BM":
        raise ValueError("not a BMP")
    off = struct.unpack_from("<I", data, 10)[0]
    hdr = struct.unpack_from("<I", data, 14)[0]
    w, h = struct.unpack_from("<ii", data, 18)
    bpp = struct.unpack_from("<H", data, 28)[0]
    comp = struct.unpack_from("<I", data, 30)[0]
    if hdr < 40 or comp != 0 or bpp not in (24, 32):
        raise ValueError(f"unsupported BMP: {bpp}bpp compression {comp}")

    flip = h > 0
    h = abs(h)
    stride = ((w * bpp // 8) + 3) & ~3
    px = bytearray(w * h * 3)
    for y in range(h):
        src = off + (h - 1 - y if flip else y) * stride
        d = y * w * 3
        for x in range(w):
            s = src + x * (bpp // 8)
            px[d + x * 3 + 0] = data[s + 2]      # BMP stores BGR
            px[d + x * 3 + 1] = data[s + 1]
            px[d + x * 3 + 2] = data[s + 0]
    return (w, h), px


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    return a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)


def _decode_png(data: bytes):
    """Non-interlaced 8-bit PNG, colour types 0/2/3/4/6."""
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    pos, idat, plte, trns = 8, bytearray(), b"", b""
    w = h = depth = ctype = interlace = 0
    while pos < len(data):
        ln = struct.unpack_from(">I", data, pos)[0]
        tag = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + ln]
        pos += 12 + ln
        if tag == b"IHDR":
            w, h, depth, ctype, _, _, interlace = struct.unpack(">IIBBBBB", body)
        elif tag == b"PLTE":
            plte = body
        elif tag == b"tRNS":
            trns = body
        elif tag == b"IDAT":
            idat += body
        elif tag == b"IEND":
            break
    if depth != 8 or interlace != 0:
        raise ValueError(f"unsupported PNG: {depth}-bit, interlace {interlace}")

    nch = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}[ctype]
    raw = zlib.decompress(bytes(idat))
    stride = w * nch
    out = bytearray(w * h * 3)
    prev = bytearray(stride)
    p = 0
    for y in range(h):
        ft = raw[p]; p += 1
        line = bytearray(raw[p:p + stride]); p += stride
        if ft:
            for i in range(stride):
                a = line[i - nch] if i >= nch else 0
                b = prev[i]
                c = prev[i - nch] if i >= nch else 0
                if ft == 1:   line[i] = (line[i] + a) & 0xFF
                elif ft == 2: line[i] = (line[i] + b) & 0xFF
                elif ft == 3: line[i] = (line[i] + ((a + b) >> 1)) & 0xFF
                elif ft == 4: line[i] = (line[i] + _paeth(a, b, c)) & 0xFF
        prev = line
        d = y * w * 3
        for x in range(w):
            s = x * nch
            if ctype in (0, 4):
                g = line[s]; out[d + x * 3:d + x * 3 + 3] = bytes((g, g, g))
            elif ctype in (2, 6):
                out[d + x * 3:d + x * 3 + 3] = line[s:s + 3]
            else:                                    # palette
                i3 = line[s] * 3
                out[d + x * 3:d + x * 3 + 3] = plte[i3:i3 + 3]
    return (w, h), out


def decode(path: Path):
    """-> ((w, h), flat HWC RGB bytes). Pillow when available, else PNG/BMP."""
    try:
        return _decode_pillow(path)
    except ImportError:
        pass
    data = path.read_bytes()
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return _decode_png(data)
    if data[:2] == b"BM":
        return _decode_bmp(data)
    raise ValueError(
        f"{path.name}: no decoder. Without Pillow only PNG and uncompressed "
        f"BMP can be read -- run 'pip install pillow' for JPEG and the rest.")


# ---------------------------------------------------------------------------
# letterbox
# ---------------------------------------------------------------------------

def letterbox(size, px, out_size: int = IMGSZ):
    """Aspect-preserving resize onto a grey canvas.

    Nearest-neighbour rather than bilinear: this path exists for the no-Pillow
    case, where a bilinear resample in pure Python over 1.2 M pixels is slow
    enough to time the browser out. When Pillow is present it does the resize
    instead, with proper filtering.
    """
    w, h = size
    scale = min(out_size / w, out_size / h)
    nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
    ox, oy = (out_size - nw) // 2, (out_size - nh) // 2

    canvas = bytearray(bytes((PAD, PAD, PAD)) * (out_size * out_size))
    for y in range(nh):
        sy = min(h - 1, int(y / scale))
        srow = sy * w * 3
        drow = (y + oy) * out_size * 3 + ox * 3
        for x in range(nw):
            sx = min(w - 1, int(x / scale))
            s = srow + sx * 3
            canvas[drow + x * 3:drow + x * 3 + 3] = px[s:s + 3]
    return canvas, scale, ox, oy


def _letterbox_pillow(path: Path, out_size: int):
    from PIL import Image
    im = Image.open(path).convert("RGB")
    w, h = im.size
    scale = min(out_size / w, out_size / h)
    nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
    im2 = im.resize((nw, nh), Image.BILINEAR)
    canvas = Image.new("RGB", (out_size, out_size), (PAD, PAD, PAD))
    ox, oy = (out_size - nw) // 2, (out_size - nh) // 2
    canvas.paste(im2, (ox, oy))
    return bytearray(canvas.tobytes()), scale, ox, oy, (w, h)


# ---------------------------------------------------------------------------
# quantise + write
# ---------------------------------------------------------------------------

def quantise(rgb: bytearray, out_size: int = IMGSZ,
             in_scale: float = IN_SCALE) -> bytes:
    """HWC uint8 -> CHW INT8, matching prepare_image.py exactly.

        int8 = clip(round((pixel/255) / in_scale), -128, 127)

    With the default scale of 1/127 that maps [0,255] onto [0,127], keeping
    the input in the positive half of the INT8 range -- which is what the
    exporter's first-layer scale assumes.
    """
    n = out_size * out_size
    k = 1.0 / (255.0 * in_scale)
    lut = bytes(max(-128, min(127, int(round(v * k)))) & 0xFF for v in range(256))

    planes = bytearray(3 * n)
    for c in range(3):
        base = c * n
        for i in range(n):
            planes[base + i] = lut[rgb[i * 3 + c]]
    return bytes(planes)


def png_preview(rgb: bytearray, path: Path, size: int = IMGSZ):
    """Write the letterboxed image as a PNG so the browser can show it."""
    row = size * 3
    raw = b"".join(b"\x00" + bytes(rgb[y * row:(y + 1) * row]) for y in range(size))

    def chunk(tag: bytes, body: bytes) -> bytes:
        return (struct.pack(">I", len(body)) + tag + body +
                struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) +
                     chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def prepare(src: Path, out_bin: Path, out_size: int = IMGSZ,
            in_scale: float = IN_SCALE, preview: Path | None = None) -> dict:
    """Full pipeline: decode -> letterbox -> .bin + .rgb (+ PNG preview)."""
    src, out_bin = Path(src), Path(out_bin)
    try:
        rgb, scale, ox, oy, orig = _letterbox_pillow(src, out_size)
        decoder = "pillow"
    except ImportError:
        orig, px = decode(src)
        rgb, scale, ox, oy = letterbox(orig, px, out_size)
        decoder = "builtin"

    out_bin.write_bytes(quantise(rgb, out_size, in_scale))
    out_rgb = out_bin.with_suffix(".rgb")
    out_rgb.write_bytes(bytes(rgb))
    if preview:
        png_preview(rgb, Path(preview), out_size)

    return {
        "ok": True,
        "source": str(src),
        "bin": str(out_bin),
        "rgb": str(out_rgb),
        "bytes": out_bin.stat().st_size,
        "decoder": decoder,
        "original": list(orig),
        "imgsz": out_size,
        "in_scale": in_scale,
        # Enough to map a detection in 640-space back onto the original.
        "letterbox": {"scale": round(scale, 6), "pad_x": ox, "pad_y": oy},
    }


def main(argv):
    import json
    if len(argv) < 3:
        print(__doc__)
        print("usage: dvcon_image.py <image> <out.bin> [imgsz] [in_scale]")
        return 2
    size = int(argv[3]) if len(argv) > 3 else IMGSZ
    sc = float(argv[4]) if len(argv) > 4 else IN_SCALE
    try:
        print(json.dumps(prepare(Path(argv[1]), Path(argv[2]), size, sc),
                         indent=2))
    except (ValueError, OSError) as exc:
        print(f"error: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main(sys.argv))
