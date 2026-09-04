#!/usr/bin/env python3
"""Relocate weight/bias pointers that export_yolo26n.py's old `if v:` skipped.

The exporter decided whether a pointer needed relocating by testing its VALUE.
Layer 0's weights are the first block in the blob, so its offset is legitimately
0 -- indistinguishable from "no weights" by value -- and it was left
unrelocated. The conv engine then read the 32-byte file header as its weight
tile and layer 0's output came back saturated at +127, with no error anywhere.

The exporter is fixed to key off the op instead. This repairs an already
exported blob in place, so a re-export (which needs the .pt and torch) is not
required.
"""
import struct
import sys
from pathlib import Path

OP_CONV, F_BIAS, DESC_BYTES = 1, 2, 64

src = Path(sys.argv[1] if len(sys.argv) > 1 else "model/yolo26n_a16.bin")
b = bytearray(src.read_bytes())

magic, ver, n_layers, stride = struct.unpack_from("<4I", b, 0)
if magic != 0x594F4C4F:
    sys.exit(f"not a YOLO blob: magic 0x{magic:08X}")
if stride != DESC_BYTES:
    sys.exit(f"unexpected descriptor stride {stride}")

hdr_bytes   = 32
weight_base = hdr_bytes + n_layers * DESC_BYTES
blob_hi     = len(b)
print(f"{n_layers} layers, weight_base = 0x{weight_base:08X}, file 0x{blob_hi:X}")

fixed = 0
for i in range(n_layers):
    rec = hdr_bytes + i * DESC_BYTES
    op    = struct.unpack_from("<I", b, rec + 0x00)[0]
    flags = struct.unpack_from("<I", b, rec + 0x04)[0]
    present = ([0x14] if op == OP_CONV else []) + ([0x18] if flags & F_BIAS else [])
    for off in present:
        v = struct.unpack_from("<I", b, rec + off)[0]
        if weight_base <= v < blob_hi:
            continue                     # already an absolute address
        a = v + weight_base
        if not (weight_base <= a < blob_hi):
            sys.exit(f"descriptor {i} offset 0x{off:02X}: 0x{v:08X} -> 0x{a:08X} "
                     f"is outside [0x{weight_base:08X}, 0x{blob_hi:08X})")
        struct.pack_into("<I", b, rec + off, a)
        print(f"  descriptor {i:3d} +0x{off:02X}: 0x{v:08X} -> 0x{a:08X}")
        fixed += 1

# Re-check every pointer that should exist, by op rather than by value.
bad = 0
for i in range(n_layers):
    rec = hdr_bytes + i * DESC_BYTES
    op    = struct.unpack_from("<I", b, rec + 0x00)[0]
    flags = struct.unpack_from("<I", b, rec + 0x04)[0]
    for off in ([0x14] if op == OP_CONV else []) + ([0x18] if flags & F_BIAS else []):
        v = struct.unpack_from("<I", b, rec + off)[0]
        if not (weight_base <= v < blob_hi):
            print(f"  [err] descriptor {i} +0x{off:02X} = 0x{v:08X} out of range")
            bad += 1

print(f"relocated {fixed} pointer(s); {bad} still out of range")
if bad:
    sys.exit(1)
src.write_bytes(bytes(b))
print(f"wrote {src}")
