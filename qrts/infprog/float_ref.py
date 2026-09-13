#!/usr/bin/env python3
"""float_ref.py -- the original FP32 YOLO26n on the same frame the NPU gets.

    <torch python> float_ref.py <frame.bin> [conf] [weights.pt]

Prints one JSON object: {"ok", "boxes": [{x1,y1,x2,y2,conf,cls,name}], ...}.

Needs torch + ultralytics, so server.py runs it with the miniconda python
(DVCON_TORCH_PY) rather than importing it. Input is the prepared .bin itself,
dequantised (code * 255/127), so the float and INT8 paths see the same 640x640
pixels and any difference between them is the NPU's quantisation alone.
Boxes come back in that 640-space frame, the same coordinates as the NPU's.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WEIGHTS = HERE.parent.parent / "Model" / "yolo26n.pt"


def main(argv):
    frame = Path(argv[1])
    conf = float(argv[2]) if len(argv) > 2 else 0.25
    weights = Path(argv[3]) if len(argv) > 3 else WEIGHTS
    codes = np.frombuffer(frame.read_bytes(), np.int8)
    side = int(round((codes.size / 3) ** 0.5))
    if 3 * side * side != codes.size:
        raise ValueError(f"{frame.name}: {codes.size} bytes is not 3 x N x N")
    rgb = np.clip(np.round(codes.astype(np.float32) * 255 / 127), 0, 255).astype(np.uint8)
    bgr = rgb.reshape(3, side, side)[::-1].transpose(1, 2, 0).copy()

    from ultralytics import YOLO
    import time
    t0 = time.time()
    res = YOLO(str(weights)).predict(bgr, imgsz=side, conf=conf, verbose=False)[0]
    names = res.names
    boxes = [{"x1": float(x1), "y1": float(y1), "x2": float(x2), "y2": float(y2),
              "conf": float(c), "cls": int(k), "name": names[int(k)], "level": -3}
             for (x1, y1, x2, y2), c, k in zip(res.boxes.xyxy.tolist(),
                                                res.boxes.conf.tolist(),
                                                res.boxes.cls.tolist())]
    boxes.sort(key=lambda b: -b["conf"])
    print(json.dumps({"ok": True, "boxes": boxes, "count": len(boxes),
                      "seconds": round(time.time() - t0, 2),
                      "weights": str(weights)}))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv))
    except Exception as exc:                  # report, never a bare traceback
        print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"}))
        sys.exit(1)
