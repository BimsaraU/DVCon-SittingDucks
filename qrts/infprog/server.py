#!/usr/bin/env python3
"""server.py -- local diagnostics dashboard for the DE2-115 accelerator.

    python server.py            # then open http://127.0.0.1:8750/

Binds 127.0.0.1 only. Every endpoint drives real hardware over the USB-Blaster,
so this is deliberately not something to expose on a network: there is no auth,
and "write 2.5 MB into SDRAM" is one HTTP call.

Only the standard library is used. Adding Flask would mean a virtualenv on a
machine whose whole job is running Quartus, and the routing here is a dozen
paths.

Long operations (model load is ~4 minutes) run on a worker thread and report
progress by polling /api/job. The browser never holds a request open for
minutes, because Quartus can stall the cable and a hung fetch looks identical
to a hung board.
"""

from __future__ import annotations

import json
import os
import threading
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import base64
import subprocess

import dvcon_link as link
import dvcon_draw as draw
import dvcon_image as image

HERE = Path(__file__).resolve().parent
QRTS = HERE.parent
UPLOADS = HERE / "uploads"
# Windows reserves scattered TCP ranges for Hyper-V/WSL (see
# `netsh int ipv4 show excludedportrange protocol=tcp`); binding inside one
# fails with WinError 10013, which reads like a permissions problem rather than
# a reserved port. 8750 falls inside 8730-8829 on this machine, so the default
# moved out of it. Override with DVCON_PORT if the new one is taken too.
PORT = int(os.environ.get("DVCON_PORT", "8321"))

# The frame most recently prepared from an uploaded image. /api/render draws on
# this, so a detection is always drawn on the picture that was actually fed to
# the accelerator rather than on whatever path happens to be typed in the box.
CURRENT = {"bin": "", "rgb": "", "source": "", "letterbox": None,
           "npu_boxes": None, "npu_from": ""}

# torch + ultralytics live in miniconda, not in the python serving this page.
TORCH_PY = os.environ.get("DVCON_TORCH_PY", r"C:\Users\Bimsara\miniconda3\python.exe")


def _iou(a, b):
    iw = min(a["x2"], b["x2"]) - max(a["x1"], b["x1"])
    ih = min(a["y2"], b["y2"]) - max(a["y1"], b["y1"])
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    area = lambda r: (r["x2"] - r["x1"]) * (r["y2"] - r["y1"])
    return inter / (area(a) + area(b) - inter)


def compare_boxes(ref: list, npu: list, thr: float = 0.5) -> dict:
    """Greedy same-class match, best confidence first, at IoU >= thr."""
    used, pairs = set(), []
    for r in ref:
        best, bi = thr, -1
        for i, n in enumerate(npu):
            if i not in used and n["cls"] == r["cls"]:
                v = _iou(r, n)
                if v >= best:
                    best, bi = v, i
        if bi >= 0:
            used.add(bi)
            pairs.append((r, npu[bi], best))
    return {"matched": len(pairs), "float_only": len(ref) - len(pairs),
            "npu_only": len(npu) - len(pairs),
            "mean_iou": round(sum(p[2] for p in pairs) / len(pairs), 3) if pairs else None,
            "mean_conf_diff": round(sum(abs(p[0]["conf"] - p[1]["conf"]) for p in pairs)
                                    / len(pairs), 3) if pairs else None,
            "missed": [f"{r['name']} {r['conf']:.2f}" for r in ref
                       if all(r is not p[0] for p in pairs)]}


def float_frame(frame: Path, conf: float) -> dict:
    """Original FP32 network on the same frame, drawn, and scored against the
    last NPU result (board or simulation) if there is one."""
    if not Path(TORCH_PY).exists():
        return {"ok": False, "error": f"no torch python at {TORCH_PY}; set DVCON_TORCH_PY"}
    try:
        p = subprocess.run([TORCH_PY, str(HERE / "float_ref.py"), str(frame), str(conf)],
                           capture_output=True, text=True, timeout=300)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "error": f"float reference: {exc}"}
    lines = [l for l in p.stdout.splitlines() if l.startswith("{")]
    if not lines:
        return {"ok": False, "error": (p.stderr or p.stdout or "no output")[-600:]}
    r = json.loads(lines[-1])
    if not r.get("ok"):
        return r
    d = draw.draw(frame, r["boxes"], HERE / "render.png")
    r.update({"drawn": d["drawn"], "boxes_list": r.pop("boxes"),
              "source": "original FP32 model (PyTorch) on this PC"})
    if CURRENT["npu_boxes"] is not None:
        r["compare"] = compare_boxes(r["boxes_list"], CURRENT["npu_boxes"])
        r["compare"]["against"] = CURRENT["npu_from"]
    return r


class Job:
    """One long-running board operation.

    Only one runs at a time: the cable is a single exclusive resource, and two
    concurrent quartus_stp processes fight over it in ways that surface as
    unrelated-looking JTAG errors.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.thread: threading.Thread | None = None
        self.name = ""
        self.running = False
        self.log: list[str] = []
        self.result: dict | None = None

    def busy(self) -> bool:
        return self.running

    def start(self, name: str, fn) -> bool:
        with self.lock:
            if self.running:
                return False
            self.running = True
            self.name = name
            self.log = [f"started: {name}"]
            self.result = None

        def wrap():
            try:
                r = fn()
                out = r.as_dict() if hasattr(r, "as_dict") else r
                with self.lock:
                    if out.get("stdout"):
                        self.log.extend(out["stdout"].splitlines())
                    self.result = out
            except Exception:
                with self.lock:
                    self.result = {"ok": False, "error": traceback.format_exc()}
            finally:
                with self.lock:
                    self.running = False
                    self.log.append("finished")

        self.thread = threading.Thread(target=wrap, daemon=True)
        self.thread.start()
        return True

    def snapshot(self) -> dict:
        with self.lock:
            return {"name": self.name, "running": self.running,
                    "log": list(self.log), "result": self.result}



# The board's LEDs, described once here so the UI and the README cannot drift
# apart from the RTL. Order matches dvcon_top's assignments.
LED_GUIDE = {
    "green": [
        {"id": "LEDG0", "name": "Heartbeat",
         "meaning": "About 1.5 Hz. Solid or dark means the clock or reset is dead."},
        {"id": "LEDG1", "name": "Model loaded",
         "meaning": "The model is in SDRAM and verified. Dark after programming or KEY0."},
        {"id": "LEDG2", "name": "Image loaded",
         "meaning": "A frame is in SDRAM and verified. With G1 lit, KEY3 runs inference."},
        {"id": "LEDG3", "name": "Ethernet RX",
         "meaning": "A frame was received and passed the MAC."},
        {"id": "LEDG4", "name": "Ethernet TX", "meaning": "Transmitting."},
        {"id": "LEDG5", "name": "Ethernet RX error",
         "meaning": "Frame failed its FCS. Steady flicker means a physical-layer problem."},
        {"id": "LEDG6", "name": "File transfer",
         "meaning": "The JTAG memory window is moving data."},
        {"id": "LEDG7", "name": "NPU busy",
         "meaning": "The sequencer is walking the descriptor table."},
        {"id": "LEDG8", "name": "Reset released",
         "meaning": "Steady on in normal operation. Dark means KEY0 is held."},
    ],
    "red": [
        {"id": "LEDR0", "name": "IDLE", "meaning": "Ready for START."},
        {"id": "LEDR1", "name": "FETCH", "meaning": "Reading the next 64-byte descriptor."},
        {"id": "LEDR2", "name": "CONV input", "meaning": "Loading the input band into the on-chip buffer."},
        {"id": "LEDR3", "name": "CONV weights", "meaning": "Loading weight tiles, per-channel parameters or the SiLU table."},
        {"id": "LEDR4", "name": "CONV array", "meaning": "The 16x16 systolic array is streaming pixels."},
        {"id": "LEDR5", "name": "CONV store", "meaning": "Writing the requantised output tile to SDRAM."},
        {"id": "LEDR6", "name": "ADD", "meaning": "Residual add."},
        {"id": "LEDR7", "name": "UPSAMPLE", "meaning": "Nearest-neighbour x2."},
        {"id": "LEDR8", "name": "MAXPOOL", "meaning": "SPPF 5x5 pooling."},
        {"id": "LEDR9", "name": "SOFTMAX", "meaning": "Attention probabilities."},
        {"id": "LEDR10", "name": "PACK", "meaning": "Converting the host frame to the NPU layout."},
        {"id": "LEDR11", "name": "DETECT", "meaning": "Decoding boxes and writing the box list."},
        {"id": "LEDR12", "name": "DONE", "meaning": "Held after a frame until the next START."},
        {"id": "LEDR13", "name": "ERROR", "meaning": "Held until the next START. STATUS gives the code."},
        {"id": "LEDR14", "name": "DMA read", "meaning": "SDRAM to NPU burst in progress."},
        {"id": "LEDR15", "name": "DMA write", "meaning": "NPU to SDRAM burst in progress."},
        {"id": "LEDR16", "name": "MAC slot", "meaning": "A pixel is entering the array this instant."},
        {"id": "LEDR17", "name": "Found boxes", "meaning": "The last frame produced at least one detection."},
    ],
    "hex": [
        {"id": "KEY3", "name": "Run button",
         "meaning": "Press to run one inference on the loaded model and image, conf 0.25 "
                    "unless the host set another. Boxes are read back with Read boxes."},
        {"id": "HEX7-6", "name": "Layer", "meaning": "Model layer (0-23) of the running descriptor."},
        {"id": "HEX5", "name": "Operation", "meaning": "C conv, A add, U upsample, P pool, S softmax, I input pack, d detect."},
        {"id": "HEX3-0", "name": "Index / result", "meaning": "Descriptor index in hex while running; 'donE' and the box count when finished; 'Err' on an error."},
    ],
}


def eth_diag() -> dict:
    """The PC-side half of the Ethernet story: link speed and Npcap.

    Worth its own endpoint because the single most useful fact about this link
    is not on the board at all. The FPGA MAC is MII, which tops out at
    100 Mbit. If Windows reports the NIC at 1 Gbps then the PHY negotiated
    gigabit, which means it is in RGMII -- 125 MHz, data on both clock edges --
    and mii_rx_adapter samples only the rising edge. Half the nibbles are
    dropped, so frames are "received" and counted but their contents are
    rubbish. That reads as a protocol bug and is a jumper.
    """
    out = {"ok": True, "adapters": [], "interfaces": [], "notes": []}

    ps = ("Get-NetAdapter | Select-Object Name,Status,LinkSpeed,"
          "InterfaceDescription | ConvertTo-Json -Compress")
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                           capture_output=True, text=True, timeout=30)
        if r.returncode == 0 and r.stdout.strip():
            got = json.loads(r.stdout)
            out["adapters"] = got if isinstance(got, list) else [got]
    except Exception as exc:
        out["notes"].append(f"could not query adapters: {exc}")

    try:
        import dvcon_eth as eth
        out["interfaces"] = eth.Pcap().interfaces()
    except Exception as exc:
        out["notes"].append(f"Npcap unavailable: {exc}")

    wired = [a for a in out["adapters"]
             if a.get("Status") == "Up"
             and "Wi-Fi" not in (a.get("Name") or "")
             and "Wireless" not in (a.get("InterfaceDescription") or "")
             and "Virtual" not in (a.get("InterfaceDescription") or "")
             and "Tunnel" not in (a.get("InterfaceDescription") or "")
             and "Bluetooth" not in (a.get("InterfaceDescription") or "")]
    out["wired"] = wired

    if not wired:
        out["verdict"] = ("no wired adapter is up. The board's ENET1 (J5) has to be "
                          "cabled to this machine's Ethernet port -- Wi-Fi "
                          "cannot inject raw L2 frames.")
        out["ok"] = False
    else:
        speeds = [w.get("LinkSpeed", "") for w in wired]
        if any("Gbps" in sp for sp in speeds):
            out["verdict"] = (
                f"wired link is up at {speeds[0]}. That is the problem: the "
                f"FPGA MAC is MII and MII stops at 100 Mbit. A gigabit link "
                f"means the PHY is in RGMII. Move the mode jumper to pins 2-3, hard-reset "
                f"the board, and force this NIC to 100 Mbps Full Duplex.")
            out["ok"] = False
        else:
            out["verdict"] = f"wired link up at {speeds[0]} -- compatible with MII."

    out["fix"] = {
        "jumper": "Pins 2-3 selects MII -- JP1 for ENET0 (J4), JP2 for ENET1 "
                  "(J5). Both ship in RGMII, and the bitstream must be pinned "
                  "to whichever PHY the cable is in. "
                  "Power-cycle after moving it; a warm reset is not enough.",
        "nic": "Set-NetAdapterAdvancedProperty -Name Ethernet "
               "-RegistryKeyword '*SpeedDuplex' -RegistryValue 4",
        "nic_note": "value 4 is 100 Mbps Full Duplex on the Realtek driver. "
                    "Needs an Administrator PowerShell; it fails with "
                    "'Access is denied' otherwise.",
    }
    return out


JOB = Job()


def simulate_frame(model: Path, frame: Path, conf: float) -> dict:
    """Run the NPU golden model on a prepared frame and draw the result."""
    import time
    import numpy as np
    isa = link.isa
    blob = model.read_bytes()
    hdr = isa.Header.unpack(blob)
    codes = np.frombuffer(frame.read_bytes(), np.uint8)
    want = 3 * hdr.imgsz * hdr.imgsz
    if codes.size != want:
        raise ValueError(f"{frame.name} is {codes.size} bytes; the model wants "
                         f"{want} (3 x {hdr.imgsz} x {hdr.imgsz})")
    mem = isa.Mem()
    mem.write(link.MODEL_BASE, blob)
    mem.write(link.FRAME_BASE, codes)
    regs = isa.Regs(img_addr=link.FRAME_BASE, box_addr=link.BOXES_BASE,
                    conf_logit=isa.conf_to_logit_q88(conf))
    t0 = time.time()
    isa.simulate(mem, link.MODEL_BASE + hdr.desc_off, regs)
    words = [int(w) for w in mem.read(link.BOXES_BASE, regs.num_boxes * 16).view(np.uint32)]
    boxes = isa.decode_boxes(words, regs.num_boxes)
    for b in boxes:
        b["name"] = isa.COCO_NAMES[b["cls"]] if b["cls"] < len(isa.COCO_NAMES) else f"#{b['cls']}"
    boxes.sort(key=lambda b: -b["conf"])
    d = draw.draw(frame, boxes, HERE / "render.png")
    CURRENT.update({"npu_boxes": boxes, "npu_from": "PC simulation"})
    return {"ok": True, "count": len(boxes), "boxes_list": boxes,
            "drawn": d["drawn"], "seconds": round(time.time() - t0, 1),
            "source": "golden model on this PC"}


def default_model() -> str:
    p = link.DEFAULT_MODEL
    return str(p) if p.exists() else ""


def default_frame() -> str:
    for cand in (QRTS.parent / "DVCFinal" / "out" / "frame.bin",):
        if cand.exists():
            return str(cand)
    return ""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):      # quieter console
        pass

    # -- helpers ----------------------------------------------------------
    def _send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code: int = 200):
        self._send(code, json.dumps(obj).encode(), "application/json")

    def _file(self, name: str, ctype: str):
        p = HERE / name
        if not p.exists():
            self._json({"ok": False, "error": f"missing {name}"}, 404)
            return
        self._send(200, p.read_bytes(), ctype)

    # -- routes -----------------------------------------------------------
    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        path = u.path

        if path in ("/", "/index.html"):
            return self._file("dashboard.html", "text/html; charset=utf-8")
        if path == "/dashboard.js":
            return self._file("dashboard.js", "application/javascript")
        if path == "/dashboard.css":
            return self._file("dashboard.css", "text/css")

        if path == "/api/defaults":
            try:
                info = link.blob_info(default_model()) if default_model() else None
            except (OSError, ValueError):
                info = None
            return self._json({
                "ok": True,
                "model": default_model(),
                "frame": CURRENT["bin"] or default_frame(),
                "model_base": f"0x{link.MODEL_BASE:08X}",
                "frame_base": f"0x{link.FRAME_BASE:08X}",
                "boxes_base": f"0x{link.BOXES_BASE:08X}",
                "arena_base": f"0x{link.ARENA_BASE:08X}",
                "blob": info,
                "sof": str(link.DEFAULT_SOF),
                "expected_array_size": link.EXPECTED_ARRAY_SIZE,
            })

        if path == "/api/blob":
            try:
                return self._json({"ok": True, **link.blob_info(
                    q.get("path", [default_model()])[0])})
            except (OSError, ValueError) as exc:
                return self._json({"ok": False, "error": str(exc)})

        if path == "/render.png":
            png = HERE / "render.png"
            if not png.exists():
                return self._json({"ok": False, "error": "nothing rendered yet"}, 404)
            return self._send(200, png.read_bytes(), "image/png")

        if path == "/preview.png":
            png = HERE / "preview.png"
            if not png.exists():
                return self._json({"ok": False, "error": "no image loaded"}, 404)
            return self._send(200, png.read_bytes(), "image/png")

        if path == "/api/current":
            return self._json({"ok": True, **CURRENT})

        if path == "/api/ethdiag":
            return self._json(eth_diag())

        if path == "/api/leds":
            return self._json({"ok": True, "leds": LED_GUIDE})

        if path == "/api/job":
            return self._json({"ok": True, **JOB.snapshot()})

        # Short operations run inline; none of these takes more than a few
        # seconds, and blocking briefly is simpler than a job for each.
        try:
            if path == "/api/cable":
                return self._json(link.cable_status().as_dict())
            if path == "/api/ident":
                return self._json(link.ident().as_dict())
            if path == "/api/regs":
                return self._json(link.registers().as_dict())
            if path == "/api/eth":
                return self._json(link.eth_counters().as_dict())
            if path == "/api/memtest":
                return self._json(link.mem_selftest().as_dict())
            if path == "/api/boxes":
                return self._json(link.read_boxes().as_dict())
            if path == "/api/render":
                # Boxes come from the board; the drawing is done here on the
                # host CPU. Putting a rasteriser in the fabric would spend
                # logic on something a laptop does in milliseconds.
                r = link.read_boxes()
                if not r.ok:
                    return self._json(r.as_dict())
                frame = q.get("frame", [CURRENT["bin"] or default_frame()])[0]
                if not frame:
                    return self._json({"ok": False,
                                       "error": "no frame file to draw on"})
                try:
                    d = draw.draw(Path(frame), r.data.get("boxes", []),
                                  HERE / "render.png")
                except (ValueError, OSError) as exc:
                    return self._json({"ok": False, "error": str(exc)})
                d["count"] = r.data.get("count")
                d["boxes_list"] = r.data.get("boxes")
                CURRENT.update({"npu_boxes": d["boxes_list"], "npu_from": "board"})
                return self._json(d)
            if path == "/api/memread":
                addr = int(q.get("addr", ["0"])[0], 0)
                n = min(int(q.get("n", ["8"])[0]), 256)
                return self._json(link.mem_read(addr, n).as_dict())
        except link.LinkError as exc:
            return self._json({"ok": False, "error": str(exc)}, 200)
        except Exception:
            return self._json({"ok": False, "error": traceback.format_exc()}, 200)

        self._json({"ok": False, "error": f"no route {path}"}, 404)

    def do_POST(self):
        u = urlparse(self.path)
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            return self._json({"ok": False, "error": "bad JSON"}, 400)

        if u.path == "/api/image":
            # The browser sends the file base64-encoded inside JSON. A
            # multipart parser would be more conventional, but this server is
            # stdlib-only and the payloads are a few megabytes at most.
            try:
                raw = base64.b64decode(body.get("data", ""), validate=True)
            except Exception:
                return self._json({"ok": False, "error": "bad base64 payload"})
            if not raw:
                return self._json({"ok": False, "error": "empty upload"})

            name = Path(body.get("name", "upload.png")).name or "upload.png"
            UPLOADS.mkdir(exist_ok=True)
            src = UPLOADS / name
            src.write_bytes(raw)

            out_bin = UPLOADS / (src.stem + ".bin")
            try:
                meta = image.prepare(src, out_bin, preview=HERE / "preview.png")
            except (ValueError, OSError) as exc:
                return self._json({"ok": False, "error": str(exc)})

            CURRENT.update({"bin": meta["bin"], "rgb": meta["rgb"],
                            "source": meta["source"],
                            "letterbox": meta["letterbox"]})
            return self._json(meta)

        if u.path == "/api/load":
            path = Path(body.get("path", ""))
            base = int(str(body.get("base", "0")), 0)
            verify = bool(body.get("verify", True))
            transport = str(body.get("transport", "jtag")).lower()

            if transport == "eth":
                iface = body.get("iface") or None
                gap = int(body.get("gap_us", 0) or 0)
                import dvcon_eth as eth
                started = JOB.start(
                    f"ethernet load {path.name} -> 0x{base:08X}",
                    lambda: eth.send_file(path, base, iface, gap_us=gap,
                                          verify=verify))
            else:
                started = JOB.start(f"load {path.name} -> 0x{base:08X}",
                                    lambda: link.load_file(path, base, verify))
            return self._json({"ok": started,
                               "error": "" if started else "a job is already running"})

        if u.path == "/api/run":
            try:
                conf = float(body.get("conf", 0.25))
            except (TypeError, ValueError):
                return self._json({"ok": False, "error": "conf must be a number 0..1"})
            if not 0.0 < conf < 1.0:
                return self._json({"ok": False, "error": "conf must be between 0 and 1"})
            model = body.get("model") or default_model()
            started = JOB.start("inference",
                                lambda: link.run_inference(conf, model))
            return self._json({"ok": started,
                               "error": "" if started else "a job is already running"})

        if u.path == "/api/abort":
            return self._json(link.abort().as_dict())

        if u.path == "/api/program":
            sof = body.get("sof") or str(link.DEFAULT_SOF)
            started = JOB.start(f"program FPGA with {Path(sof).name}",
                                lambda: link.program_fpga(sof))
            return self._json({"ok": started,
                               "error": "" if started else "a job is already running"})

        if u.path == "/api/simulate":
            # The golden model on the host: what the board must return for
            # this frame, bit for bit. Needs numpy only.
            frame = body.get("frame") or CURRENT["bin"]
            model = body.get("model") or default_model()
            try:
                conf = float(body.get("conf", 0.25))
                return self._json(simulate_frame(Path(model), Path(frame), conf))
            except (OSError, ValueError) as exc:
                return self._json({"ok": False, "error": str(exc)})

        if u.path == "/api/float":
            frame = body.get("frame") or CURRENT["bin"]
            if not frame:
                return self._json({"ok": False, "error": "prepare an image first"})
            try:
                return self._json(float_frame(Path(frame), float(body.get("conf", 0.25))))
            except (OSError, ValueError) as exc:
                return self._json({"ok": False, "error": str(exc)})

        self._json({"ok": False, "error": f"no route {u.path}"}, 404)


def main():
    try:
        srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    except OSError as e:
        # WinError 10013 here almost never means "no permission" -- it means the
        # port sits in a range Windows reserved for Hyper-V/WSL. Say so, because
        # the raw message sends people looking for an admin shell instead.
        print(f"cannot bind 127.0.0.1:{PORT}: {e}")
        print("  netsh int ipv4 show excludedportrange protocol=tcp")
        print("  then: set DVCON_PORT=<free port> and re-run")
        return

    print(f"dvcon diagnostics dashboard: http://127.0.0.1:{PORT}/")
    print("bound to localhost only; every button drives the real board")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")


if __name__ == "__main__":
    main()
