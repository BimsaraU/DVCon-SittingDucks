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
CURRENT = {"bin": "", "rgb": "", "source": "", "letterbox": None}


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
         "meaning": "~1.5 Hz blink. Solid or dark means the clock or reset is dead."},
        {"id": "LEDG1", "name": "SDRAM traffic",
         "meaning": "Any master reading or writing memory."},
        {"id": "LEDG2", "name": "Ethernet RX",
         "meaning": "A frame was received and passed the MAC."},
        {"id": "LEDG3", "name": "Ethernet TX", "meaning": "Transmitting."},
        {"id": "LEDG4", "name": "Ethernet RX error",
         "meaning": "Frame failed its FCS. Steady flicker means a physical-layer problem."},
        {"id": "LEDG5", "name": "JTAG register",
         "meaning": "A control register was read or written."},
        {"id": "LEDG6", "name": "FILE TRANSFER",
         "meaning": "The JTAG memory window is moving data. Solid during a model or frame load."},
        {"id": "LEDG7", "name": "INFERENCE RUNNING",
         "meaning": "The layer sequencer is walking the network."},
        {"id": "LEDG8", "name": "Reset released",
         "meaning": "Steady on in normal operation. Dark means KEY0 is held or the board is in reset."},
    ],
    "red": [
        {"id": "LEDR0", "name": "Systolic array",
         "meaning": "The 16x16 MAC array is busy."},
        {"id": "LEDR1", "name": "Weight load",
         "meaning": "Weights are being pushed into the array."},
        {"id": "LEDR2", "name": "Activation stream",
         "meaning": "Activations are streaming through the array."},
        {"id": "LEDR3", "name": "Vector unit",
         "meaning": "Requantise and activation stage produced a result."},
        {"id": "LEDR4", "name": "Conv engine", "meaning": "Convolution in progress."},
        {"id": "LEDR5", "name": "Elementwise engine",
         "meaning": "Add / concat / split / upsample."},
        {"id": "LEDR6", "name": "Detect engine",
         "meaning": "Decoding detections and writing the box list."},
        {"id": "LEDR7", "name": "Engine error",
         "meaning": "An engine raised an error. Cross-check the STATUS register."},
        {"id": "LEDR8-17", "name": "Layer progress",
         "meaning": "Ten-segment bar over the 181 descriptors. A frozen bar shows the layer a run wedged on."},
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


def default_model() -> str:
    p = QRTS / "model" / "yolo26n_a16.bin"
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
            return self._json({
                "ok": True,
                "model": default_model(),
                "frame": default_frame(),
                "model_base": f"0x{link.MODEL_BASE:08X}",
                "frame_base": f"0x{link.FRAME_BASE:08X}",
                "boxes_base": f"0x{link.BOXES_BASE:08X}",
                "arena_base": f"0x{link.ARENA_BASE:08X}",
                "desc_base": f"0x{link.MODEL_BASE + link.MODEL_HDR_BYTES:08X}",
                "expected_array_size": link.EXPECTED_ARRAY_SIZE,
            })

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
            conf = int(str(body.get("conf", "0x20")), 0)
            desc = body.get("desc_base")
            desc_i = int(str(desc), 0) if desc not in (None, "") else None
            started = JOB.start("inference",
                                lambda: link.run_inference(conf, desc_i))
            return self._json({"ok": started,
                               "error": "" if started else "a job is already running"})

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
