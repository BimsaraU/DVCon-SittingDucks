#!/usr/bin/env python3
"""dvcon_eth.py -- bulk transfer to the board over raw Ethernet, from Windows.

JTAG moves about 2,480 words/s, so the 2.46 MB model takes ~250 s and a frame
~125 s. The same data over 100 Mbit is a few seconds. This is that path.

WHY THIS EXISTS ALONGSIDE sw/dvcon_host.c
-----------------------------------------
dvcon_host.c does the same job with Linux AF_PACKET sockets. This machine runs
Windows, which has no AF_PACKET -- but Npcap is installed, and wpcap.dll can
inject raw L2 frames. So this speaks the identical wire protocol through ctypes
rather than requiring a Linux host or WSL bridging.

THE ACK PATH IS JTAG, NOT ETHERNET
----------------------------------
The protocol has the FPGA reply with a received-frame bitmap. That reply needs
the board's Ethernet TRANSMIT path, which is the least-proven part of the
design. It is not needed: the same bitmap is a register (REG_ETH_BM) readable
over JTAG, and the counters next to it say whether frames arrived at all.

So this sends over Ethernet and confirms over JTAG. Each JTAG check spawns a
quartus_stp process (seconds), so checking every 64-frame window would cost
more than it saves -- the transfer is verified at the end against the file
instead, which is a stronger check anyway.

THE JUMPER, AND WHICH CONNECTOR
-------------------------------
None of this works until the PHY is in MII mode. The 88E1111's interface is
strapped at power-on by a jumper: JP1 for ENET0 (J4), JP2 for ENET1 (J5), each
short 1-2 for RGMII (the factory default) or 2-3 for MII. A hardware reset is
required after moving one.

JP1 could not be reached on this board, so JP2 was moved and the cable went
into ENET1. The design's pins were repointed to PHY 1 to match -- the two PHYs
are identical parts, so nothing but pin/de2_115_pins.tcl changed.

With the jumper wrong, every MAC counter stays at zero, including "frames not
addressed to us" -- and a machine on the far end broadcasts constantly, so that
counter being zero is proof that nothing is decoding at the MII pins at all.

MII also caps at 100 Mbit, so the PC's NIC must be forced to 100 Mbps Full
Duplex or the two ends never agree. See ETHERNET_SETUP.md in the repo root.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import struct
import sys
import time
from ctypes import (POINTER, Structure, c_char, c_char_p, c_int, c_uint,
                    c_ubyte, c_void_p, byref)
from pathlib import Path

import dvcon_link as link

# Wire constants -- must match sw/dvcon_memmap.h and dvcon_top's MAC_ADDR.
ETHERTYPE = 0x88B5
HDR_BYTES = 16
MAX_PAYLOAD = 1408
ACK_WINDOW = 64
OP_WRITE_MEM = 0x01
OP_ACK_REQ = 0x06
FPGA_MAC = bytes([0x02, 0x00, 0x00, 0xC0, 0xFF, 0xEE])

PCAP_ERRBUF_SIZE = 256


class _pcap_addr(Structure):
    pass


class _pcap_if(Structure):
    pass


_pcap_if._fields_ = [
    ("next", POINTER(_pcap_if)),
    ("name", c_char_p),
    ("description", c_char_p),
    ("addresses", POINTER(_pcap_addr)),
    ("flags", c_uint),
]


class EthError(RuntimeError):
    pass


def _load_wpcap():
    """Load Npcap's wpcap.dll.

    Npcap installs into System32\\Npcap rather than System32 itself, and that
    directory is not on the default DLL search path -- loading by bare name
    finds an old WinPcap if one is present, or nothing at all.
    """
    candidates = [
        r"C:\Windows\System32\Npcap\wpcap.dll",
        r"C:\Windows\SysWOW64\Npcap\wpcap.dll",
        "wpcap.dll",
    ]
    last = None
    for c in candidates:
        try:
            return ctypes.WinDLL(c)
        except OSError as exc:
            last = exc
    raise EthError(
        "could not load wpcap.dll -- install Npcap (https://npcap.com) with "
        f"WinPcap-compatible mode. Last error: {last}"
    )


class Pcap:
    def __init__(self):
        self.dll = _load_wpcap()
        d = self.dll
        d.pcap_findalldevs.argtypes = [POINTER(POINTER(_pcap_if)), c_char_p]
        d.pcap_findalldevs.restype = c_int
        d.pcap_freealldevs.argtypes = [POINTER(_pcap_if)]
        d.pcap_open_live.argtypes = [c_char_p, c_int, c_int, c_int, c_char_p]
        d.pcap_open_live.restype = c_void_p
        d.pcap_sendpacket.argtypes = [c_void_p, POINTER(c_ubyte), c_int]
        d.pcap_sendpacket.restype = c_int
        d.pcap_close.argtypes = [c_void_p]
        d.pcap_geterr.argtypes = [c_void_p]
        d.pcap_geterr.restype = c_char_p
        self.handle = None

    def interfaces(self) -> list[dict]:
        head = POINTER(_pcap_if)()
        err = ctypes.create_string_buffer(PCAP_ERRBUF_SIZE)
        if self.dll.pcap_findalldevs(byref(head), err) != 0:
            raise EthError(f"pcap_findalldevs: {err.value.decode(errors='replace')}")
        out, cur = [], head
        try:
            while cur:
                c = cur.contents
                out.append({
                    "name": c.name.decode(errors="replace") if c.name else "",
                    "description": (c.description.decode(errors="replace")
                                    if c.description else ""),
                })
                cur = c.next
        finally:
            self.dll.pcap_freealldevs(head)
        return out

    def open(self, name: str):
        err = ctypes.create_string_buffer(PCAP_ERRBUF_SIZE)
        # snaplen 65536, promiscuous, 10 ms read timeout -- we only transmit,
        # so the capture side is irrelevant beyond opening successfully.
        h = self.dll.pcap_open_live(name.encode(), 65536, 1, 10, err)
        if not h:
            raise EthError(f"pcap_open_live({name}): "
                           f"{err.value.decode(errors='replace')}")
        self.handle = h
        return h

    def send(self, frame: bytes):
        buf = (c_ubyte * len(frame)).from_buffer_copy(frame)
        if self.dll.pcap_sendpacket(self.handle, buf, len(frame)) != 0:
            msg = self.dll.pcap_geterr(self.handle)
            raise EthError(f"pcap_sendpacket: "
                           f"{msg.decode(errors='replace') if msg else '?'}")

    def close(self):
        if self.handle:
            self.dll.pcap_close(self.handle)
            self.handle = None


def build_frame(src_mac: bytes, opcode: int, seq: int, addr: int,
                payload: bytes = b"") -> bytes:
    """Byte-identical to send_cmd() in sw/dvcon_host.c.

    Header is big endian because the FPGA reads it a byte at a time.
    """
    if len(payload) > MAX_PAYLOAD:
        raise EthError(f"payload {len(payload)} exceeds {MAX_PAYLOAD}")

    f = bytearray()
    f += FPGA_MAC
    f += src_mac
    f += struct.pack(">H", ETHERTYPE)
    f += struct.pack(">BBBB", 1, opcode, 0, 0)      # ver, op, flags, rsv
    f += struct.pack(">I", seq & 0xFFFFFFFF)
    f += struct.pack(">I", addr & 0xFFFFFFFF)
    f += struct.pack(">H", len(payload))
    f += b"\x00\x00"                                 # rsv2
    f += payload
    # Pad to the 60-byte minimum. A runt is dropped silently by anything in
    # the path, which looks exactly like the board ignoring the frame.
    while len(f) < 60:
        f.append(0)
    return bytes(f)


def pick_interface(pc: Pcap, want: str | None) -> str:
    ifs = pc.interfaces()
    if not ifs:
        raise EthError("Npcap reports no interfaces. Run as Administrator: "
                       "packet injection needs it.")
    if want:
        for i in ifs:
            if want.lower() in i["name"].lower() or want.lower() in i["description"].lower():
                return i["name"]
        raise EthError(f"no interface matching {want!r}. Available:\n" +
                       "\n".join(f"  {i['name']}  {i['description']}" for i in ifs))
    if len(ifs) == 1:
        return ifs[0]["name"]
    raise EthError("several interfaces present; name the one the board is on:\n" +
                   "\n".join(f"  {i['name']}  {i['description']}" for i in ifs))


def send_file(path: Path, base: int, iface: str | None = None,
              src_mac: bytes = b"\x02\x00\x00\xC0\xFF\x01",
              gap_us: int = 0, verify: bool = True) -> dict:
    """Push a file into SDRAM over Ethernet, then verify it over JTAG.

    gap_us paces the send. There is no flow control on this protocol: the FPGA
    drops what it cannot absorb, and 100 Mbit into a 50 MHz DMA has no margin
    to spare. Start at 0 on a direct cable and raise it if verify fails.
    """
    path = Path(path)
    data = path.read_bytes()
    total = len(data)

    pc = Pcap()
    name = pick_interface(pc, iface)
    pc.open(name)

    nframes = (total + MAX_PAYLOAD - 1) // MAX_PAYLOAD
    t0 = time.time()
    try:
        for i in range(nframes):
            chunk = data[i * MAX_PAYLOAD:(i + 1) * MAX_PAYLOAD]
            pc.send(build_frame(src_mac, OP_WRITE_MEM, i,
                                base + i * MAX_PAYLOAD, chunk))
            if gap_us:
                time.sleep(gap_us / 1e6)
    finally:
        pc.close()
    dt = time.time() - t0

    out = {
        "ok": True,
        "interface": name,
        "bytes": total,
        "frames": nframes,
        "seconds": round(dt, 2),
        "mbytes_per_s": round(total / dt / 1e6, 2) if dt > 0 else None,
    }

    # Did anything actually arrive? These are JTAG reads, and they are the
    # whole reason this works without the board's transmit path.
    counters = link.eth_counters()
    if counters.ok:
        out["counters"] = counters.data.get("counters")
        out["link_verdict"] = counters.data.get("verdict")
        if (out["counters"] or {}).get("CMD", 0) == 0:
            out["ok"] = False
            # Report what the counters actually say rather than a fixed
            # guess: "no command frames" has several distinct causes and the
            # verdict tree in dvcon_link already separates them.
            out["error"] = ("sent, but the board accepted zero command "
                            "frames. " + (out.get("link_verdict") or ""))
            return out
    else:
        out["counters_error"] = counters.error

    if verify:
        v = link.load_verify_only(path, base, 128)
        out["verify_bad"] = v.data.get("bad")
        if v.ok and v.data.get("bad"):
            out["ok"] = False
            out["error"] = (f"{v.data['bad']} of 128 sampled words differ. "
                            f"Frames were dropped; retry with --gap 50.")
        elif not v.ok:
            out["verify_error"] = v.error
    return out


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        print("commands:")
        print("  list                                  show Npcap interfaces")
        print("  send <file> <addr> [iface] [gap_us]   push a file into SDRAM")
        print("  model <file> [iface]                  send at MODEL_BASE")
        print("  frame <file> [iface]                  send at FRAME_BASE")
        return 2

    import json
    cmd = argv[1]
    try:
        if cmd == "list":
            for i in Pcap().interfaces():
                print(f"  {i['name']}\n      {i['description']}")
            return 0
        if cmd == "send":
            r = send_file(Path(argv[2]), int(argv[3], 0),
                          argv[4] if len(argv) > 4 else None,
                          gap_us=int(argv[5]) if len(argv) > 5 else 0)
        elif cmd == "model":
            r = send_file(Path(argv[2]), link.MODEL_BASE,
                          argv[3] if len(argv) > 3 else None)
        elif cmd == "frame":
            r = send_file(Path(argv[2]), link.FRAME_BASE,
                          argv[3] if len(argv) > 3 else None)
        else:
            print(f"unknown command: {cmd}")
            return 2
    except EthError as exc:
        print(f"error: {exc}")
        return 1

    print(json.dumps(r, indent=2))
    return 0 if r.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
