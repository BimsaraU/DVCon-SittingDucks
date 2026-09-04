#!/usr/bin/env python3
"""Cross-check pin assignments against the top-level port list.

Quartus reports an assignment to a nonexistent port as a warning, not an error,
and then leaves the real port unassigned -- so a typo becomes a pin the fitter
places wherever it likes. On an SDRAM data line or a PHY clock that is a dead
interface, discovered on the bench rather than in the tool.

The reverse is worse: a port with NO location assignment gets placed
arbitrarily, and on a board where every pin already goes somewhere, an
arbitrary placement drives a real signal.

This checks both directions, plus that every port has an I/O standard.

It also checks that every port named in dvcon.sdc still exists. `get_ports` on
a name that is not there returns an EMPTY collection, and create_clock on an
empty collection creates nothing -- silently. Renaming the Ethernet ports from
ENET0_* to ENET1_* left both PHY clocks undeclared that way, which does not
fail the build: it removes them from timing analysis altogether, along with
every exception written against them.

It also checks every location against the Terasic pin table in
resources/DE2-115 Pin Assignments.csv. That is the check that matters: the
consistency checks above pass happily on a pin that is legal, in the right
bank, and wired to the wrong peripheral. Six DRAM_DQ lines were transcribed
one position off the real table, which put DRAM_DQ[26] on SRAM_ADDR[9], and
three ENET0 pins pointed at ENETCLK_25 and at PHY 1. Nothing but the board
table catches that.
"""

import csv
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
QRTS = HERE.parent

TOP_SV = QRTS / "rtl" / "platform" / "dvcon_top.sv"
PIN_TCL = QRTS / "pin" / "de2_115_pins.tcl"
QSF     = QRTS / "quartus" / "dvcon.qsf"
SDC     = QRTS / "quartus" / "dvcon.sdc"
REF_CSV = QRTS / "resources" / "DE2-115 Pin Assignments.csv"


def parse_ports(path):
    """Port names and widths from the top-level module declaration.

    Deliberately simple: the module header is written in one style, and a
    parser that accepts more would hide a header that drifted out of it.
    """
    text = path.read_text(encoding="utf-8")

    m = re.search(r"module\s+dvcon_top\s*(#\s*\((?:[^()]|\([^()]*\))*\)\s*)?\(",
                  text)
    if not m:
        sys.exit(f"could not find the dvcon_top module header in {path}")

    # Walk from the opening paren of the port list to its match.
    i = text.index("(", m.end() - 1)
    depth = 0
    for j in range(i, len(text)):
        if text[j] == "(":
            depth += 1
        elif text[j] == ")":
            depth -= 1
            if depth == 0:
                break
    body = text[i + 1:j]

    # Strip comments before looking for declarations.
    body = re.sub(r"//[^\n]*", "", body)

    ports = {}
    for decl in body.split(","):
        d = decl.strip()
        if not d:
            continue
        pm = re.match(
            r"(input|output|inout)\s+(?:wire|reg|logic)?\s*"
            r"(?:\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*)?"
            r"([A-Za-z_][A-Za-z0-9_]*)\s*$", d)
        if pm:
            hi, lo, name = pm.group(2), pm.group(3), pm.group(4)
            width = 1 if hi is None else (int(hi) - int(lo) + 1)
            ports[name] = width
    return ports


def parse_pins(path):
    """Return (locations, io_standard-strings) keyed by base port name."""
    locs = {}
    stds = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("#"):
            continue

        m = re.search(r"set_location_assignment\s+(\S+)\s+-to\s+(\S+)", line)
        if m:
            pin, port = m.group(1), m.group(2)
            base = port.split("[")[0]
            locs.setdefault(base, []).append((port, pin))

        m = re.search(r"set_instance_assignment\s+-name\s+IO_STANDARD\s+"
                      r"\"([^\"]*)\"\s+-to\s+(\S+)", line)
        if m:
            std, port = m.group(1), m.group(2)
            stds[port.split("[")[0]] = std
            stds[port] = std
    return locs, stds


def parse_reference(path):
    """Signal -> PIN_ from the Terasic pin table.

    The CSV is the board vendor's own export and carries a licence header and
    blank rows; anything whose third column is not a PIN_ name is not data.
    """
    ref = {}
    with path.open(encoding="utf-8", errors="replace", newline="") as fh:
        for row in csv.reader(fh):
            if len(row) >= 6 and row[2].startswith("PIN_"):
                ref[row[0].strip()] = (row[2].strip(), row[5].strip())
    return ref


def volts(std):
    """Bank voltage implied by an I/O standard string, or None.

    "3.3-V LVTTL" and "2.5 V" name the same thing in two spellings; comparing
    the strings finds differences that are not differences, and misses the one
    that is.
    """
    m = re.search(r"(\d+(?:\.\d+)?)\s*-?\s*V", std.upper())
    return float(m.group(1)) if m else None



def parse_sdc_ports(path):
    """Port names referenced by get_ports in the SDC, base names only.

    Commented lines are skipped -- the SDC carries several blocks of
    deliberately disabled input/output delay constraints, and flagging those
    would train the reader to ignore this check.
    """
    if not path.exists():
        return set()
    names = set()
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        code = line.split("#", 1)[0]
        for m in re.finditer(r"get_ports\s*(?:\{([^}]*)\}|(\S+?)\s*\])", code):
            blob = m.group(1) or m.group(2) or ""
            for tok in blob.replace("]", " ").split():
                base = tok.split("[")[0].strip()
                if base and not base.startswith("*"):
                    names.add(base)
    return names


def main():
    ports = parse_ports(TOP_SV)
    locs, stds = parse_pins(PIN_TCL)

    if not ports:
        sys.exit("parsed no ports from the top level -- check the header style")

    problems = 0

    # 1. Assignments that name a port the top level does not have.
    for base in sorted(locs):
        if base not in ports:
            print(f"  [err] pin file assigns '{base}', which is not a "
                  f"dvcon_top port")
            problems += 1

    # 2. Ports with no location at all.
    for name in sorted(ports):
        if name not in locs:
            print(f"  [err] port '{name}' has no location assignment")
            problems += 1

    # 3. Bus widths: every bit needs its own pin.
    for name, width in sorted(ports.items()):
        if name not in locs:
            continue
        assigned = len(locs[name])
        if width == 1:
            if assigned != 1:
                print(f"  [err] scalar port '{name}' has {assigned} locations")
                problems += 1
        else:
            want = {f"{name}[{i}]" for i in range(width)}
            have = {p for p, _ in locs[name]}
            missing = want - have
            extra  = have - want
            if missing:
                print(f"  [err] port '{name}' missing {sorted(missing)}")
                problems += 1
            if extra:
                print(f"  [err] port '{name}' has unexpected {sorted(extra)}")
                problems += 1

    # 3b. SDC references. An SDC naming a port that no longer exists does not
    # fail anything -- get_ports returns empty and the constraint evaporates,
    # taking the clock definition and its exceptions with it.
    for name in sorted(parse_sdc_ports(SDC)):
        if name not in ports:
            print(f"  [err] dvcon.sdc constrains '{name}', which is not a "
                  f"dvcon_top port -- the constraint is silently doing nothing")
            problems += 1

    # 4. Duplicate pins: two ports on one ball is a short.
    seen = {}
    for base, entries in locs.items():
        for port, pin in entries:
            if pin in seen:
                print(f"  [err] pin {pin} assigned to both {seen[pin]} "
                      f"and {port}")
                problems += 1
            seen[pin] = port

    # 5. Missing I/O standard. Not fatal -- there is a device default -- but on
    #    a board with mixed bank voltages the default is a guess.
    for name in sorted(ports):
        if name not in stds:
            print(f"  [warn] port '{name}' has no explicit IO_STANDARD")

    # 6. Every location against the board's own pin table.
    #
    # Checks 1-5 all passed on a pinout in which DRAM_DQ[26:30] were shifted
    # one place against the real table and [28]/[31] were missing entirely --
    # a self-consistent set of legal pins wired to the wrong nets. This is the
    # only check that reads the board rather than the design.
    checked = 0
    if not REF_CSV.exists():
        print(f"  [err] board pin table missing: {REF_CSV}")
        problems += 1
    else:
        ref = parse_reference(REF_CSV)
        if not ref:
            print(f"  [err] parsed no rows from {REF_CSV.name}")
            problems += 1
        for base in sorted(locs):
            for port, pin in sorted(locs[base]):
                entry = ref.get(port)
                want = entry[0] if entry else None
                if want is None:
                    # Every top-level port on this board is a board signal, so
                    # a name the table does not know is a typo.
                    print(f"  [err] '{port}' is not a signal in "
                          f"{REF_CSV.name} -- check the name")
                    problems += 1
                elif want != pin:
                    owner = next((s for s, (q, _) in ref.items() if q == pin),
                                 None)
                    owns = f" ({pin} is {owner})" if owner else ""
                    print(f"  [err] '{port}' assigned {pin}, board table "
                          f"says {want}{owns}")
                    problems += 1
                else:
                    checked += 1
                    # The bank voltage is the board's, not the design's. A
                    # 3.3 V standard declared on a 2.5 V bank is accepted by
                    # Quartus and sets the wrong input threshold; KEY[3:0]
                    # were declared that way.
                    std = stds.get(port) or stds.get(base)
                    if std is not None:
                        dv, bv = volts(std), volts(entry[1])
                        if dv is not None and bv is not None and dv != bv:
                            print(f"  [err] '{port}' declares {std}, but the "
                                  f"board runs that pin at {entry[1]}")
                            problems += 1

    # 7. The pinout must live in one file.
    #
    # Saving the project in the Quartus GUI flattens the sourced .tcl back into
    # the .qsf. Both copies then exist, the later one wins, and editing the
    # .tcl silently changes nothing. That is how the wrong DRAM_DQ locations
    # survived being looked at.
    if QSF.exists():
        stray = [n for n, line in enumerate(
                     QSF.read_text(encoding="utf-8").splitlines(), 1)
                 if line.strip().startswith("set_location_assignment")]
        if stray:
            print(f"  [err] {QSF.name} has {len(stray)} location assignment(s) "
                  f"of its own (line {stray[0]}...) -- the pinout belongs in "
                  f"{PIN_TCL.name}; delete them so one file is authoritative")
            problems += 1

    print(f"\n  {len(ports)} port(s), {len(seen)} pin(s) assigned, "
          f"{checked} confirmed against {REF_CSV.name}")
    if problems:
        sys.exit(f"  {problems} problem(s) -- fix before synthesis")
    print("  pin assignments consistent with the top level and the board table")


if __name__ == "__main__":
    main()
