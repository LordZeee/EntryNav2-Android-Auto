# Copyright (c) 2026 Jiraiya43
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
"""Generate platform/kombi_ric_template.hpp from a capture of the F-series RIC traffic on the session bus.

The capture (never in git) holds the D-Bus messages the head unit's own navigation sent to BMW.RueKoIC on a route
start, as recorded by a passive eavesdropper: records of [u32 sec][u32 usec][u32 length][D-Bus message], little-endian.
The models that kombi_ric.cpp sends unchanged are copied from it: road data (40), thresholds (34), guiding graph (31)
and order (7), the six sub-distances (29) and their info events (36). Nothing personal goes in: the graph's points are
moved to an arbitrary place (their shape kept), and the templates hold no text.

usage: python3 tools/gen_ric_template.py <capture.bin> [platform/kombi_ric_template.hpp]
"""

import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEST = "BMW.RueKoIC"
NEUTRAL_LAT = 50.0  # where the graph's first point goes; the others keep their offsets from it
NEUTRAL_LON = 10.0
BASIC = {
    "y": ("B", 1),
    "b": ("I", 4),
    "n": ("h", 2),
    "q": ("H", 2),
    "i": ("i", 4),
    "u": ("I", 4),
    "x": ("q", 8),
    "t": ("Q", 8),
    "d": ("d", 8),
    "h": ("I", 4),
}


def type_end(sig, i):
    """Index just after the single complete type starting at sig[i]."""
    c = sig[i]
    if c == "a":
        return type_end(sig, i + 1)
    if c in "({":
        depth, j = 0, i
        while True:
            if sig[j] in "({":
                depth += 1
            elif sig[j] in ")}":
                depth -= 1
                if depth == 0:
                    return j + 1
            j += 1
    return i + 1


def split(sig):
    out, i = [], 0
    while i < len(sig):
        j = type_end(sig, i)
        out.append(sig[i:j])
        i = j
    return out


def alignment(t):
    if t[0] in "({xtd":
        return 8
    if t[0] in "nq":
        return 2
    if t[0] in "ygv":
        return 1
    return 4


class Reader:
    def __init__(self, data, at=0):
        self.data, self.at = data, at

    def align(self, n):
        self.at = (self.at + n - 1) // n * n

    def read(self, t):
        c = t[0]
        if c == "y":
            v = self.data[self.at]
            self.at += 1
            return v
        if c in BASIC:
            fmt, size = BASIC[c]
            self.align(size)
            v = struct.unpack_from("<" + fmt, self.data, self.at)[0]
            self.at += size
            return v
        if c in "so":
            n = self.read("u")
            v = self.data[self.at : self.at + n].decode("utf-8")
            self.at += n + 1
            return v
        if c == "g":
            n = self.data[self.at]
            v = self.data[self.at + 1 : self.at + 1 + n].decode()
            self.at += n + 2
            return v
        if c == "v":
            s = self.read("g")
            return (s, self.read(s))
        if c == "a":
            n = self.read("u")
            self.align(alignment(t[1:]))
            end, items = self.at + n, []
            while self.at < end:
                items.append(self.read(t[1:]))
            return items
        if c == "(":
            self.align(8)
            return tuple(self.read(x) for x in split(t[1:-1]))
        raise ValueError(f"type {t}")


class Writer:
    def __init__(self):
        self.data = bytearray()

    def align(self, n):
        while len(self.data) % n:
            self.data.append(0)

    def write(self, t, v):
        c = t[0]
        if c == "y":
            self.data.append(v)
        elif c in BASIC:
            fmt, size = BASIC[c]
            self.align(size)
            self.data += struct.pack("<" + fmt, v)
        elif c in "so":
            b = v.encode("utf-8")
            self.write("u", len(b))
            self.data += b + b"\0"
        elif c == "g":
            self.data += bytes([len(v)]) + v.encode() + b"\0"
        elif c == "v":
            self.write("g", v[0])
            self.write(v[0], v[1])
        elif c == "a":
            self.align(4)
            at = len(self.data)
            self.data += b"\0\0\0\0"
            self.align(alignment(t[1:]))
            start = len(self.data)
            for item in v:
                self.write(t[1:], item)
            struct.pack_into("<I", self.data, at, len(self.data) - start)
        elif c == "(":
            self.align(8)
            for x, item in zip(split(t[1:-1]), v):
                self.write(x, item)
        else:
            raise ValueError(f"type {t}")


def calls(capture):
    """(member, signature, body values, body bytes) of every call to BMW.RueKoIC, in order."""
    data = capture.read_bytes()
    at = 0
    while at + 12 <= len(data):
        length = struct.unpack_from("<III", data, at)[2]
        msg = data[at + 12 : at + 12 + length]
        at += 12 + length
        if msg[1] != 1:
            continue
        body_len, _, fields_len = struct.unpack_from("<III", msg, 4)
        fields = dict(Reader(msg, 12).read("a(yv)"))
        if fields.get(6, ("s", ""))[1] != DEST:
            continue
        sig = fields.get(8, ("g", ""))[1]
        start = (16 + fields_len + 7) // 8 * 8
        body = msg[start : start + body_len]
        r = Reader(body)
        values = [r.read(t) for t in split(sig)]
        yield fields[3][1], sig, values, body


def first(all_calls, member, model_id, model_type):
    for c in all_calls:
        if c[0] == member and c[2][1] == model_id and c[2][2] == model_type:
            return c
    raise SystemExit(f"{member} {model_id}/{model_type} not in the capture")


def encode(sig, values):
    w = Writer()
    for t, v in zip(split(sig), values):
        w.write(t, v)
    return bytes(w.data)


def neutral_graph(values):
    """The graph with its points moved: the first point to NEUTRAL_LAT/LON, the others at the same offsets."""
    tag, (vsig, model) = values[3]
    points = model[2]
    lat0, lon0 = points[0][1][0], points[0][1][1]
    moved = [(pid, (lat - lat0 + NEUTRAL_LAT, lon - lon0 + NEUTRAL_LON, alt)) for pid, (lat, lon, alt) in points]
    return values[:3] + [(tag, (vsig, (model[0], model[1], moved)))]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    capture = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "platform" / "kombi_ric_template.hpp"
    cs = list(calls(capture))
    picks = [
        ("road_data", first(cs, "ric_setRoadData", 800, 40)),
        ("thresholds", first(cs, "ric_setManeuverData", 700, 34)),
        ("graph", first(cs, "ric_setGuidingData", 1300, 31)),
        ("graph_order", first(cs, "ric_setGuidingData", 1200, 7)),
    ]
    picks += [(f"sub_distance_{i}", first(cs, "ric_setManeuverData", 4001 + i, 29)) for i in range(6)]
    picks += [(f"event_{i}", first(cs, "ric_setNaviInfoEventData", 15000 + i, 36)) for i in range(6)]

    lines = [
        "// Copyright (c) 2026 Jiraiya43",
        "// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0",
        "// The F-series route-guidance models that kombi_ric.cpp sends unchanged, as D-Bus call bodies (uui(yv)).",
        "",
        "// Captured from the head unit's own navigation on an X1 F48 (BMW.RueKoIC, route start), by",
        "// tools/gen_ric_template.py. The graph's points are moved to an arbitrary place, their shape kept; no text.",
        "// Generated: do not edit by hand. The bytes are what BMW's software sends; the notice above covers their",
        "// arrangement in this file, not the bytes (see LICENSE.md, Credits).",
        "#ifndef ENTRY_AAD_PLATFORM_KOMBI_RIC_TEMPLATE_HPP",
        "#define ENTRY_AAD_PLATFORM_KOMBI_RIC_TEMPLATE_HPP",
        "",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        "namespace evo {",
        "namespace kombi {",
        "namespace rictmpl {",
        "",
        "struct Call {",
        "    const char* member;",
        "    const uint8_t* body;",
        "    size_t len;",
        "};",
        "",
    ]
    for name, (_member, sig, values, body) in picks:
        if name == "graph":
            values = neutral_graph(values)
            body = encode(sig, values)
        elif encode(sig, values) != body:
            raise SystemExit(f"{name}: re-encoding differs from the capture")
        nums = [str(b) for b in body]
        rows = [", ".join(nums[i : i + 20]) for i in range(0, len(nums), 20)]
        lines.append(f"static const uint8_t {name}[] = {{")
        lines += [f"    {row}," for row in rows]
        lines.append("};")
    lines.append("")
    lines.append(
        "// In the order the factory sends them, after the distance of maneuver 0 (sub-distances interleave with"
    )
    lines.append("// their events; the graph and its order come first).")
    order = ["road_data", "thresholds", "graph", "graph_order"]
    lines.append("static const Call before_distance[] = {")
    lines += [f'    {{"{dict(picks)[n][0]}", {n}, sizeof({n})}},' for n in order[:1]]
    lines.append("};")
    lines.append("static const Call after_distance[] = {")
    lines += [f'    {{"{dict(picks)[n][0]}", {n}, sizeof({n})}},' for n in order[1:]]
    for i in range(6):
        for n in (f"sub_distance_{i}", f"event_{i}"):
            lines.append(f'    {{"{dict(picks)[n][0]}", {n}, sizeof({n})}},')
    lines.append("};")
    lines += ["", "}  // namespace rictmpl", "}  // namespace kombi", "}  // namespace evo", "", "#endif", ""]
    out.write_text("\n".join(lines), encoding="ascii")
    print(f"{out}: {len(picks)} models")
    return 0


if __name__ == "__main__":
    sys.exit(main())
