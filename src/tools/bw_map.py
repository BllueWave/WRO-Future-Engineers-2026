"""The map and cloud commands of `bw` (docs/BRAIN_SPEC.md 8.5): the Map page's data as files the team opens.
tools/bw.py dispatches here; `req` is its HTTP helper (exits with the agent's message on an error status).

    bw map [OUT.png] [--overlay] [--json]   the served map (SLAM's, or the known-pose field map) as a PNG (default
                                            runs/map-<time>.png); --overlay = colour with the pose, trail, scan and
                                            lidar pillars (what the review reads); --json = its meta only
    bw map save NAME                        runs/maps/NAME.png / .json / .npy on the robot
    bw map clear --yes                      forget the served map (refused while slam runs)
    bw maps                                 the saved maps
    bw cloud OUT.ply [--png OUT.png [--view top|side]]   the HP60C cloud (depth.on = 1, the 3D view open)
"""
from __future__ import annotations

import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMMANDS = ("map", "maps", "cloud")


def _meta_line(m: dict) -> str:
    c = m.get("crop") or [0, 0, 0, 0]
    res = m.get("res_mm") or 0.0
    return ("%s  frame %s  v %s  %.1f m²  %g mm cells  %d scans  crop %d x %d cells = %.2f x %.2f m" % (
        m.get("src"), m.get("frame"), m.get("v"), m.get("known_m2") or 0.0, res, m.get("scans") or 0, c[3], c[2],
        c[3] * res / 1000.0, c[2] * res / 1000.0))


def cmd_map(a, req):
    if a and a[0] == "save":
        if len(a) < 2:
            sys.exit("bw map save NAME")
        r = req("POST", "/api/map/save", {"name": a[1]})
        print("saved %s on the robot:" % r["name"])
        for p in r["paths"]:
            print("  " + p)
        return 0
    if a and a[0] == "clear":
        if "--yes" not in a:
            typed = input("type clear_map to forget the served map: ").strip()
            if typed != "clear_map":
                print("not confirmed")
                return 1
        req("POST", "/api/map/clear", {"confirm": "clear_map"})
        print("map cleared")
        return 0
    m = req("GET", "/api/map").get("map")
    if m is None:
        print("no map: `bw run slam` and drive by hand, or run a program with a pose (the known-pose map builds "
              "while it runs)")
        return 1
    if "--json" in a:
        print(json.dumps(m, indent=1))
        return 0
    names = [x for x in a if not x.startswith("--")]
    out = names[0] if names else os.path.join(ROOT, "runs", "map-%s.png" % time.strftime("%Y%m%d-%H%M%S"))
    ov = 1 if "--overlay" in a else 0
    png = req("GET", "/api/map.png?crop=1&scale=1&overlay=%d" % ov, raw=True)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "wb") as f:
        f.write(png)
    print(out)
    print(_meta_line(m))
    if not ov:
        print("black = occupied, white = free, grey = unknown; top row = highest y.  --overlay adds the car, its "
              "trail and the latest scan")
    return 0


def cmd_maps(a, req):
    ms = req("GET", "/api/maps")
    if not ms:
        print("no saved maps (bw map save NAME, or slam saves one when it ends)")
        return 0
    for m in ms:
        t = time.strftime("%m-%d %H:%M", time.localtime(m["saved"])) if m.get("saved") else "-"
        print("%-24s %s  %-5s v %-5s %5.1f m²  %6.0f kB" % (m["name"], t, m.get("frame"), m.get("v"),
                                                          m.get("known_m2") or 0.0, (m.get("bytes") or 0) / 1024.0))
    return 0


def cmd_cloud(a, req):
    names = [x for x in a if not x.startswith("--")]
    if not names:
        sys.exit("bw cloud OUT.ply [--png OUT.png [--view top|side]]")
    ply = req("GET", "/api/cloud.ply", raw=True)
    with open(names[0], "wb") as f:
        f.write(ply)
    head = ply[:400].split(b"end_header")[0].decode("ascii", "replace")
    n = next((ln.split()[-1] for ln in head.splitlines() if ln.startswith("element vertex")), "?")
    print("%s  %s points, %.0f kB" % (names[0], n, len(ply) / 1024.0))
    if "--png" in a:
        i = a.index("--png")
        if i + 1 >= len(a):
            sys.exit("--png OUT.png")
        view = a[a.index("--view") + 1] if "--view" in a and a.index("--view") + 1 < len(a) else "top"
        png = req("GET", "/api/cloud.png?view=%s" % ("side" if view == "side" else "top"), raw=True)
        with open(a[i + 1], "wb") as f:
            f.write(png)
        print(a[i + 1])
    return 0


def main(c: str, a: list, req) -> int:
    if c == "map":
        return cmd_map(a, req)
    if c == "maps":
        return cmd_maps(a, req)
    if c == "cloud":
        return cmd_cloud(a, req)
    sys.exit("unknown command %s" % c)
