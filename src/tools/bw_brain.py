"""The obstacle brain's and the calibrate flow's commands of `bw` (docs/BRAIN_SPEC.md 8.5, 12.4): what the lidar sees,
the sensor tree, the recordings and the camera fit -- the same numbers the console's Map page and Calibrate card show.
tools/bw.py dispatches here; `req` is its HTTP helper (exits with the agent's message on an error status).

    bw perc [SECONDS] [--json]      the lidar's perception twice a second: segments, signs, limitations, free run
    bw tf                           the frame tree with xyz / rpy, rates and ages
    bw recs                         the recordings (runs/rec) the fit can use
    bw calib fit [REC|latest] [--kind replay|lidar] [--apply]   the camera fit from a recorded drive (a separate
                                    process on the robot; --apply saves its diff with the reason calib:fit)
    bw calib status | cancel
"""
from __future__ import annotations

import json
import math
import sys
import time

COMMANDS = ("perc", "tf", "recs", "calib")


def perc_line(d: dict) -> str:
    pil = d.get("pil") or []
    lim = d.get("lim") or []
    sec = [v for v in d.get("sec") or [] if v is not None and v >= 0]
    near = ""
    if sec:
        k = min(range(len(d["sec"])), key=lambda i: d["sec"][i] if d["sec"][i] >= 0 else 1e9)
        near = "  nearest %d mm @ %d deg" % (d["sec"][k], -175 + 10 * k)
    ps = ", ".join("(%d,%d) %d mm %d %%" % (q[0], q[1], q[2], q[3]) for q in pil[:4])
    fr = d.get("free") or {}
    return "perc %.1f ms  %d pts  %d seg  pillars %d%s  lim %d  free fwd %s back %s%s  [%s %d ms]" % (
        d.get("ms") or 0.0, d.get("n") or 0, len(d.get("seg") or []) // 5, len(pil), (": " + ps) if ps else "", len(lim),
        fr.get("fwd"), fr.get("back"), near, d.get("src"), d.get("age_ms") or 0)


def cmd_perc(a, req):
    js = "--json" in a
    rest = [x for x in a if not x.startswith("--")]
    secs = float(rest[0]) if rest else 0.0
    t_end = time.time() + secs
    while True:
        d = req("GET", "/api/perc")
        print(json.dumps(d) if js else perc_line(d), flush=True)
        if time.time() >= t_end:
            return 0
        time.sleep(0.5)


def cmd_tf(a, req):
    d = req("GET", "/api/tf")
    for f in d.get("frames", []):
        xyz = f.get("xyz")
        rpy = f.get("rpy")
        extra = []
        for k, u in (("rpm", "rpm"), ("fps", "fps"), ("hz", "Hz"), ("bins", "bins"), ("age_ms", "ms")):
            if f.get(k) is not None:
                extra.append("%s %s" % (f[k], u))
        if f.get("pose"):
            p = f["pose"]
            extra.insert(0, "pose %d, %d mm  %.1f deg" % (p["x"], p["y"], p["th"]))
        print("%-10s <- %-9s xyz %-22s rpy %-18s %s" % (f["name"], f.get("parent"), json.dumps(xyz), json.dumps(rpy),
                                                       "  ".join(extra)))
    return 0


def cmd_recs(a, req):
    rs = req("GET", "/api/recs")
    if not rs:
        print("no recordings (run wro_next with record=1)")
    for r in rs:
        print("%-24s %5d frames  %5d scans  %6.1f MB  %s" % (r["id"], r["frames"], r["scans"], r["bytes"] / 1e6,
                                                         time.strftime("%m-%d %H:%M", time.localtime(r["mtime"]))))
    return 0


def _fit_lines(st: dict) -> list:
    out = ["fit %s  %s (%s)  %s" % (st.get("state"), st.get("rec"), st.get("kind"),
                                    "%d %%" % round(100 * (st.get("progress") or 0)) if st.get("state") == "running"
                                    else "")]
    r = st.get("result") or {}
    if st.get("state") == "done":
        out.append("score %s -> %s mm   (%s evals, %s s, %s of %s frames used)" % (
            r.get("score_start"), r.get("score_best"), r.get("evals"), r.get("seconds"), r.get("frames_used"),
            r.get("frames")))
        out.append("start %s" % json.dumps(r.get("start")))
        out.append("best  %s" % json.dumps(r.get("best")))
        out.append("diff  %s" % (" ".join("%s=%s" % kv for kv in (r.get("diff") or {}).items()) or "(none)"))
    elif st.get("state") == "failed":
        out.append("error: %s" % (st.get("error") or r.get("error")))
    return out


def cmd_calib(a, req):
    sub = a[0] if a else "status"
    if sub == "status":
        print("\n".join(_fit_lines(req("GET", "/api/calib/fit"))))
        return 0
    if sub == "cancel":
        print(req("POST", "/api/calib/fit/cancel", {}))
        return 0
    if sub != "fit":
        sys.exit("bw calib fit [REC|latest] [--kind replay|lidar] [--apply] | calib status | calib cancel")
    rest = [x for x in a[1:] if not x.startswith("--")]
    kind = "replay"
    if "--kind" in a:
        kind = a[a.index("--kind") + 1]
        rest = [x for x in rest if x != kind]
    r = req("POST", "/api/calib/fit", {"rec": rest[0] if rest else "latest", "kind": kind})
    print("started %s on %s (%s)" % (r.get("job"), r.get("rec"), r.get("kind")), flush=True)
    last = -1
    while True:
        time.sleep(1.0)
        st = req("GET", "/api/calib/fit")
        pc = int(math.floor(100 * (st.get("progress") or 0)))
        if st.get("state") == "running":
            if pc // 10 != last // 10:
                print("  %d %%" % pc, flush=True)
                last = pc
            continue
        print("\n".join(_fit_lines(st)))
        diff = (st.get("result") or {}).get("diff") or {}
        if st.get("state") == "done" and "--apply" in a and diff:
            req("PUT", "/api/params?reason=calib:fit", diff)
            print("applied (reason calib:fit; bw params-undo takes it back)")
        return 0 if st.get("state") == "done" else 1


def main(c: str, a: list, req) -> int:
    return dict(perc=cmd_perc, tf=cmd_tf, recs=cmd_recs, calib=cmd_calib)[c](a, req)
