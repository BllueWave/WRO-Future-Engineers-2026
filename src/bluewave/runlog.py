"""Run files (runs/<id>.jsonl) read back for the console: a summary card and a columnar replay.

A run file is JSON lines written by the agent's Runner (or race/race_main.py): one `meta` record, the program's `log`
records, `tel` (robot.snapshot() at 20 Hz) and, since console v2, `live` records (the program's published pose at
5 Hz, its path on each version change, its signs and lot when they change).  Readers ignore unknown kinds.
tools/bw.py keeps its own `summarise()` because it runs offline on pulled files; `summary()` here is a superset.
"""
from __future__ import annotations

import json
import math
import os
import re
import threading

_CACHE: dict = {}
_LOCK = threading.Lock()
RUN_ID = re.compile(r"^(\d{8})-(\d{6})(?:\.\d{3})?(-race)?-(.+)$")     # .mmm since 2026-09-23: ids never collide


def parse_id(run_id: str) -> dict:
    """`YYYYmmdd-HHMMSS[.mmm][-race]-<program>` -> {program, race, started ('YYYY-mm-dd HH:MM:SS' local)}."""
    m = RUN_ID.match(run_id)
    if not m:
        return dict(program=None, race=False, started=None)
    d, t = m.group(1), m.group(2)
    return dict(program=m.group(4), race=bool(m.group(3)),
                started="%s-%s-%s %s:%s:%s" % (d[:4], d[4:6], d[6:], t[:2], t[2:4], t[4:]))


def classify(x):
    """(kind, ev) of one program log record: error / ev / pose / text / data (CONSOLE_SPEC 5.4)."""
    if isinstance(x, str):
        return "text", None
    if isinstance(x, dict):
        if "error" in x:
            return "error", None
        if "ev" in x:
            return "ev", str(x["ev"])
        if ("x" in x and "y" in x and "state" in x) or "mode" in x:
            return "pose", None
        return "data", None
    return "data", None


def _fmt(v) -> str:
    if isinstance(v, float):
        return ("%.2f" % v).rstrip("0").rstrip(".") if math.isfinite(v) else "nan"
    if isinstance(v, (list, tuple)):
        s = "[" + ",".join(_fmt(a) for a in v[:8]) + (",..." if len(v) > 8 else "") + "]"
        return s
    if isinstance(v, dict):
        return "{%d keys}" % len(v)
    return str(v)


def _deg(v) -> str:
    return "%.1f" % v if isinstance(v, (int, float)) and math.isfinite(v) else str(v)


def event_text(r: dict) -> str:
    """One line for a log event: `direction ccw at -300,-1000, 0.0 deg`, `seat [0,2] red at 498,-903 (err 4 mm)`,
    otherwise `ev k=v k=v` without the bulky keys."""
    ev = r.get("ev")
    try:
        if ev == "direction" and "x" in r:
            return "direction %s at %s,%s, %s deg" % (r.get("dir"), r.get("x"), r.get("y"), _deg(r.get("th")))
        if ev == "seat":
            at = r.get("at") or [None, None]
            return "seat %s %s at %s,%s (err %s mm)" % (_fmt(r.get("seat")), r.get("colour"), at[0], at[1],
                                                        r.get("err"))
        if ev == "end":
            return "end %s, laps %s, %s s" % (r.get("reason"), _fmt(r.get("laps")), _fmt(r.get("seconds")))
        if ev in ("lost", "relocalised", "search", "start_search") and "x" in r:
            return "%s at %s,%s, %s deg (cost %s)" % (ev, r.get("x"), r.get("y"), _deg(r.get("th")),
                                                      _fmt(r.get("cost")))
    except Exception:
        pass
    skip = {"ev", "params", "legs", "rec", "seats"}
    parts = ["%s=%s" % (k, _fmt(v)) for k, v in r.items() if k not in skip]
    s = " ".join([str(ev)] + parts)
    return s if len(s) <= 160 else s[:157] + "..."


def _records(path: str):
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except ValueError:
                continue                                  # a half-written last line of a run still going


def cached(path: str, kind: str) -> bool:
    """True when summary / replay of `path` would come from the cache (the file has not changed since)."""
    try:
        st = os.stat(path)
    except OSError:
        return False
    with _LOCK:
        hit = _CACHE.get((path, kind))
        return bool(hit and hit[0] == (st.st_mtime, st.st_size))


def _cached(path: str, kind: str, fn):
    try:
        st = os.stat(path)
    except OSError:
        return None
    key = (path, kind)
    with _LOCK:
        hit = _CACHE.get(key)
        if hit and hit[0] == (st.st_mtime, st.st_size):
            return hit[1]
    out = fn(path)
    with _LOCK:
        if len(_CACHE) > 64:
            _CACHE.clear()
        _CACHE[key] = ((st.st_mtime, st.st_size), out)
    return out


# ---------------------------------------------------------------------------------------------------------- summary
def summary(path: str) -> dict:
    """bw.py's summarise() plus started, race, pose_source, battery_v [first, last] and live_records."""
    return _cached(path, "summary", _summary)


def _summary(path: str) -> dict:
    tel, logs, meta, live_n, live_pose, t_first = [], [], None, 0, 0, None
    for r in _records(path):
        if t_first is None and "t" in r:
            t_first = r["t"]
        if "tel" in r:
            tel.append(r["tel"])
        elif "log" in r:
            logs.append(r["log"])
        elif "meta" in r:
            meta = r["meta"]
        elif "live" in r:
            live_n += 1
            live_pose += "pose" in r["live"]
    rid = os.path.basename(path)[:-6] if path.endswith(".jsonl") else os.path.basename(path)
    ids = parse_id(rid)
    meta = meta or {}
    pose_log = sum(1 for l in logs if isinstance(l, dict) and "x" in l and "y" in l and "th" in l and "ev" not in l)
    out = dict(id=rid, program=meta.get("program") or ids["program"], params=meta.get("params"),
               race=bool(meta.get("race") or ids["race"]), started=t_first, live_records=live_n,
               pose_source="live" if live_pose else "log" if pose_log else None, samples=len(tel))
    if not tel:
        out.update(seconds=None, battery_v=None)
    else:
        def col(k):
            return [t[k] for t in tel if t.get(k) is not None]
        yaw = col("yaw")
        turned = 0.0
        for a, b in zip(yaw, yaw[1:]):
            turned += ((b - a + 180) % 360) - 180
        bat = col("battery_v")
        out.update(seconds=round(tel[-1]["t"] - tel[0]["t"], 2), min_front_mm=min(col("front_mm"), default=None),
                   min_left_mm=min(col("left_mm"), default=None), min_right_mm=min(col("right_mm"), default=None),
                   max_abs_steer=max((abs(x) for x in col("steer")), default=None), yaw_turned_deg=round(turned, 1),
                   battery_v=[bat[0], bat[-1]] if bat else None)
        if tel[-1].get("sim"):
            out["sim_contacts"] = tel[-1]["sim"].get("contacts")
    out["errors"] = [l for l in logs if isinstance(l, dict) and "error" in l][:3]
    evs = [l for l in logs if isinstance(l, dict) and "ev" in l]
    out["events"] = {k: sum(1 for l in evs if l["ev"] == k) for k in sorted({str(l["ev"]) for l in evs})}
    end = next((l for l in reversed(evs) if l["ev"] == "end"), None)
    if end:
        out["end"] = {k: end.get(k) for k in ("reason", "laps", "seconds")}
    return out


# ---------------------------------------------------------------------------------------------------------- replay
def replay(path: str) -> dict:
    """The columnar replay of CONSOLE_SPEC 6; `t` in s from the run's first record, any key null when absent."""
    return _cached(path, "replay", _replay)


def _replay(path: str) -> dict:
    recs = list(_records(path))
    rid = os.path.basename(path)[:-6] if path.endswith(".jsonl") else os.path.basename(path)
    meta = next((r["meta"] for r in recs if "meta" in r), {}) or {}
    ts = [r["t"] for r in recs if isinstance(r.get("t"), (int, float))]
    t0 = ts[0] if ts else 0.0
    out = dict(id=rid, program=meta.get("program") or parse_id(rid)["program"], t0=t0,
               duration_s=round(ts[-1] - t0, 2) if ts else 0.0, pose=None, truth=None, tel=None, paths=None,
               pillars=None, lot=None, state=None, events=[])

    def rt(r):
        return round(r["t"] - t0, 2)

    # pose: live records first (5 Hz), else the program's periodic pose dicts
    lp = [(rt(r), r["live"]["pose"]) for r in recs if "live" in r and r["live"].get("pose")]
    if lp:
        out["pose"] = dict(src="live", t=[a for a, _ in lp], x=[p[0] for _, p in lp], y=[p[1] for _, p in lp],
                           th=[p[2] for _, p in lp], sxy=[p[3] for _, p in lp])
    else:
        pl = [(rt(r), r["log"]) for r in recs if isinstance(r.get("log"), dict) and "ev" not in r["log"]
              and all(k in r["log"] for k in ("x", "y", "th"))]
        if pl:
            out["pose"] = dict(src="log", t=[a for a, _ in pl], x=[l["x"] for _, l in pl], y=[l["y"] for _, l in pl],
                               th=[l["th"] for _, l in pl], sxy=[l.get("sxy") for _, l in pl])
    # telemetry and mock truth, 10 Hz
    tel = [r for r in recs if "tel" in r]
    keep, last = [], -1e9
    for r in tel:
        if r["t"] - last >= 0.095:
            keep.append(r)
            last = r["t"]
    if keep:
        cols = ("v", "steer", "yaw", "front_mm", "left_mm", "right_mm", "battery_v")
        out["tel"] = dict(t=[rt(r) for r in keep], **{k: [r["tel"].get(k) for r in keep] for k in cols})
        sim = [(rt(r), r["tel"]["sim"]) for r in keep if r["tel"].get("sim")]
        if sim:
            out["truth"] = dict(t=[a for a, _ in sim], x=[s["x"] for _, s in sim], y=[s["y"] for _, s in sim],
                                th=[round(math.degrees(s["th"]), 1) for _, s in sim])
    # plan versions
    paths = [dict(t=rt(r), v=r["live"]["path_v"], xy=r["live"].get("path") or [])
             for r in recs if "live" in r and "path_v" in r["live"]]
    out["paths"] = paths or None
    # signs: the first time each is known (live lists, else the seat events)
    pil = []

    def add_pillar(t, x, y, c):
        for q in pil:
            if q["c"] == c and math.hypot(q["x"] - x, q["y"] - y) < 150.0:
                return
        pil.append(dict(t=t, x=int(x), y=int(y), c=c))

    for r in recs:
        if "live" in r and r["live"].get("pillars"):
            for q in r["live"]["pillars"]:
                add_pillar(rt(r), q[0], q[1], q[2])
    if not pil:
        for r in recs:
            l = r.get("log")
            if isinstance(l, dict) and l.get("ev") == "seat" and l.get("colour") and l.get("at"):
                add_pillar(rt(r), l["at"][0], l["at"][1], l["colour"])
    out["pillars"] = pil or None
    # the lot: first time known, last (refined) rectangle
    lots = [(rt(r), r["live"]["lot"]) for r in recs if "live" in r and r["live"].get("lot")]
    if lots:
        out["lot"] = dict(t=lots[0][0], rect=lots[-1][1])
    else:
        dirn = next((l.get("dir") for l in (r.get("log") for r in recs)
                     if isinstance(l, dict) and l.get("ev") == "direction"), None)
        lot_ev = [(rt(r), r["log"]) for r in recs if isinstance(r.get("log"), dict) and r["log"].get("ev") in
                  ("lot", "lot_start") and "x0" in r["log"]]
        if lot_ev and dirn:
            lot_len = float(((meta.get("robot_params") or {}).get("car") or {}).get("lot_len_mm", 320.0))
            x0 = float(lot_ev[-1][1]["x0"])
            x1 = x0 + (lot_len if str(dirn).startswith("ccw") else -lot_len)
            a, b = sorted((x0, x1))
            out["lot"] = dict(t=lot_ev[0][0], rect=[int(a), -1500, int(b), -1300])
    # state changes
    st_t, st_v = [], []
    for r in recs:
        s = None
        if "live" in r and r["live"].get("state"):
            s = r["live"]["state"]
        elif isinstance(r.get("log"), dict) and "ev" not in r["log"]:
            s = r["log"].get("state") or r["log"].get("mode")
        if s is not None and (not st_v or st_v[-1] != s):
            st_t.append(rt(r))
            st_v.append(s)
    out["state"] = dict(t=st_t, v=st_v) if st_v else None
    out["events"] = [dict(t=rt(r), ev=str(r["log"]["ev"]), text=event_text(r["log"]))
                     for r in recs if isinstance(r.get("log"), dict) and "ev" in r["log"]]
    out["events"] += [dict(t=rt(r), ev="error", text=str(r["log"]["error"]).strip().splitlines()[-1][:200])
                      for r in recs if isinstance(r.get("log"), dict) and "error" in r["log"]]
    out["events"].sort(key=lambda e: e["t"])
    return out
