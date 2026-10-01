"""The survey file (BRAIN4_SPEC 5.3): what one slow lap learnt about THIS field -- every sign seat with its colour, the
lot, the direction, how well each straight's walls fit the map -- saved as runs/maps/survey-<YYYYmmdd-HHMMSS>.json
(the directory SLAM maps use: gridmap.maps_dir()).

    s = from_result(r, run_id, profile, start_pose, fit_mm)    # wro_next's result dict (W18) -> the file's content
    path = save(s)                                            # atomic, never replaces (name-2 as gridmap.unique_base)
    s = load("latest")                                        # a practice run's prior: `wro_next survey=latest`
    v = votes(s)                                              # {(section, along): {"red", "green", "empty", "xy"}}

PRACTICE ONLY across runs: the judges re-draw the signs every round [RULE], so a saved survey is a prior for practice;
the race strategy is the in-run handover (programs/survey.py: the survey lap counted as lap 1).
"""
from __future__ import annotations

import glob
import json
import math
import os
import time

from . import field as F

SCHEMA = "bw.survey/1"
FIT_BAD = {"lidar": 25.0, "camera": 40.0}       # mm: a straight whose wall points sit farther from the map is "not
#                                                 standard" (5.3): the lidar's 12 + 1 % noise, the camera's floor model


def maps_dir() -> str:
    from .gridmap import maps_dir as _md
    return _md()


def _unique(base: str) -> str:
    path, k = base + ".json", 2
    while os.path.exists(path):
        path = "%s-%d.json" % (base, k)
        k += 1
    return path


def save(obj: dict, name: str = "") -> str:
    """Write the survey atomically (tmp + replace) as maps_dir()/<name or survey-<time>>.json; an existing name gets
    -2, -3 ... (a save never replaces a survey).  Returns the path."""
    from .gridmap import safe_name
    d = maps_dir()
    os.makedirs(d, exist_ok=True)
    nm = safe_name(name) if name else time.strftime("survey-%Y%m%d-%H%M%S")
    if not nm.startswith("survey-"):
        nm = "survey-" + nm
    path = _unique(os.path.join(d, nm))
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, indent=1)
    os.replace(tmp, path)
    return path


def latest() -> str | None:
    """The newest survey file (by mtime), or None."""
    fs = [p for p in glob.glob(os.path.join(maps_dir(), "survey-*.json")) if not p.endswith(".tmp")]
    return max(fs, key=os.path.getmtime) if fs else None


def load(name: str = "latest") -> dict | None:
    """"latest", a name (with or without survey- / .json) or a path -> the survey dict; None when there is none or it
    is not a survey."""
    if not name:
        return None
    if name == "latest":
        path = latest()
    elif os.path.isfile(name):
        path = name
    else:
        nm = name if name.endswith(".json") else name + ".json"
        path = os.path.join(maps_dir(), nm)
        if not os.path.isfile(path):
            path = os.path.join(maps_dir(), "survey-" + nm)
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, encoding="utf-8") as f:
            s = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(s, dict) or s.get("schema") != SCHEMA:
        return None
    s["_path"] = path
    return s


def _seat_row(j: int, **kw) -> dict:
    k, ai, ci, x, y = F.SEATS[j]
    out = dict(j=j, section=k, along=ai, row=ci, x=round(x), y=round(y), colour=None, exists=None, hits=0, empty=0,
               red=0, green=0, conf=0.0)
    out.update(kw)
    return out


def from_result(r: dict, run_id=None, profile=None, start_pose=None, fit_mm=None) -> dict:
    """The survey file's content from a wro_next result (W18).  Seats: the WLtoys fills them from the SeatMap export
    (lidar existence and empties + camera colours), the stock A1 from the planner's camera votes (a seat exists when a
    colour has >= 2 votes; after a complete lap a (section, along) the camera never voted on is empty)."""
    complete = r.get("reason") in ("laps", "parked")
    seats = []
    smap = r.get("smap")
    if smap and smap.get("seats"):
        from .seats import SeatMap
        sm = SeatMap({})
        sm.import_(smap)
        for j in range(len(F.SEATS)):
            st = sm.state(j)
            xy = st["xy"]
            ex = True if st["exists"] else (False if st["empty"] >= 2 else None)
            seats.append(_seat_row(j, colour=st["colour"], exists=ex, hits=st["hits"], empty=st["empty"],
                                   red=st["red"], green=st["green"], conf=st["conf"],
                                   **({"x": round(xy[0]), "y": round(xy[1])} if xy and st["exists"] else {})))
    else:
        by_key = {}
        for key, v in (r.get("seats") or {}).items():
            k_, ai = (int(q) for q in str(key).split(","))
            by_key[(k_, ai)] = v
        for j in range(len(F.SEATS)):
            k_, ai, ci = F.SEATS[j][:3]
            v = by_key.get((k_, ai))
            if v is None:
                seats.append(_seat_row(j, exists=False if complete else None))
                continue
            red, green, empty = int(v.get("red", 0)), int(v.get("green", 0)), int(v.get("empty", 0))
            col = None
            if max(red, green) >= 2 and max(red, green) > empty and red != green:
                col = "red" if red > green else "green"
            xy = v.get("xy")
            # the row the sign stood in: the one nearer the measured position
            row = None
            if xy and xy[2]:
                mx, my = xy[0] / xy[2], xy[1] / xy[2]
                d = [math.hypot(F.SEATS[k_ * 6 + ai * 2 + c][3] - mx, F.SEATS[k_ * 6 + ai * 2 + c][4] - my)
                     for c in range(2)]
                row = 0 if d[0] <= d[1] else 1
            if col is None:
                seats.append(_seat_row(j, exists=None if not complete else False, red=red, green=green, empty=empty))
            elif row == ci or row is None and ci == 0:
                kw = dict(colour=col, exists=True, hits=red + green, red=red, green=green, empty=empty,
                          conf=round(max(red, green) / max(1.0, red + green + empty), 2))
                if xy and xy[2]:
                    kw.update(x=round(xy[0] / xy[2]), y=round(xy[1] / xy[2]))
                seats.append(_seat_row(j, **kw))
            else:
                seats.append(_seat_row(j, exists=False))           # its along-position's sign stands in the other row
    fx = r.get("fix") or {}
    src = "lidar" if smap else "camera"
    fit = {k: (None if v is None else round(float(v), 1)) for k, v in (fit_mm or r.get("fit_mm") or {}).items()}
    bad = [k for k, v in fit.items() if v is not None and v > FIT_BAD[src]]
    notes = ["straight %s: walls %.0f mm off the map (> %.0f, %s): not the standard field there" % (k, fit[k],
             FIT_BAD[src], src) for k in bad]
    lot = None
    if r.get("lot_x0") is not None:
        d = int(r.get("direction") or 1)
        x0 = float(r["lot_x0"])
        ll = float(r.get("lot_len") or 320.0)
        a, b = (x0, x0 + ll) if d > 0 else (x0 - ll, x0)
        lot = dict(x0=round(a, 1), x1=round(b, 1), by=src, points=int(r.get("lot_points") or 0), section=0)
    pose = r.get("pose")
    return dict(schema=SCHEMA, t=round(time.time(), 1), run_id=run_id, profile=profile, complete=bool(complete),
                layout=dict(name=str(fx.get("layout") or "standard"), fit_mm=fit, standard=not bad),
                frame=dict(by=r.get("frame_by") or "none", k=r.get("frame_k")),
                direction="ccw" if int(r.get("direction") or 1) > 0 else "cw",
                start_pose=None if start_pose is None else [round(start_pose[0]), round(start_pose[1]),
                                                            round(math.degrees(start_pose[2]), 1)],
                end_pose=None if pose is None else [round(pose[0]), round(pose[1]), round(math.degrees(pose[2]), 1)],
                laps=r.get("laps"), seats=seats, lot=lot,
                obstacles=[[round(o[0]), round(o[1])] for o in (smap or {}).get("obstacles", []) if o[2] >= 3],
                camera=dict(pitch_deg=r.get("pitch_deg"), fitted=r.get("pitch_deg") is not None),
                grid=None, notes=notes)


def votes(s: dict) -> dict:
    """The survey's signs as planner seat votes: {(section, along): {"red", "green", "empty", "xy": (sx, sy, n)}},
    each known colour worth 2 votes (wro_next seat_votes: known at once)."""
    out = {}
    for q in (s or {}).get("seats", []):
        if q.get("exists") is True and q.get("colour") in ("red", "green"):
            key = (int(q["section"]), int(q["along"]))
            v = {"red": 0, "green": 0, "empty": 0}
            v[q["colour"]] = 2
            v["xy"] = (float(q["x"]), float(q["y"]), 1)
            out[key] = v
    return out


def rotate(s: dict, k: int) -> dict:
    """The whole survey in the frame rotated by k x 90 deg CCW (its seats move to straight (s + k) mod 4)."""
    import copy
    k %= 4
    out = copy.deepcopy(s)
    if not k:
        return out
    seats = [None] * len(F.SEATS)
    for q in s.get("seats", []):
        j2 = F.rot_seat(int(q["j"]), k)
        q2 = dict(q, j=j2, section=F.SEATS[j2][0])
        q2["x"], q2["y"] = (round(v) for v in F.rot(k, float(q["x"]), float(q["y"])))
        seats[j2] = q2
    out["seats"] = [q for q in seats if q is not None]
    for key in ("start_pose", "end_pose"):
        if s.get(key):
            x, y, th = F.rot_pose((s[key][0], s[key][1], math.radians(s[key][2])), k)
            out[key] = [round(x), round(y), round(math.degrees(th), 1)]
    if s.get("lot"):
        out["lot"] = dict(s["lot"], section=(int(s["lot"].get("section", 0)) + k) % 4)
    out["obstacles"] = [[round(v) for v in F.rot(k, o[0], o[1])] for o in s.get("obstacles", [])]
    fr = dict(s.get("frame") or {})
    fr["k"] = None if fr.get("k") is None else (int(fr["k"]) + k) % 4
    out["frame"] = fr
    return out
