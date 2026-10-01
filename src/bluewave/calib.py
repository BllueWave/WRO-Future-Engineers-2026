"""Fit the camera from one short recorded drive (BRAIN_SPEC 12.4): the replay core of tools/replay.py, deployed with
bluewave/ so the console can run it on the robot, and fast enough to.

    python -m bluewave.calib fit runs/rec/<id> [--kind replay] [--json]     progress JSON lines, then {"result": ...}

The mat 2026-09-23 (stock A1): the mount moved when touched, pitch 1.0 -> 5.5 (static wall height) -> 4.7 with h 127,
roll 0.4, latency 0.09 from `tools/replay.py fit` on a real drive (fit 98 -> 20 mm).  That fit re-decoded and re-ran
vision on every frame for every candidate and cached every decoded frame (0.92 MB each: 1.35 GB for the 1467-frame mat
recording), so it could not run on the Pi.

Here the wall bases are found ONCE per frame: vision.edges with the horizon band widened by `band_deg` of pitch, so
every candidate pitch's band is inside it (the base row is the LOWEST obstacle row: extra rows above change nothing).
A candidate camera then only re-projects the cached (u, row) pairs through its own vision.Ground -- the same pixels,
another floor model -- and the particle filter runs on those points: the score is tools/replay.py's (median + 0.3 x p90
of the per-frame median distance of the points to the map), the search its coordinate descent (h 4 mm, pitch 0.4,
roll 0.4, latency 0.03 s, speed scale 0.04 when odo.source = "cmd"; halved when a round finds nothing; 4 rounds).

`vo` in a step line (wro_next W9: the drive's own speed estimate) is the odometry when odo.source = "lidar" (the
encoderless WLtoys: its command is not its speed); otherwise the command x speed_scale, as on the car.

Measured on the laptop 2026-09-23 against tools/replay.py fit, both from the pre-fit params (h 123, pitch 5.5, roll 0,
latency 0.12): the mat's first 120 frames -> the SAME answer, h 127 / pitch 4.7 / roll 0.4 / latency 0.09, score 35
(the mat's fit), 20 s vs 23.5 s; the whole 1467-frame recording -> the same answer both ways (h 135 / pitch 5.1 / roll
-0.4 / scale 0.96: after frame 151 the lens is against the wall and the replayed pose is lost, so fit a clean drive),
29 s / 126 MB vs 103 s / 1436 MB.  On 120 frames of one straight h, roll and latency sit in a flat valley (h 127-133
and latency 0.06-0.15 score within 2 mm of each other); pitch is the one number such a drive pins down.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
import time

import numpy as np

from . import loc as L
from . import params as P
from . import vision as V
from .lidar_perc import field_wall_dist as wall_dist   # exact distance for the lidar-pose kind (no filter there)

STEPS = dict(h_mm=4.0, pitch_deg=0.4, roll_deg=0.4, latency_s=0.03, speed_scale=0.04)


def load_rec(rec_dir: str, params_file: str | None = None) -> dict:
    """frames [(n, t)], steps [dict], scans (W9 lines, may be empty), robot_params (merged over DEFAULTS): the
    recording's own robot_params.json, else `params_file` (the agent passes the robot's params), else the DEFAULTS
    (`params_src` says which)."""
    frames, steps, scans = [], [], []
    with open(os.path.join(rec_dir, "steps.jsonl"), encoding="utf-8") as f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if "frame" in r:
                frames.append((int(r["frame"]), float(r["t"])))
            elif "scan" in r:
                scans.append(r)
            elif "t" in r and "v" in r:
                steps.append(r)
    params, src = copy.deepcopy(P.DEFAULTS), "defaults"
    for rp, s_ in ((os.path.join(rec_dir, "robot_params.json"), "recording"), (params_file, "robot")):
        if rp and os.path.exists(rp):
            with open(rp, encoding="utf-8") as f:
                params, src = P._merge(P.DEFAULTS, json.load(f)), s_
            break
    frames.sort(key=lambda q: q[1])
    steps.sort(key=lambda q: q["t"])
    return dict(dir=rec_dir, frames=frames, steps=steps, scans=scans, robot_params=params, params_src=src)


def rows(rec: dict, cam: dict, vp: dict, band_deg: float = 3.0, progress=None) -> list:
    """Per frame (t, u, row): the wall-base pixels (seen bases, not clipped) and those seen over a limitation, found
    once with the horizon band widened by band_deg of pitch."""
    import cv2
    g = None
    out = []
    n = len(rec["frames"])
    for i, (k, t) in enumerate(rec["frames"]):
        bgr = cv2.imread(os.path.join(rec["dir"], "f%05d.jpg" % k))
        if bgr is None:
            continue
        if g is None:
            g = V.Ground(dict(cam, pitch_deg=float(cam["pitch_deg"]) + band_deg), bgr.shape[1], bgr.shape[0])
        e = V.edges(bgr, g, vp)
        ev = np.asarray(e.vv, float)
        m = (e.cls == V.WALL) & ~e.clip & np.isfinite(ev) & np.isfinite(e.X)
        u, vv = e.u[m].astype(float), ev[m]
        if len(e.behind[0]):
            u, vv = np.concatenate([u, e.behind[2].astype(float)]), np.concatenate([vv, e.behind_v.astype(float)])
        out.append((t, u, vv))                     # float64: the filter then sees exactly tools/replay.py's points
        if progress is not None and i % 50 == 0:
            progress(i / max(n, 1))
    return out


def replay(rec: dict, cam: dict, vp: dict, scale: float, lat: float, rws=None, init=None, n: int = 300,
           seed: int = 0, src: str = "cmd", max_mm: float = 2600.0) -> dict:
    """The localiser on the recording with this camera model: fit (median of the per-frame median point-to-map
    distance, mm), fit_p90, frames used, poses [(t, x, y, th, sxy)]."""
    if rws is None:
        rws = rows(rec, cam, vp)
    steps = rec["steps"]
    if not steps or not rws:
        return dict(fit=1e9, fit_p90=1e9, frames=0, poses=[])
    W, H = int(cam.get("width", 640)), int(cam.get("height", 480))
    g = V.Ground(cam, W, H)
    loc = L.Localizer(n, seed=seed)
    odo = L.Odo()
    fi = 0
    yaw_prev, t_prev = steps[0]["yaw"], steps[0]["t"]
    fits, poses = [], []
    started = False
    use_vo = src == "lidar"
    for s in steps:
        dt = s["t"] - t_prev
        t_prev = s["t"]
        dth = math.radians(((s["yaw"] - yaw_prev + 180.0) % 360.0) - 180.0)
        yaw_prev = s["yaw"]
        v = s["vo"] if (use_vo and "vo" in s) else s["v"] * scale
        ds = v * 1000.0 * dt
        if started:
            loc.predict(ds, dth)
        odo.add(s["t"], ds, dth)
        while fi < len(rws) and rws[fi][0] <= s["t"]:
            tf, u, vv = rws[fi]
            fi += 1
            X, Y = g.floor(u, vv)
            ok = np.isfinite(X) & (np.hypot(X, Y) < max_mm)
            if ok.sum() < 8:
                continue
            Xn, Yn = L.to_now(X[ok], Y[ok], odo.since(tf - lat))
            if not started:
                if init is not None:
                    loc.init([(init[0], init[1], init[2], 30.0, 30.0, math.radians(2.0), 1.0)])
                else:
                    loc.global_start(Xn, Yn, g.pos[0])
                started = True
            loc.update(Xn, Yn)
            x, y, th, sxy, _sth = loc.estimate()
            c, sn = math.cos(th), math.sin(th)
            fits.append(float(np.median(loc.df(x + c * Xn - sn * Yn, y + sn * Xn + c * Yn))))
            poses.append((s["t"], x, y, th, sxy))
    return dict(fit=float(np.median(fits)) if fits else 1e9, fit_p90=float(np.percentile(fits, 90)) if fits else 1e9,
                frames=len(fits), poses=poses)


def fit_camera(rec: dict, params: dict | None = None, progress=None, rounds: int = 4) -> dict:
    """The coordinate search of tools/replay.py fit on cached rows: h_mm, pitch_deg, roll_deg, latency_s (and
    odo.speed_scale when odo.source = "cmd").  Returns start, best, score_start, score_best, fit / fit_p90 before and
    after, evals, seconds, diff ({dotted param: value} to apply)."""
    t0 = time.monotonic()
    p = params or rec["robot_params"]
    cam, vp = copy.deepcopy(p["camera"]), p.get("vision", {})
    odo = p.get("odo", {})
    src = str(odo.get("source", "cmd"))
    start = dict(h_mm=float(cam["h_mm"]), pitch_deg=float(cam["pitch_deg"]), roll_deg=float(cam.get("roll_deg", 0.0)),
                 latency_s=float(cam.get("latency_s", 0.12)))
    if src == "cmd":
        start["speed_scale"] = float(odo.get("speed_scale", 0.92))
    say = progress or (lambda *_a, **_k: None)
    say(0.0, stage="rows")
    rws = rows(rec, cam, vp, progress=lambda f: say(0.15 * f, stage="rows"))
    evals = [0]

    def run(b):
        c2 = dict(cam, h_mm=b["h_mm"], pitch_deg=b["pitch_deg"], roll_deg=b["roll_deg"])
        r = replay(rec, c2, vp, b.get("speed_scale", 1.0), b["latency_s"], rws=rws, n=300, src=src)
        evals[0] += 1
        return r

    r0 = run(start)
    best, cur = dict(start), r0["fit"] + 0.3 * r0["fit_p90"]
    best_r = r0
    steps = {k: v for k, v in STEPS.items() if k in start}
    total = rounds * 2 * len(steps)
    done = 0
    say(0.15, stage="search", score=round(cur, 1))
    for _rnd in range(rounds):
        improved = False
        for k in steps:
            for sgn in (1, -1):
                done += 1
                cand = dict(best)
                cand[k] = best[k] + sgn * steps[k]
                r = run(cand)
                sc = r["fit"] + 0.3 * r["fit_p90"]
                say(0.15 + 0.85 * min(1.0, done / total), stage="search", key=k, score=round(sc, 1),
                    best=round(cur, 1))
                if sc < cur - 0.3:
                    best, cur, best_r, improved = cand, sc, r, True
                    break
        if not improved:
            steps = {k: v / 2.0 for k, v in steps.items()}
    rnd = dict(h_mm=1, pitch_deg=2, roll_deg=2, latency_s=3, speed_scale=3)
    best = {k: round(v, rnd[k]) for k, v in best.items()}
    diff = {("odo." if k == "speed_scale" else "camera.") + k: v for k, v in best.items() if abs(v - start[k]) > 1e-9}
    return dict(kind="replay", source=src, frames=len(rec["frames"]), frames_used=best_r["frames"],
                start=start, best=best, score_start=round(r0["fit"] + 0.3 * r0["fit_p90"], 1), score_best=round(cur, 1),
                fit_start=round(r0["fit"], 1), fit_best=round(best_r["fit"], 1), p90_start=round(r0["fit_p90"], 1),
                p90_best=round(best_r["fit_p90"], 1), evals=evals[0], seconds=round(time.monotonic() - t0, 1),
                diff=diff)


def fit_camera_to_lidar(rec: dict, params: dict | None = None, progress=None, rounds: int = 6,
                        max_sxy: float = 60.0, max_mm: float = 2600.0) -> dict:
    """The WLtoys build (the lidar localised the run): the recorded poses are HELD FIXED and only the camera moves --
    h_mm, pitch_deg, roll_deg, latency_s by the same coordinate search, scored by the map distance of the re-projected
    wall bases at the pose of each exposure (t - latency, interpolated in the recorded poses with sxy < max_sxy).  No
    filter in the loop: a candidate costs one projection per frame, and the camera cannot drag the pose along with it
    (in the replay fit a wrong pitch and a wrong pose partly explain each other).

    Measured (mock agent, WLtoys, one recorded lap of wro_next, 204 frames): the recording's pitch bumped 16 -> 17.5
    comes back as 16.02 (h 139.1 for 138.3, roll 0.01), the unbumped one as 16.03 / 139.4, 2-3 s each.  `latency_s`
    comes back ~0.17 for a true 0.12: it also absorbs how far the RECORDED pose lags the frame stamp (the mock agent's
    lidar.scan_lag_s 0.05 on an instantaneous simulated revolution) -- on the car, take it with that in mind."""
    t0 = time.monotonic()
    p = params or rec["robot_params"]
    cam, vp = copy.deepcopy(p["camera"]), p.get("vision", {})
    st = [s for s in rec["steps"] if "x" in s and float(s.get("sxy", 1e9)) < max_sxy]
    if len(st) < 10:
        raise ValueError("kind lidar: fewer than 10 confident poses in the recording")
    ts = np.array([s["t"] for s in st])
    xs, ys = np.array([s["x"] for s in st]), np.array([s["y"] for s in st])
    ths = np.unwrap(np.array([s["th"] for s in st]))
    say = progress or (lambda *_a, **_k: None)
    say(0.0, stage="rows")
    rws = rows(rec, cam, vp, progress=lambda f: say(0.3 * f, stage="rows"))
    rws = [r for r in rws if ts[0] <= r[0] <= ts[-1] and len(r[1]) >= 8]
    if not rws:
        raise ValueError("kind lidar: no frame inside the confident poses")
    W, H = int(cam.get("width", 640)), int(cam.get("height", 480))
    evals = [0]

    def score(b):
        g = V.Ground(dict(cam, h_mm=b["h_mm"], pitch_deg=b["pitch_deg"], roll_deg=b["roll_deg"]), W, H)
        fits = []
        for tf, u, vv in rws:
            te = tf - b["latency_s"]
            if te < ts[0] or te > ts[-1]:
                continue
            x, y, th = np.interp(te, ts, xs), np.interp(te, ts, ys), np.interp(te, ts, ths)
            X, Y = g.floor(u, vv)
            ok = np.isfinite(X) & (np.hypot(X, Y) < max_mm)
            if ok.sum() < 8:
                continue
            c, s = math.cos(th), math.sin(th)
            fits.append(float(np.median(wall_dist(x + c * X[ok] - s * Y[ok], y + s * X[ok] + c * Y[ok]))))
        evals[0] += 1
        if not fits:
            return 1e9, 1e9, 0
        return float(np.median(fits)), float(np.percentile(fits, 90)), len(fits)

    start = dict(h_mm=float(cam["h_mm"]), pitch_deg=float(cam["pitch_deg"]), roll_deg=float(cam.get("roll_deg", 0.0)),
                 latency_s=float(cam.get("latency_s", 0.12)))
    f0, p0, n0 = score(start)
    best, cur, bf = dict(start), f0 + 0.3 * p0, (f0, p0, n0)
    # Nelder-Mead in units of STEPS, restarted once from its best.  h and pitch are coupled (both move the far wall
    # points: a long, narrow valley), and the replay fit's coordinate rounds stopped in it -- a pitch bumped +1.5 deg
    # came back as h +14 mm / pitch +0.7, and a compass search as h +8 / pitch +0.3 (sim).  A candidate costs one
    # projection per frame here (~20-30 ms), so a few hundred are affordable
    keys = [k for k in STEPS if k in start]
    sc_ = {k: STEPS[k] for k in keys}

    def f_of(z):
        cand = {k: start[k] + float(z[i]) * sc_[k] for i, k in enumerate(keys)}
        f, p9, n_ = score(cand)
        return f + 0.3 * p9, cand, (f, p9, n_)

    z0 = np.zeros(len(keys))
    for rs in range(2):
        simplex = [z0] + [z0 + np.eye(len(keys))[i] * (2.0 if rs == 0 else 0.5) for i in range(len(keys))]
        vals = [f_of(z) for z in simplex]
        for it in range(150):
            order = np.argsort([v[0] for v in vals])
            simplex, vals = [simplex[i] for i in order], [vals[i] for i in order]
            say(0.3 + 0.7 * min(1.0, (rs * 150 + it) / 200.0), stage="search", score=round(vals[0][0], 1))
            if vals[-1][0] - vals[0][0] < 0.05 and it > 10:
                break
            cen = np.mean(simplex[:-1], axis=0)
            zr = cen + (cen - simplex[-1])
            vr = f_of(zr)
            if vr[0] < vals[0][0]:
                ze = cen + 2.0 * (cen - simplex[-1])
                ve = f_of(ze)
                simplex[-1], vals[-1] = (ze, ve) if ve[0] < vr[0] else (zr, vr)
            elif vr[0] < vals[-2][0]:
                simplex[-1], vals[-1] = zr, vr
            else:
                zc = cen + 0.5 * (simplex[-1] - cen)
                vc = f_of(zc)
                if vc[0] < vals[-1][0]:
                    simplex[-1], vals[-1] = zc, vc
                else:
                    simplex = [simplex[0]] + [simplex[0] + 0.5 * (z - simplex[0]) for z in simplex[1:]]
                    vals = [vals[0]] + [f_of(z) for z in simplex[1:]]
        i0 = int(np.argmin([v[0] for v in vals]))
        z0 = simplex[i0]
        if vals[i0][0] < cur:
            cur, best, bf = vals[i0]
    rnd = dict(h_mm=1, pitch_deg=2, roll_deg=2, latency_s=3)
    best = {k: round(v, rnd[k]) for k, v in best.items()}
    diff = {"camera." + k: v for k, v in best.items() if abs(v - start[k]) > 1e-9}
    return dict(kind="lidar", frames=len(rec["frames"]), frames_used=bf[2], poses=len(st), start=start, best=best,
                score_start=round(f0 + 0.3 * p0, 1), score_best=round(cur, 1), fit_start=round(f0, 1),
                fit_best=round(bf[0], 1), p90_start=round(p0, 1), p90_best=round(bf[1], 1), evals=evals[0],
                seconds=round(time.monotonic() - t0, 1), diff=diff)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m bluewave.calib")
    ap.add_argument("cmd", choices=["fit"])
    ap.add_argument("rec")
    ap.add_argument("--kind", default="replay", choices=["replay", "lidar"])
    ap.add_argument("--json", action="store_true", help="progress as JSON lines, then {\"result\": ...}")
    ap.add_argument("--params", default=None, help="robot params JSON for a recording without robot_params.json")
    a = ap.parse_args(argv)
    rec = load_rec(a.rec, a.params)
    if not rec["frames"]:
        print(json.dumps(dict(error="no frames in %s" % a.rec)), flush=True)
        return 2

    def progress(frac, **info):
        if a.json:
            print(json.dumps(dict(progress=round(float(frac), 3), **info)), flush=True)

    try:
        if a.kind == "lidar":
            if not rec["scans"] and not int(rec["robot_params"].get("lidar", {}).get("loc", 0)):
                raise ValueError("kind lidar needs a drive the lidar localised (lidar.loc = 1, scans in the "
                                 "recording): this one is camera-only -- use kind replay")
            res = fit_camera_to_lidar(rec, progress=progress)
        else:
            res = fit_camera(rec, progress=progress)
    except ValueError as e:
        print(json.dumps(dict(error=str(e))), flush=True)
        return 2
    res["rec"] = os.path.basename(os.path.normpath(a.rec))
    res["params_src"] = rec["params_src"]
    if a.json:
        print(json.dumps(dict(result=res)), flush=True)
    else:
        print("start %s  score %.1f" % (res["start"], res["score_start"]))
        print("best  %s  score %.1f  (%d evals, %.0f s)" % (res["best"], res["score_best"], res["evals"], res["seconds"]))
        print("apply: " + " ".join("%s=%s" % kv for kv in res["diff"].items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
