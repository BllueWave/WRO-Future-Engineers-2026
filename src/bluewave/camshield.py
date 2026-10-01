"""The camera shield for the stock A1 (BRAIN4_SPEC 4.8): shield.Shield fed with CAMERA points instead of a scan.

The stock A1's LD19 scans at 145.7 mm, above the 100 mm walls (mat 2026-09-23, M8): it has no lidar walls, and its
map guard guards the BELIEVED pose -- in run 185306 the pose was wrong and the car pushed a black wall for 85 s (M4).
The camera IS its range sensor, so the camshield is the existing lidar shield with the camera as the "lidar": its
position is the lens, its only seen sector is the field of view, its nearest seen range is where the image bottom
meets the floor, its memory keeps what the camera saw for camshield.memory_s (3 s) for the zones it no longer sees.

    camsh = make(robot.p)                                       # a shield.Shield
    bot = shield.command(robot, camsh, ttl)                      # every drive command through it
    per new frame:  camsh.scan(perception(e, dets, g, tf, lat, car), odo.since)
    per loop:       camsh.odom(ds, dth)                          # or set_pose() with a confident field pose
    confident pose: camsh.virtual(shield.virtual_points(...))    # the map fills what the camera cannot see (5 Hz)

A CLIPPED wall column (the wall reaches the image bottom) says the wall is somewhere NEARER than the image shows: a
single point at the image bottom would move with the car and never close in (the car would drive into it), so each
clipped column adds points every 20 mm along its bearing from the image-bottom floor point back to the footprint's
front edge (clip_rays).

Numbers (vision.Ground, 2026-09-25): the stock A1 (pitch 4.7 deg, h 127 mm, fy 569.5) sees the floor from 392 mm ahead
of the rear axle, 213 mm ahead of the nose; the WLtoys camera (pitch 16, h 138.3) from 320 mm; hfov 58.6 deg.
need(v) = v^2 / (2 x 2.2) + 0.25 v + 25 mm: 120 mm at 0.30 m/s, 182 mm at 0.45 -- a wall first seen at the edge of the
near band still stops the A1 below ~0.45 m/s; walls in a lap are seen 0.5-2.6 m ahead.
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass

import numpy as np

from . import lidar_perc as LP
from . import shield as SH

# BRAIN4_SPEC 2.1: camshield.* (params.py DEFAULTS gains them: B4; read with .get here)
DEFAULTS = dict(memory_s=3.0, margin_mm=30.0, side_mm=10.0,
                latency_s=0.25,     # camera latency + frame period + loop period [EST: 0.09 + 0.067 + 0.025, rounded up]
                clip_rays=1, v_floor=0.10)
MAX_MM = 2600.0                     # perc.max_mm of the camera "scan": the wall bases the localiser trusts


@dataclass(eq=False)
class CamPerc:
    """Duck-typed lidar_perc.Perception for Shield.scan: t, lag_s, X, Y, cls (car frame AT THE EXPOSURE, mm)."""
    t: float
    lag_s: float
    X: np.ndarray
    Y: np.ndarray
    cls: np.ndarray


def cam_cfg(robot_p: dict) -> dict:
    """camshield.* over DEFAULTS."""
    return dict(DEFAULTS, **{k: v for k, v in ((robot_p or {}).get("camshield") or {}).items() if k in DEFAULTS})


def hfov_deg(cam: dict) -> float:
    fx = float(cam["intrinsics"][0]) * float(cam.get("width", 640)) / float(cam.get("calib_size", [640, 480])[0])
    return math.degrees(2.0 * math.atan(float(cam.get("width", 640)) / 2.0 / fx))


def near_range_mm(cam: dict) -> float:
    """The camera's nearest seen floor range: Ground.floor of the bottom-centre pixel, measured from the lens."""
    from .vision import Ground
    g = Ground(cam, int(cam.get("width", 640)), int(cam.get("height", 480)))
    X, Y = g.floor([g.W / 2.0], [g.H - 0.5])
    if not np.isfinite(X[0]):
        return 400.0
    return float(math.hypot(X[0] - g.pos[0], Y[0] - g.pos[1]))


def config(robot_p: dict) -> dict:
    """The robot_p view the camera's Shield is built from: the lens as the "lidar" (lidar.pos_mm), everything outside
    the field of view as its blind sector (lidar.block_deg = [[hfov/2 - 1, 360 - hfov/2 + 1]]), the near-ground range
    as lidar.min_mm, perc.max_mm 2600, shield.* = the params' shield over BRAIN_SPEC's defaults, overridden by
    camshield.* (memory_s, margin_mm, side_mm, latency_s, v_floor), on = 1."""
    p = copy.deepcopy(robot_p or {})
    cam = p.get("camera") or {}
    hf = hfov_deg(cam)
    li = dict(p.get("lidar") or {})
    li.update(pos_mm=[float(cam.get("x_mm", 150.0)), float(cam.get("y_mm", 0.0))],
              block_deg=[[hf / 2.0 - 1.0, 360.0 - hf / 2.0 + 1.0]], min_mm=near_range_mm(cam), loc=0)
    p["lidar"] = li
    p["perc"] = dict(p.get("perc") or {}, max_mm=MAX_MM)
    cc = cam_cfg(robot_p)
    sh = dict(SH.DEFAULTS)
    sh.update({k: v for k, v in (p.get("shield") or {}).items() if k in SH.DEFAULTS})
    sh.update(memory_s=float(cc["memory_s"]), margin_mm=float(cc["margin_mm"]), side_mm=float(cc["side_mm"]),
              latency_s=float(cc["latency_s"]), v_floor=float(cc["v_floor"]), on=1, virtual=1)
    p["shield"] = sh
    return p


def make(robot_p: dict) -> "SH.Shield":
    """The camera's shield.Shield (enabled whatever shield.on says: one shield per command path is the caller's rule,
    S18 -- the camshield is off when the lidar shield is on)."""
    s = SH.Shield(config(robot_p), {"on": 1})
    s.camera = True
    return s


def perception(e, dets, g, t_frame: float, lat: float, car: tuple, clip_rays: bool = True, pose=None,
               island=None) -> CamPerc:
    """One frame -> the camera's "scan": every SEEN base (wall, red, green, magenta) as a point (CLS_WALL /
    CLS_PILLAR / CLS_LIMITER), each sign detection's 50 x 50 outline, and with clip_rays each clipped wall column's
    ray of points every 20 mm from its image-bottom floor point back to the footprint's front edge.  t = t_frame,
    lag_s = lat (camera.latency_s): Shield.scan carries them from the exposure to now.

    pose: the CONFIDENT field pose at the frame (GO with cam_shield): a clipped column the MAP explains -- a map wall
    along its bearing nearer than the image bottom (+ 60 mm) -- adds no ray: the virtual map points already hold that
    wall.  At the edge of the view a ray ends 16 mm off the nose's centre, inside the grown footprint, so a wall beside
    the car entering the image's bottom corner in a turn braked EVERY command (sim 2026-09-25, A1 clean laps: 0 / 4
    with the camshield, 1 / 4 without, 200-480 braked commands a run).  A clipped column the map does not explain (an
    object, or a wrong pose) keeps its ray; without a pose (LOCATE, LOOK) every clipped column does."""
    from . import vision as V
    fin = np.isfinite(e.X)
    seen = fin & ~e.clip & np.isin(e.cls, (V.WALL, V.RED, V.GREEN, V.MAGENTA))
    X = [e.X[seen].astype(float)]
    Y = [e.Y[seen].astype(float)]
    cmap = {V.WALL: LP.CLS_WALL, V.RED: LP.CLS_PILLAR, V.GREEN: LP.CLS_PILLAR, V.MAGENTA: LP.CLS_LIMITER}
    C = [np.array([cmap[int(c)] for c in e.cls[seen]], np.int8)]
    for q in dets or []:
        if q.get("colour") == "magenta":
            continue
        bx = SH.box_pts(float(q["X"]), float(q["Y"]), 25.0, 25.0, step=10.0)
        X.append(bx[:, 0])
        Y.append(bx[:, 1])
        C.append(np.full(len(bx), LP.CLS_PILLAR, np.int8))
    if clip_rays:
        front = float(car[1])
        cx, cy = float(g.pos[0]), float(g.pos[1])
        cl = fin & e.clip & (e.cls == V.WALL)
        if pose is not None and cl.any():
            from .globloc import ray_aa
            from . import field as F
            px, py, pth = (float(v) for v in pose)
            ox = px + cx * math.cos(pth) - cy * math.sin(pth)
            oy = py + cx * math.sin(pth) + cy * math.cos(pth)
            b = pth + np.arctan2(e.Y[cl] - cy, e.X[cl] - cx)
            t_map = ray_aa(ox, oy, b, island or (-F.ISL, F.ISL, -F.ISL, F.ISL))
            r_bot = np.hypot(e.X[cl] - cx, e.Y[cl] - cy)
            explained = np.zeros(len(e.X), bool)
            explained[np.flatnonzero(cl)] = t_map <= r_bot + 60.0
            cl = cl & ~explained
        for x0, y0 in zip(e.X[cl].tolist(), e.Y[cl].tolist()):
            dx, dy = x0 - cx, y0 - cy
            dist = math.hypot(dx, dy)
            if dist < 1e-6 or x0 <= front:
                continue
            # from the image-bottom point back toward the lens, until the ray reaches x = front (the nose line)
            ux, uy = dx / dist, dy / dist
            s_end = (x0 - front) / max(ux, 1e-6) if ux > 0 else 0.0
            s = np.arange(0.0, min(s_end, dist) + 1e-6, 20.0)
            X.append(x0 - ux * s)
            Y.append(y0 - uy * s)
            C.append(np.full(len(s), LP.CLS_WALL, np.int8))
    return CamPerc(t=float(t_frame), lag_s=float(lat), X=np.concatenate(X), Y=np.concatenate(Y),
                   cls=np.concatenate(C))
