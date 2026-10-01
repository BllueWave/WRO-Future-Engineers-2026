"""Calibration tests that measure with the lidar (BRAIN_SPEC 12): still_check, lidar_tilt, turn_radius_lidar.  Same
contract as bluewave/tests.py (result(ok, summary, data, suggest); a moving test checks `stop` and the latch, never
arms).  lidar_tilt is described in its own docstring.

still_check   the mat 2026-09-23: field_check scored 1.2-1.5 on a car that was moving or had just been moved, 0.009
              on the same field standing still.  Still = the gyro rate, the 80 x 60 grey thumbnail difference of two
              frames (wro_next's stuck measure) and the median |range change| of two lidar scans all under their
              limits for `seconds` in a row, within `max_wait_s`.
turn_radius_lidar   the mat 2026-09-23: the stock A1 turned on R ~510 mm at full lock where the code assumed ~300
              (M1).  Without an encoder the old test's R = v dt / angle is the COMMAND, so it could not see that.  Here
              the car turns `deg` at full lock each way, stands still, and two scans are registered: the second is
              first rotated by the gyro's angle and moved by the nominal chord (tests.icp starts at identity and does
              not converge across a 90 deg turn), then ICP refines the residual.  R = chord / (2 sin(dtheta / 2)), with
              dtheta from the gyro (the ICP rotation must agree within 3 deg).  Suggests the EFFECTIVE wheelbase
              chassis.wheelbase_m = mean(R_left, R_right) tan(steer.max_deg) / 1000, and reports a left / right
              difference over 10 % (a steering-centre offset: re-trim steer.center_us).
"""
from __future__ import annotations

import math
import time

import numpy as np

from . import lidar_perc as LP
from .tests import _halted, _stopped, _wait, icp, result


def _gyro_dps(robot) -> float | None:
    im = robot.p.get("imu", {})
    try:
        v = robot.imu_raw()
        return abs(float(v[int(im.get("gyro_axis", 5))]) * float(im.get("gyro_sign", 1.0)) - float(im.get("gyro_bias", 0.0)))
    except (TypeError, ValueError, IndexError):
        return None


def _thumb(bgr):
    import cv2
    return cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (80, 60), interpolation=cv2.INTER_AREA).astype(np.float32)


def still_check(robot, seconds: float = 1.0, max_wait_s: float = 5.0, gyro_dps: float = 0.8, img_diff: float = 1.0,
                scan_mm: float = 10.0, stop=None, **_):
    """Nothing moves.  ok when the car holds still for `seconds` within `max_wait_s` (module docstring)."""
    t0 = time.monotonic()
    since = None
    worst: dict = dict(gyro_dps=0.0, img_diff=0.0, scan_mm=0.0)
    last_img, last_ft, last_scan = None, 0.0, None
    cur: dict = dict(gyro_dps=None, img_diff=None, scan_mm=None)
    while time.monotonic() - t0 < max_wait_s:
        if stop is not None and stop.is_set():
            return _stopped("the still check")
        now = time.monotonic()
        g = _gyro_dps(robot)
        cur["gyro_dps"] = g
        bgr, tf = robot.frame()
        if bgr is not None and tf != last_ft:
            th = _thumb(bgr)
            if last_img is not None:
                cur["img_diff"] = float(np.mean(np.abs(th - last_img)))
            last_img, last_ft = th, tf
        sc = robot.scan
        if sc is not None and (last_scan is None or sc.t != last_scan.t):
            if last_scan is not None:
                a, b = np.asarray(last_scan.dist, float), np.asarray(sc.dist, float)
                m = (a > 0) & (b > 0)
                cur["scan_mm"] = float(np.median(np.abs(a[m] - b[m]))) if m.sum() > 20 else None
            last_scan = sc
        # + the drive's own speed: a wall parallel to the travel does not change its range, so the scan median alone
        # passed a car driving at 0.2 m/s (10 mm per scan, sim)
        v_now = abs(float(robot.v_odo()))
        ok = ((g is None or g <= gyro_dps) and (cur["img_diff"] is None or cur["img_diff"] <= img_diff)
              and (cur["scan_mm"] is None or cur["scan_mm"] <= scan_mm) and v_now < 0.02)
        for k in worst:
            if cur[k] is not None:
                worst[k] = max(worst[k], cur[k]) if since is not None else cur[k]
        if ok:
            since = since or now
            if now - since >= seconds and (robot.scan is None or cur["scan_mm"] is not None):
                data = dict(gyro_dps=round(worst["gyro_dps"], 2), img_diff=round(worst["img_diff"], 2),
                            scan_mm=round(worst["scan_mm"], 1), waited_s=round(now - t0 - seconds, 1))
                return result(True, "still: gyro %.2f deg/s, image %.2f, scan %.1f mm over %.1f s" % (
                    data["gyro_dps"], data["img_diff"], data["scan_mm"], seconds), data)
        else:
            since = None
        time.sleep(0.03)
    return result(False, "the car is moving -- hold it still, then run again (gyro %s deg/s, image %s, scan %s mm, "
                  "drive %.2f m/s)" % (_r(cur["gyro_dps"], 2), _r(cur["img_diff"], 2), _r(cur["scan_mm"], 1),
                                       abs(float(robot.v_odo()))),
                  dict(gyro_dps=cur["gyro_dps"], img_diff=cur["img_diff"], scan_mm=cur["scan_mm"], waited_s=max_wait_s))


def _r(v, nd):
    return None if v is None else round(v, nd)


def _scan_pts(robot, n_max: int = 300):
    """The newest scan's returns in the car frame, the car's own body out, <= n_max (evenly subsampled)."""
    pr = LP.perceive(robot.scan, LP.config(robot.p))
    if pr is None:
        return None
    m = pr.cls != LP.CLS_CLUTTER
    P_ = np.stack([pr.X[m], pr.Y[m]], 1).astype(np.float64)
    if len(P_) > n_max:
        P_ = P_[np.linspace(0, len(P_) - 1, n_max).astype(int)]
    return P_


def _fresh_scan_pts(robot, stop, after: float, timeout: float = 2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        sc = robot.scan
        if sc is not None and sc.t > after:
            return _scan_pts(robot)
        if _halted(robot, stop):
            return None
        time.sleep(0.02)
    return None


def _one_turn(robot, sign: int, v: float, deg: float, steer: float, wb_mm: float, stop):
    """Full lock `sign` (+1 left), drive until the gyro turned `deg`; before / after scans registered -> dict."""
    if not _wait(robot, stop, 0.4):
        return None
    t_still = time.monotonic()
    p0 = _fresh_scan_pts(robot, stop, t_still)
    if p0 is None or len(p0) < 40:
        return dict(error="no lidar scan before the turn")
    y0 = robot.yaw
    turned, last = 0.0, y0
    t0 = time.monotonic()
    if not robot.drive(v, sign * steer):
        return dict(error="refused")
    while abs(turned) < deg and time.monotonic() - t0 < 10.0:
        if _halted(robot, stop):
            robot.stop()
            return None
        d = ((robot.yaw - last + 180.0) % 360.0) - 180.0
        turned, last = turned + d, robot.yaw
        time.sleep(0.01)
    robot.drive(0.0, sign * steer)
    if not _wait(robot, stop, 0.8):                      # coast, then stand still for the scan
        return None
    d = ((robot.yaw - last + 180.0) % 360.0) - 180.0
    turned += d
    p1 = _fresh_scan_pts(robot, stop, time.monotonic())
    robot.stop()
    if p1 is None or len(p1) < 40:
        return dict(error="no lidar scan after the turn")
    if abs(turned) < 0.5 * deg:
        return dict(error="only %.0f deg turned in %.1f s (blocked?)" % (turned, time.monotonic() - t0), turned=turned)
    th = math.radians(turned)
    R0 = wb_mm / math.tan(math.radians(steer)) * sign          # nominal, signed (+ = centre to the left)
    T0 = np.array([R0 * math.sin(th), R0 * (1.0 - math.cos(th))])
    Rt = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    src = p1 @ Rt.T + T0                                        # the after-scan placed at the nominal end pose
    rot_r, t_r = icp(src, p0)
    rr = math.radians(rot_r)
    Rr = np.array([[math.cos(rr), -math.sin(rr)], [math.sin(rr), math.cos(rr)]])
    T = Rr @ T0 + np.asarray(t_r)
    chord = float(np.hypot(*T))
    R = chord / (2.0 * abs(math.sin(th / 2.0)))
    return dict(turned_deg=round(turned, 1), icp_extra_deg=round(rot_r, 2), chord_mm=round(chord, 1), R_mm=round(R, 1),
                nominal_R_mm=round(abs(R0), 1), agree=abs(rot_r) <= 3.0)


def lidar_tilt(robot, far_mm: float = 2000.0, stop=None, **_):
    """STILL.  Is the scan plane level?  The WLtoys lidar scans at 50 mm under 100 mm walls: tilted UP by a, the beam
    passes over a wall farther than r_v = (100 - h) / tan(a) (1.4 deg at h 50 loses everything beyond 2 m); tilted
    DOWN, it meets the mat at r_f = h / tan(a).  The car anywhere in the field, no signs: the lidar pose by a ray-cast
    search over the whole field (every wall direction), then per scan bin the map's expected range r_exp; a bin sees
    its wall when it returns within 5 % of r_exp.  Per 45 deg sector: the farthest wall still seen, and the bins that
    return well short of their wall (the mat).  PASS when every sector whose walls lie beyond far_mm sees them there.
    Suggests nothing: shim the mount (BRAIN_SPEC 12.6)."""
    from . import field as F
    from . import loc as L
    st = still_check(robot, stop=stop)
    if not st["ok"]:
        return result(False, "not still: " + st["summary"], st["data"])
    sc = robot.scan
    if sc is None:
        return result(False, "no lidar scan")
    cfg = LP.config(robot.p)
    h = float(robot.p.get("lidar", {}).get("h_mm", 145.7))
    if h >= F.WALL_H:
        return result(False, "lidar.h_mm %.1f: the scan plane is above the %d mm walls -- nothing to test" % (h, F.WALL_H))
    pr = LP.perceive(sc, cfg)
    if pr is None:
        return result(False, "no lidar scan")
    X, Y = pr.points(exclude=(LP.CLS_CLUTTER,))
    if len(X) < 40:
        return result(False, "too few returns (%d)" % len(X))
    lx = float(cfg["pos"][0])
    m = np.arange(len(X))[:: max(1, len(X) // 64)]
    az, rng = np.arctan2(Y[m] - cfg["pos"][1], X[m] - lx), np.hypot(X[m] - lx, Y[m] - cfg["pos"][1])
    best = (math.inf, None)
    grid = np.arange(-1440.0, 1441.0, 80.0)
    heads = np.radians(np.concatenate([hd + np.arange(-44.0, 44.1, 3.0) for hd in (0, 90, 180, 270)]))
    for hd in np.array_split(heads, 8):
        G = np.array(np.meshgrid(grid, grid, hd, indexing="ij")).reshape(3, -1).T
        G = G[L.Localizer.valid(G)]
        c = L.Localizer.beam_cost(G, lx, az, rng) / len(m)
        i = int(np.argmin(c))
        if c[i] < best[0]:
            best = (float(c[i]), G[i])
        if _halted(robot, stop):
            return _stopped("the pose search")
    cost, pose = best
    if pose is not None:                                 # refine: 20 mm / 0.5 deg around the coarse best
        fx = np.arange(-80.0, 80.1, 20.0)
        ft = np.radians(np.arange(-3.0, 3.01, 0.5))
        R_ = np.array(np.meshgrid(pose[0] + fx, pose[1] + fx, pose[2] + ft, indexing="ij")).reshape(3, -1).T
        R_ = R_[L.Localizer.valid(R_)]
        if len(R_):
            c = L.Localizer.beam_cost(R_, lx, az, rng) / len(m)
            i = int(np.argmin(c))
            if c[i] < cost:
                cost, pose = float(c[i]), R_[i]
    if pose is None or cost > 0.8:
        return result(False, "the scan does not match the field map (cost %.2f): car inside the field, no signs?" % cost)
    x, y, th = (float(v) for v in pose)
    c_, s_ = math.cos(th), math.sin(th)
    ox, oy = x + c_ * lx - s_ * cfg["pos"][1], y + s_ * lx + c_ * cfg["pos"][1]
    d = np.asarray(sc.dist, float)
    a = np.radians(-180.0 + (np.arange(len(d)) + 0.5) * float(sc.res))
    _t, _c, _s, blind = LP._tables(len(d), float(sc.res), cfg["blind"])
    r_exp = F.raycast(ox, oy, th + a, max_mm=6000.0)
    seen = (d > 0) & (np.abs(d - r_exp) <= 0.05 * r_exp)
    short = (d > cfg["min_mm"]) & (d < 0.8 * r_exp)
    sectors, ok = [], True
    for k in range(8):
        lo = -180.0 + 45.0 * k
        sel = (np.degrees(a) >= lo) & (np.degrees(a) < lo + 45.0) & ~blind
        if not sel.any():
            continue
        far = sel & (r_exp > far_mm)
        r_v = float(r_exp[sel & seen].max()) if (sel & seen).any() else 0.0
        row: dict = dict(sector=[lo, lo + 45.0], seen_to_mm=round(r_v), walls_to_mm=round(float(r_exp[sel].max())),
                   short_bins=int((sel & short).sum()), bins=int(sel.sum()))
        if far.any():
            frac = float((far & seen).sum()) / float(far.sum())
            row["far_seen_pct"] = round(100.0 * frac)
            row["tilt_up_max_deg"] = round(math.degrees(math.atan2(F.WALL_H - h, max(r_v, 1.0))), 2) if r_v else None
            if frac < 0.6:
                ok = False
        sectors.append(row)
    data = dict(pose=[round(x), round(y), round(math.degrees(th), 1)], cost=round(cost, 3), h_mm=h, sectors=sectors)
    bad = [s for s in sectors if s.get("far_seen_pct") is not None and s["far_seen_pct"] < 60]
    if ok:
        return result(True, "level: every sector sees its walls beyond %.1f m (pose %d, %d, %.0f deg)" % (
            far_mm / 1000.0, x, y, math.degrees(th)), data)
    return result(False, "TILTED: %d sector(s) lose the walls beyond %.1f m (%s) -- shim the lidar mount" % (
        len(bad), far_mm / 1000.0, ", ".join("%d..%d deg" % tuple(s["sector"]) for s in bad)), data)


def turn_radius_lidar(robot, v: float = 0.2, deg: float = 90.0, steer=None, stop=None, **_):
    """MOVES.  Full lock left `deg`, then right `deg`, at `v`; the radius each way from two lidar scans."""
    try:
        v, deg = float(v), float(deg)
    except (TypeError, ValueError):
        return result(False, "v and deg must be numbers")
    mx = float(robot.p["steer"]["max_deg"])
    steer = mx if steer is None else float(steer)
    if not (0.05 <= v <= min(0.35, float(robot.p["drive"]["max_mps"]))):
        return result(False, "v must be 0.05-0.35 m/s")
    wb = float(robot.p["chassis"]["wheelbase_m"]) * 1000.0
    st = still_check(robot, stop=stop)
    if not st["ok"]:
        return result(False, "not still before the turn: " + st["summary"], st["data"])
    out = {}
    for name, sign in (("left", 1), ("right", -1)):
        r = _one_turn(robot, sign, v, deg, steer, wb, stop)
        if r is None:
            return _stopped("the %s turn" % name)
        if "error" in r:
            return result(False, "%s turn: %s" % (name, r["error"]), dict(out, **{name: r}))
        out[name] = r
    robot.stop()
    Rl, Rr = out["left"]["R_mm"], out["right"]["R_mm"]
    agree = out["left"]["agree"] and out["right"]["agree"]
    Rm = 0.5 * (Rl + Rr)
    wb_eff = Rm * math.tan(math.radians(steer)) / 1000.0
    diff = abs(Rl - Rr) / max(Rm, 1.0)
    data = dict(out, R_mean_mm=round(Rm, 1), lr_diff_pct=round(100 * diff, 1), steer_deg=steer,
                wheelbase_eff_m=round(wb_eff, 4), wheelbase_now_m=round(wb / 1000.0, 4))
    if not agree:
        return result(False, "CHECK: the scans disagree with the gyro by > 3 deg (%.1f / %.1f) -- more static things "
                      "around the car, then again" % (out["left"]["icp_extra_deg"], out["right"]["icp_extra_deg"]),
                      data)
    msg = "R left %.0f mm, right %.0f mm at %.1f deg -> effective wheelbase %.3f m (now %.3f)" % (
        Rl, Rr, steer, wb_eff, wb / 1000.0)
    if diff > 0.10:
        msg += "; left / right differ %.0f %%: re-trim steer.center_us" % (100 * diff)
    return result(True, msg, data, {"chassis.wheelbase_m": round(wb_eff, 4)})
