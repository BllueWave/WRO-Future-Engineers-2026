"""Component tests -- one per [DAY1] number in params.py.  Each returns {ok, summary, data, suggest} where `suggest` is the
params change the measurement supports (the UI offers it as one click; nothing is saved without that click).

Tests that move the car say so in MOVES.  Tests that need a person say what to do in HOW; the test then measures
what the person did, so the answer is a reading, not a guess.

A moving test gets `stop`, a threading.Event the agent sets on E-STOP, on the Stop button, when the console that
started it disconnects and on the link watchdog.  It checks `stop` and the latch between phases (`_wait`) and never
arms the robot: auto_calib once called robot.arm() and so undid an E-STOP pressed in its first 0.4 s (measured).
"""
from __future__ import annotations

import math
import statistics
import time

import numpy as np

SPEED_RUN_MAX_S = 5.0


def _halted(robot, stop) -> bool:
    return bool(robot.estopped or (stop is not None and stop.is_set()))


def _wait(robot, stop, seconds: float) -> bool:
    """Sleep `seconds` in 20 ms steps; False (and the car stopped) as soon as the latch or `stop` is set."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if _halted(robot, stop):
            robot.stop()
            return False
        time.sleep(min(0.02, max(0.0, end - time.monotonic())))
    if _halted(robot, stop):
        robot.stop()
        return False
    return True


def _stopped(what: str):
    return result(False, "stopped: E-STOP or Stop during %s" % what)

MOVES = {"servo_sweep", "motor_pulse", "speed_run", "turn_radius"}
HOW = {
    "board": "Nothing to do.  Checks the controller board link: IMU rate, battery, frame errors.",
    "imu_bias": "Keep the car completely still for 3 s.",
    "imu_direction": "Within 4 s, turn the car about 90 deg to the LEFT (anticlockwise seen from above) by hand.",
    "servo_sweep": "Wheels OFF the ground or car on the mat with room.  Watch: the wheels must go LEFT first.",
    "motor_pulse": "Wheels OFF the ground.  Both wheels must spin FORWARD for 1 s, then backward for 1 s.",
    "lidar_health": "Nothing to do.  Checks the lidar spins at its rate and returns a full circle.",
    "lidar_direction": "Start with nothing near the car.  After the BEEP, put a box ~30 cm from the car on its LEFT.",
    "camera": "Point the camera at a red and a green pillar under the hall's light.",
    "speed_run": "Car on the mat, pointing straight at a wall 1.5-2.5 m away.  It drives 0.8 m forward at 0.3 m/s.",
    "turn_radius": "Car on an open mat area (1 m clear around it).  It drives one full circle at max steer, left.",
}


def _clean(x):
    """JSON-safe: NaN / inf -> None, numpy scalars -> Python.  A lidar sector with no return is NaN -- measured on
    the real car (the simulator never produced one), and JSON cannot carry it."""
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, np.generic):
        x = x.item()
    if isinstance(x, float) and (x != x or x in (float("inf"), float("-inf"))):
        return None
    return x


def result(ok, summary, data=None, suggest=None):
    return dict(ok=bool(ok), summary=summary, data=_clean(data or {}), suggest=_clean(suggest or {}))


def board(robot, **_):
    st = robot.board.state
    time.sleep(1.0)
    hz, bat = st.imu_rate_hz(), robot.battery_v()
    par = getattr(robot.board, "parser", None)
    data = dict(imu_hz=round(hz, 1), imu_age_ms=round(st.imu.age() * 1000), battery_v=bat,
                frames=getattr(par, "frames", None), crc_errors=getattr(par, "crc_errors", None))
    ok = hz > 20 and st.imu.age() < 0.2
    low = bat is not None and bat < 7.2
    return result(ok and not low, "IMU %.0f Hz, battery %s V%s" % (hz, bat, " -- CHARGE IT" if low else ""), data)


def imu_bias(robot, seconds: float = 3.0, **_):
    axis = robot.p["imu"]["gyro_axis"]
    vals, last = [], 0.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        st = robot.board.state.imu
        if st.t != last and st.value:
            last = st.t
            vals.append(st.value[axis] * robot.p["imu"]["gyro_sign"])
        time.sleep(0.005)
    if len(vals) < 20:
        return result(False, "only %d IMU samples in %.0f s" % (len(vals), seconds))
    m, sd = statistics.fmean(vals), statistics.pstdev(vals)
    ok = sd < 0.5
    return result(ok, "gyro bias %.3f deg/s (noise %.3f)%s" % (m, sd, "" if ok else " -- the car moved?"),
                  dict(samples=len(vals), mean=m, std=sd, drift_per_min_deg=round(m * 60, 1)),
                  {"imu.gyro_bias": round(m, 4)} if ok else {})


def imu_direction(robot, seconds: float = 4.0, **_):
    y0 = robot.yaw
    t0, peak = time.monotonic(), 0.0
    while time.monotonic() - t0 < seconds:
        d = ((robot.yaw - y0 + 180) % 360) - 180
        peak = d if abs(d) > abs(peak) else peak
        time.sleep(0.02)
    if abs(peak) < 30:
        return result(False, "only %.0f deg of rotation seen -- turn it further" % peak, dict(peak=peak))
    sign = robot.p["imu"]["gyro_sign"] * (1 if peak > 0 else -1)
    return result(peak > 0, "turned %.0f deg -> gyro_sign should be %+d" % (peak, sign), dict(peak=peak),
                  {} if peak > 0 else {"imu.gyro_sign": sign})


def servo_sweep(robot, stop=None, **_):
    mx = robot.p["steer"]["max_deg"]
    for d in (0, mx, 0, -mx, 0):
        if _halted(robot, stop) or not robot.steer_only(d) or not _wait(robot, stop, 0.7):
            return _stopped("the sweep")
    return result(True, "swept 0 -> +%.0f (LEFT) -> 0 -> -%.0f (RIGHT) -> 0.  If it went RIGHT first, set "
                        "steer.invert." % (mx, mx), dict(max_deg=mx))


def motor_pulse(robot, rps: float = 1.0, stop=None, **_):
    d = robot.p["drive"]
    v = rps * math.pi * d["wheel_d_m"]
    for sign in (1, -1):
        if not robot.drive(sign * v, 0.0) or not _wait(robot, stop, 1.0):
            robot.stop()
            return _stopped("the pulse")
    robot.stop()
    return result(True, "forward then back at %.2f m/s (%.1f rps) on motors %s" % (v, rps, list(d["motors"])),
                  dict(v=v, rps=rps, motors=d["motors"]))


def lidar_health(robot, **_):
    time.sleep(1.2)
    sc = robot.scan
    if sc is None:
        return result(False, "no lidar scan -- check the port (params.lidar.device) and that it spins")
    valid = int((sc.dist > 0).sum())
    frac = valid / len(sc.dist)
    age = time.monotonic() - sc.t
    ok = frac > 0.6 and age < 0.5 and 300 < sc.rpm < 1200
    return result(ok, "%.0f rpm, %d / %d bins valid, last scan %.0f ms ago" % (sc.rpm, valid, len(sc.dist), age * 1e3),
                  dict(rpm=sc.rpm, valid=valid, bins=len(sc.dist), age_ms=round(age * 1e3),
                       front=sc.at(0), left=sc.at(90), right=sc.at(-90), back=sc.at(180)))


def _median_scan(robot, n: int = 3, gap: float = 0.12):
    arr = []
    for _ in range(n):
        sc = robot.scan
        if sc is not None:
            arr.append(np.where(sc.dist > 0, sc.dist, np.nan))
        time.sleep(gap)
    return (np.nanmedian(np.array(arr), axis=0), robot.scan.angles()) if arr else (None, None)


def find_new_object(base, now, angles, drop_mm: float = 150.0, min_bins: int = 3):
    """The angle (deg) of the widest run of adjacent bins that came >= drop_mm NEARER than the baseline, or None.
    Differential on purpose: a wall is in both scans and cancels, so only what the person placed is left."""
    closer = (base - now) >= drop_mm
    closer &= ~np.isnan(base) & ~np.isnan(now)
    idx = np.flatnonzero(closer)
    if len(idx) == 0:
        return None
    runs, cur = [], [idx[0]]
    for i in idx[1:]:
        if i == cur[-1] + 1:
            cur.append(i)
        else:
            runs.append(cur)
            cur = [i]
    runs.append(cur)
    best = max(runs, key=len)
    if len(best) < min_bins:
        return None
    return float(np.mean(angles[best])), float(np.nanmedian(now[best]))


def lidar_direction(robot, seconds: float = 6.0, **_):
    """Baseline first, THEN the person places the object: whatever moved closer is the object."""
    if robot.scan is None:
        return result(False, "no lidar scan")
    base, ang = _median_scan(robot)
    robot.beep(2400)                                  # the cue: place the object now
    t0 = time.monotonic()
    hit = None
    while time.monotonic() - t0 < seconds and hit is None:
        now, _a = _median_scan(robot, 2, 0.1)
        hit = find_new_object(base, now, ang)
    if hit is None:
        return result(False, "nothing new appeared -- start the test, THEN place the object after the beep")
    a, mm = hit
    ok = 60 <= a <= 120
    sug = {}
    if not ok:
        # the object is truly at +90.  A mirror (it shows near -90) is the direction; anything else is the offset.
        if abs(a + 90) < 30:
            sug = {"lidar.cw": not robot.p["lidar"]["cw"]}
        else:
            sug = {"lidar.offset_deg": round((robot.p["lidar"]["offset_deg"] + (a - 90) *
                                              (-1 if robot.p["lidar"]["cw"] else 1)) % 360, 1)}
    return result(ok, "the object appeared at %.0f deg, %.0f mm (want about +90 = LEFT)" % (a, mm),
                  dict(angle=a, mm=mm), sug)


def camera(robot, **_):
    from . import camera as C
    time.sleep(0.5)
    f, t = robot.frame()
    if f is None:
        return result(False, "no camera frame -- check params.camera.device")
    ms = C.masks(f, robot.p["hsv"])
    counts = {c: len(C.blobs(ms[c])) for c in ("red", "green", "magenta", "orange", "blue") if c in ms}
    fps = robot.camera.fps() if robot.camera else 0
    return result(fps > 10, "%dx%d at %.0f fps; blobs %s" % (f.shape[1], f.shape[0], fps, counts),
                  dict(w=f.shape[1], h=f.shape[0], fps=fps, blobs=counts))


def speed_run(robot, v: float = 0.3, dist_m: float = 0.8, stop=None, **_):
    try:
        v, dist_m = float(v), float(dist_m)
    except (TypeError, ValueError):
        return result(False, "v and dist_m must be numbers")
    if not (0.05 <= v <= robot.p["drive"]["max_mps"]) or not (0.1 <= dist_m <= 1.5):
        return result(False, "v must be 0.05-%.2f m/s and dist_m 0.1-1.5 m" % robot.p["drive"]["max_mps"])
    sc0 = robot.scan
    if sc0 is None:
        return result(False, "needs the lidar to measure the distance travelled")
    f0 = sc0.at(0, 2.0)
    if not (1400 <= f0 <= 2600):
        return result(False, "front wall %.0f mm away -- put it 1.5-2.5 m ahead" % f0)
    robot.reset_yaw()
    t0 = time.monotonic()
    if not robot.drive(v, 0.0) or not _wait(robot, stop, min(dist_m / v, SPEED_RUN_MAX_S)):
        robot.stop()
        return _stopped("the run")
    robot.stop()
    dt = time.monotonic() - t0
    time.sleep(0.6)                                   # let it stop and a fresh scan arrive
    f1 = robot.scan.at(0, 2.0)
    travelled = (f0 - f1) / 1000.0
    ratio = travelled / (v * dt) if dt > 0 else float("nan")
    scale = robot.p["drive"]["rps_scale"] / ratio if ratio > 0.3 else None
    return result(scale is not None and abs(ratio - 1) < 0.5,
                  "commanded %.2f m, travelled %.2f m (x%.2f), heading drift %.1f deg" % (v * dt, travelled, ratio,
                                                                                          robot.yaw),
                  dict(f0=f0, f1=f1, commanded_m=v * dt, travelled_m=travelled, ratio=ratio, yaw=robot.yaw),
                  {"drive.rps_scale": round(scale, 3)} if scale else {})


def turn_radius(robot, v: float = 0.25, stop=None, **kw):
    if robot.scan is not None:
        # the lidar measures the radius the car REALLY turns (tests_cal.turn_radius_lidar, BRAIN_SPEC 12.3): the
        # command-based R below is v dt / angle, meaningless without an encoder -- it missed the mat's R 510 (M1)
        from .tests_cal import turn_radius_lidar
        return turn_radius_lidar(robot, v=min(float(v), 0.2), stop=stop, **kw)
    mx = robot.p["steer"]["max_deg"]
    try:
        v = float(v)
    except (TypeError, ValueError):
        return result(False, "v must be a number")
    if not (0.05 <= v <= robot.p["drive"]["max_mps"]):
        return result(False, "v must be 0.05-%.2f m/s" % robot.p["drive"]["max_mps"])
    robot.reset_yaw()
    turned, last = 0.0, 0.0
    t0 = time.monotonic()
    if not robot.drive(v, mx):
        return _stopped("the circle")
    while turned < 355 and time.monotonic() - t0 < 12:
        if _halted(robot, stop):
            robot.stop()
            return _stopped("the circle")
        d = ((robot.yaw - last + 180) % 360) - 180
        turned, last = turned + d, robot.yaw
        time.sleep(0.01)
    robot.stop()
    dt = time.monotonic() - t0
    if turned < 300:
        return result(False, "only %.0f deg turned in %.1f s" % (turned, dt), dict(turned=turned))
    r = v * dt / math.radians(turned) * robot.p["drive"]["rps_scale"]
    L = robot.p["chassis"]["wheelbase_m"]
    eff = math.degrees(math.atan(L / r))
    return result(True, "radius %.0f mm at %.0f deg commanded -> the wheels really reach %.1f deg" % (r * 1e3, mx, eff),
                  dict(radius_m=r, seconds=dt, effective_steer_deg=eff))


def travel_direction(s0, s1, angles, clip: float = 400.0):
    """Where the car went between two scans, in the scan's own angle frame: a static point at angle t comes
    d(t) = r0 - r1 ~ D cos(t - tf) nearer, so the first Fourier term of d gives tf and D.  (Verified on a synthetic
    3 x 3 m box: 0 / 180 / 37 deg recovered within 1.2 deg, 150 mm within 2 mm.)"""
    d = s0 - s1
    m = ~np.isnan(d) & (np.abs(d) < clip)
    if m.sum() < 40:
        return None, None
    t = np.radians(angles[m])
    a, b = float((d[m] * np.cos(t)).sum()), float((d[m] * np.sin(t)).sum())
    return math.degrees(math.atan2(b, a)), 2.0 * math.hypot(a, b) / m.sum()


def scan_rotation(s1, s2, res: float, maxdeg: int = 70):
    """The circular shift k (deg) that best maps s2 onto s1: s2(t) = s1(t + k).  A car turning LEFT (CCW) by alpha
    sees static points move to t - alpha, so k = +alpha in a correctly-handed frame and -alpha in a mirrored one."""
    best, bk = None, 0
    n = len(s1)
    for k in range(-int(maxdeg / res), int(maxdeg / res) + 1):
        a, b = s2, np.roll(s1, -k)
        m = ~np.isnan(a) & ~np.isnan(b)
        if m.sum() < 60:
            continue
        err = float(np.median(np.abs(a[m] - b[m])))
        if best is None or err < best:
            best, bk = err, k
    return bk * res, best


def scan_points(dist, angles, cap: int = 260):
    """Valid returns as (x, y) mm in the scan's frame, evenly subsampled to at most `cap` points."""
    m = ~np.isnan(dist) & (dist > 60) & (dist < 6000)
    a = np.radians(angles[m])
    p = np.stack([dist[m] * np.cos(a), dist[m] * np.sin(a)], 1)
    if len(p) > cap:
        p = p[np.linspace(0, len(p) - 1, cap).astype(int)]
    return p


def icp(src, dst, iters: int = 40):
    """Rigid 2-D point-to-point ICP: R, t minimising |R src + t - dst|, as (rotation deg CCW, t).
    Scans taken before (dst) and after (src) a car motion of translation T and CCW rotation a give t = T in the
    FIRST scan's frame and rotation = a -- in a correctly-handed frame; a mirrored frame gives -a.
    Matching points, not angle bins: a wall parallel to the motion reads the same range at a fixed angle before and
    after, which is what threw the bin-difference fit 17 deg off in the corridor simulator."""
    R, t = np.eye(2), np.zeros(2)
    for it in range(iters):
        q = src @ R.T + t
        d2 = ((q[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dist = np.sqrt(d2[np.arange(len(q)), j])
        # the outlier gate starts WIDE and tightens: in a corridor the side walls match at ~0 from the start, and a
        # median-based gate alone rejected the far wall -- the only thing that shows a move along the corridor
        m = dist < max(3.0 * float(np.median(dist)), 400.0 * 0.85 ** it, 30.0)
        if m.sum() < 20:
            break
        A, B = src[m], dst[j[m]]
        ca, cb = A.mean(0), B.mean(0)
        U, _S, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        Rn = Vt.T @ U.T
        if np.linalg.det(Rn) < 0:
            Vt[1] *= -1
            Rn = Vt.T @ U.T
        R, t = Rn, cb - Rn @ ca
    return math.degrees(math.atan2(R[1, 0], R[0, 0])), t


def auto_calib(robot, v: float = 0.15, steer: float = 20.0, stop=None, **_):
    """No person needed: ~0.15 m straight, then a short LEFT arc.  The straight gives the lidar's front (offset);
    the arc, with the CAMERA as the independent reference for left (turning left shifts the image right --
    cv2.phaseCorrelate dx > 0, verified on a synthetic roll), gives the steering direction, the gyro sign and
    whether the lidar is mirrored.  It never arms: the agent refuses it while the latch is set, and a latch set
    during it ends it."""
    import cv2
    sc = robot.scan
    if sc is None or robot.frame()[0] is None:
        return result(False, "needs the lidar and the camera")
    near = np.where(sc.dist > 60, sc.dist, 99999).min()
    if near < 350:
        return result(False, "something is %d mm from the car -- give it 35 cm of room all round" % near)
    s0, ang = _median_scan(robot)
    if _halted(robot, stop) or not robot.drive(v, 0.0) or not _wait(robot, stop, 1.0):
        robot.stop()
        return _stopped("the straight")
    robot.stop()
    if not _wait(robot, stop, 0.8):
        return _stopped("the straight")
    s1, _a = _median_scan(robot)
    f1 = cv2.cvtColor(robot.frame()[0], cv2.COLOR_BGR2GRAY).astype(np.float32)
    rs, ts = icp(scan_points(s1, ang), scan_points(s0, ang))     # the straight: where did the car go?
    tf, dist = (math.degrees(math.atan2(ts[1], ts[0])), float(np.hypot(*ts))) if np.hypot(*ts) > 40 else (None, None)
    y0 = robot.yaw
    if _halted(robot, stop) or not robot.drive(v, steer) or not _wait(robot, stop, 1.3):
        robot.stop()
        return _stopped("the arc")
    robot.stop()
    if not _wait(robot, stop, 0.9):
        return _stopped("the arc")
    s2, _a = _median_scan(robot)
    f2 = cv2.cvtColor(robot.frame()[0], cv2.COLOR_BGR2GRAY).astype(np.float32)
    (dx, _dy), resp = cv2.phaseCorrelate(f1, f2)
    dyaw = ((robot.yaw - y0 + 180) % 360) - 180
    k, _ta = icp(scan_points(s2, ang), scan_points(s1, ang))      # the arc: how far, and which way, it turned
    L = robot.p["lidar"]
    s = -1 if L["cw"] else 1
    data = dict(travel_dir_deg=tf, travel_mm=dist, straight_rot_deg=rs, camera_dx_px=dx, camera_response=resp,
                gyro_dyaw_deg=dyaw, lidar_rot_deg=k)
    if tf is None:
        return result(False, "the lidar did not see the car move -- is it on the mat?", data)
    sug = {}
    front = ((tf + (180.0 if v < 0 else 0.0) + 180) % 360) - 180      # reversing: it travelled toward its BACK
    data["lidar_front_deg"] = front
    if abs(front) > 15:
        sug["lidar.offset_deg"] = round((L["offset_deg"] + s * front) % 360, 1)
    if resp < 0.05 or abs(dx) < 8:
        return result(False, "the camera could not tell the turn (dx %.0f px, response %.2f) -- more light or "
                             "more texture in view" % (dx, resp), data, sug)
    turn_sign = 1 if dx > 0 else -1                      # the camera: the car REALLY turned left (+) or right (-)
    expected = (1 if v > 0 else -1) * (1 if steer > 0 else -1)   # + steer is LEFT going forward, RIGHT in reverse
    if turn_sign != expected:
        sug["steer.invert"] = not robot.p["steer"]["invert"]
    if abs(dyaw) > 5 and (dyaw > 0) != (turn_sign > 0):
        sug["imu.gyro_sign"] = -robot.p["imu"]["gyro_sign"]
    if abs(k) > 5 and (k > 0) != (turn_sign > 0):
        sug["lidar.cw"] = not L["cw"]
    summary = ("straight (%s): lidar front at %.0f deg (want 0), moved %.0f mm | arc: camera says the car turned %s "
               "(dx %+.0f px), gyro %+.0f deg, lidar rotation %+.0f deg"
               % ("reverse" if v < 0 else "forward", front, dist, "LEFT" if turn_sign > 0 else "RIGHT", dx, dyaw, k))
    return result(not sug, summary, data, sug)


def cam_pitch(robot, dist_mm: float = 500.0, bumper_mm: float = 180.0, frames: int = 8, **_):
    """Camera pitch from one pillar whose NEAR face stands `dist_mm` in front of the front bumper, centred: the base
    row of the pillar is a ray to a floor point at a known distance, which fixes the pitch for the known lens height
    (params.camera.h_mm, x_mm).  Needed because the stock mount moved ~10 deg in one day (2026-09-22/23)."""
    import cv2
    import numpy as np
    from . import vision as V
    cam = robot.p["camera"]
    x_face = bumper_mm + dist_mm                       # rear-axle frame, as every car-frame number in vision.py
    rows = []
    for _ in range(frames * 3):
        f, _t = robot.frame()
        if f is None:
            time.sleep(0.05)
            continue
        hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
        _d, red, green, _m = V._classes(hsv, dict(V.DEFAULT_VISION, **robot.p.get("vision", {})))
        best = None
        for m in (red, green):
            n, _lab, st, _c = cv2.connectedComponentsWithStats(m.astype(np.uint8))
            for i in range(1, n):
                x, y, w, h, a = st[i]
                if a > 400 and (best is None or a > best[4]):
                    best = (x, y, w, h, a)
        if best is not None:
            x, y, w, h, a = best
            core = (red | green)[:, x + w // 4:x + 3 * w // 4].mean(1) > 0.5
            ys = np.nonzero(core)[0]
            if len(ys) and ys.max() < f.shape[0] - 2:
                rows.append((x + w / 2.0, ys.min() - 0.5, ys.max() + 0.5, f.shape[1], f.shape[0], float(w)))
        if len(rows) >= frames:
            break
        time.sleep(0.07)
    if len(rows) < 3:
        return result(False, "no whole pillar in view (base must be above the image bottom) -- put one pillar "
                             "centred, its near face %.0f mm from the front bumper" % dist_mm)
    uc, vt, vb = (float(np.median([r[i] for r in rows])) for i in (0, 1, 2))
    W, H = rows[0][3], rows[0][4]

    def x_at(h_mm, pitch, v, z):
        g = V.Ground(dict(cam, h_mm=h_mm, pitch_deg=pitch), W, H)
        return float(g.floor([uc], [v], z=z)[0][0])

    h_mm = float(cam["h_mm"])
    lo, hi = -25.0, 35.0                                   # pitch: the base ray must land on the near face
    for _ in range(40):
        mid = (lo + hi) / 2
        xb = x_at(h_mm, mid, vb, 0.0)
        if not math.isfinite(xb) or xb > x_face:
            lo = mid
        else:
            hi = mid
    pitch = (lo + hi) / 2
    # the top face is ~2 px tall at this range, so its edge cannot fix the height (1.4 px of blur = 37 mm, measured on
    # a rendered frame): the height stays params.camera.h_mm -- a mount that rotates barely moves the lens up or down
    x_top = x_at(h_mm, pitch, vt, 100.0)
    # IS it the pillar at dist_mm?  The base row alone always gives SOME pitch: a mat sign 800 mm ahead (the test pillar
    # forgotten) gave 8.65 deg for a true 6.66 and the field check after it still said STANDARD (review 2026-09-25).
    # Two pitch-free-enough checks: (1) the blob's WIDTH against the pillar's 50 mm projected at x_face (a far sign is
    # narrower: ~0.6 x at 800 mm); (2) its TOP edge: with the lens above the pillar (h > 100) the top row is the top
    # face's far edge, x_face + 50 (mock: +38 mm of blur when right, -209 with no pillar there, -262 at 650 mm)
    w_px = float(np.median([r[5] for r in rows]))
    g_fit = V.Ground(dict(cam, h_mm=h_mm, pitch_deg=pitch), W, H)
    yc = float(g_fit.floor([uc], [vb])[1][0])
    pil = np.array([[x_face + dx, yc + dy, z] for dx in (0.0, 50.0) for dy in (-25.0, 25.0) for z in (0.0, 100.0)]).T
    pu, _pv, pd = g_fit.project(pil)
    w_exp = float(np.max(pu[pd > 0]) - np.min(pu[pd > 0])) if np.any(pd > 0) else float("nan")
    top_exp = x_face + (50.0 if h_mm > 100.0 else 0.0)
    why = []
    if abs(uc - W / 2) >= W * 0.2:
        why.append("the pillar is not centred, put it straight ahead")
    if math.isfinite(w_exp) and w_exp > 0 and abs(w_px / w_exp - 1.0) > 0.15:
        why.append("the blob is %.0f px wide, a pillar at %.0f mm would be %.0f: no pillar at %.0f mm from the bumper "
                   "(a sign farther away?)" % (w_px, dist_mm, w_exp, dist_mm))
    if math.isfinite(x_top) and abs(x_top - top_exp) > 100.0:
        why.append("its top edge reads %.0f mm, the pillar's would be %.0f: not the pillar at %.0f mm"
                   % (x_top, top_exp, dist_mm))
    data = dict(base_row=round(vb, 1), top_row=round(vt, 1), u=round(uc, 1), frames=len(rows), pitch_deg=round(pitch, 2),
                h_mm=h_mm, top_edge_x_mm=round(x_top, 1) if math.isfinite(x_top) else None,
                top_expect_mm=round(top_exp, 1), w_px=round(w_px, 1),
                w_expect_px=round(w_exp, 1) if math.isfinite(w_exp) else None,
                was=dict(pitch_deg=cam["pitch_deg"], h_mm=cam["h_mm"]))
    ok = not why
    return result(ok, "camera pitch %.2f deg (%s) at height %.1f mm; params had %.2f deg%s"
                  % (pitch, "down" if pitch > 0 else "up", h_mm, cam["pitch_deg"],
                     "" if ok else " -- NOT APPLIED: " + "; ".join(why)),
                  data, {"camera.pitch_deg": round(pitch, 2)} if ok else {})


HOW["cam_pitch"] = ("One pillar on the mat, centred in front of the car, its near face 500 mm from the front bumper "
                    "(dist_mm=... to change).  Nothing moves.  Measures the camera's pitch from the pillar.")

def depth(robot, dist_mm: float = 500.0, bumper_mm: float = -1.0, frames: int = 8, **_):
    """[DAY1] the HP60C depth against a tape measure (the cam_pitch setup): one pillar centred, its NEAR face `dist_mm`
    in front of the front bumper (bumper_mm -1 = params car.front_mm).  The median of the pillar's central depth
    pixels (the RGB colour says which), as forward distance, against the true one: depth.scale is their ratio -- one
    red pillar at ~1.25 m read 1.43 m on the team's field (2026-09-22).  Also the stream's rate, the stamp's age and
    the fill (the share of the pillar's pixels with a return).  Nothing moves."""
    import cv2
    from . import depth as D
    from . import vision as V
    dcam = getattr(robot, "depth_cam", None)
    if dcam is None:
        return result(False, "no depth stream: `bw apply depth.on=1`, restart the agent, and the bridge must run in the "
                             "camera container (os/install_on_stock.sh)")
    cfg, cam = D.config(robot.p), robot.p["camera"]
    if bumper_mm < 0:
        bumper_mm = float(robot.p.get("car", {}).get("front_mm", 179.0))
    x_face = bumper_mm + dist_mm                         # rear-axle frame
    vis = dict(V.DEFAULT_VISION, **robot.p.get("vision", {}))
    xs, fills, us, t_last, dg = [], [], [], 0.0, None
    end = time.monotonic() + max(3.0, 0.5 * frames)
    while len(xs) < frames and time.monotonic() < end:
        d, td = robot.depth()
        f, _tf = robot.frame()
        if d is None or f is None or td == t_last:
            time.sleep(0.03)
            continue
        t_last = td
        _dk, red, green, _mg = V._classes(cv2.cvtColor(f, cv2.COLOR_BGR2HSV), vis)
        best = None
        for m in (red, green):
            n, _lab, st, _c = cv2.connectedComponentsWithStats(m.astype(np.uint8))
            for i in range(1, n):
                if st[i][4] > 400 and (best is None or st[i][4] > best[4]):
                    best = st[i]
        if best is None:
            continue
        x, y, w, h, _a = (int(v) for v in best)
        H, W = f.shape[:2]
        dh, dw = d.shape[:2]
        if dg is None:
            dg = D.DepthGeom(V.Ground(cam, W, H), cfg, dw, dh)
        u0, u1 = int((x + w / 4) * dw / W), int((x + 3 * w / 4) * dw / W) + 1
        v0, v1 = int((y + h / 4) * dh / H), int((y + 3 * h / 4) * dh / H) + 1
        patch = d[v0:v1, u0:u1]
        ok = patch > 0
        fills.append(float(ok.mean()) if patch.size else 0.0)
        us.append(x + w / 2.0)
        if ok.sum() < 5:
            continue
        vv, uu = np.nonzero(ok)
        z = patch[ok].astype(float) * dg.scale
        xp, yp = (uu + u0 - dg.cx) / dg.fx, (vv + v0 - dg.cy) / dg.fy
        R = dg.R
        xs.append(float(np.median(z * (R[0, 0] * xp + R[0, 1] * yp + R[0, 2]) + dg.pos[0])))
    data = dict(fps=round(dcam.fps(), 1), age_ms=getattr(dcam, "age_ms", None), frames=len(xs),
                fill=round(float(np.median(fills)), 2) if fills else None, face_true_mm=round(x_face, 1))
    if len(xs) < 3:
        return result(False, "no pillar with depth returns in view -- one pillar centred, its near face %.0f mm from "
                             "the front bumper (fill %s)" % (dist_mm, data["fill"]), data)
    x_meas, lens = float(np.median(xs)), float(dg.pos[0])
    ratio = (x_face - lens) / max(1.0, x_meas - lens)
    scale = round(float(cfg.get("scale", 1.0)) * ratio, 3)
    data.update(face_depth_mm=round(x_meas, 1), err_mm=round(x_meas - x_face, 1), scale=scale)
    ok = abs(float(np.median(us)) - W / 2) < 0.2 * W and (data["fill"] or 0) >= 0.3
    return result(ok, "pillar face at %.0f mm by depth, %.0f by tape (%+.0f mm): depth.scale %.3f; fill %.0f %%, %.1f fps%s"
                  % (x_meas, x_face, x_meas - x_face, scale, 100 * (data["fill"] or 0), data["fps"],
                     "" if ok else " -- centre the pillar / too few returns"),
                  data, {"depth.scale": scale} if ok else {})


HOW["depth"] = ("depth.on = 1 and the bridge running.  One pillar on the mat, centred in front of the car, its near "
                "face 500 mm from the front bumper (dist_mm=... for others: 300, 800, 1200).  Nothing moves.  Measures "
                "depth.scale against the tape, the rate and the fill.")

MOVES.add("auto_calib")
HOW["auto_calib"] =("Car on the mat with 35 cm of room all round.  It drives ~15 cm straight, then a short left "
                     "arc, and measures lidar front, lidar mirror, gyro sign and steering direction by itself.")


# ---------------------------------------------------------------------------------------------- the WLtoys drive
# gpio_pwm (an encoderless motor on a PWM driver): the plant is measured with the LIDAR as the only speed sensor --
# the front wall's distance, one sample per revolution.  Every duty is battery-normalised to drive.duty.v_ref (the
# model divides by V_bat), so a fit on a full pack holds on a tired one.

def _front_samples(robot, since: float, until: float, stop, out: list) -> bool:
    """Append (t, front mm) of every NEW scan until `until`; False when halted."""
    last = out[-1][0] if out else 0.0
    while time.monotonic() < until:
        if _halted(robot, stop):
            robot.stop()
            return False
        sc = robot.scan
        if sc is not None and sc.t > max(last, since):
            f = sc.at(0, 2.5)
            if f == f:
                out.append((sc.t, float(f)))
            last = sc.t
        time.sleep(0.01)
    return True


def _slope(rows):
    """Least-squares d(front)/dt of [(t, f)], mm/s (negative = approaching the wall)."""
    if len(rows) < 3:
        return float("nan")
    t = np.array([r[0] for r in rows])
    f = np.array([r[1] for r in rows])
    t = t - t.mean()
    return float((t * (f - f.mean())).sum() / max((t * t).sum(), 1e-9))


def _still_front(robot, stop, n: int = 3, tol: float = 3.0, max_s: float = 2.0):
    """The front distance once `n` consecutive scans agree within `tol` mm (the car has stopped), or None."""
    rows = []
    end = time.monotonic() + max_s
    while time.monotonic() < end:
        if not _front_samples(robot, 0.0, min(end, time.monotonic() + 0.12), stop, rows):
            return None
        if len(rows) >= n and max(r[1] for r in rows[-n:]) - min(r[1] for r in rows[-n:]) <= tol:
            return float(np.median([r[1] for r in rows[-n:]]))
    return float(np.median([r[1] for r in rows[-n:]])) if len(rows) >= n else None


def _need_pwm(robot):
    if getattr(robot, "motor", None) is None:
        return result(False, "needs the gpio_pwm drive (drive.backend=gpio_pwm, `bw apply --profile wltoys`)%s"
                      % ((": " + robot.motor_error) if getattr(robot, "motor_error", "") else ""))
    if robot.scan is None:
        return result(False, "needs the lidar: it is the only speed sensor")
    return None


def duty_sweep(robot, start: float = 0.06, step: float = 1 / 255.0, dwell_s: float = 0.3,
               speeds=(0.15, 0.25, 0.35), hold_s: float = 1.0, stop=None, **_):
    """[DAY1] the plant: BREAKAWAY (the duty ramped one 8-bit count per dwell from rest until the lidar sees the car
    move 8 mm), then the running line v = mps_per_duty x (duty - deadband) from held duties (kick, 0.4 s to settle,
    `hold_s` of lidar slope each).  research/dynamics_wl.md 9 M4."""
    bad = _need_pwm(robot)
    if bad:
        return bad
    if isinstance(speeds, str):
        import json
        speeds = json.loads(speeds)
    dm, c = robot.duty_model, robot.p["drive"]["duty"]
    f0 = _still_front(robot, stop)
    if f0 is None or not (1400 <= f0 <= 2600):
        return result(False, "front wall %s mm away -- put it 1.5-2.5 m straight ahead" % (None if f0 is None else round(f0)))
    vbat = robot.battery_v() or float(c.get("v_ref", 8.0))
    norm = vbat / float(c.get("v_ref", 8.0))                  # duty at the pin x this = duty at v_ref
    d, brk = float(start), None
    while d < 0.4 and brk is None:
        # once it breaks free it lunges toward the running speed of that duty (stick-slip): two consecutive scans
        # 8 mm nearer stop it at once -- one scan is inside the lidar's noise
        if not robot.duty_only(d):
            return _stopped("the breakaway ramp")
        samp = []
        t_end = time.monotonic() + float(dwell_s)
        while time.monotonic() < t_end:
            if not _front_samples(robot, 0.0, min(t_end, time.monotonic() + 0.03), stop, samp):
                return _stopped("the breakaway ramp")
            if len(samp) >= 2 and all(f0 - s[1] > 8.0 for s in samp[-2:]):
                brk = d
                break
        d += float(step)
    robot.stop()
    if brk is None:
        return result(False, "no movement up to duty %.2f -- motor wired?  DIR/PWM pins?  main switch?" % d)
    if not _wait(robot, stop, 0.8):
        return _stopped("the pause")
    rows = []
    kick = max(brk + 5 / 255.0, float(c["kick"]))
    for v in speeds:
        f_start = _still_front(robot, stop)
        if f_start is None or f_start < 700.0:
            break
        duty = dm.ff(float(v), vbat)
        if not robot.duty_only(kick) or not _wait(robot, stop, float(c.get("kick_ms", 60)) / 1000.0):
            robot.stop()
            return _stopped("a kick")
        if not robot.duty_only(duty) or not _wait(robot, stop, 0.4):
            robot.stop()
            return _stopped("a hold")
        samp = []
        t_end = time.monotonic() + float(hold_s)
        ok = _front_samples(robot, time.monotonic(), t_end, stop, samp)
        robot.stop()
        if not ok:
            return _stopped("a hold")
        if samp and samp[-1][1] < 450.0:
            samp = [s for s in samp if s[1] >= 450.0]
        vm = -_slope(samp) / 1000.0
        if vm == vm:
            rows.append((duty * norm, vm))
        if not _wait(robot, stop, 0.6):
            return _stopped("the pause")
    data = dict(breakaway=round(brk * norm, 4), vbat=vbat, holds=[[round(a, 4), round(b, 3)] for a, b in rows])
    moving = [(a, b) for a, b in rows if b > 0.03]
    if len(moving) < 2:
        return result(False, "breakaway %.3f (%.0f/255); too few moving holds for the running line (room? speeds=)"
                      % (brk * norm, brk * norm * 255), data, {"drive.duty.breakaway": round(brk * norm, 4)})
    A = np.array([[a, 1.0] for a, _b in moving])
    k, b0 = np.linalg.lstsq(A, np.array([b for _a, b in moving]), rcond=None)[0]
    if k <= 0:
        return result(False, "the speed did not grow with the duty -- wheels slipping?  lidar front blocked?", data)
    dead = -b0 / k
    data.update(mps_per_duty=round(float(k), 2), deadband=round(float(dead), 4))
    sug = {"drive.duty.breakaway": round(brk * norm, 4), "drive.duty.deadband": round(float(max(0.0, dead)), 4),
           "drive.duty.mps_per_duty": round(float(k), 2), "drive.duty.kick": round(brk * norm + 5 / 255.0, 4)}
    return result(0 < dead < brk * norm, "breakaway %.3f (%.0f/255), running line %.2f m/s per duty from %.3f "
                  "(%.1f/255), at %.2f V" % (brk * norm, brk * norm * 255, k, dead, dead * 255, vbat), data, sug)


def coast(robot, speeds=(0.15, 0.25, 0.35), run_s: float = 1.3, stop=None, **_):
    """[DAY1] the braked stop: from each speed, the distance the car still travels after the stop command -- the
    front wall's lidar LINE over the steady part of the run (every scan from 0.5 s on, 5-degree median) evaluated at
    the command, against the distance once the scans agree.  Fits d = v^2 / (2 a) + v t0: t0 soaks up what is
    proportional to v (where the front ray sits in the revolution, the command's latency), so a timing offset does
    not read as braking.  a -> drive.stop_decel, which the park's step mode stops early by.  dynamics_wl.md 9 M5."""
    bad = _need_pwm(robot)
    if bad:
        return bad
    if isinstance(speeds, str):
        import json
        speeds = json.loads(speeds)
    rows = []
    for v in speeds:
        f_start = _still_front(robot, stop)
        if f_start is None or f_start < 900.0:
            break
        samp = []
        if not robot.drive(float(v), 0.0):
            return _stopped("a run")
        t0 = time.monotonic()
        if not _front_samples(robot, t0 + 0.5, t0 + float(run_s), stop, samp):
            return _stopped("a run")
        t_stop = time.monotonic()
        robot.stop()
        if not _wait(robot, stop, 0.3):
            return _stopped("the stop")
        f_end = _still_front(robot, stop, n=4, tol=4.0)
        if f_end is None or len(samp) < 4:
            continue
        sl = _slope(samp)
        f_at = float(np.mean([s[1] for s in samp])) + sl * (t_stop - float(np.mean([s[0] for s in samp])))
        rows.append((-sl / 1000.0, f_at - f_end))
        if not _wait(robot, stop, 0.4):
            return _stopped("the pause")
    data = dict(runs=[[round(a, 3), round(b, 1)] for a, b in rows])
    good = [(a, b) for a, b in rows if a > 0.05]
    if not good:
        return result(False, "no usable run (room ahead >= 0.9 m?  did the car move?)", data)
    a, t_0 = None, 0.0
    if len(good) >= 3:
        A = np.array([[v * v / 2.0, v] for v, _d in good])
        (inv_a, t_0), *_r = np.linalg.lstsq(A, np.array([d / 1000.0 for _v, d in good]), rcond=None)
        if inv_a > 0 and 0.0 <= t_0 < 0.15:              # a ray is never younger than its stamp, a command
            a = 1.0 / inv_a                                #   never acts early: t0 < 0 is noise -> v^2 only
    if a is None:                                            # two runs, or a fit that makes no sense: v^2 only
        t_0 = 0.0
        den = 2.0 * sum(v * v * max(d, 0.1) / 1000.0 for v, d in good)
        a = sum(v ** 4 for v, _d in good) / den
    data.update(stop_decel=round(float(a), 2), t0_s=round(float(t_0), 3))
    return result(0.3 < a < 20.0, "braked stop: " + ", ".join("%.2f m/s -> %.0f mm" % r for r in good)
                  + " -> %.2f m/s^2 (+ %.0f ms x v)" % (a, t_0 * 1000), data, {"drive.stop_decel": round(float(a), 2)})


def step_table(robot, repeats: int = 3, stop=None, **_):
    """[DAY1] the step mode's kicks: each row of params step.table fired `repeats` times from rest, its travel read
    on the lidar once the car is still.  Suggests the table with the measured mean travel (the duty and time stay).
    research/dynamics_wl.md 9 M6."""
    bad = _need_pwm(robot)
    if bad:
        return bad
    rows_in = robot.p.get("step", {}).get("table", [])
    out, data = [], []
    for mm, duty, ms in rows_in:
        got = []
        for _ in range(int(repeats)):
            f0 = _still_front(robot, stop)
            if f0 is None or f0 < 500.0:
                break
            if robot.pulse(1, 0.0, float(mm)) <= 0.0:
                return _stopped("a pulse")
            while robot.pulse_busy():
                if _halted(robot, stop):
                    robot.stop()
                    return _stopped("a pulse")
                time.sleep(0.005)
            if not _wait(robot, stop, 0.25):
                return _stopped("the settle")
            f1 = _still_front(robot, stop)
            if f1 is not None:
                got.append(f0 - f1)
        if got:
            m = float(np.mean(got))
            data.append(dict(duty=duty, ms=ms, want_mm=mm, got_mm=[round(g, 1) for g in got], mean=round(m, 1),
                             sd=round(float(np.std(got)), 1)))
            out.append([round(m, 1), duty, ms])
    if not out:
        return result(False, "no pulse measured (room ahead >= 0.5 m?  duty above breakaway?)", dict(rows=data))
    moved = all(r[0] > 2.0 for r in out)
    return result(moved, "; ".join("%.3f x %d ms -> %.1f mm (sd %.1f)" % (d["duty"], d["ms"], d["mean"], d["sd"])
                                   for d in data), dict(rows=data), {"step.table": sorted(out)} if moved else {})


MOVES.update({"duty_sweep", "coast", "step_table"})
HOW["duty_sweep"] = ("WLtoys drive.  Car on the mat pointing straight at a wall 1.5-2.5 m ahead.  It creeps until it "
                     "moves (breakaway), then drives three short held duties: the duty -> speed line.")
HOW["coast"] = ("WLtoys drive.  Car on the mat pointing straight at a wall 2-2.5 m ahead.  Three short runs "
                "(0.15 / 0.25 / 0.35 m/s), each braked: how far it still rolls.")
HOW["step_table"] = ("WLtoys drive.  Car on the mat pointing at a wall >= 1 m ahead.  Fires each park kick three times "
                     "and measures its travel.")


ALL = dict(auto_calib=auto_calib, board=board, imu_bias=imu_bias, imu_direction=imu_direction, servo_sweep=servo_sweep,
           motor_pulse=motor_pulse, lidar_health=lidar_health, lidar_direction=lidar_direction, camera=camera,
           speed_run=speed_run, turn_radius=turn_radius, cam_pitch=cam_pitch, duty_sweep=duty_sweep, coast=coast,
           step_table=step_table, depth=depth)

def still_check(robot, **kw):
    """bluewave/tests_cal.still_check (imported here, not at the top: tests_cal imports this module's helpers)."""
    from .tests_cal import still_check as _still
    return _still(robot, **kw)


ALL["still_check"] = still_check


def lidar_tilt(robot, **kw):
    """bluewave/tests_cal.lidar_tilt (BRAIN_SPEC 12.6)."""
    from .tests_cal import lidar_tilt as _tilt
    return _tilt(robot, **kw)


ALL["lidar_tilt"] = lidar_tilt
HOW["lidar_tilt"] = ("Car anywhere inside the field, no signs, still.  The lidar's pose from the whole-field search, then "
                     "per sector how far it still sees the walls: a plane tilted up passes over far walls (1.4 deg at "
                     "50 mm loses everything beyond 2 m).  It must PASS; otherwise shim the lidar mount.")
HOW["still_check"] = ("Nothing moves.  Hands off the car: the gyro, the camera image and two lidar scans must hold still "
                      "for 1 s (field_check needs it: a moving car scored 1.2-1.5 on a field that scores 0.009).")
HOW["turn_radius"] = ("0.6 m clear around the WLtoys car (1 m for the stock A1), static things around it.  Full lock 90 "
                      "deg left, then 90 deg right; two lidar scans each way give the radius the car REALLY turns and "
                      "the effective wheelbase (chassis.wheelbase_m).  Without a lidar: one full circle, command-based.")
