"""Run a program on the simulator, headless, and score it: localisation error against the truth, contacts, laps.

    py -3 tools/sim_run.py wro --seed 3                   # random WRO sign layout (1-2 signs per straight)
    py -3 tools/sim_run.py wro --seed 3 --cw              # start facing -x = clockwise round
    py -3 tools/sim_run.py wro --scene my_scene.json      # {"start": [x, y, deg], "pillars": [[x, y, "red"], ...]}
    py -3 tools/sim_run.py wro --seeds 1-8                # a batch, one line each
    py -3 tools/sim_run.py wro_next --wltoys --seed 1     # the WLtoys build: profiles/wltoys.json (car, sensors, drive)
                                                          # on the duty-driven plant (mock.WL_PLANT)
    py -3 tools/sim_run.py wro_next --profile stock_a1    # a profile's patch (paramstore.profiles()): the plant's
                                                          # wheelbase, body and camera follow it -- the mat's A1 (G12)
    py -3 tools/sim_run.py wro_next --depth --pitch-error 1   # the HP60C depth on (walls, pillars, start search), A/B
    py -3 tools/sim_run.py wro_next --depth walls --depth-truth '{"scale": 0.14}'   # one use; a worse sensor
    py -3 tools/sim_run.py wro_next --wltoys --set '{"drive.speed_loop.on": 0}'     # A/B: one robot param changed
                                                          # (dotted keys, applied after the profile)
    py -3 tools/sim_run.py wro_next --wltoys --plant '{"servo_dps": 250}'   # the steering servo's slew (0.12 s to
                                                          # lock); the default servo is instant
    py -3 tools/sim_run.py BEST_OPEN --profile wltoys_bw2 --mat1001 --open --cw   # the plant FITTED to the mat
                                                          # (tools/fit_plant.py -> profiles/plant_mat1001.json: the
                                                          # drive, the brake, steer bias +4, the understeer, the mat's
                                                          # steer.max_deg 26 / camera latency); --plant-file F = another fit
    py -3 tools/sim_run.py wro_next --start-any --params '{"start_any": 1, "laps": 1, "park": 0}' --seeds 1-4
                                                          # BRAIN4_SPEC 5.6: the start anywhere on the corridor ring
    py -3 tools/sim_run.py survey --start-any --lot --params '{"then": "race", "then_laps": 1}'
    py -3 tools/sim_run.py wro_next --runfile runs/sim   # + a run file in the Runner's format (the analyzer's input)
    py -3 tools/sim_run.py wro_next --fault '{"kind": "pitch_jump", "t": 20, "in_state": "GO", "deg": 2}'
                                                          # fault injection mid-run: pitch_jump, wall_stuck,
                                                          # lidar_dropout, battery_sag, camera_freeze (inject_faults);
                                                          # a list = several

WLtoys results add the plant's dynamics: ay_max (m/s^2 at the CG; the design cap is 0.35 SSF g = 1.99, the tip 5.68)
and where it happened (ay_at: x, y, v mm/s, wheel deg), and tight_fast_s -- seconds on a radius under 250 mm above
0.30 m/s, dynamics_wl.md S-1's tight-arc rule.

Start anywhere (--start-any, BRAIN4_SPEC 5.6): the start drawn uniformly over the scene's corridor ring, the car's
footprint >= 200 mm from every wall and >= 150 mm from every sign and limitation, any heading, random.Random(7000 +
seed, + 500 clockwise); the program's start_any is NOT implied (pass --params).  The result gains `fix` (the committed
LOCATE fix against the truth: err_mm / err_deg, twin-aware when the frame did not matter, wrong_fix = the car moved on a
pose > 150 mm / 10 deg off) and, for the survey program, `survey` (seat recall / precision, colour errors, lot error,
direction).  CLEAN there = 0 contacts and either the laps done or an honest refusal -- never a wrong fix.

Every result has `touch_s`: the seconds the body touched something while told to move (a push: the mat's M4 pushed 85 s).
--fault (inject_faults) adds `faults`: what fired when, where, and the contacts after it.

The sim is not the judge (team rule): this finds bugs and regressions before the mat, nothing more.
"""
from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import math
import os
import random
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
os.environ["BW_MOCK"] = "1"

from bluewave import field as F        # noqa: E402
from bluewave import params as P       # noqa: E402

REFUSALS = ("no_fix", "boxed_in", "no_join", "start_mismatch", "not_standard")


def random_scene(seed: int, cw: bool = False, lot: bool = False, lot_start: bool = False, car: dict | None = None) -> dict:
    rng = random.Random(seed)
    pillars = []
    for k in range(4):
        n = rng.choice([1, 1, 2, 2]) if k else rng.choice([0, 1, 1, 2])
        alongs = rng.sample([0, 1, 2], n)
        for ai in alongs:
            # [RULE] lot straight: inner seats only -- with a lot START too: --lot-start put signs 400 mm off the wall
            # right at the lot's exit, a scene the rules never set (the exit then met the sign, 2026-09-23)
            ci = 1 if ((lot or lot_start) and k == 0) else rng.randrange(2)
            x, y = F.rot(k, F.SEAT_ALONG[ai], -F.HALF + F.SEAT_ACROSS[ci])
            pillars.append([round(x), round(y), rng.choice(["red", "green"])])
    sx = rng.uniform(-350.0, -150.0) * (-1 if cw else 1)
    sy = rng.uniform(-1150.0, -850.0)
    # the judges never place the car against a sign: nothing within 550 mm ahead of it in the start straight
    fwd = -1 if cw else 1
    pillars = [p for p in pillars if not (p[1] < -500 and 0 < (p[0] - sx) * fwd < 550 and abs(p[1] - sy) < 300)]
    uniq = {(p[0], p[1]): p for p in pillars}
    sdeg = (180.0 if cw else 0.0) + rng.uniform(-6.0, 6.0)
    out = {"start": [round(sx), round(sy), round(sdeg, 1)], "pillars": list(uniq.values()), "true_speed": 0.93}
    if lot or lot_start:
        from bluewave import park as K
        lot_len = float(car.get("lot_len_mm", K.LOT_LEN)) if car else K.LOT_LEN
        x0 = rng.uniform(-480.0, 480.0 - lot_len - 20.0)                # interior x0 .. x0 + lot_len (field x)
        out["lot"] = [round(x0 - 10.0, 1), round(x0 + lot_len + 10.0, 1)]
        if lot_start:
            d = -1 if cw else 1
            x_travel0 = x0 if d > 0 else x0 + lot_len
            kc = K.Car(rear=car["rear_mm"], front=car["front_mm"], half_w=car["half_w_mm"]) if car else K.Car()
            px, py, pth = K.to_field(K.parked_pose(kc, lot_len), x_travel0, d)
            out["start"] = [round(px, 1), round(py, 1), round(math.degrees(pth) + rng.uniform(-1.0, 1.0), 1)]
        else:
            # the middle zone of the lot half of the straight, clear of the lot [RULE p.16]
            out["start"][1] = round(rng.uniform(-1050.0, -950.0))
    return out


def scene_island(scene: dict):
    """(left, right, bottom, top) of a scene's island, as mock.World builds it."""
    if scene.get("corridors"):
        return F.island_of(scene["corridors"])
    return tuple(float(v) for v in scene.get("island", (-F.ISL, F.ISL, -F.ISL, F.ISL)))


def random_start(seed: int, scene: dict, body, cw: bool = False, wall_mm: float = 200.0, sign_mm: float = 150.0):
    """BRAIN4_SPEC 5.6 --start-any: a pose drawn uniformly over the corridor ring with the footprint (rear, front,
    half_w) >= wall_mm from the outer walls and the island and >= sign_mm from every sign (a 50 x 50 box) and
    limitation, any heading.  Deterministic per (seed, cw).  Returns [x, y, deg]."""
    rng = random.Random(7000 + seed + (500 if cw else 0))
    rear, front, half = (float(v) for v in body)
    l, r, b, t = scene_island(scene)
    boxes = [(float(px), float(py), 25.0, 25.0) for px, py, _c in scene.get("pillars", [])]
    for xa in scene.get("lot") or []:
        boxes.append((float(xa), -1500.0 + 100.0, 10.0, 100.0))
    ea = [(a, bb) for a in range(int(-rear), int(front) + 1, 10) for bb in (-half, half)]
    ea += [(a, bb) for a in (-rear, front) for bb in range(int(-half), int(half) + 1, 10)]
    for _ in range(200000):
        x, y = rng.uniform(-F.HALF, F.HALF), rng.uniform(-F.HALF, F.HALF)
        th = rng.uniform(-math.pi, math.pi)
        c, s = math.cos(th), math.sin(th)
        ok = True
        for a, bb in ea:
            px, py = x + c * a - s * bb, y + s * a + c * bb
            if abs(px) > F.HALF - wall_mm or abs(py) > F.HALF - wall_mm:
                ok = False
                break
            dx = max(l - px, px - r, 0.0)
            dy = max(b - py, py - t, 0.0)
            if math.hypot(dx, dy) < wall_mm:
                ok = False
                break
            for bx, by, hx, hy in boxes:
                if math.hypot(max(abs(px - bx) - hx, 0.0), max(abs(py - by) - hy, 0.0)) < sign_mm:
                    ok = False
                    break
            if not ok:
                break
        if ok:
            return [round(x), round(y), round(math.degrees(th), 1)]
    raise RuntimeError("no start pose found")


NEW_BODY = {"rear_mm": 34.2, "front_mm": 262.6, "half_w_mm": 80.5, "lot_len_mm": 445.2}   # design/LAYOUT.py
# the BLUE WAVE body's sensors (3d/bluewave_body/design/LAYOUT.py + layout_report.json, rear-axle frame = floor + 70.31):
# camera on the keyed 20 deg wedge, lens 241.8 ahead / 120 high; LD19 inverted at the nose, axis 232.3 ahead,
# scan plane 50 mm, hidden rear sector 130.7-229.35 deg
NEW_BODY_SENSORS = {"camera": {"x_mm": 241.8, "h_mm": 120.0, "pitch_deg": 20.0, "roll_deg": 0.0},
                    "lidar": {"loc": 1, "pos_mm": [232.3, 0.0], "block_deg": [[130.7, 229.35]],
                              "scan_lag_s": 0.0}}           # the simulated revolution is instantaneous
CAR = None
WLTOYS = None                          # the profile's patch when its drive is the duty plant (--wltoys, a gpio_pwm profile)
PROFILE = None                         # --profile: any other profile's patch (the stock A1: the mat's wheelbase, camera)
PROFILE_NAME = None
PLANT = {}                             # --plant: overrides of the simulated WLtoys truth (mock.WL_PLANT)
PLANT_SCENE = {}                       # --mat1001 / --plant-file: scene defaults of the fit (the camera's latency)
DEPTH = None                           # --depth: the uses switched on ("walls", "pillars", "start"); None = depth off
DEPTH_TRUTH = {}                       # --depth-truth: overrides of the simulated sensor (mock.DEPTH_TRUTH)
SET = {}                               # --set: robot params (dotted keys) over the defaults and the profile, for A/B
RUNFILE = None                         # --runfile: the directory the Runner-format run files go to
FAULTS = []                            # --fault: what goes wrong mid-run (inject_faults)
FAULT_KINDS = ("pitch_jump", "wall_stuck", "lidar_dropout", "battery_sag", "camera_freeze")


def wltoys_profile() -> dict:
    with open(os.path.join(ROOT, "profiles", "wltoys.json"), encoding="utf-8") as f:
        return json.load(f)["patch"]


def profile_patch(name: str) -> dict:
    from bluewave import paramstore as PS
    profs = PS.profiles()
    if name not in profs:
        sys.exit("--profile: no profile %r (%s)" % (name, ", ".join(sorted(profs))))
    return dict(profs[name]["patch"])


def obstacles(seed: int, n: int) -> list:
    """`n` 50 x 50 obstacles NO seat explains (BRAIN_SPEC 11.1 --obstacles): on the planned 500 mm lane of straights
    E, N, W (S holds the start and the lot), midway between two seat columns (along +-250 mm, across 500 +- 20): >= 262
    mm from every seat, so no camera vote snaps to a seat (seat_snap 200) and no lidar sign to one (seat_snap_mm 120),
    >= 480 mm from the walls.  Rendered red (mock.RenderCamera knows red, green, magenta).  Deterministic per seed."""
    rng = random.Random(5000 + seed)
    slots = [(k, a) for k in (1, 2, 3) for a in (-250.0, 250.0)]
    out = []
    for k, a in rng.sample(slots, min(n, len(slots))):
        x, y = F.rot(k, a, -F.HALF + 500.0 + rng.uniform(-20.0, 20.0))
        out.append([round(x), round(y), "red"])
    return out


def random_open_scene(seed: int, cw: bool = False) -> dict:
    """Open Challenge: every corridor 600 or 1000 mm wide [RULE], no signs, the car anywhere in the start straight's
    corridor facing the round's direction."""
    rng = random.Random(1000 + seed)
    cor = [rng.choice([600, 1000]) for _ in range(4)]            # S, E, N, W
    cS = cor[0]
    sx = rng.uniform(-350.0, 150.0) * (-1 if cw else 1)
    sy = -1500.0 + cS * rng.uniform(0.35, 0.65)
    sdeg = (180.0 if cw else 0.0) + rng.uniform(-5.0, 5.0)
    return {"start": [round(sx), round(sy), round(sdeg, 1)], "pillars": [], "corridors": cor, "true_speed": 0.93}


def robot_params(scene: dict):
    """(robot params, scene) for a run: the DEFAULTS, then the profile (--wltoys / --profile) with the simulated body
    and plant, the BLUE WAVE body (--new-body), the depth, --set."""
    p = copy.deepcopy(P.DEFAULTS)
    if WLTOYS:
        from bluewave import paramstore as PS
        PS.patch(p, WLTOYS)                                  # car, sensors, steering, drive, speed estimation
        p["lidar"]["scan_lag_s"] = 0.0                       # the simulated revolution is instantaneous
        scene = dict(PLANT_SCENE, **dict(scene, body=list(p["mock"]["body"]), plant=dict(p["mock"]["plant"], **PLANT)))
    elif PROFILE is not None:
        from bluewave import paramstore as PS
        PS.patch(p, PROFILE)
        c = p["car"]
        scene = dict(scene, body=[-float(c["rear_mm"]), float(c["front_mm"]), float(c["half_w_mm"])])
        # the simulated camera's frame age = the latency the profile believes (stock_a1: 0.09 s, the mat's fit): the
        # mock's default 0.12 s left a 30 ms mismatch, a 0.7 deg heading bias on every wall point in a corner
        # (0.43 rad/s at R 606); a scene's own latency_s (a latency-error test) still wins
        scene.setdefault("latency_s", float(p["camera"].get("latency_s", 0.12)))
    elif CAR:
        p["car"] = dict(CAR)
        scene = dict(scene, body=[-CAR["rear_mm"], CAR["front_mm"], CAR["half_w_mm"]])
        for sec, vals in NEW_BODY_SENSORS.items():
            p[sec] = dict(p[sec], **vals)
    if DEPTH is not None:
        p["depth"] = dict(p["depth"], on=1, **{k: int(k in DEPTH) for k in ("walls", "pillars", "start")})
        scene = dict(scene, depth_truth=dict(DEPTH_TRUTH))
    if SET:
        from bluewave import paramstore as PS
        PS.patch(p, SET)
    return p, scene


def body_of(p: dict, scene: dict):
    """The simulated footprint (rear, front, half_w) the scene will use."""
    if scene.get("body"):
        b = scene["body"]
        return -float(b[0]), float(b[1]), float(b[2])
    from bluewave.mock import World
    return -World.BODY[0], World.BODY[1], World.BODY[2]


def _twin_err(pose, truth, twins: bool):
    """(mm, deg) of pose vs the truth; with twins, the smallest over the field's 4 rotations of the pose."""
    best = None
    for k in (range(4) if twins else (0,)):
        q = F.rot_pose((pose[0], pose[1], math.radians(pose[2])), k)
        e = (math.hypot(q[0] - truth[0], q[1] - truth[1]), abs(((math.degrees(q[2]) - truth[2] + 180) % 360) - 180))
        if best is None or e[0] + 10 * e[1] < best[0] + 10 * best[1]:
            best = e
    return best


def fix_score(logs: list, free: bool = False) -> dict | None:
    """The committed LOCATE fix (the last confident `fix` event, else the last one) against the truth logged with it:
    twin-aware when the frame did not matter to the run ("any") or nothing in the scene breaks the field's symmetry
    (`free`: no lot, no start mark, no survey)."""
    fx = [d for d in logs if d.get("ev") == "fix"]
    if not fx:
        return None
    conf = [d for d in fx if d.get("confident")]
    d = conf[-1] if conf else fx[-1]
    tw = free or d.get("frame_by") in ("any", "none")
    e = _twin_err((d["x"], d["y"], d["th"]), d["truth"], tw) if d.get("truth") else (None, None)
    moved = any(q.get("ev") == "locate" and q.get("phase") == "commit" for q in logs)
    wrong = bool(conf) and moved and e[0] is not None and (e[0] > 150.0 or e[1] > 10.0)
    return dict(p=d.get("p"), confident=bool(d.get("confident")), frame_by=d.get("frame_by"),
                err_mm=None if e[0] is None else round(e[0]), err_deg=None if e[1] is None else round(e[1], 1),
                wrong_fix=wrong, fixes=len(fx), looks=sum(1 for q in logs if q.get("ev") == "look"
                                                           and q.get("phase") == "move"),
                why=d.get("why"), cols=d.get("cols"), cov=d.get("cov_deg"))


def survey_score(logs: list, scene: dict, direction_true: int | None) -> dict | None:
    """The saved survey file against the scene: seat recall / precision (a sign within 120 mm), colour errors, the
    lot's error, the direction -- twin-aware when the survey's frame was not resolved by the lot / a mark / a survey."""
    ev = next((d for d in reversed(logs) if d.get("ev") == "survey_saved"), None)
    if ev is None or not ev.get("path"):
        return None
    try:
        with open(ev["path"], encoding="utf-8") as f:
            s = json.load(f)
    except (OSError, ValueError):
        return None
    from bluewave import survey_map as SVM
    truth = [(float(x), float(y), c) for x, y, c in scene.get("pillars", []) if [x, y, c] not in
             scene.get("obstacles", [])]
    fb = (s.get("frame") or {}).get("by")
    best = None
    for k in (range(4) if fb not in ("lot", "expect", "survey") else (0,)):
        sk = SVM.rotate(s, k)
        found = [(q["x"], q["y"], q.get("colour")) for q in sk["seats"] if q.get("exists") is True]
        hit = cerr = cmiss = 0
        for tx, ty, tc in truth:
            m = [f for f in found if math.hypot(f[0] - tx, f[1] - ty) < 120.0]
            if m:
                hit += 1
                if m[0][2] is None:
                    cmiss += 1                   # the sign exists, its colour was not seen (the race avoids it)
                else:
                    cerr += int(m[0][2] != tc)
        tp = sum(1 for f in found if any(math.hypot(f[0] - tx, f[1] - ty) < 120.0 for tx, ty, _c in truth))
        sc = (hit, -cerr, tp, k, len(found), cmiss)
        if best is None or sc[:5] > best[:5]:
            best = sc
    hit, ncerr, tp, k, nf, cmiss = best
    lot_err = None
    lt = SVM.rotate(s, k).get("lot")
    if scene.get("lot") and lt and int(lt.get("section", 0)) == 0:
        xa, xb = sorted(scene["lot"])
        lot_err = round(min(abs(lt["x0"] - (xa + 10.0)), 99999.0))
    d_ok = None
    if direction_true:
        d_ok = (s.get("direction") == "ccw") == (direction_true > 0)
    return dict(seat_recall=round(hit / max(1, len(truth)), 2), seat_precision=round(tp / max(1, nf), 2),
                colour_errors=-ncerr, colour_missing=cmiss, lot_err_mm=lot_err, dir_ok=d_ok, frame_by=fb, twin=k,
                complete=s.get("complete"), path=ev["path"])


class RunFile:
    """--runfile: the Runner's run-file format (agent.Runner): meta, clock (first after meta, then every 10 s), the
    program's log (Unix t), tel at 20 Hz (robot.snapshot(), the truth in its `sim` block) and the board's IMU samples
    every 0.2 s (bluewave/blackbox.py, when present)."""

    def __init__(self, d: str, program: str, params: dict, robot, scene: dict, seed, cw):
        os.makedirs(d, exist_ok=True)
        now = time.time()
        self.run_id = "%s.%03d-sim-%s" % (time.strftime("%Y%m%d-%H%M%S", time.localtime(now)), int(now * 1000) % 1000,
                                          program)
        self.path = os.path.join(d, self.run_id + ".jsonl")
        self.f = open(self.path, "w", encoding="utf-8", newline="\n")
        self.lock = threading.Lock()
        self.robot = robot
        robot.run_id = self.run_id
        self.write(dict(t=time.time(), meta=dict(program=program, params=params, robot_params=robot.p, drives=True,
                                                  manual_ok=False, by="sim", wait=None,
                                                  sim=dict(scene=scene, seed=seed, cw=cw, faults=FAULTS))))
        self.clock()
        try:
            from bluewave import blackbox
            self.bb = blackbox
        except Exception:
            self.bb = None
        self.imu_last = time.monotonic()
        self.run = True
        threading.Thread(target=self._tel, daemon=True).start()

    def write(self, rec: dict):
        line = json.dumps(rec, default=str) + "\n"
        with self.lock:
            if self.f:
                self.f.write(line)

    def clock(self):
        self.write(dict(t=time.time(), clock=dict(mono=round(time.monotonic(), 4), run_id=self.run_id,
                                                  agent_started=None)))

    def log(self, x):
        self.write(dict(t=time.time(), log=x))

    def _tel(self):
        k = 0
        while self.run:
            try:
                self.write(dict(t=time.time(), tel=self.robot.snapshot()))
            except Exception:
                pass
            k += 1
            if k % 4 == 0 and self.bb is not None:
                try:
                    rec, self.imu_last = self.bb.take(self.robot, self.imu_last)
                    if rec:
                        self.write(dict(t=time.time(), imu=rec))
                except Exception:
                    pass
            if k % 200 == 0:
                self.clock()
            time.sleep(0.05)

    def close(self):
        self.run = False
        time.sleep(0.06)
        self.log("-- program ended --")
        with self.lock:
            f, self.f = self.f, None
        if f:
            f.close()


def _outer_gap(w) -> float:
    """mm from the car's footprint to the nearest outer wall (the truth)."""
    from bluewave.mock import OUTER
    cs = w.corners(w.x, w.y, w.th)
    return min(OUTER - max(abs(p[0]) for p in cs), OUTER - max(abs(p[1]) for p in cs))


def _wall_stuck(w, yaw_deg: float, gap_mm: float = 1.0) -> dict:
    """--fault wall_stuck: the car put against the OUTER wall nearest its footprint -- its heading turned toward that
    wall by <= yaw_deg, then moved along the wall's normal until the footprint is gap_mm from it.  The program's pose
    does not follow (the mat's M4: a pose the program does not know and a wall in front of the nose).  A sign or a
    limitation in the way: the car stops short of it, against that instead."""
    from bluewave.mock import OUTER
    normals = {"E": 0.0, "N": math.pi / 2, "W": math.pi, "S": -math.pi / 2}
    with w.lock:
        x0, y0, th0 = w.x, w.y, w.th
        cs = w.corners(x0, y0, th0)
        gaps = {"E": OUTER - max(p[0] for p in cs), "N": OUTER - max(p[1] for p in cs),
                "W": OUTER + min(p[0] for p in cs), "S": OUTER + min(p[1] for p in cs)}
        wall = min(gaps, key=gaps.get)
        n = normals[wall]
        dth = ((n - th0 + math.pi) % (2 * math.pi)) - math.pi
        yaw = math.radians(yaw_deg)
        th = ((th0 + max(-yaw, min(yaw, dth))) + math.pi) % (2 * math.pi) - math.pi
        cs = w.corners(x0, y0, th)
        ux, uy = math.cos(n), math.sin(n)
        reach = min(OUTER - (px * ux + py * uy) for px, py in cs) - gap_mm   # along the normal to gap_mm
        d = reach
        while d > 0.0 and w.hit(x0 + ux * d, y0 + uy * d, th):
            d -= 10.0                                   # something (a sign, the island) in the way: short of it
        d = max(0.0, d)
        w.x, w.y, w.th, w.v, w.touching = x0 + ux * d, y0 + uy * d, th, 0.0, ""
        w.hist.append((time.monotonic(), w.x, w.y, w.th))
    turned = ((th - th0 + math.pi) % (2 * math.pi)) - math.pi
    return dict(wall=wall, moved_mm=round(d), yaw_deg=round(math.degrees(turned), 1),
                to=[round(w.x), round(w.y), round(math.degrees(w.th), 1)])


def inject_faults(robot, faults: list, t0: float, logs: list, log, stop, out: list):
    """--fault: each {"kind": ..., "t": seconds after the program started, "in_state": "GO"} fires at the first moment
    >= t that the program's last logged state is in_state ("" or absent = any state):
      pitch_jump     {"deg": 2.0}                 the camera's TRUE pitch += deg (a bumped mount); the program keeps its own
      wall_stuck     {"yaw_deg": 20, "near_mm": 0} the car against the nearest outer wall, nose turned into it
                                                  (_wall_stuck); near_mm > 0: only once its footprint runs that near one
      lidar_dropout  {"dur": 1.0}                 no new scan for dur s (the last one ages: a USB hiccup)
      battery_sag    {"to_v": 6.9, "ramp_s": 20}  the pack (the WLtoys plant's truth AND the board's reading) ramps from
                                                  its voltage to to_v (the mat: 8.2 -> 7.05 V in 1.5 h; the stock A1's
                                                  encoder loop holds its speed, so only the reading changes there)
      camera_freeze  {"dur": 0}                   the camera grabber stops: read() keeps returning the last frame and its
                                                  stamp (what camera.Camera does when cap.read() fails or hangs); dur > 0
                                                  = frames again after dur s
    Each fires once, is logged as ev "sim_fault" (the run file too) and lands in `out` for the result."""
    sim, w = robot.sim, robot.sim.world
    pending, sags, thaws = [dict(f) for f in faults], [], []
    while not stop.is_set() and (pending or sags or thaws):
        now = time.monotonic()
        rel = now - t0
        state = next((d.get("state") for d in reversed(logs) if d.get("state")), None)
        for f in list(pending):
            want = f.get("in_state") or ""
            if rel < float(f.get("t", 0.0)) or (want and state != want):
                continue
            if f["kind"] == "wall_stuck" and f.get("near_mm") and _outer_gap(w) > float(f["near_mm"]):
                continue                                # only where the car already runs near an outer wall
            pending.remove(f)
            k = f["kind"]
            rec = dict(kind=k, at_s=round(rel, 2), at_state=state, at_truth=[round(w.x), round(w.y),
                                                                            round(math.degrees(w.th), 1)])
            if k == "pitch_jump":
                cam = dict(sim.camera.cam)
                cam["pitch_deg"] = float(cam["pitch_deg"]) + float(f.get("deg", 2.0))
                sim.camera.set_camera(cam)
                rec.update(deg=float(f.get("deg", 2.0)), pitch_true=round(cam["pitch_deg"], 2))
            elif k == "wall_stuck":
                rec.update(_wall_stuck(w, float(f.get("yaw_deg", 20.0))))
            elif k == "lidar_dropout":
                sim.lidar.mute_until = now + float(f.get("dur", 1.0))
                rec.update(dur=float(f.get("dur", 1.0)))
            elif k == "battery_sag":
                sags.append((now, float(f.get("ramp_s", 20.0)), float(w.vbat), float(f.get("to_v", 6.9))))
                rec.update(from_v=round(float(w.vbat), 2), to_v=float(f.get("to_v", 6.9)),
                           ramp_s=float(f.get("ramp_s", 20.0)))
            elif k == "camera_freeze":
                sim.camera._run = False                  # the render loop ends: read() gives the last frame, old stamp
                dur = float(f.get("dur", 0.0) or 0.0)
                if dur > 0.0:
                    thaws.append(now + dur)
                rec.update(dur=dur or None)
            log(dict(ev="sim_fault", **rec))
            rec["contacts_before"] = w.contacts
            out.append(rec)
        for s in list(sags):
            ts, ramp, v0, v1 = s
            a = min(1.0, (now - ts) / max(ramp, 1e-3))
            v = v0 + (v1 - v0) * a
            w.vbat = v
            st = sim.board.state
            st.battery_mv.value, st.battery_mv.t = int(round(v * 1000)), now
            if a >= 1.0:
                sags.remove(s)
        for tw in list(thaws):
            if now >= tw:
                thaws.remove(tw)
                sim.camera._run = True
                threading.Thread(target=sim.camera._loop, daemon=True).start()
                log(dict(ev="sim_fault", kind="camera_thaw", at_s=round(now - t0, 2)))
        time.sleep(0.05)


def run_one(program: str, scene: dict, prog_params: dict, seconds: float, quiet: bool, seed=None, cw=None) -> dict:
    from bluewave.hw import Robot
    p, scene = robot_params(scene)
    p["mock"] = scene
    robot = Robot(p, mock=True)
    spec = importlib.util.spec_from_file_location(program, os.path.join(ROOT, "programs", program + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    w = robot.sim.world
    errs, logs = [], []
    errs_park = []                                          # the pose error while it parks (APPROACH / PARK / EXIT)
    t0 = time.monotonic()
    rf = RunFile(RUNFILE, program, prog_params, robot, scene, seed, cw) if RUNFILE else None
    # a start-anywhere run in a scene nothing breaks the symmetry of (no lot, no start mark, no survey prior) may commit
    # in any twin of the true frame: the walls cannot tell them apart and the signs are learnt in the run's own frame.
    # Its errors are then taken in that twin (twin_k: the rotation of the truth the confident fix matched)
    free = not scene.get("lot") and not prog_params.get("expect") and not prog_params.get("survey") and \
        not scene.get("corridors")
    twin = [0]

    def truth_k():
        x_, y_, t_ = F.rot_pose((w.x, w.y, w.th), twin[0]) if twin[0] else (w.x, w.y, w.th)
        return x_, y_, t_

    def log(d):
        if rf is not None:
            rf.log(d)
        d = dict(d)
        d["t"] = round(time.monotonic() - t0, 2)
        d["truth"] = [round(w.x), round(w.y), round(math.degrees(w.th), 1)]
        if free and d.get("ev") == "fix" and d.get("confident"):
            twin[0] = min(range(4), key=lambda k: math.hypot(*[a - b for a, b in zip(
                F.rot_pose((w.x, w.y, w.th), k)[:2], (d["x"], d["y"]))]))
        tx, ty, tth = truth_k()
        if "x" in d and d.get("state") in ("GO", "CREEP"):
            e = math.hypot(d["x"] - tx, d["y"] - ty)
            eh = abs(((d["th"] - math.degrees(tth) + 180) % 360) - 180)
            errs.append((e, eh))
            d["err"] = [round(e), round(eh, 1)]
        elif "x" in d and d.get("state") in ("APPROACH", "PARK", "EXIT"):
            errs_park.append(math.hypot(d["x"] - tx, d["y"] - ty))
        logs.append(d)
        if not quiet and ("ev" in d or len(logs) % 6 == 0):
            print(json.dumps(d, separators=(",", ":"))[:300], flush=True)

    stop = threading.Event()
    touches = []
    touch_s = [0.0]                                         # seconds in contact while told to move (a push: M4 85 s)

    v_err, v_true = [], []                                  # the speed estimate against the truth (WLtoys)
    # every pass of a sign beside the car and on which side (the truth): red = kept on the car's LEFT, green = on its
    # RIGHT [RULE].  A wrong-side pass costs points in WRO -- "CLEAN" used to count those runs (review 2026-09-25)
    obst_ = [tuple(o) for o in scene.get("obstacles") or []]
    signs_ = [q for q in scene.get("pillars") or [] if tuple(q) not in obst_]
    passes, cur_pass = [], {}

    def sign_sides():
        rear_, front_, half_ = -w.BODY[0], w.BODY[1], w.BODY[2]
        x_, y_, th_ = w.x, w.y, w.th
        c_, s_ = math.cos(th_), math.sin(th_)
        for i_, (px_, py_, col_) in enumerate(signs_):
            X_ = c_ * (px_ - x_) + s_ * (py_ - y_)
            Y_ = -s_ * (px_ - x_) + c_ * (py_ - y_)
            if -rear_ <= X_ <= front_ and abs(Y_) < 420.0:
                if i_ not in cur_pass:
                    cur_pass[i_] = ("L" if Y_ > 0 else "R", round(time.monotonic() - t0, 1))
            elif i_ in cur_pass:
                sd_, t1_ = cur_pass.pop(i_)
                passes.append((col_, sd_, (px_, py_), t1_))

    def watch_contacts():                                   # where and on what every new contact happened
        seen = 0
        while not stop.is_set():
            if signs_:
                sign_sides()
            if w.driver is not None and abs(w.v) > 50.0:
                v_true.append(abs(w.v) / 1000.0)
                v_err.append(abs(robot.v_odo() - w.v / 1000.0))
            if w.touching and abs(robot.v_cmd) > 0.03:
                touch_s[0] += 0.02
            if w.contacts > seen:
                seen = w.contacts
                last_state = next((d.get("state") or d.get("mode") for d in reversed(logs) if d.get("state") or d.get("mode")), None)
                touches.append(dict(t=round(time.monotonic() - t0, 1), what=w.touching, state=last_state,
                                    at=[round(w.x), round(w.y), round(math.degrees(w.th))]))
            time.sleep(0.02)

    threading.Thread(target=watch_contacts, daemon=True).start()
    fault_out = []
    if FAULTS:
        threading.Thread(target=inject_faults, args=(robot, FAULTS, t0, logs, log, stop, fault_out),
                         daemon=True).start()
    th = threading.Thread(target=mod.run, args=(robot, prog_params, log, stop), daemon=True)
    th.start()
    th.join(seconds)
    stop.set()
    th.join(5)
    if rf is not None:
        rf.close()
    end = next((d for d in reversed(logs) if d.get("ev") == "end"), {})
    # the truth's own lap count: the angle around the field centre, unwrapped, from every logged pose
    tr = [d["truth"] for d in logs if "truth" in d]
    ang, prev = 0.0, None
    for x, y, _th in tr:
        a = math.atan2(y, x)
        if prev is not None:
            ang += ((a - prev + math.pi) % (2 * math.pi)) - math.pi
        prev = a
    true_laps = abs(ang) / (2 * math.pi)
    l_, r_, b_, t_ = w.island
    in_start = (l_ <= w.x <= r_) and (w.y < b_) and not w.touching
    cam_ = robot.sim.camera
    cam_state = dict(frames=cam_.frames, alive=bool(getattr(cam_, "_thread", None) and cam_._thread.is_alive()),
                     errors=getattr(cam_, "errors", 0), last_error=getattr(cam_, "last_error", "") or None,
                     age_s=round(time.monotonic() - cam_._t, 2) if cam_._t else None)
    robot.close()
    e = sorted(x[0] for x in errs) or [float("nan")]
    eh = sorted(x[1] for x in errs) or [float("nan")]

    def rnd(v, nd=0):                                     # programs without a pose estimate (lapcam) log none
        return None if v != v else round(v, nd) if nd else round(v)
    for i_, (sd_, t1_) in cur_pass.items():
        passes.append((signs_[i_][2], sd_, tuple(signs_[i_][:2]), t1_))
    wrong_ = [(c_, xy_, t1_) for c_, sd_, xy_, t1_ in passes if (c_ == "red" and sd_ == "R") or (c_ == "green" and sd_ == "L")]
    out = dict(reason=end.get("reason"), laps=end.get("laps"), secs=end.get("seconds"), contacts=w.contacts,
               sign_passes=len(passes), wrong_side=len(wrong_),
               wrong_at=["%s(%d,%d)@%.0fs" % (c_, xy_[0], xy_[1], t1_) for c_, xy_, t1_ in wrong_][:6],
               touching=w.touching, err_med=rnd(e[len(e) // 2]), err_p95=rnd(e[int(len(e) * 0.95)]),
               err_max=rnd(e[-1]), hdg_p95=rnd(eh[int(len(eh) * 0.95)], 1),
               end_truth=[round(w.x), round(w.y), round(math.degrees(w.th))], seats=end.get("seats"),
               park=_park_score(w, scene), true_laps=round(true_laps, 2), in_start=in_start, touches=touches[:8],
               touch_s=round(touch_s[0], 1))
    if not cam_state["alive"] or cam_state["errors"] or (cam_state["age_s"] or 0) > 1.0:
        out["sim_camera"] = cam_state                        # the RENDERER stopped (not a fault): the run is suspect
    if FAULTS:                                               # --fault: what fired, and the contacts after it
        for rec in fault_out:
            rec["contacts_after"] = w.contacts - rec.pop("contacts_before")
        out["faults"] = fault_out or "none fired"
    if errs_park:
        out["err_park_max"] = round(max(errs_park))
    sh = next((d for d in reversed(logs) if d.get("ev") == "shield_stats"), None)
    if sh is not None:                                       # the lidar shield (BRAIN_SPEC 5.3): how often it acted
        out["shield"] = {k: sh.get(k) for k in ("n_slow", "n_steer", "n_brake")}
    cp = next((d for d in reversed(logs) if d.get("ev") == "cpu"), None)
    if cp is not None:                                       # W8: the program thread's CPU share (x3 on the Pi)
        out["cpu_share"] = cp.get("share")
    ss = next((d for d in logs if d.get("ev") == "start_search"), None)
    if ss is not None:
        out["start_by"] = ss.get("by", "camera")
    ds = next((d for d in reversed(logs) if d.get("ev") == "depth_stats"), None)
    if ds is not None:
        out["depth"] = {k: ds.get(k) for k in ("pairs", "unpaired", "wall_pts", "rgb_pts", "signs")}
    pf = next((d for d in reversed(logs) if d.get("ev") == "pitch_final"), None)
    if pf is not None and scene.get("camera_truth"):
        out["pitch_final_err"] = round(pf["pitch"] - float(scene["camera_truth"]["pitch_deg"]), 2)
    if w.driver is not None:                                 # the WLtoys plant: dynamics and the speed estimate
        out.update(rollover=w.rolled, ay_max=round(w.ay_max, 2), ay_over_cap_s=round(w.ay_over_cap_s, 2),
                   ay_at=w.ay_at, tight_fast_s=round(w.tight_fast_s, 2),
                   lost=sum(1 for d in logs if d.get("ev") == "lost"),
                   v_err_med=rnd(sorted(v_err)[len(v_err) // 2], 3) if v_err else None,
                   v_true_mean=rnd(sum(v_true) / len(v_true), 3) if v_true else None,
                   parked_legs=[{k: g.get(k) for k in ("leg", "mode", "pulses", "rem_mm", "timeout") if k in g}
                                for g in next((d.get("legs") for d in reversed(logs) if d.get("ev") == "parked"), [])])
    fs = fix_score(logs, free)
    if fs is not None:                                       # BRAIN4_SPEC 5.6: the LOCATE fix against the truth
        out["fix"] = fs
    svs = survey_score(logs, scene, (1 if ang > 0 else -1) if abs(ang) > 0.5 else None)
    if svs is not None:
        out["survey"] = svs
    if rf is not None:
        out["runfile"] = rf.path
    return out


def _base_pitch() -> float:
    if WLTOYS:
        return float(WLTOYS["camera.pitch_deg"])
    if PROFILE is not None and "camera.pitch_deg" in PROFILE:
        return float(PROFILE["camera.pitch_deg"])
    return NEW_BODY_SENSORS["camera"]["pitch_deg"] if CAR else P.DEFAULTS["camera"]["pitch_deg"]


def _park_score(w, scene):
    """How the car ended against the lot: fully inside the 320 x 200 interior = "full", else the overlap share."""
    if not scene.get("lot"):
        return None
    xa, xb = scene["lot"]
    lo, hi = min(xa, xb) + 10.0, max(xa, xb) - 10.0              # limitation centres +- half their 20 mm
    cs = w.corners(w.x, w.y, w.th)
    inside = sum(1 for px, py in cs if lo <= px <= hi and -1500.0 <= py <= -1300.0)
    return "full" if inside == 4 else "%d/4 corners in" % inside


def start_pose_arg(arg: str) -> list:
    """--start-pose: JSON [x, y, deg], or PLAN.json:MARK (a plan's `starts` entry, BRAIN4_SPEC 7.2)."""
    if arg.strip().startswith("["):
        return [float(v) for v in json.loads(arg)]
    path, mark = arg.rsplit(":", 1)
    if not os.path.isabs(path) and not os.path.exists(path):
        path = os.path.join(ROOT, path)
    with open(path, encoding="utf-8") as f:
        return [float(v) for v in json.load(f)["starts"][mark]["pose"]]


def clean(r: dict, start_any: bool) -> bool:
    """CLEAN: 0 contacts, 0 signs passed on the wrong side, and the laps done or parked -- with --start-any also an
    honest refusal, never a wrong fix."""
    if r["contacts"] or r.get("wrong_side"):
        return False
    if start_any:
        if (r.get("fix") or {}).get("wrong_fix"):
            return False
        return r["reason"] in ("laps", "parked") or r["reason"] in REFUSALS
    return r["reason"] in ("laps", "parked")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("program")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--seeds", default="")
    ap.add_argument("--cw", action="store_true")
    ap.add_argument("--scene", default="")
    ap.add_argument("--params", default="{}", help="JSON overrides for the program's DEFAULTS")
    ap.add_argument("--seconds", type=float, default=200.0)
    ap.add_argument("--lot", action="store_true", help="put a parking lot in the start straight")
    ap.add_argument("--lot-start", action="store_true", help="start parked in the lot")
    ap.add_argument("--open", action="store_true", help="Open Challenge: random 600/1000 corridors, no signs")
    ap.add_argument("--corridors", default="", help="with --open: the corridors S,E,N,W in mm instead of the seed's "
                                                    "(1000,1000,1000,1000 = the mat 2026-09-30/10-01); the start keeps "
                                                    "its share of the S corridor")
    ap.add_argument("--pitch-error", type=float, default=0.0, help="true camera pitch minus what the program believes")
    ap.add_argument("--new-body", action="store_true", help="the BLUE WAVE body: 296.8 mm car, 445.2 mm lot")
    ap.add_argument("--wltoys", action="store_true", help="the WLtoys build: profiles/wltoys.json on the duty plant")
    ap.add_argument("--profile", default="", help="a profile's patch (paramstore.profiles()); a gpio_pwm drive takes "
                                                  "the WLtoys plant path")
    ap.add_argument("--plant", default="{}", help="JSON overrides of the simulated WLtoys truth (mock.WL_PLANT)")
    ap.add_argument("--plant-file", default="", help="a fitted plant file (tools/fit_plant.py): its `plant` under "
                                                     "--plant, its `robot_set` under --set")
    ap.add_argument("--mat1001", action="store_true", help="the mat-calibrated plant: --plant-file "
                                                           "profiles/plant_mat1001.json (needs a gpio_pwm profile)")
    ap.add_argument("--depth", nargs="?", const="walls,pillars,start", default=None,
                    help="the HP60C depth on: its uses, comma-separated (walls, pillars, start; default all three)")
    ap.add_argument("--depth-truth", default="{}", help="JSON overrides of the simulated depth sensor (mock.DEPTH_TRUTH)")
    ap.add_argument("--set", default="{}", help="JSON robot params, dotted keys, over the profile (A/B one mechanism)")
    ap.add_argument("--obstacles", type=int, default=0,
                    help="N off-seat 50 x 50 red obstacles on the lane of straights E / N / W (only the shield sees them)")
    ap.add_argument("--shield", type=int, default=None, choices=(0, 1), help="params shield.on (a --set shortcut)")
    ap.add_argument("--start-any", action="store_true", help="the start anywhere on the corridor ring (5.6)")
    ap.add_argument("--expect", action="store_true", help="pass the true start as the program's `expect` (bw mat)")
    ap.add_argument("--start-pose", default="", help='JSON [x, y, deg]: the start (a taped mark, e.g. from plans/*.json '
                                                      'starts); PLAN.json:MARK reads it from a plan file')
    ap.add_argument("--runfile", default="", help="a directory: write a Runner-format run file per run there")
    ap.add_argument("--fault", default="", help='JSON fault(s) injected mid-run, e.g. {"kind": "pitch_jump", "t": 20, '
                                                '"in_state": "GO", "deg": 2} -- kinds %s (inject_faults)'
                                                % ", ".join(FAULT_KINDS))
    a = ap.parse_args()
    global PLANT_SCENE
    global CAR, WLTOYS, PLANT, DEPTH, DEPTH_TRUTH, SET, PROFILE, PROFILE_NAME, RUNFILE, FAULTS
    if a.fault:
        fl = json.loads(a.fault)
        FAULTS = [fl] if isinstance(fl, dict) else list(fl)
        bad = [f.get("kind") for f in FAULTS if f.get("kind") not in FAULT_KINDS]
        if bad:
            sys.exit("--fault: unknown kind(s) %s (%s)" % (bad, ", ".join(FAULT_KINDS)))
    if a.wltoys and a.new_body:
        sys.exit("--wltoys and --new-body are two different cars")
    if a.profile and (a.wltoys or a.new_body):
        sys.exit("--profile names the car: not with --wltoys / --new-body")
    CAR = NEW_BODY if a.new_body else None
    PLANT = json.loads(a.plant)
    if a.depth is not None:
        DEPTH = {s.strip() for s in a.depth.split(",") if s.strip()}
        bad = DEPTH - {"walls", "pillars", "start"}
        if bad:
            sys.exit("--depth: unknown use(s) %s (walls, pillars, start)" % ", ".join(sorted(bad)))
    DEPTH_TRUTH = json.loads(a.depth_truth)
    SET = json.loads(a.set)
    pf = a.plant_file or (os.path.join(ROOT, "profiles", "plant_mat1001.json") if a.mat1001 else "")
    if pf:
        # the mat's calibration (tools/fit_plant.py): the fitted truth under any --plant, the mat's robot params (as
        # the run logs show them) under any --set -- the command line still wins
        if not os.path.isabs(pf) and not os.path.exists(pf):
            pf = os.path.join(ROOT, pf)
        with open(pf, encoding="utf-8") as f:
            fitted = json.load(f)
        PLANT = dict(fitted.get("plant") or {}, **PLANT)
        SET = dict(fitted.get("robot_set") or {}, **SET)
        PLANT_SCENE = dict(fitted.get("scene") or {})     # a scene's own keys still win
        print("plant file %s: %s" % (os.path.relpath(pf, ROOT), json.dumps(fitted.get("robot_set") or {})), flush=True)
    if a.shield is not None:
        SET["shield.on"] = a.shield
    if a.wltoys:
        WLTOYS = wltoys_profile()
        CAR = {k.split(".", 1)[1]: v for k, v in WLTOYS.items() if k.startswith("car.")}
    elif a.profile:
        pt = profile_patch(a.profile)
        PROFILE_NAME = a.profile
        if str(pt.get("drive.backend", "rrc")) == "gpio_pwm":
            WLTOYS = pt                                      # the duty plant, scan_lag 0, the body (as --wltoys)
            CAR = {k.split(".", 1)[1]: v for k, v in WLTOYS.items() if k.startswith("car.")}
        else:
            PROFILE = pt
            CAR = None
    if pf and not WLTOYS:
        sys.exit("--mat1001 / --plant-file: the WLtoys duty plant only (--profile wltoys_bw2 or --wltoys)")
    if a.runfile:
        RUNFILE = os.path.abspath(a.runfile)
        os.environ["BW_RUNS"] = RUNFILE                      # the recording lands beside the run file
        os.environ["BW_MOCK_IMU_BUMP"] = "1"                 # the simulated IMU shows contacts (bluewave/mock.py)
    elif not os.environ.get("BW_RUNS"):
        # a simulated recording or survey file never lands in the tree's runs/ (the team's pulled runs): the survey
        # lap records by definition (BRAIN4_SPEC 5.2) and wrote rec/ and maps/survey-* there (2026-09-25)
        import shutil
        import tempfile
        os.environ["BW_RUNS"] = tempfile.mkdtemp(prefix="bwsim-")
        cache = os.path.join(ROOT, "runs", "exit_plans.json")    # the park's exit-plan cache: no 2 s cold start
        if os.path.isfile(cache):
            shutil.copy(cache, os.path.join(os.environ["BW_RUNS"], "exit_plans.json"))
    pp = dict({"record": 1 if a.runfile else 0}, **json.loads(a.params))
    start_any = bool(int(pp.get("start_any", 0))) or a.start_any

    def make_scene(s, cw):
        sc = random_open_scene(s, cw) if a.open else random_scene(s, cw, a.lot, a.lot_start, CAR)
        if a.open and a.corridors:
            cor = [float(v) for v in a.corridors.split(",")]
            if len(cor) != 4:
                sys.exit("--corridors: four widths S,E,N,W")
            share = (sc["start"][1] + 1500.0) / float(sc["corridors"][0])
            sc["corridors"], sc["start"][1] = cor, round(-1500.0 + cor[0] * share)
        if a.obstacles:
            sc["obstacles"] = obstacles(s, a.obstacles)
            sc["pillars"] = sc["pillars"] + sc["obstacles"]
        if a.pitch_error:
            sc["camera_truth"] = {"pitch_deg": _base_pitch() + a.pitch_error}
        if a.start_any:
            p_, sc_ = robot_params(sc)
            sc["start"] = random_start(s, sc, body_of(p_, sc_), cw)
        if a.start_pose:
            sc["start"] = start_pose_arg(a.start_pose)
        if a.start_pose or (a.start_any and a.expect):
            # a MARKED start (bw mat): the judges' rule the marks follow -- nothing within 550 mm ahead or 250 mm beside
            # the start (plans/*.json) -- so the scene's signs there go
            from bluewave import globloc as GL
            x_, y_, d_ = sc["start"]
            c_, s_ = math.cos(math.radians(d_)), math.sin(math.radians(d_))
            p_, sc_ = robot_params(sc)
            rr_, ff_, hh_ = body_of(p_, sc_)

            def in_zone(q):
                a_, b_ = c_ * (q[0] - x_) + s_ * (q[1] - y_), -s_ * (q[0] - x_) + c_ * (q[1] - y_)
                return -rr_ - GL.CLEAR_BESIDE < a_ < ff_ + GL.CLEAR_AHEAD and abs(b_) < hh_ + GL.CLEAR_BESIDE
            sc["pillars"] = [q for q in sc["pillars"] if not in_zone(q)]
            if sc.get("obstacles"):
                sc["obstacles"] = [q for q in sc["obstacles"] if not in_zone(q)]
        return sc

    def prog_params(sc):
        q = dict(pp)
        if a.expect:
            q["expect"] = list(sc["start"])
        return q

    if a.seeds:
        lo, hi = (int(x) for x in a.seeds.split("-"))
        rows = []
        for s in range(lo, hi + 1):
            for cw in ([False, True] if not a.cw else [True]):
                sc = make_scene(s, cw)
                r = run_one(a.program, sc, prog_params(sc), a.seconds, quiet=True, seed=s, cw=cw)
                rows.append(r)
                print("seed %2d %s  %s" % (s, "cw " if cw else "ccw", json.dumps({k: v for k, v in r.items() if k != "seats"})),
                      flush=True)
        if a.open and not start_any:
            ok = sum(1 for r in rows if r["contacts"] == 0 and 2.75 <= r["true_laps"] <= 3.25 and r["in_start"])
            print("CLEAN OPEN (3 true laps, stopped in the start straight, 0 contacts): %d / %d" % (ok, len(rows)))
            return 0
        ok = sum(1 for r in rows if clean(r, start_any))
        wrong_n, pass_n = sum(r.get("wrong_side") or 0 for r in rows), sum(r.get("sign_passes") or 0 for r in rows)
        if start_any:
            laps = sum(1 for r in rows if r["reason"] in ("laps", "parked") and r["contacts"] == 0)
            wrong = sum(1 for r in rows if (r.get("fix") or {}).get("wrong_fix"))
            refused = sum(1 for r in rows if r["reason"] in REFUSALS)
            print("CLEAN START-ANY (0 contacts, 0 wrong-side passes; laps done or an honest refusal; no wrong fix): "
                  "%d / %d  -- laps %d, refusals %d, wrong fixes %d, contacts %d, wrong-side passes %d of %d"
                  % (ok, len(rows), laps, refused, wrong, sum(r["contacts"] for r in rows), wrong_n, pass_n))
            return 0
        print("CLEAN (laps done or parked, 0 contacts, 0 wrong-side passes): %d / %d  -- wrong-side passes %d of %d"
              % (ok, len(rows), wrong_n, pass_n))
        return 0
    if a.scene:
        with open(a.scene, encoding="utf-8") as f:
            sc = json.load(f)
        if a.obstacles:
            sc["obstacles"] = obstacles(a.seed, a.obstacles)
            sc["pillars"] = sc.get("pillars", []) + sc["obstacles"]
        if a.pitch_error:
            sc["camera_truth"] = {"pitch_deg": _base_pitch() + a.pitch_error}
    else:
        sc = make_scene(a.seed, a.cw)
    print("scene", json.dumps(sc))
    r = run_one(a.program, sc, prog_params(sc), a.seconds, quiet=False, seed=a.seed, cw=a.cw)
    print("RESULT", json.dumps(r))
    return 0


if __name__ == "__main__":
    sys.exit(main())
