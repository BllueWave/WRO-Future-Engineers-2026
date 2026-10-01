"""The crash / near-miss analyzer (BRAIN4_SPEC 6): one run file (+ its recording) -> WHERE and WHY the car touched
something, as runs/<id>.report.json and a standalone runs/<id>.report.html.

    python -m bluewave.analyze RUN.jsonl [--rec DIR|auto|none] [--out-dir DIR] [--no-html] [--json-progress]
                                         [--set an.near_mm=20 ...] [--truth]

Exit 0; 2 = the run file cannot be read; 3 = an internal error (a minimal report with "error" is still written).
It never imports hw and never opens a device: the agent runs it as a separate niced process after a run
(bluewave/runs_api.py), the laptop runs it on pulled or simulated runs (tools/bw_analyze.py).

The mechanism, in the order it runs:
  1. read the run file (meta, log, tel 20 Hz, live 5 Hz, clock, imu 50 Hz) and the recording (steps.jsonl 40 Hz pose
     + command + state, JPEG frames, lidar scans); tie the recording's monotonic clock to the run file's Unix time
     (the `clock` record, the recording header, else start.t - first step - 0.05 s: "estimated +-0.1 s");
  2. per frame (160 x 120 decode): the wall bases of vision.edges, "near wall" (>= 60 % of the central half of the
     columns clipped WALL: a wall inside the band the camera cannot see, 213 mm ahead of the A1's nose), "lens
     blocked" (below the horizon >= 95 % dark and <= 5 % of the columns with a seen base, for >= 0.5 s -- measured on
     the M4 frames 151-1467: 99-100 % dark below the horizon, 0-4 seen columns; the room above the wall keeps the
     WHOLE image at 66 % dark, so the whole-image 80 % of the first draft never fires), and the 80 x 60 grey
     thumbnail difference (still 0.44, driving 8-12 on the mat recording: the A1's speed sensor);
  3. per lidar scan (lidar.loc = 1 only: the stock LD19 scans above the walls): the body clearance of the nearest
     return, the car's own returns (bins near the body in > 50 % of the scans) left out;
  4. the detectors D1-D13 of 6.3 -> candidates; fused into incidents (6.3 Fusion); each incident gets WHERE (pose,
     section, the map object and the part of the car, 6.4), WHAT the program was doing (the decision), WHY (the first
     of 12 attribution rules that holds, 6.5) and a fix hint;
  5. the report JSON (schema bw.report/1, 6.7) with a 10 Hz timeline, and the HTML (bluewave/report_html.py).

Thresholds are the an.* params (DEFAULTS below; the run's robot_params an.* override them, then --set):
[MAT 2026-09-23] contacts read 0.54-1.08 g in tel's 3-D deviation, a clean lap's max 0.33 g, a lone 0.62 g spike
with nothing near (M4 6.72 s) is NOT a contact -- a spike needs corroboration.
[MAT 2026-09-30] the WLtoys' own stick-slip start and brake jolt the IMU ~0.5 g: inside the command's start / brake
window and under an.jolt_g (from the body profile) a spike is the drive, not a contact (D1, stats.jolts).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback

import numpy as np

from . import blackbox as BB
from . import field as F

VERSION = 1
SCHEMA = "bw.report/1"

DEFAULTS = {                    # the an.* keys of BRAIN4_SPEC 2.1 (without the prefix)
    "auto": 1,                  # the agent analyses every run when it ends
    "acc_contact_g": 0.45,      # [MAT 2026-09-23 M4-M6] 3-D deviation of a contact (tel 20 Hz); [DAY1] re-check at 50 Hz
    "acc_bump_g": 0.28,         # a "bump?" candidate (weak)
    "gyro_jerk_dps": 60.0,      # yaw-rate residual jump within 0.1 s
    "push_s": 0.6,              # commanded >= 0.08 m/s this long while the measured motion < ratio x command
    "push_ratio": 0.2,
    "touch_mm": 8.0,            # lidar body clearance of a contact
    "near_mm": 30.0,            # footprint clearance of a near miss
    "merge_s": 0.6,             # candidates this close in time are one incident
    "loop_gap_ms": 150.0,       # a control step gap this long (while driving) is an overrun
    "pose_jump_mm": 80.0,       # a relocalisation that moves the pose this far marks the pose before it as wrong
    "sxy_bad_mm": 60.0,
    "bat_rest_v": 7.5, "bat_min_v": 7.0, "bat_sag_v": 0.5,
    "thumbs": 3,                # frames per incident (at t, t - 0.5 s, t - 1.0 s)
    "max_incidents": 40,
    "cam_frames": 1,            # analyse the recorded frames (the A1's camera signals, D6)
    "why_truth": 0,             # 1 = the simulator's truth (tel.sim) may decide WHERE / WHY too.  0 (always, for any
    #                             report the team reads): the truth only SCORES the detectors (D13, the report's
    #                             `truth` / `detector` blocks) -- the causes are what the mat, which has no truth, would
    #                             get (review 2026-09-25: with the truth the M4-type push read LOC_POSE_OFF, without it
    #                             UNKNOWN_OBSTACLE / TURN_RADIUS, and "the analyzer names the cause" was graded on it)
    # the drive's OWN jolts (D1): the WLtoys' stick-slip start and its shorted-motor brake shake the IMU ~0.5 g
    # [MAT 2026-09-30] -- over acc_contact_g, and one or two samples long like a hit, so every start from rest read
    # "contact".  A spike inside a jolt window of the command timeline and under jolt_g is the drive, not a contact,
    # unless something independent of the spike says contact (a near obstacle, a push, a stall).
    "jolt_g": 0.0,              # the ceiling: 0 = from the body -- a gpio_pwm drive (the WLtoys) JOLT_X x its measured
    #                             start jolt (the run's mock.plant.start_jolt_g, else its profile file's, else
    #                             acc_contact_g); the stock A1's encoder drive: none (off)
    "jolt_start_s": 0.3,        # a start from rest (or a reversal): the 60 ms kick breaks it loose at once -- [SIM
    #                             2026-09-29, the WLtoys plant with its 0.5 g start jolt] every jolt came 0.00-0.03 s
    #                             after the command (the 20 Hz tel's jitter is inside 0.3 s)
    "jolt_brake_s": 0.3,        # a brake (to rest, a reversal, a cut to <= half): the shorted motor bites at once
    "jolt_block_s": 1.5,        # ...but a car braking as it COASTS into a wall hits inside that window too: [SIM
    #                             2026-09-29, lapstop with the mirrored lidar] 8 of 9 true contacts came 0.02-0.26 s
    #                             after a brake command, each followed 1.2 s later by the program's turn_blocked.  A
    #                             program's blocked / stuck event (PUSH_EVENTS) this long after a spike = a contact
    "cam_stale_ms": 600.0,      # tel.cam_age_ms above this while driving: the camera stopped (D11b).  The robot's
    #                             age is 0-70 ms at 15-30 fps; the simulator stamps a frame before rendering it, so its
    #                             age reads 120-480 ms on a loaded machine -- 600 keeps that out, a frozen grabber grows
    #                             past it in 0.6 s
}

SEV = {"contact": 3, "error": 3, "sensor": 3, "near": 2, "loc_loss": 2, "pose_out": 2, "start_mismatch": 2,
       "bump": 1, "intervention": 1, "overrun": 1, "battery": 1}
COUNT_KEYS = ("contact", "bump", "near", "loc_loss", "pose_out", "intervention", "overrun", "sensor", "battery",
              "start_mismatch")
CAUSES = ("CAMERA_STALE", "CAMERA_PITCH", "LOC_POSE_OFF", "PUSH", "REVERSE_BLIND", "UNKNOWN_OBSTACLE", "SIGN_SIDE",
          "TURN_RADIUS", "TOO_FAST", "LOOP_STALL", "GUARD_SATURATED", "PARK_LEG", "CAMERA_BLIND", "SENSOR_LOSS",
          "UNEXPLAINED")
STRAIGHT = "SENW"
CORNER = ("SE", "NE", "NW", "SW")          # field.where corner k joins straight k to straight k + 1 (CCW)
PARK_STATES = ("APPROACH", "PARK", "EXIT")
REFUSALS = ("no_fix", "boxed_in", "no_join", "start_mismatch", "not_standard")   # wro_next ends WITHOUT moving (S14)
STILL_STATES = ("SETTLE", "LOCATE", "LOOK", "WAIT")
# lapcam logs its mode once a second; these events switch it at once (turn_blocked is left out: the back-off command
# starts ~0.1 s after it, and a hit in between happened while still turning)
LAPCAM_EVENT_STATE = {"corner": "TURN", "corner_done": "FOLLOW", "finish_plan": "FINISH", "finish": "FINISH"}
PUSH_EVENTS = ("stuck", "stuck_gyro", "turn_blocked", "follow_stuck", "blocked_stop", "creep_blocked", "park_blocked")
SENSOR_EVENTS = ("lidar_dead", "board_lost", "camera_stale", "camera_dead", "no_start")
V_MOVE = 0.05                   # m/s: a command this large is motion
V_PUSH = 0.08                   # m/s: D4 needs at least this commanded
LAG_S = 0.15                    # the drive's speed lag behind the command (mock.World, the RRC's loop)
GZ_STILL = 2.0                  # deg/s: "the heading does not move"
GZ_EXP_MIN = 10.0               # deg/s: the gyro form of D4 only where the command should turn at least this fast
STEER_MIN_GYRO = 12.0           # deg: ...and steers at least this much.  [MAT 2026-09-23 M5 2.9 s] at 5-8 deg the
#                                 A1 turned ~0 deg/s against an expected 7.6 (trim, the believed wheelbase 0.145 vs
#                                 0.28): a push by the looser rule, and nothing happened there
SUSPECT_S = 3.0                 # s: the pose is suspect this long before a loss / a jumping relocalisation and after an
#                                 accepted poor relocalisation (cost > reloc_max)
POSE_OUT_MM = 200.0             # the pose this far inside a wall body is not a pose (D8 pose_out)
JOLT_X = 1.6                    # an.jolt_g from the body: this x the measured start jolt (0.5 g -> 0.8 g; the mat's A1
#                                 contacts read 0.54-1.08 g, the mock's hit reads 3 g per m/s: a 0.27 m/s hit and up
#                                 stays a contact on the spike alone)
BRAKE_CUT = 0.5                 # a speed cut to <= this x the speed before, by >= BRAKE_DV, is a brake
BRAKE_DV = 0.1                  # m/s


# ================================================================================================== small helpers
def _f(v, nd=None):
    """JSON-safe number: None for None / NaN / inf, rounded."""
    if v is None:
        return None
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x):
        return None
    if nd is None:
        return x
    return int(round(x)) if nd == 0 else round(x, nd)


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def _hold(ts, vals, T):
    """Step-hold: the last value at or before each T (float arrays); NaN before the first sample."""
    ts = np.asarray(ts, dtype=np.float64)
    vals = np.asarray(vals, dtype=np.float64)
    T = np.asarray(T, dtype=np.float64)
    if not len(ts):
        return np.full(T.shape, np.nan)
    i = np.searchsorted(ts, T, side="right") - 1
    return np.where(i >= 0, vals[np.clip(i, 0, len(vals) - 1)], np.nan)


def _runs(mask, ts, gap_s=0.0):
    """[(i0, i1)] index ranges of consecutive True in `mask` (time order); two runs whose times are <= gap_s apart are
    joined."""
    mask = np.asarray(mask, dtype=bool)
    out = []
    i, n = 0, len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1]:
            j += 1
        if out and gap_s > 0 and ts[i] - ts[out[-1][1]] <= gap_s:
            out[-1] = (out[-1][0], j)
        else:
            out.append((i, j))
        i = j + 1
    return out


def _sec_name(x, y) -> str:
    kind, k = F.where(float(x), float(y))
    if kind == "straight":
        return STRAIGHT[k]
    if kind == "corner":
        return "corner " + CORNER[k]
    return "out"


def _sclear(X, Y, car):
    """Signed distance (mm) of car-frame points to the body rectangle [-rear, front] x [-half, half]; < 0 inside."""
    rear, front, half = car
    X = np.asarray(X, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)
    dx = np.maximum(-rear - X, X - front)
    dy = np.abs(Y) - half
    out = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
    inside = (dx < 0) & (dy < 0)
    return np.where(inside, np.maximum(dx, dy), out)


def _box_sdf(px, py, cx, cy, hx, hy):
    """Signed distance of field points to an axis-aligned box (outside +, inside -)."""
    dx = np.abs(px - cx) - hx
    dy = np.abs(py - cy) - hy
    out = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
    return np.where((dx < 0) & (dy < 0), np.maximum(dx, dy), out)


def _part(Xc, Yc, car) -> str:
    """The footprint side / corner nearest a car-frame point: front, front-left corner, left, ..."""
    rear, front, half = car
    lon = "front" if Xc >= front - 15.0 else "rear" if Xc <= -rear + 15.0 else ""
    lat = "left" if Yc >= half - 15.0 else "right" if Yc <= -half + 15.0 else ""
    if lon and lat:
        return "%s-%s corner" % (lon, lat)
    if lon or lat:
        return lon or lat
    # inside the body: the nearest face
    d = {"front": front - Xc, "rear": Xc + rear, "left": half - Yc, "right": Yc + half}
    return min(d, key=d.get)


def _bearing_part(bearing_deg: float) -> str:
    b = _wrap_deg(bearing_deg)
    names = [(-22.5, 22.5, "front"), (22.5, 67.5, "front-left corner"), (67.5, 112.5, "left"),
             (112.5, 157.5, "rear-left corner"), (-67.5, -22.5, "front-right corner"), (-112.5, -67.5, "right"),
             (-157.5, -112.5, "rear-right corner")]
    for lo, hi, n in names:
        if lo <= b < hi:
            return n
    return "rear"


def _steer_text(steer, mx) -> str:
    if steer is None or not math.isfinite(steer):
        return "steer ?"
    side = "left" if steer > 0 else "right"
    if abs(steer) >= mx - 1.0:
        return "full lock %s" % side
    if abs(steer) < 3.0:
        return "straight"
    return "steer %.0f deg %s" % (abs(steer), side)


# ================================================================================================== reading
class Run:
    """One run file, read."""

    def __init__(self, path: str):
        self.path = os.path.abspath(path)
        base = os.path.basename(path)
        self.id = base[:-6] if base.endswith(".jsonl") else base
        self.meta, self.t0 = {}, None
        self.logs, self.tel, self.live, self.clocks, self.imu_recs, self.errors = [], [], [], [], [], []
        n_ok = 0
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except ValueError:
                    continue                                  # a half-written last line
                if not isinstance(r, dict):
                    continue
                t = r.get("t")
                if not isinstance(t, (int, float)):
                    continue
                n_ok += 1
                if self.t0 is None:
                    self.t0 = float(t)
                if "meta" in r and not self.meta:
                    self.meta = r["meta"] or {}
                elif "tel" in r and isinstance(r["tel"], dict):
                    self.tel.append((float(t), r["tel"]))
                elif "log" in r:
                    x = r["log"]
                    self.logs.append((float(t), x))
                    if isinstance(x, dict) and "error" in x:
                        self.errors.append((float(t), str(x["error"])))
                elif "live" in r and isinstance(r["live"], dict):
                    self.live.append((float(t), r["live"]))
                elif "clock" in r and isinstance(r["clock"], dict):
                    self.clocks.append((float(t), r["clock"]))
                elif "imu" in r and isinstance(r["imu"], dict):
                    self.imu_recs.append((float(t), r["imu"]))
        if not n_ok:
            raise ValueError("no records in %s" % path)
        # a run started by the car's START button (bw mat, --wait-button): its times count from the BUTTON, not from
        # when the file opened -- a 44 s wait made a 32 s run's NEAR read "65.7 s" (review 2026-09-25)
        self.file_t0 = self.t0
        self.button_t = next((t for t, x in self.logs if isinstance(x, dict) and x.get("ev") == "button"), None)
        self.waited = any(isinstance(x, dict) and x.get("ev") == "wait_button" for _t, x in self.logs)
        if self.button_t is not None:
            self.t0 = float(self.button_t)
        self.t_last = max([self.t0] + [t for t, _ in self.tel] + [t for t, _ in self.logs] + [t for t, _ in self.live])
        self.rp = dict(self.meta.get("robot_params") or {})
        self.params = dict(self.meta.get("params") or {})
        self.program = self.meta.get("program") or self._id_program()
        self.events = [(t - self.t0, x) for t, x in self.logs if isinstance(x, dict) and "ev" in x]
        start = next((x for _t, x in self.events if x.get("ev") == "start"), None)
        self.start_ev = start
        self.start_t = next((t for t, x in self.events if x.get("ev") == "start"), None)
        if start and isinstance(start.get("params"), dict):
            self.params = dict(start["params"], **self.params)       # the program's full params (DEFAULTS merged)
        self.end = next((x for _t, x in reversed(self.events) if x.get("ev") == "end"), None)

    def _id_program(self):
        parts = self.id.split("-", 2)
        return parts[2] if len(parts) == 3 else None

    def rel(self, t_unix):
        return float(t_unix) - self.t0


class Rec:
    """A wro_next / survey recording: steps.jsonl (header, frames, steps, scans), JPEG frames, robot_params.json."""

    def __init__(self, d: str):
        self.dir = d
        self.header = None
        fr_n, fr_t = [], []
        st = {k: [] for k in ("t", "v", "steer", "yaw", "x", "y", "th", "sxy")}
        st_state = []
        self.scans = []                                     # (k, t, res, dist uint16 array)
        with open(os.path.join(d, "steps.jsonl"), encoding="utf-8", errors="replace") as f:
            for i, line in enumerate(f):
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if not isinstance(r, dict):
                    continue
                if i == 0 and "clock" in r and "t" not in r:
                    self.header = r
                    continue
                if "frame" in r and "t" in r:
                    fr_n.append(int(r["frame"]))
                    fr_t.append(float(r["t"]))
                elif "scan" in r and "t" in r and "d" in r:
                    try:
                        import base64
                        dist = np.frombuffer(base64.b64decode(r["d"]), dtype="<u2")
                        self.scans.append((int(r["scan"]), float(r["t"]), float(r.get("res", 1.0)), dist))
                    except (ValueError, TypeError):
                        pass
                elif "t" in r and "v" in r:
                    for k in st:
                        v = r.get(k)
                        st[k].append(float(v) if isinstance(v, (int, float)) else np.nan)
                    st_state.append(r.get("state"))
        self.frames_n = np.array(fr_n, dtype=np.int64)
        self.frames_t = np.array(fr_t, dtype=np.float64)
        o = np.argsort(self.frames_t, kind="stable")
        self.frames_n, self.frames_t = self.frames_n[o], self.frames_t[o]
        self.steps = {k: np.array(v, dtype=np.float64) for k, v in st.items()}
        self.steps_state = st_state
        self.rp = None
        try:
            with open(os.path.join(d, "robot_params.json"), encoding="utf-8") as f:
                self.rp = json.load(f)
        except (OSError, ValueError):
            pass

    def frame_path(self, n: int) -> str:
        return os.path.join(self.dir, "f%05d.jpg" % int(n))


def find_rec(run: Run, rec_arg: str | None):
    """The recording of a run: an explicit dir, else the `start` event's `rec` basename under the run file's rec/
    (then ../rec, ../runs/rec), else the recording whose header names this run_id; None = none / "none"."""
    if rec_arg in (None, "none", ""):
        return None
    if rec_arg != "auto":
        return rec_arg if os.path.isfile(os.path.join(rec_arg, "steps.jsonl")) else None
    base_dir = os.path.dirname(run.path)
    cands = []
    rec_path = (run.start_ev or {}).get("rec") or (run.end or {}).get("rec")
    if isinstance(rec_path, str) and rec_path:
        b = os.path.basename(rec_path.rstrip("/\\"))
        cands += [os.path.join(base_dir, "rec", b), os.path.join(base_dir, "..", "rec", b),
                  os.path.join(base_dir, "..", "runs", "rec", b)]
    for c in cands:
        if os.path.isfile(os.path.join(c, "steps.jsonl")):
            return os.path.normpath(c)
    for root in (os.path.join(base_dir, "rec"), os.path.join(base_dir, "..", "rec")):
        if not os.path.isdir(root):
            continue
        for name in sorted(os.listdir(root), reverse=True):
            p = os.path.join(root, name, "steps.jsonl")
            try:
                with open(p, encoding="utf-8") as f:
                    h = json.loads(f.readline() or "null")
            except (OSError, ValueError):
                continue
            if isinstance(h, dict) and h.get("run_id") == run.id:
                return os.path.normpath(os.path.join(root, name))
    return None


def _get(d, dotted, default=None):
    cur = d
    for k in dotted.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def _profile_value(name: str, key: str, sub: str | None = None):
    """profiles/<name>.json's patch[key] (its [sub]) -- mentorpi/profiles on the laptop, ~/bluewave/profiles on the
    robot (bw deploy copies it) -- or None."""
    if not name or os.sep in name or "/" in name:
        return None
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "profiles", name + ".json")
    try:
        with open(path, encoding="utf-8") as f:
            v = (json.load(f).get("patch") or {}).get(key)
    except (OSError, ValueError, AttributeError):
        return None
    return v.get(sub) if sub is not None and isinstance(v, dict) else (v if sub is None else None)


# ================================================================================================== the analysis
class Analysis:
    def __init__(self, run: Run, rec: Rec | None, cfg: dict, truth: bool = False, progress=None):
        self.run, self.rec, self.cfg = run, rec, cfg
        self.want_truth = truth
        self.progress = progress or (lambda p, stage: None)
        self.notes = []
        rp = run.rp
        car = rp.get("car") or {}
        self.car = (float(car.get("rear_mm", 34.0)), float(car.get("front_mm", 179.0)),
                    float(car.get("half_w_mm", 80.5)))
        self.park_margin = float(car.get("park_margin_mm", 13.0))
        self.L = float(_get(rp, "chassis.wheelbase_m", 0.145)) * 1000.0
        self.steer_max = float(_get(rp, "steer.max_deg", 29.0))
        self.speed_scale = float(_get(rp, "odo.speed_scale", 0.92))
        self.lidar_loc = int(_get(rp, "lidar.loc", 0) or 0)
        self.wltoys = _get(rp, "drive.backend", "rrc") == "gpio_pwm"
        self.jolt_g, self.jolt_src = self._jolt_ceiling(rp, cfg)
        self.jolts = []           # D1 spikes read as the drive's own start / brake jolt (not candidates)
        self.mock = any(bool(te.get("mock")) for _t, te in run.tel[:3])
        self.dur = max(0.0, run.t_last - run.t0)
        self.cands = []           # contact-type candidates
        self.others = []          # non-contact incidents (already incidents)
        self.frame_stats = None
        self.scan_stats = None
        self.offset = None        # unix - mono
        self.clock_src = None

    def _jolt_ceiling(self, rp: dict, cfg: dict) -> tuple:
        """(an.jolt_g in g, where it came from).  Set (> 0): as set.  0: from the body -- a gpio_pwm drive JOLT_X x
        its measured start jolt: the run's mock.plant.start_jolt_g (the profile carries the mat's number there), else
        the stamped profile file's (an older params.json on the robot lacks it), else acc_contact_g; the rrc drive
        (the stock A1: encoder motors, a smooth start) 0 = no jolt gate."""
        set_g = float(cfg.get("jolt_g") or 0.0)
        if set_g > 0.0:
            return set_g, "an.jolt_g"
        if not self.wltoys:
            return 0.0, "rrc drive: no jolt"
        meas, src = _get(rp, "mock.plant.start_jolt_g", None), "run params mock.plant.start_jolt_g"
        if not (isinstance(meas, (int, float)) and meas > 0):
            name = str(_get(rp, "body.profile", "") or "")
            meas, src = _profile_value(name, "mock.plant", "start_jolt_g"), "profiles/%s.json" % name
        if not (isinstance(meas, (int, float)) and meas > 0):
            meas, src = float(cfg["acc_contact_g"]), "an.acc_contact_g (no measured jolt)"
        return JOLT_X * float(meas), "%.2f x %.2f g, %s" % (JOLT_X, float(meas), src)

    # ---------------------------------------------------------------------------------------------- clock
    def _clock(self):
        run, rec = self.run, self.rec
        if run.clocks:
            t, c = run.clocks[0]
            if isinstance(c.get("mono"), (int, float)):
                self.offset, self.clock_src = t - float(c["mono"]), "run file clock"
        if self.offset is None and rec is not None and rec.header and isinstance(rec.header.get("clock"), dict):
            c = rec.header["clock"]
            if isinstance(c.get("mono"), (int, float)) and isinstance(c.get("unix"), (int, float)):
                self.offset, self.clock_src = float(c["unix"]) - float(c["mono"]), "recording header"
        if self.offset is None and rec is not None and len(rec.steps["t"]):
            first = float(np.nanmin(np.concatenate([rec.steps["t"], rec.frames_t]))) if len(rec.frames_t) else \
                float(np.nanmin(rec.steps["t"]))
            st = run.start_t + run.t0 if run.start_t is not None else run.t0
            self.offset, self.clock_src = st - first - 0.05, "estimated +-0.1 s"
        if self.offset is None and run.imu_recs:
            offs = []
            for t, body in run.imu_recs:
                arr = BB.decode(body)
                if len(arr):
                    offs.append(t - float(arr[-1, 0]))
            if offs:
                self.offset, self.clock_src = float(np.median(offs)), "estimated from the imu records +-0.1 s"

    def mono_rel(self, t_mono):
        return np.asarray(t_mono, dtype=np.float64) + self.offset - self.run.t0

    # ---------------------------------------------------------------------------------------------- signals
    def _signals(self):
        run, rec = self.run, self.rec
        tel = run.tel
        self.tel_t = np.array([run.rel(t) for t, _ in tel], dtype=np.float64)

        def col(k, sub=None):
            out = []
            for _t, te in tel:
                v = te.get(k) if sub is None else (te.get(k) or {}).get(sub) if isinstance(te.get(k), dict) else None
                out.append(float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else np.nan)
            return np.array(out, dtype=np.float64)
        if len(tel):
            self.cmd_t, self.cmd_v, self.cmd_s = self.tel_t, np.nan_to_num(col("v")), np.nan_to_num(col("steer"))
        elif rec is not None and len(rec.steps["t"]) and self.offset is not None:
            self.cmd_t = self.mono_rel(rec.steps["t"])
            self.cmd_v = np.nan_to_num(rec.steps["v"])
            self.cmd_s = np.nan_to_num(rec.steps["steer"])
        else:
            self.cmd_t, self.cmd_v, self.cmd_s = np.zeros(1), np.zeros(1), np.zeros(1)
        # the drive's speed behind the command (first-order lag): what the car was really doing a moment after a step
        ve = np.zeros(len(self.cmd_t))
        for i in range(1, len(ve)):
            dt = max(0.0, self.cmd_t[i] - self.cmd_t[i - 1])
            ve[i] = ve[i - 1] + (self.cmd_v[i - 1] - ve[i - 1]) * (1.0 - math.exp(-dt / LAG_S))
        self.v_est = ve
        self.bat = col("battery_v") if len(tel) else np.zeros(0)
        self.yaw = col("yaw") if len(tel) else np.zeros(0)
        self.imu_hz = col("imu_hz") if len(tel) else np.zeros(0)
        self.cam_fps = col("cam_fps") if len(tel) else np.zeros(0)
        self.deadman = col("deadman_trips") if len(tel) else np.zeros(0)
        self.board_err = np.array([float((te.get("board_error") or {}).get("n", 0)) if isinstance(te.get("board_error"), dict)
                                   else 0.0 for _t, te in tel]) if len(tel) else np.zeros(0)
        # the WLtoys drive block: the lidar-measured speed when fresh, the stall flag
        self.v_lidar = np.full(len(tel), np.nan)
        self.stall = np.zeros(len(tel), bool)
        for i, (_t, te) in enumerate(tel):
            d = te.get("drive")
            if isinstance(d, dict):
                if d.get("lidar_fresh") and isinstance(d.get("v_lidar"), (int, float)):
                    self.v_lidar[i] = float(d["v_lidar"])
                self.stall[i] = bool(d.get("stall"))
        # the gyro: the board's samples when the run has imu records (50 Hz), else the tel yaw's differences (20 Hz)
        imu = [BB.decode(b) for _t, b in run.imu_recs]
        imu = [a for a in imu if len(a)]
        self.imu = np.concatenate(imu) if imu else np.zeros((0, 7))
        if len(self.imu):
            self.imu = self.imu[np.argsort(self.imu[:, 0], kind="stable")]
        if len(self.imu) >= 10 and self.offset is not None:
            it = self.mono_rel(self.imu[:, 0])
            ax = int(_get(run.rp, "imu.gyro_axis", 5))
            sign = float(_get(run.rp, "imu.gyro_sign", 1.0))
            bias = float(_get(run.rp, "imu.gyro_bias", 0.0))
            ax = ax if 0 <= ax < 6 else 5
            self.gz_t, self.gz = it, self.imu[:, 1 + ax] * sign - bias
            self.acc_t, self.acc = it, self.imu[:, 1:4]
            per = float(np.median(np.diff(it))) if len(it) > 2 else 0.02
            self.imu_src = "imu %d Hz" % int(round(1.0 / max(per, 1e-3)))
        else:
            if len(self.yaw) >= 3:
                yu = np.degrees(np.unwrap(np.radians(np.nan_to_num(self.yaw))))
                self.gz_t, self.gz = self.tel_t, np.gradient(yu, self.tel_t)
            else:
                self.gz_t, self.gz = np.zeros(0), np.zeros(0)
            acc = [(run.rel(t), te["acc"]) for t, te in tel if isinstance(te.get("acc"), list) and len(te["acc"]) == 3]
            self.acc_t = np.array([a for a, _ in acc], dtype=np.float64)
            self.acc = np.array([b for _, b in acc], dtype=np.float64).reshape(-1, 3)
            self.imu_src = "tel 20 Hz" if len(acc) else "none"
        self.dev = BB.deviation(self.acc_t, self.acc) if len(self.acc_t) else np.zeros(0)
        # the gyro smoothed over 0.3 s (the sustained push test) and the expected yaw rate from the command
        if len(self.gz_t) > 2:
            per = float(np.median(np.diff(self.gz_t)))
            k = max(1, int(round(0.15 / max(per, 1e-3))))
            ker = np.ones(2 * k + 1) / (2 * k + 1)
            self.gz_s = np.convolve(np.pad(self.gz, k, mode="edge"), ker, mode="valid")
        else:
            self.gz_s = self.gz.copy()
        self.exp_t = self.cmd_t
        self.exp = np.degrees(self.v_est * 1000.0 * self.speed_scale * np.tan(np.radians(self.cmd_s)) / max(self.L, 1.0))
        # the pose: steps 40 Hz > live 5 Hz > the log's pose dicts 2 Hz
        self.pose = None
        if rec is not None and self.offset is not None and np.isfinite(rec.steps["x"]).sum() > 5:
            m = np.isfinite(rec.steps["x"]) & np.isfinite(rec.steps["y"])
            self.pose = dict(src="steps", t=self.mono_rel(rec.steps["t"][m]), x=rec.steps["x"][m], y=rec.steps["y"][m],
                             th=np.degrees(rec.steps["th"][m]), sxy=rec.steps["sxy"][m])
        if self.pose is None:
            lp = [(run.rel(t), l["pose"]) for t, l in run.live if isinstance(l.get("pose"), list) and len(l["pose"]) >= 4]
            if lp:
                self.pose = dict(src="live", t=np.array([a for a, _ in lp]), x=np.array([p[0] for _, p in lp], float),
                                 y=np.array([p[1] for _, p in lp], float), th=np.array([p[2] for _, p in lp], float),
                                 sxy=np.array([p[3] if p[3] is not None else np.nan for _, p in lp], float))
        if self.pose is None:
            pl = [(run.rel(t), x) for t, x in run.logs if isinstance(x, dict) and "ev" not in x
                  and all(isinstance(x.get(k), (int, float)) for k in ("x", "y", "th"))]
            if pl:
                self.pose = dict(src="log", t=np.array([a for a, _ in pl]), x=np.array([p["x"] for _, p in pl], float),
                                 y=np.array([p["y"] for _, p in pl], float), th=np.array([p["th"] for _, p in pl], float),
                                 sxy=np.array([p.get("sxy") if isinstance(p.get("sxy"), (int, float)) else np.nan
                                               for _, p in pl], float))
        # the state: steps > live > log dicts (state / lapcam's mode) + the lapcam events that switch its mode
        st = []
        if rec is not None and self.offset is not None and rec.steps_state and any(rec.steps_state):
            ts = self.mono_rel(rec.steps["t"])
            st = [(float(t), s) for t, s in zip(ts, rec.steps_state) if s]
        if not st:
            st = [(run.rel(t), l["state"]) for t, l in run.live if l.get("state")]
        if not st:
            for t, x in run.logs:
                if isinstance(x, dict) and "ev" not in x and (x.get("state") or x.get("mode")):
                    st.append((run.rel(t), str(x.get("state") or x.get("mode"))))
                elif isinstance(x, dict) and x.get("ev") in LAPCAM_EVENT_STATE and run.program == "lapcam":
                    st.append((run.rel(t), LAPCAM_EVENT_STATE[x["ev"]]))
            st.sort(key=lambda a: a[0])
        self.state_t = np.array([a for a, _ in st], dtype=np.float64)
        self.state_v = [str(b) for _, b in st]
        # the sim's truth
        self.truth = None
        sim = [(run.rel(t), te["sim"]) for t, te in tel if isinstance(te.get("sim"), dict)]
        if sim:
            self.truth = dict(t=np.array([a for a, _ in sim]), x=np.array([s.get("x", np.nan) for _, s in sim], float),
                              y=np.array([s.get("y", np.nan) for _, s in sim], float),
                              th=np.degrees(np.array([s.get("th", np.nan) for _, s in sim], float)),
                              contacts=np.array([s.get("contacts", 0) or 0 for _, s in sim], float))

    # command / state / pose lookups ---------------------------------------------------------------------------------
    def v_at(self, t):
        return float(_hold(self.cmd_t, self.cmd_v, [t])[0]) if len(self.cmd_t) else 0.0

    def s_at(self, t):
        return float(_hold(self.cmd_t, self.cmd_s, [t])[0]) if len(self.cmd_t) else 0.0

    def vest_at(self, t):
        return float(_hold(self.cmd_t, self.v_est, [t])[0]) if len(self.cmd_t) else 0.0

    def moving(self, t0, t1):
        """max |commanded v| over [t0, t1] (the tel samples in it and the one before) > V_MOVE."""
        if not len(self.cmd_t):
            return False
        i0 = max(0, int(np.searchsorted(self.cmd_t, t0, side="right")) - 1)
        i1 = int(np.searchsorted(self.cmd_t, t1, side="right"))
        seg = self.cmd_v[i0:max(i1, i0 + 1)]
        return bool(len(seg) and np.nanmax(np.abs(seg)) > V_MOVE)

    def state_at(self, t):
        if not len(self.state_t):
            return None
        i = int(np.searchsorted(self.state_t, t, side="right")) - 1
        return self.state_v[i] if i >= 0 else self.state_v[0]

    def pose_at(self, t):
        """(x, y, th deg, sxy) interpolated, or None outside the pose's span (+-0.5 s)."""
        p = self.pose
        if p is None or not len(p["t"]) or t < p["t"][0] - 0.5 or t > p["t"][-1] + 0.5:
            return None
        x = float(np.interp(t, p["t"], p["x"]))
        y = float(np.interp(t, p["t"], p["y"]))
        if "thu" not in p:                                  # unwrapped once: pose_at runs per 10 Hz tick
            p["thu"] = np.degrees(np.unwrap(np.radians(p["th"])))
        th = _wrap_deg(float(np.interp(t, p["t"], p["thu"])))
        i = int(np.clip(np.searchsorted(p["t"], t), 0, len(p["t"]) - 1))
        sxy = float(p["sxy"][i]) if np.isfinite(p["sxy"][i]) else float("nan")
        return x, y, th, sxy

    def truth_at(self, t):
        tr = self.truth
        if tr is None or not len(tr["t"]):
            return None
        i = int(np.clip(np.searchsorted(tr["t"], t), 0, len(tr["t"]) - 1))
        return float(tr["x"][i]), float(tr["y"][i]), float(tr["th"][i])

    def truth_why(self, t):
        """The truth for WHERE / WHY: None unless an.why_truth = 1 -- the mat has none, so a report must not lean on
        it (the truth scores the detectors only: d13_truth, the report's truth blocks)."""
        return self.truth_at(t) if int(self.cfg.get("why_truth", 0) or 0) else None

    def ev(self, names, t0=-1e9, t1=1e9):
        names = (names,) if isinstance(names, str) else names
        return [(t, x) for t, x in self.run.events if x.get("ev") in names and t0 <= t <= t1]

    # ---------------------------------------------------------------------------------------------- the map
    def _map(self):
        """The map elements as axis-aligned boxes (cx, cy, hx, hy, kind, name) plus the outer walls: what the run
        knew (live pillars, seat events, the seatmap, obstacle events, the lot)."""
        run = self.run
        sc = _get(run.meta, "sim.scene", None) or {}
        isl = None
        if isinstance(sc, dict) and sc.get("corridors"):
            cS, cE, cN, cW = (float(v) for v in sc["corridors"])
            isl = (-F.HALF + cW, F.HALF - cE, -F.HALF + cS, F.HALF - cN)
            self.layout = "open-%d-%d-%d-%d" % (cS, cE, cN, cW)
        else:
            isl = (-F.ISL, F.ISL, -F.ISL, F.ISL)
            self.layout = "standard"
        self.island = isl
        signs = []                              # [x, y, colour, t_first_known, t_colour_change or None]

        def add(x, y, c, t):
            for s in signs:
                if math.hypot(s[0] - x, s[1] - y) < 150.0:
                    if c and s[2] != c:
                        if s[2] is None:
                            s[2], s[3] = c, min(s[3], t)
                        elif s[4] is None:
                            s[4] = t                  # the colour changed
                            s[2] = c
                    return
            signs.append([float(x), float(y), c, t, None])
        for t, l in run.live:
            for q in l.get("pillars") or []:
                if isinstance(q, (list, tuple)) and len(q) >= 3:
                    add(q[0], q[1], q[2], run.rel(t))
        for t, x in run.events:
            if x.get("ev") == "seat" and isinstance(x.get("at"), list) and len(x["at"]) == 2:
                add(x["at"][0], x["at"][1], x.get("colour"), t)
            elif x.get("ev") == "seatmap":
                for s in x.get("seats") or []:
                    if isinstance(s, dict) and isinstance(s.get("x"), (int, float)):
                        add(s["x"], s["y"], s.get("colour"), t)
        # a "sign" nowhere near a seat is an obstacle (the rules put signs on the 24 seats only)
        obst = []
        keep = []
        for s in signs:
            if float(np.min(np.hypot(F.SEAT_XY[:, 0] - s[0], F.SEAT_XY[:, 1] - s[1]))) > 150.0:
                obst.append((s[0], s[1], s[3]))
            else:
                keep.append(s)
        self.signs = signs = keep
        for t, x in run.events:
            if x.get("ev") == "obstacle":
                for a in x.get("at") or []:
                    if isinstance(a, list) and len(a) == 2 and not any(
                            math.hypot(o[0] - a[0], o[1] - a[1]) < 60.0 for o in obst):
                        obst.append((float(a[0]), float(a[1]), t))
        self.obstacles = obst
        lot = None
        for t, l in run.live:
            if isinstance(l.get("lot"), list) and len(l["lot"]) == 4:
                lot = [float(v) for v in l["lot"]]
        if lot is None:
            dirn = next((x.get("dir") for _t, x in run.events if x.get("ev") == "direction"), None)
            lev = [x for _t, x in run.events if x.get("ev") in ("lot", "lot_start") and isinstance(x.get("x0"), (int, float))]
            if lev and dirn:
                ll = float((run.rp.get("car") or {}).get("lot_len_mm", 320.0))
                x0 = float(lev[-1]["x0"])
                x1 = x0 + (ll if str(dirn).startswith("ccw") else -ll)
                a, b = sorted((x0, x1))
                lot = [a, -F.HALF, b, -F.HALF + F.LOT_DEPTH]
        self.lot = lot
        dirn = next((x.get("dir") for _t, x in run.events if x.get("ev") == "direction"), None)
        self.direction = -1 if (isinstance(dirn, str) and dirn.startswith("cw")) else 1
        boxes = []
        l, r, b, t = isl
        boxes.append(((l + r) / 2, (b + t) / 2, (r - l) / 2, (t - b) / 2, "island", "island"))
        for s in signs:
            boxes.append((s[0], s[1], F.PILLAR / 2, F.PILLAR / 2, "sign", self._seat_name(s)))
        for ox, oy, _t in obst:
            boxes.append((ox, oy, F.PILLAR / 2, F.PILLAR / 2, "obstacle", "obstacle"))
        if lot:
            a, _y0, b, _y1 = lot
            hy = F.LIMIT[0] / 2
            cy = -F.HALF + hy
            first, second = (a - F.LIMIT[1] / 2, b + F.LIMIT[1] / 2) if self.direction > 0 else \
                (b + F.LIMIT[1] / 2, a - F.LIMIT[1] / 2)
            boxes.append((first, cy, F.LIMIT[1] / 2, hy, "limitation", "limitation near"))
            boxes.append((second, cy, F.LIMIT[1] / 2, hy, "limitation", "limitation far"))
        self.boxes = boxes
        # boundary samples of every box, every <= 10 mm (a crossing without a corner inside is still seen)
        self.box_pts = []
        for cx, cy, hx, hy, _k, _n in boxes:
            nx, ny = max(2, int(math.ceil(2 * hx / 10.0)) + 1), max(2, int(math.ceil(2 * hy / 10.0)) + 1)
            xs, ys = np.linspace(cx - hx, cx + hx, nx), np.linspace(cy - hy, cy + hy, ny)
            px = np.concatenate([xs, xs, np.full(ny, cx - hx), np.full(ny, cx + hx)])
            py = np.concatenate([np.full(nx, cy - hy), np.full(nx, cy + hy), ys, ys])
            self.box_pts.append((px, py))

    def _seat_name(self, s) -> str:
        d = np.hypot(F.SEAT_XY[:, 0] - s[0], F.SEAT_XY[:, 1] - s[1])
        j = int(np.argmin(d))
        k, ai, _ci, _x, _y = F.SEATS[j]
        return "sign seat %s:%d (%s)" % (STRAIGHT[k], ai, s[2] or "colour unknown")

    def clearance(self, x, y, th_deg):
        """Footprint clearance (mm, signed) to every map element at poses (arrays): (min clearance, index of the
        element: -1..-4 outer walls S E N W, 0.. the boxes)."""
        x, y, th = np.atleast_1d(x).astype(float), np.atleast_1d(y).astype(float), np.radians(np.atleast_1d(th_deg))
        rear, front, half = self.car
        c, s = np.cos(th), np.sin(th)
        cxs = np.array([-rear, front, front, -rear])
        cys = np.array([-half, -half, half, half])
        Xc = x[:, None] + c[:, None] * cxs[None, :] - s[:, None] * cys[None, :]
        Yc = y[:, None] + s[:, None] * cxs[None, :] + c[:, None] * cys[None, :]
        # the outer walls: a convex footprint inside a square -- the nearest point is a corner
        m = np.maximum(np.abs(Xc), np.abs(Yc))
        wd = F.HALF - m
        k = np.argmin(wd, axis=1)
        best = wd[np.arange(len(x)), k]
        cx_, cy_ = Xc[np.arange(len(x)), k], Yc[np.arange(len(x)), k]
        wall = np.where(np.abs(cx_) >= np.abs(cy_), np.where(cx_ > 0, -2, -4), np.where(cy_ > 0, -3, -1))
        idx = wall.astype(int)
        for bi, (bx, by, hx, hy, _kind, _n) in enumerate(self.boxes):
            d1 = _box_sdf(Xc, Yc, bx, by, hx, hy).min(axis=1)
            px, py = self.box_pts[bi]
            dx, dy = px[None, :] - x[:, None], py[None, :] - y[:, None]
            Xb = c[:, None] * dx + s[:, None] * dy
            Yb = -s[:, None] * dx + c[:, None] * dy
            d2 = _sclear(Xb, Yb, self.car).min(axis=1)
            d = np.minimum(d1, d2)
            better = d < best
            best = np.where(better, d, best)
            idx = np.where(better, bi, idx)
        return best, idx

    def element_name(self, idx: int, x=None, y=None) -> str:
        if idx < 0:
            return "outer wall %s" % "SENW"[-idx - 1]
        cx, cy, hx, hy, kind, name = self.boxes[idx]
        if kind == "island" and x is not None:
            # the island face nearest the car
            dx, dy = x - cx, y - cy
            if abs(dx) / max(hx, 1.0) >= abs(dy) / max(hy, 1.0):
                return "island face %s" % ("E" if dx > 0 else "W")
            return "island face %s" % ("N" if dy > 0 else "S")
        return name

    def nearest_part(self, idx, x, y, th_deg) -> str | None:
        """The part of the footprint nearest element idx at the pose."""
        c, s = math.cos(math.radians(th_deg)), math.sin(math.radians(th_deg))
        if idx < 0:
            k = -idx - 1                                       # S E N W: the wall's inward normal
            px, py = {0: (x, -F.HALF), 1: (F.HALF, y), 2: (x, F.HALF), 3: (-F.HALF, y)}[k]
            Xc, Yc = c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)
            # the nearest footprint point to a straight wall lies toward the wall: take the corner nearest the wall
            rear, front, half = self.car
            corners = [(-rear, -half), (front, -half), (front, half), (-rear, half)]
            wx, wy = {0: (0, -1), 1: (1, 0), 2: (0, 1), 3: (-1, 0)}[k]
            proj = [(x + c * a - s * b) * wx + (y + s * a + c * b) * wy for a, b in corners]
            order = np.argsort(proj)[::-1]
            a0, b0 = corners[order[0]]
            a1, b1 = corners[order[1]]
            if abs(proj[order[0]] - proj[order[1]]) < 25.0:          # a side lies along the wall
                return _part((a0 + a1) / 2, (b0 + b1) / 2, self.car)
            return _part(a0, b0, self.car)
        px, py = self.box_pts[idx]
        dx, dy = px - x, py - y
        Xb, Yb = c * dx + s * dy, -s * dx + c * dy
        d = _sclear(Xb, Yb, self.car)
        i = int(np.argmin(d))
        return _part(float(Xb[i]), float(Yb[i]), self.car)

    # ---------------------------------------------------------------------------------------------- D6: frames
    def _frames(self):
        rec = self.rec
        if rec is None or not len(rec.frames_n) or not int(self.cfg.get("cam_frames", 1)) or self.offset is None:
            return
        try:
            import cv2
            from . import vision as V
        except Exception as e:                               # no OpenCV: the frame signals are skipped, not the run
            self.notes.append("frames skipped: %s" % e)
            return
        rp = rec.rp or self.run.rp
        cam = dict(rp.get("camera") or {})
        if "intrinsics" not in cam or "h_mm" not in cam:
            self.notes.append("frames skipped: no camera params")
            return
        vp = dict(V.DEFAULT_VISION, **(rp.get("vision") or {}))
        red = 4
        g = None
        n = len(rec.frames_n)
        near = np.zeros(n, bool)
        close = np.zeros(n, bool)
        blocked = np.zeros(n, bool)
        seen_wall = np.zeros(n, np.int32)
        diff = np.full(n, np.nan)
        front = np.full(n, np.nan)
        prev = None
        ok = np.zeros(n, bool)
        vps = None
        for i, fn in enumerate(rec.frames_n):
            if i % 50 == 0:
                self.progress(0.2 + 0.5 * i / max(n, 1), "frames")
            im = cv2.imread(rec.frame_path(fn), cv2.IMREAD_REDUCED_COLOR_4)
            if im is None:
                continue
            H, W = im.shape[:2]
            if g is None or g.W != W or g.H != H:
                g = V.Ground(cam, W, H)
                vps = dict(vp, K=max(2, int(round(vp["K"] / red))), col_step=max(1, int(round(vp["col_step"] / red))))
                hz_f = g.horizon_row()
                g.horizon_row = lambda _h=hz_f: _h              # edges() asks per frame: 90 bisections each (47 % of
                hz = int(np.clip(hz_f, 0, H - 1))               # the frame work, measured on the M4 recording)
            e = V.edges(im, g, vps)
            u = e.u
            cen = (u >= W / 4.0) & (u < 3.0 * W / 4.0)
            near[i] = bool(cen.any() and np.mean((e.cls[cen] == V.WALL) & e.clip[cen]) >= 0.6)
            close[i] = bool(cen.any() and np.mean(e.clip[cen]) >= 0.6)   # anything (a wall, a sign) inside the band
            seen = np.isfinite(e.X) & ~e.clip
            seen_wall[i] = int(np.sum(seen & (e.cls == V.WALL)))
            hsv = cv2.cvtColor(im[hz:], cv2.COLOR_BGR2HSV)
            dark = float(np.mean(hsv[..., 2] < vp["v_dark"])) if hsv.size else 0.0
            blocked[i] = dark >= 0.95 and np.sum(seen) <= 0.05 * len(u)
            fr = e.front(self.car[2])
            front[i] = fr if math.isfinite(fr) else np.nan
            grey = cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (80, 60), interpolation=cv2.INTER_AREA).astype(np.float32)
            if prev is not None:
                diff[i] = float(np.mean(np.abs(grey - prev)))
            prev = grey
            ok[i] = True
        ft = self.mono_rel(rec.frames_t)
        # the still floor of the image difference: frames while the car was commanded 0 (the start, a stop)
        still = [diff[i] for i in range(n) if np.isfinite(diff[i]) and abs(self.v_at(ft[i])) < 1e-3]
        floor = float(np.median(still)) if len(still) >= 5 else 0.5
        self.frame_stats = dict(t=ft, n=rec.frames_n, near=near, close=close, blocked=blocked, seen_wall=seen_wall,
                                diff=diff, front=front, ok=ok, still_thr=max(1.5, 3.0 * floor), floor=floor, cam=cam, vp=vp)

    # ---------------------------------------------------------------------------------------------- D5: scans
    def _scans(self):
        rec = self.rec
        if rec is None or not rec.scans or not self.lidar_loc or self.offset is None:
            return
        rp = rec.rp or self.run.rp
        li = rp.get("lidar") or {}
        pos = li.get("pos_mm") or (0.0, 0.0)
        blind = tuple((float(a), float(b)) for a, b in (li.get("block_deg") or ((130.0, 230.0),)))
        min_mm = float(li.get("min_mm", 60.0))
        max_mm = float(_get(rp, "perc.max_mm", 3000.0))
        ts, clr, brg = [], [], []
        near_cnt = None
        pts_all = []
        for _k, t, res, dist in rec.scans:
            nb = len(dist)
            a = -180.0 + (np.arange(nb) + 0.5) * res
            aa = a % 360.0
            bl = np.zeros(nb, bool)
            for lo, hi in blind:
                if hi - lo >= 360.0:
                    bl[:] = True
                    continue
                lo, hi = lo % 360.0, hi % 360.0
                bl |= ((aa >= lo) & (aa <= hi)) if lo <= hi else ((aa >= lo) | (aa <= hi))
            d = dist.astype(np.float64)
            # a 3-bin median: the minimum over ~360 noisy bins reads 1-2 sigma short (the LD19 +-10 mm, the sim's 6 mm:
            # a 6 mm "touch" where the truth passed clear, SIM 2026-09-25); every real object here spans several bins
            dz = np.where(d > 0, d, np.nan)
            stack = np.vstack([np.roll(dz, 1), dz, np.roll(dz, -1)])
            with np.errstate(all="ignore"):
                med = np.nanmedian(stack, axis=0) if np.isfinite(stack).any() else dz
            d = np.where(np.isfinite(med) & (d > 0), med, d)
            valid = (d > 0) & (d >= min_mm) & (d <= max_mm) & ~bl
            X = float(pos[0]) + d * np.cos(np.radians(a))
            Y = float(pos[1]) + d * np.sin(np.radians(a))
            c = np.where(valid, _sclear(X, Y, self.car), np.inf)
            if near_cnt is None or len(near_cnt) != nb:
                near_cnt = np.zeros(nb)
            near_cnt += c < 8.0
            pts_all.append((t, c, a))
        if not pts_all:
            return
        body = near_cnt > 0.5 * len(pts_all)                 # the car's own parts: near the body in most scans
        for t, c, a in pts_all:
            cc = np.where(body[:len(c)] if len(body) == len(c) else False, np.inf, c)
            i = int(np.argmin(cc))
            ts.append(t)
            clr.append(float(cc[i]))
            brg.append(float(a[i]))
        self.scan_stats = dict(t=self.mono_rel(np.array(ts)), clr=np.array(clr), brg=np.array(brg),
                               body_bins=int(body.sum()), n=len(pts_all))

    # ---------------------------------------------------------------------------------------------- detectors
    def _cand(self, t, det, kind, strong=False, t_end=None, span=False, **ev):
        c = dict(t=float(t), t_end=float(t if t_end is None else t_end), det=det, kind=kind, strong=bool(strong),
                 span=span, ev=ev)
        self.cands.append(c)
        return c

    def _other(self, kind, t, t_end=None, evidence=None, text="", **extra):
        inc = dict(kind=kind, sev=SEV[kind], t=float(t), t_end=float(t if t_end is None else t_end),
                   evidence=list(evidence or []), text=text)
        inc.update(extra)
        self.others.append(inc)
        return inc

    def _near_thr(self, t):
        st = self.state_at(t)
        if st in PARK_STATES:
            return min(float(self.cfg["near_mm"]), self.park_margin - 5.0)
        return float(self.cfg["near_mm"])

    def _spikes(self):
        """D1's raw material: groups of acc deviation >= acc_bump_g (samples <= 0.1 s apart) -> [{t (the peak), dev,
        n (samples >= bump), t0, t1}]."""
        self.spikes = []
        if not len(self.dev):
            return
        thr_b = float(self.cfg["acc_bump_g"])
        t, dev = self.acc_t, self.dev
        for i0, i1 in _runs(dev >= thr_b, t, gap_s=0.1):
            j = i0 + int(np.argmax(dev[i0:i1 + 1]))
            idx = np.arange(i0, i1 + 1)
            second = bool(np.any((idx != j) & (dev[i0:i1 + 1] >= thr_b) & (np.abs(t[i0:i1 + 1] - t[j]) <= 0.1 + 1e-6)))
            self.spikes.append(dict(t=float(t[j]), dev=float(dev[j]), n=int(np.sum(dev[i0:i1 + 1] >= thr_b)),
                                    t0=float(t[i0]), t1=float(t[i1]), second=second))

    def _suspect_intervals(self):
        """Where the believed pose is known to be wrong (so D7 is not evidence there): SUSPECT_S before a `lost`,
        before a relocalisation that moved the pose > pose_jump_mm, after one accepted at cost > reloc_max, and while
        the state is LOST."""
        cfg = self.cfg
        jump_mm = float(cfg["pose_jump_mm"])
        reloc_max = float(self.run.params.get("reloc_max", 0.35) or 0.35)
        iv = []
        self.reloc_jumps = []
        for t, x in self.ev("relocalised"):
            before = self.pose_at(t - 0.4)
            d = None
            if before and isinstance(x.get("x"), (int, float)) and isinstance(x.get("y"), (int, float)):
                d = math.hypot(before[0] - x["x"], before[1] - x["y"])
                self.reloc_jumps.append((t, d, x.get("cost")))
            if d is not None and d > jump_mm:
                iv.append((t - SUSPECT_S, t, "relocalised %.0f mm away at %.1f s" % (d, t)))
            if isinstance(x.get("cost"), (int, float)) and x["cost"] > reloc_max:
                iv.append((t, t + SUSPECT_S, "relocalised at cost %.2f > %.2f at %.1f s" % (x["cost"], reloc_max, t)))
        for t, x in self.ev("lost"):
            iv.append((t - SUSPECT_S, t, "lost at %.1f s (fit %s)" % (t, x.get("fit"))))
        for i0, i1 in _runs(np.array([s == "LOST" for s in self.state_v], bool), self.state_t):
            iv.append((self.state_t[i0], self.state_t[min(i1 + 1, len(self.state_t) - 1)] if i1 + 1 < len(self.state_t)
                       else self.dur, "state LOST"))
        self.suspect_iv = iv

    def suspect(self, t):
        return next((w for a, b, w in getattr(self, "suspect_iv", []) if a <= t <= b), None)

    def d4_push(self):
        """Commanded motion that does not happen: the heading does not follow the steering (the gyro form), the image
        does not change (the A1 with frames: image static and |gz| < 2 deg/s), or the lidar speed stays below
        push_ratio x the command (WLtoys).  Sustained >= push_s -> a push span (strong).

        The gyro form has two shapes.  STUCK: the heading does not move.  WRONG WAY: it turns the other way -- the car
        pivoting on a wall it presses (M5 6.05-8.49 s: +40 expected, -17 deg/s measured, from a 1.08 g hit on) -- or
        the steering / a wheel not following the command with nothing touched (M4 11.7-13.3 s: full lock right, the
        heading +15 deg/s left, the frames moving freely).  A wrong-way span is a push only when it starts with an IMU
        spike (>= acc_bump_g within 0.3 s before .. 0.5 s after its start); otherwise it is reported as an anomaly
        in the causes of what follows."""
        cfg = self.cfg
        pr = float(cfg["push_ratio"])
        spans = []
        self.heading_wrong = []
        # the gyro form, on the command's samples
        if len(self.gz_t) > 2 and len(self.cmd_t) > 2:
            gz = np.interp(self.cmd_t, self.gz_t, self.gz_s)
            e = self.exp
            along = gz * np.sign(e)
            lim = np.maximum(GZ_STILL, pr * np.abs(e))
            base = (np.abs(self.cmd_v) >= V_PUSH) & (np.abs(self.cmd_s) >= STEER_MIN_GYRO) & (np.abs(e) >= GZ_EXP_MIN)
            for i0, i1 in _runs(base & (along < lim), self.cmd_t, gap_s=0.3):
                t0, t1 = self.cmd_t[i0], self.cmd_t[i1]
                if t1 - t0 < float(cfg["push_s"]):
                    continue
                g_med = float(np.median(along[i0:i1 + 1]))
                info = dict(exp_dps=_f(np.median(e[i0:i1 + 1]), 1), gz_dps=_f(np.median(gz[i0:i1 + 1]), 1))
                if g_med < -lim[i0:i1 + 1].mean():
                    hit = [s for s in self.spikes if t0 - 0.3 <= s["t"] <= t0 + 0.5]
                    if not hit:
                        self.heading_wrong.append((t0, t1, info))
                        continue
                    spans.append((t0, t1, "gyro wrong way", info))
                else:
                    spans.append((t0, t1, "gyro", info))
        # the image form (the A1's recorded frames).  A static image alone is no push: driving straight toward or away
        # from a far wall barely changes an 80 x 60 thumbnail (no rotation, sub-pixel flow), and the front camera
        # cannot see a reverse push at all.  So: FORWARD, the command sustained (the lagged v_est, not a shield's
        # 0 / 0.1 m/s creep), and the thing pushed inside the near band (>= 60 % of the central columns clipped, or
        # the lens blocked) within the span or 0.5 s before it.  [SIM 2026-09-25, integration: a clean survey on the
        # stock_a1 mock (truth 0 contacts) read two image-form "pushes" -- a straight LOST back-off at -0.18 m/s and
        # a camshield brake / creep at 0 / 0.1 m/s -- and CONTACT would have stopped a bw mat series]
        fs = self.frame_stats
        if fs is not None and len(fs["t"]) > 2:
            ft = fs["t"]
            vc = _hold(self.cmd_t, self.cmd_v, ft)
            vs = _hold(self.cmd_t, self.v_est, ft)
            gzf = np.interp(ft, self.gz_t, self.gz_s) if len(self.gz_t) > 2 else np.zeros(len(ft))
            dfl = np.where(np.isfinite(fs["diff"]), fs["diff"], np.inf)
            cond = (vc >= V_PUSH) & (vs >= V_PUSH) & (dfl < fs["still_thr"]) & (np.abs(gzf) < GZ_STILL)
            held = fs.get("close", fs["near"]) | fs["blocked"]
            for i0, i1 in _runs(cond, ft, gap_s=0.4):
                t0, t1 = ft[i0], ft[i1]
                w = (ft >= t0 - 0.5) & (ft <= t1)
                if t1 - t0 >= float(cfg["push_s"]) and bool(np.any(held & w)):
                    spans.append((t0, t1, "image", dict(img_diff=_f(np.nanmedian(fs["diff"][i0:i1 + 1]), 2),
                                                        still=_f(fs["still_thr"], 2))))
        # the lidar speed (WLtoys)
        if np.isfinite(self.v_lidar).any():
            vl = self.v_lidar
            cond = (np.abs(self.cmd_v) >= V_PUSH) & np.isfinite(vl) & (np.abs(vl) < pr * np.abs(self.cmd_v))
            for i0, i1 in _runs(cond, self.tel_t, gap_s=0.3):
                t0, t1 = self.tel_t[i0], self.tel_t[i1]
                if t1 - t0 >= float(cfg["push_s"]):
                    spans.append((t0, t1, "lidar", dict(v_lidar=_f(np.nanmedian(vl[i0:i1 + 1]), 3))))
        # union of the forms
        spans.sort(key=lambda s: s[0])
        merged = []
        for s in spans:
            if merged and s[0] <= merged[-1][1] + 0.3:
                a = merged[-1]
                merged[-1] = (a[0], max(a[1], s[1]), a[2] + "+" + s[2] if s[2] not in a[2] else a[2], dict(a[3], **s[3]))
            else:
                merged.append(s)
        self.pushes = merged
        pev = self.ev(PUSH_EVENTS)
        for t0, t1, how, info in merged:
            evs = sorted({x["ev"] for t, x in pev if t0 - 0.5 <= t <= t1 + 1.0})
            self._cand(t0, "D4", "contact", strong=True, t_end=t1, span=True, sig="push", dur_s=_f(t1 - t0, 1),
                       how=how, events=evs, **info)
        # the programs' own blocked / stuck events without a detected push: weak
        for t, x in pev:
            if not any(t0 - 0.5 <= t <= t1 + 1.0 for t0, t1, _h, _i in merged):
                self._cand(t, "D4ev", "contact", sig=x["ev"], by=x.get("by"))

    def d3_stall(self):
        if len(self.stall):
            prev = False
            for i, s in enumerate(self.stall):
                if s and not prev and abs(self.cmd_v[min(i, len(self.cmd_v) - 1)]) > V_MOVE:
                    self._cand(self.tel_t[i], "D3", "contact", strong=True, sig="stall")
                prev = bool(s)
        for t, x in self.ev(("lost", "park_blocked", "creep_blocked")):
            if x.get("by") == "stall":
                self._cand(t, "D3", "contact", strong=True, sig="stall", event=x.get("ev"))

    def d5_lidar(self):
        ss = self.scan_stats
        if ss is None:
            return
        touch, t_ = float(self.cfg["touch_mm"]), ss["t"]
        for i0, i1 in _runs(ss["clr"] <= touch, t_, gap_s=0.25):
            t0 = t_[i0]
            if not self.moving(t0 - 0.3, t_[i1] + 0.1):
                continue
            j = i0 + int(np.argmin(ss["clr"][i0:i1 + 1]))
            n = int(np.sum(ss["clr"][i0:i1 + 1] <= touch))
            # one scan is one noisy look: strong from two scans on (a real touch lasts: the car stops against it)
            self._cand(t0, "D5", "contact", strong=n >= 2, t_end=t_[i1], sig="lidar", clr=_f(ss["clr"][j], 0),
                       bearing=_f(ss["brg"][j], 0), scans=n)
        near_all = ss["clr"] <= float(self.cfg["near_mm"])
        for i0, i1 in _runs(near_all & (ss["clr"] > touch), t_, gap_s=0.25):
            j = i0 + int(np.argmin(ss["clr"][i0:i1 + 1]))
            if ss["clr"][j] > self._near_thr(t_[j]) or not self.moving(t_[i0] - 0.3, t_[i1] + 0.1):
                continue
            self._cand(t_[j], "D5", "near", t_end=t_[i1], sig="lidar", clr=_f(ss["clr"][j], 0),
                       bearing=_f(ss["brg"][j], 0))

    def d6_camera(self):
        fs = self.frame_stats
        if fs is None:
            return
        ft = fs["t"]
        vc = _hold(self.cmd_t, self.cmd_v, ft)
        # lens blocked while driving: strong (the M4 signature)
        for i0, i1 in _runs(fs["blocked"], ft, gap_s=0.3):
            t0, t1 = ft[i0], ft[i1]
            if t1 - t0 < 0.5 or not self.moving(t0 - 1.0, t1):
                continue
            self._cand(t0, "D6", "contact", strong=True, t_end=t1, span=True, sig="lens_blocked", **{"from": _f(t0, 2)},
                       dur_s=_f(t1 - t0, 1), frames=[int(fs["n"][i0]), int(fs["n"][i1])])
        # near wall while moving forward: near / weak contact
        for i0, i1 in _runs(fs["near"] & (vc > V_MOVE), ft, gap_s=0.2):
            if i1 - i0 < 1 and ft[i1] - ft[i0] < 0.1:
                continue
            self._cand(ft[i0], "D6", "near", t_end=ft[i1], sig="near_wall",
                       frames=[int(fs["n"][i0]), int(fs["n"][i1])])

    def d7_map(self):
        p = self.pose
        if p is None:
            self.map_clear = None
            return
        T = self.T
        cfg = self.cfg
        pz = [self.pose_at(t) for t in T]
        have = np.array([q is not None for q in pz])
        x = np.array([q[0] if q else np.nan for q in pz])
        y = np.array([q[1] if q else np.nan for q in pz])
        th = np.array([q[2] if q else np.nan for q in pz])
        sxy = np.array([q[3] if q else np.nan for q in pz])
        clr = np.full(len(T), np.nan)
        idx = np.full(len(T), -99, int)
        m = have & np.isfinite(x)
        if m.any():
            c, k = self.clearance(x[m], y[m], th[m])
            clr[m], idx[m] = c, k
        self.map_clear, self.map_idx = clr, idx
        self.pose_T = (x, y, th, sxy)
        # pose_out: the pose deep inside a wall body (outside the outer walls, inside the island) -- not a pose
        l, r, b, t_ = self.island
        out = have & ((np.maximum(np.abs(x), np.abs(y)) > F.HALF + POSE_OUT_MM) |
                      ((x > l + POSE_OUT_MM) & (x < r - POSE_OUT_MM) & (y > b + POSE_OUT_MM) & (y < t_ - POSE_OUT_MM)))
        self.pose_out = out
        conf = have & np.isfinite(sxy) & (sxy < float(cfg["sxy_bad_mm"])) & ~out
        mov = np.array([self.moving(tt - 0.3, tt) for tt in T])
        st = [self.state_at(tt) for tt in T]
        still = np.array([s in STILL_STATES for s in st])
        sus = np.array([self.suspect(tt) is not None for tt in T])
        self.map_suspect = sus
        use = conf & mov & ~still & ~sus & np.isfinite(clr)
        self.map_use = use
        bad = float(cfg["sxy_bad_mm"])
        # overlap within a pose's error: a weak contact; deeper: the pose, not the car, is inside the wall (D8)
        for i0, i1 in _runs(use & (clr < 0) & (clr >= -bad), T, gap_s=0.2):
            j = i0 + int(np.argmin(clr[i0:i1 + 1]))
            self._cand(T[i0], "D7", "contact", t_end=T[i1], sig="map", clr=_f(clr[j], 0),
                       object=self.element_name(int(idx[j]), x[j], y[j]))
        self.in_wall = conf & mov & ~still & np.isfinite(clr) & (clr < -bad)
        for i0, i1 in _runs(use & (clr >= 0), T, gap_s=0.0):
            seg = clr[i0:i1 + 1]
            thr = np.array([self._near_thr(T[i]) for i in range(i0, i1 + 1)])
            nm = seg < thr
            for a0, a1 in _runs(nm, T[i0:i1 + 1], gap_s=0.2):
                j = i0 + a0 + int(np.argmin(seg[a0:a1 + 1]))
                self._cand(T[j], "D7", "near", t_end=T[i0 + a1], sig="map", clr=_f(clr[j], 0),
                           object=self.element_name(int(idx[j]), x[j], y[j]))

    def d1_acc(self):
        """IMU spikes: dev >= acc_bump_g while the car was commanded to move in the last 0.5 s.  >= acc_contact_g is
        strong only when corroborated: a second sample >= acc_bump_g within 0.1 s, a near obstacle within +-0.5 s
        (D5 / D6 near wall / D7), or a motion change after it (a push overlapping it, the measured speed halving
        within 0.3 s).

        The drive's own jolt (an.jolt_g > 0: the WLtoys): a spike under jolt_g inside a start / brake window of the
        command (_jolt_window) is the stick-slip start or the shorted-motor brake -- [MAT 2026-09-30] ~0.5 g at every
        start from rest, one-two samples, read 'contact'.  It is left out (stats.jolts) unless a near obstacle, a push
        or a stall within 0.5 s says otherwise; then it is judged on those alone."""
        cfg = self.cfg
        thr_c = float(cfg["acc_contact_g"])
        near_c = [c for c in self.cands if (c["det"] in ("D5", "D6", "D7") and c["ev"].get("sig") != "lens_blocked")]
        for sp in self.spikes:
            tj = sp["t"]
            if not self.moving(tj - 0.5, tj):
                continue
            n_hot = sp["n"]
            second = sp["second"]
            why = []
            if second:
                why.append("second sample")
            near = [c for c in near_c if abs(c["t"] - tj) <= 0.5 or (c["t"] - 0.5 <= tj <= c["t_end"] + 0.5)]
            if near:
                why.append("near " + ",".join(sorted({c["ev"].get("sig") for c in near})))
            push = [s for s in getattr(self, "pushes", []) if s[0] - 0.5 <= tj <= s[1] + 0.3]
            if push:
                why.append("push")
            if self._speed_halved(tj):
                why.append("speed halved")
            jolt = self._jolt_window(tj) if self.jolt_g > 0.0 and sp["dev"] < self.jolt_g else None
            if jolt is not None:
                # the drive's own jolt window: a second sample and the speed halving are the jolt's own shape (a
                # stick-slip lurch, a shorted motor biting), not evidence of a wall.  Only what the spike cannot make
                # itself keeps it: a near obstacle, a push, a stall within 0.5 s, the program blocked right after
                stall = [c for c in self.cands if c["det"] == "D3" and abs(c["t"] - tj) <= 0.5]
                blk = sorted({x["ev"] for _t, x in self.ev(PUSH_EVENTS, tj, tj + float(cfg["jolt_block_s"]))})
                indep = [w for w in why if w not in ("second sample", "speed halved")] + (["stall"] if stall else []) \
                    + (["then " + ",".join(blk)] if blk else [])
                if not indep:
                    self.jolts.append(dict(t=_f(tj, 2), g=_f(sp["dev"], 2), after=jolt[0], dt=_f(jolt[1], 2),
                                           n=n_hot))
                    continue
                why = indep
            strong = sp["dev"] >= thr_c and bool(why)
            trans = self._command_step(tj)
            self._cand(tj, "D1", "contact", strong=strong, sig="acc", val=_f(sp["dev"], 2), unit="g",
                       thr=thr_c, n=n_hot, src=self.imu_src, corroborated=why or None,
                       after_command_step=trans or None, in_jolt_window=("%s +%.2f s" % jolt) if jolt else None)

    def _jolt_window(self, t):
        """(kind, seconds after it) when t is inside the drive's own jolt window, else None: `start` -- the command
        went from |v| < V_MOVE to >= V_MOVE (or reversed) at most jolt_start_s before t; `brake` -- it went to
        < V_MOVE, reversed, or was cut to <= BRAKE_CUT x by >= BRAKE_DV at most jolt_brake_s before t.  0.1 s of slack
        before the step: tel samples the command at 20 Hz."""
        if len(self.cmd_t) < 2:
            return None
        s_win, b_win = float(self.cfg["jolt_start_s"]), float(self.cfg["jolt_brake_s"])
        i0 = max(1, int(np.searchsorted(self.cmd_t, t - max(s_win, b_win), side="left")))
        i1 = int(np.searchsorted(self.cmd_t, t + 0.1, side="right"))
        best = None
        for i in range(i0, min(i1, len(self.cmd_t))):
            a, b = abs(float(self.cmd_v[i - 1])), abs(float(self.cmd_v[i]))
            rev = float(self.cmd_v[i - 1]) * float(self.cmd_v[i]) < 0.0 and max(a, b) > V_MOVE
            dt = t - float(self.cmd_t[i])
            kinds = []
            if (a < V_MOVE <= b) or rev:
                kinds.append(("start", s_win))
            if (b < V_MOVE <= a) or rev or (a > V_MOVE and b <= BRAKE_CUT * a and a - b >= BRAKE_DV):
                kinds.append(("brake", b_win))
            for k, win in kinds:
                if -0.1 <= dt <= win and (best is None or abs(dt) < abs(best[1])):
                    best = (k, dt)
        return best

    def _command_step(self, t):
        """A command step just before t (a drive reversal, a steering slam): the transient that looks like a hit."""
        if not len(self.cmd_t):
            return None
        i1 = int(np.searchsorted(self.cmd_t, t, side="right"))
        i0 = int(np.searchsorted(self.cmd_t, t - 0.3, side="left"))
        if i1 - i0 < 2:
            return None
        v, s = self.cmd_v[max(0, i0 - 1):i1], self.cmd_s[max(0, i0 - 1):i1]
        if np.any(np.sign(v[:-1]) * np.sign(v[1:]) < 0):
            return "drive reversal"
        if np.max(np.abs(np.diff(s))) > 15.0:
            return "steering step"
        return None

    def _speed_halved(self, t):
        """The measured speed within 0.3 s after t below half of what it was in the 0.3 s before."""
        if np.isfinite(self.v_lidar).any():
            a = self.v_lidar[(self.tel_t >= t - 0.3) & (self.tel_t <= t)]
            b = self.v_lidar[(self.tel_t > t + 0.05) & (self.tel_t <= t + 0.35)]
            a, b = a[np.isfinite(a)], b[np.isfinite(b)]
            if len(a) and len(b) and np.mean(np.abs(a)) >= 0.05 and np.mean(np.abs(b)) < 0.5 * np.mean(np.abs(a)):
                return True
        fs = self.frame_stats
        if fs is not None:
            ft, d = fs["t"], fs["diff"]
            a = d[(ft >= t - 0.4) & (ft <= t)]
            b = d[(ft > t + 0.05) & (ft <= t + 0.4)]
            a, b = a[np.isfinite(a)], b[np.isfinite(b)]
            if len(a) and len(b) and np.median(a) >= 3.0 * fs["floor"] and np.median(b) < 0.5 * np.median(a):
                return True
        return False

    def d2_gyro(self):
        if len(self.gz_t) < 5 or not len(self.cmd_t):
            return
        e = np.interp(self.gz_t, self.exp_t, self.exp)
        r = self.gz - e
        t = self.gz_t
        per = float(np.median(np.diff(t)))
        k = max(1, int(round(0.1 / max(per, 1e-3))))
        if len(r) <= k:
            return
        jump = np.abs(r[k:] - r[:-k])
        tj = t[k:]
        hot = jump >= float(self.cfg["gyro_jerk_dps"])
        for i0, i1 in _runs(hot, tj, gap_s=0.2):
            j = i0 + int(np.argmax(jump[i0:i1 + 1]))
            tt = float(tj[j])
            if not self.moving(tt - 0.5, tt) or self._command_step(tt):
                continue
            self._cand(tt, "D2", "contact", sig="yaw_jerk", val=_f(jump[j], 0), unit="deg/s")

    def d8_loc(self):
        run, cfg = self.run, self.cfg
        bad = float(cfg["sxy_bad_mm"])
        jump_mm = float(cfg["pose_jump_mm"])
        losses = []
        for t, x in self.ev("lost"):
            # the loss ends at the next accepted relocalisation (a relocalised not followed by reloc_rejected) or
            # when the state leaves LOST
            t_end = self.dur
            for t2, x2 in self.ev("relocalised", t, 1e9):
                rej = self.ev("reloc_rejected", t2, t2 + 0.2)
                if not rej:
                    t_end = t2
                    break
            ev = [dict(sig="lost", fit=x.get("fit"), by=x.get("by"))]
            for t2, x2 in self.ev(("reloc_rejected", "reloc_heading", "relocalised"), t, t_end + 0.01):
                e = dict(sig=x2["ev"], t=_f(t2, 2), cost=x2.get("cost"))
                jm = next((d for tj, d, _c in self.reloc_jumps if tj == t2), None)
                if jm is not None:
                    e["jump_mm"] = _f(jm, 0)
                ev.append(e)
            txt = "lost at %.1f s (fit %s)" % (t, x.get("fit")) + (" by %s" % x["by"] if x.get("by") else "")
            losses.append(self._other("loc_loss", t, t_end, ev, txt))
        for t, d, cost in self.reloc_jumps:
            if d > jump_mm and not any(i["t"] - 0.1 <= t <= i["t_end"] + 0.1 for i in losses):
                losses.append(self._other("loc_loss", t, t, [dict(sig="relocalised", jump_mm=_f(d, 0), cost=cost)],
                                          "the pose jumped %.0f mm at a relocalisation (cost %s)" % (d, cost)))
        # sxy above the bad line for > 1 s while driving
        if self.pose is not None:
            T = self.T
            x, y, th, sxy = self.pose_T
            st = [self.state_at(tt) for tt in T]
            drv = np.array([s not in STILL_STATES and s not in ("LOST", "CREEP") for s in st])
            mov = np.array([self.moving(tt - 0.3, tt) for tt in T])
            outs = [(T[i0], T[i1]) for i0, i1 in _runs(self.pose_out, T, gap_s=5.0)]
            for a, b in outs:
                i0 = int(np.searchsorted(T, a))
                ev = dict(sig="pose_out", at=[_f(x[i0], 0), _f(y[i0], 0)], dur_s=_f(b - a, 1))
                self._other("pose_out", a, b, [ev], "the believed pose left the field (deep inside a wall body from "
                            "(%.0f, %.0f)) for %.1f s: the pose, not the car" % (x[i0], y[i0], b - a))
            m = np.isfinite(sxy) & (sxy > bad) & drv & mov
            for i0, i1 in _runs(m, T, gap_s=0.2):
                if T[i1] - T[i0] < 1.0 or any(i["t"] - 0.5 <= T[i0] <= i["t_end"] + 0.5 for i in losses):
                    continue
                if any(a - 1.0 <= T[i1] and T[i0] <= b + 1.0 for a, b in outs):
                    continue                                  # the same failure as a pose_out
                losses.append(self._other("loc_loss", T[i0], T[i1], [dict(sig="sxy", max_mm=_f(np.nanmax(sxy[i0:i1 + 1]), 0))],
                                          "pose uncertain (sxy up to %.0f mm) for %.1f s" % (np.nanmax(sxy[i0:i1 + 1]), T[i1] - T[i0])))
        for t, x in self.ev("start_check"):
            if x.get("ok") is False:
                self._other("start_mismatch", t, t, [dict(sig="start_check", d_mm=x.get("d_mm"), d_deg=x.get("d_deg"))],
                            "the fix is %s mm / %s deg off the start mark" % (x.get("d_mm"), x.get("d_deg")))
        fx = self.ev("fix")
        if fx and fx[-1][1].get("confident") is False and (run.end or {}).get("reason") in ("no_fix", "boxed_in", "not_standard"):
            t, x = fx[-1]
            self._other("loc_loss", t, t, [dict(sig="fix", p=x.get("p"), why=x.get("why"))],
                        "no confident fix: %s" % (x.get("why") or "p %s" % x.get("p")))
        self.losses = losses

    def d9_interventions(self):
        for name in ("guard", "shield"):
            evs = self.ev(name)
            if name == "shield":
                evs = [(t, x) for t, x in evs if x.get("act") in ("slow", "steer", "brake")]
            groups = []
            for t, x in evs:
                if groups and t - groups[-1][-1][0] <= 1.0:
                    groups[-1].append((t, x))
                else:
                    groups.append([(t, x)])
            for g in groups:
                t0, t1 = g[0][0], g[-1][0]
                if name == "guard":
                    cl = [x.get("clr") for _t, x in g if isinstance(x.get("clr"), (int, float))]
                    txt = "guard x%d: steering %s -> %s deg (clearance %s mm)" % (
                        len(g), g[0][1].get("cmd"), g[0][1].get("steer"), min(cl) if cl else "?")
                    ev = dict(sig="guard", n=len(g), clr_min=min(cl) if cl else None, cmd=g[0][1].get("cmd"),
                              steer=g[0][1].get("steer"))
                else:
                    acts = sorted({x.get("act") for _t, x in g})
                    txt = "shield %s x%d (free %s mm, need %s mm)" % ("/".join(acts), len(g), g[0][1].get("free"),
                                                                      g[0][1].get("need"))
                    ev = dict(sig="shield", n=len(g), acts=acts, free=g[0][1].get("free"), need=g[0][1].get("need"))
                self._other("intervention", t0, t1, [ev], txt)

    def d10_overruns(self):
        cfg, rec = self.cfg, self.rec
        gaps_ms = []
        if rec is not None and self.offset is not None and len(rec.steps["t"]) > 2:
            ts = rec.steps["t"]
            g = np.diff(ts) * 1000.0
            gaps_ms = g
            v = rec.steps["v"]
            tr = self.mono_rel(ts)
            hot = [i for i in np.nonzero(g > float(cfg["loop_gap_ms"]))[0] if abs(np.nan_to_num(v[i])) > V_MOVE]
            last = -1e9
            for i in hot:
                if tr[i] - last < 1.0:
                    continue
                last = tr[i]
                self._other("overrun", tr[i], tr[i + 1], [dict(sig="loop_gap", ms=_f(g[i], 0))],
                            "the control loop stood %.0f ms while driving" % g[i])
        self.step_gaps_ms = np.asarray(gaps_ms, dtype=np.float64)
        if len(self.deadman):
            d = np.nan_to_num(self.deadman)
            for i in np.nonzero(np.diff(d) > 0)[0]:
                # the command in force when it tripped: the tel samples just before the trip.  The 0.6 s window
                # counted every LOCATE search after a LOOK move (the car stopped at 2.01 s, tripped at 2.27 s while
                # standing: 3 false overruns in one survey, integration 2026-09-25)
                if not self.moving(self.tel_t[i] - 0.05, self.tel_t[i]):
                    continue                          # the car was commanded 0 anyway: the dead-man changed nothing
                self._other("overrun", self.tel_t[i + 1], None, [dict(sig="deadman", trips=_f(d[i + 1], 0))],
                            "the dead-man stopped the car (trip %d): a command came late" % int(d[i + 1]))
        if not self.mock:
            for t, x in self.ev("cpu"):
                p99, sh = x.get("loop_p99_ms"), x.get("share")
                if (isinstance(p99, (int, float)) and p99 > 100) or (isinstance(sh, (int, float)) and sh > 0.25):
                    self._other("overrun", t, None, [dict(sig="cpu", loop_p99_ms=p99, share=sh)],
                                "CPU: loop p99 %s ms, share %s (limits 100 ms, 0.25)" % (p99, sh))
        end = self.run.end or {}
        if isinstance(end.get("rec_dropped"), (int, float)) and end["rec_dropped"] > 0:
            self._other("overrun", self.dur, None, [dict(sig="rec_dropped", n=end["rec_dropped"])],
                        "the recorder dropped %d items (SD card too slow)" % end["rec_dropped"])
        if len(self.imu_hz):
            m = (self.imu_hz < 40.0) & (self.tel_t > 1.0) & np.isfinite(self.imu_hz)
            for i0, i1 in _runs(m, self.tel_t, gap_s=0.2):
                if self.tel_t[i1] - self.tel_t[i0] >= 1.0:
                    self._other("overrun", self.tel_t[i0], self.tel_t[i1], [dict(sig="imu_hz", min=_f(np.min(self.imu_hz[i0:i1 + 1]), 1))],
                                "the IMU fell to %.0f Hz" % np.min(self.imu_hz[i0:i1 + 1]))
        if len(self.cam_fps) and np.nanmax(np.nan_to_num(self.cam_fps)) >= 10.0:
            m = (self.cam_fps < 10.0) & (self.tel_t > 2.0) & np.isfinite(self.cam_fps)
            for i0, i1 in _runs(m, self.tel_t, gap_s=0.2):
                if self.tel_t[i1] - self.tel_t[i0] >= 2.0:
                    self._other("overrun", self.tel_t[i0], self.tel_t[i1], [dict(sig="cam_fps", min=_f(np.min(self.cam_fps[i0:i1 + 1]), 1))],
                                "the camera fell to %.1f fps" % np.min(self.cam_fps[i0:i1 + 1]))

    def d11b_camera_stale(self):
        """The camera stopped while the car was told to move: tel.cam_age_ms (hw.snapshot, 2026-09-25) above
        an.cam_stale_ms, or a gap in the recording's frames that long, or the programs' camera_stale / camera_dead
        events.  -> self.cam_stale [(t0, t1, how)] and one `sensor` incident per interval (the events already give
        theirs).  A frozen grabber keeps its last frame: tel.cam_fps used to hold its old rate and nothing saw it."""
        thr = float(self.cfg.get("cam_stale_ms", 300.0)) / 1000.0
        iv = []
        tel = self.run.tel
        if len(tel):
            age = np.array([float(te["cam_age_ms"]) / 1000.0 if isinstance(te.get("cam_age_ms"), (int, float))
                            else np.nan for _t, te in tel])
            # told to move AT that moment (not 0.5 s before: a LOST back-off ends in a stop, and the simulator's renderer
            # starves while the relocalisation search runs -- a sim-only frame gap with the car standing)
            mov = np.abs(self.cmd_v[np.clip(np.searchsorted(self.cmd_t, self.tel_t, side="right") - 1, 0,
                                            max(0, len(self.cmd_v) - 1))]) > V_MOVE if len(self.cmd_t) else                 np.zeros(len(self.tel_t), bool)
            for i0, i1 in _runs(np.isfinite(age) & (age > thr) & mov, self.tel_t, gap_s=0.2):
                iv.append((float(self.tel_t[i0]) - float(age[i0]), float(self.tel_t[i1]), "tel cam_age %.0f ms" % (
                    1000.0 * float(np.nanmax(age[i0:i1 + 1])))))
        rec = self.rec
        has_age = any(isinstance(te.get("cam_age_ms"), (int, float)) for _t, te in tel)
        dropped = float((self.run.end or {}).get("rec_dropped") or 0)
        # the recording's frame gaps only for run files without tel.cam_age_ms (before 2026-09-25) and a recorder that
        # dropped nothing: a slow SD card drops frames and would read as a stopped camera
        if not has_age and not dropped and rec is not None and self.offset is not None and len(rec.frames_t) > 2:
            ft = self.mono_rel(rec.frames_t)
            st_t = self.mono_rel(rec.steps["t"]) if len(rec.steps["t"]) else np.zeros(0)
            g = np.diff(ft)
            for i in np.nonzero(g > max(thr, 0.3))[0]:
                a, b = float(ft[i]), float(ft[i + 1])
                # the control loop went on stepping meanwhile (>= 10 Hz): the CAMERA stopped.  A loop that stood
                # records no frame either -- that is LOOP_STALL (M4 on the Pi: 1.8 s without a frame or a step)
                n_st = int(np.sum((st_t > a) & (st_t < b)))
                if n_st >= 10.0 * (b - a) and abs(self.v_at(a + 0.3)) > V_MOVE and                         (self.state_at(a) or "") not in STILL_STATES + ("LOST",):
                    iv.append((a, b, "no recorded frame for %.1f s" % (b - a)))
        end_t = self.dur
        for t, x in self.ev(("camera_stale", "camera_dead")):
            iv.append((t - float(x.get("age_ms", 0) or 0) / 1000.0 - float(x.get("age_s", 0) or 0), t, x["ev"]))
        iv.sort()
        merged = []
        for a, b, how in iv:
            if merged and a <= merged[-1][1] + 0.5:
                merged[-1] = (merged[-1][0], max(merged[-1][1], b), merged[-1][2])
            else:
                merged.append((a, min(b, end_t), how))
        self.cam_stale = merged
        have_ev = [t for t, _x in self.ev(("camera_stale", "camera_dead"))]
        for a, b, how in merged:
            if not any(a - 0.5 <= t <= b + 0.5 for t in have_ev):
                self._other("sensor", a, b, [dict(sig="camera_stale", how=how, dur_s=_f(b - a, 1))],
                            "the camera stopped while driving (%s)" % how)

    def _pitch_evidence(self, t):
        """Text when the camera view stopped matching the map before t -- a moved camera (pitch) or a wrong pose: the
        seat errors of the last 30 s (median of the last <= 4 > 60 mm, the run's earlier ones below it), or the
        program's own wall fit in GO over the last 10 s above 3x its clean start (>= 30 mm).  None otherwise."""
        errs_all = [(t2, x.get("err")) for t2, x in self.ev("seat", -1e9, t) if isinstance(x.get("err"), (int, float))]
        rec_e = [e for t2, e in errs_all if t2 >= t - 30.0][-4:]
        old_e = [e for t2, e in errs_all if t2 < t - 30.0]
        out = []
        if len(rec_e) >= 2 and float(np.median(rec_e)) > 60.0 and (not old_e or float(np.median(old_e)) <= 60.0):
            out.append("the signs read %s mm off their seats" % " / ".join(str(int(e)) for e in rec_e))
        # losses by the FIT (not a bump / a stall / the shield) again and again after a clean start: the view no longer
        # fits the map where the pose is (sim pitch_jump 2 deg: 4-7 such losses in 30 s, none before the jump)
        lf = [t2 for t2, x in self.ev("lost", -1e9, t) if not x.get("by") or x.get("by") in ("frames", "sxy", "pose_out")]
        go_t = next((t2 for t2, _x in self.ev("direction")), None)
        if go_t is not None and len([q for q in lf if q >= t - 20.0]) >= 2 and not [q for q in lf if q < go_t + 15.0]:
            out.append("lost %d times by the fit in the 20 s before, none in the first 15 s" % len(
                [q for q in lf if q >= t - 20.0]))
        fits = [(self.run.rel(t2), x.get("fit")) for t2, x in self.run.logs if isinstance(x, dict) and "ev" not in x
                and x.get("state") == "GO" and isinstance(x.get("fit"), (int, float))]
        if fits:
            base = [f for t2, f in fits if t2 <= fits[0][0] + 20.0 and t2 < t - 10.0]
            now = [f for t2, f in fits if t - 10.0 <= t2 <= t]
            if len(base) >= 5 and len(now) >= 3:
                b0, n0 = float(np.median(base)), float(np.median(now))
                if n0 >= 30.0 and n0 > 3.0 * b0:
                    out.append("the wall fit grew from %.0f to %.0f mm" % (b0, n0))
        return "; ".join(out) or None

    def d11_sensors(self):
        for t, x in self.ev(SENSOR_EVENTS):
            self._other("sensor", t, None, [dict(sig=x["ev"], **{k: v for k, v in x.items() if k in ("age_s", "error", "state", "n")})],
                        "%s%s" % (x["ev"].replace("_", " "), (": " + str(x.get("error"))) if x.get("error") else ""))
        if len(self.board_err):
            for i in np.nonzero(np.diff(self.board_err) > 0)[0]:
                self._other("sensor", self.tel_t[i + 1], None, [dict(sig="board_error", n=_f(self.board_err[i + 1], 0))],
                            "the RRC board refused a command (error %d)" % int(self.board_err[i + 1]))
        for t, e in self.run.errors:
            last = [l for l in str(e).strip().splitlines() if l.strip()]
            self._other("error", self.run.rel(t), None, [dict(sig="error")], last[-1].strip()[:200] if last else "error")

    def d12_battery(self):
        cfg = self.cfg
        b = self.bat
        if not len(b) or not np.isfinite(b).any():
            self.battery = None
            return
        t, v = self.tel_t, self.cmd_v
        first = (t <= 2.0) & np.isfinite(b)
        rest_m = first & (np.abs(v) < 1e-3)
        rest = float(np.median(b[rest_m])) if rest_m.any() else float(np.median(b[first])) if first.any() else \
            float(b[np.isfinite(b)][0])
        load = np.isfinite(b) & (np.abs(v) > V_MOVE)
        mn = float(np.min(b[load])) if load.any() else float(np.nanmin(b))
        end = float(b[np.isfinite(b)][-1])
        sag = rest - mn
        self.battery = dict(rest_v=round(rest, 2), min_v=round(mn, 2), end_v=round(end, 2), sag_v=round(sag, 2))
        why = []
        if rest < float(cfg["bat_rest_v"]):
            why.append("rest %.2f V < %.2f" % (rest, float(cfg["bat_rest_v"])))
        if mn < float(cfg["bat_min_v"]):
            why.append("min %.2f V < %.2f under load" % (mn, float(cfg["bat_min_v"])))
        if sag > float(cfg["bat_sag_v"]):
            why.append("sag %.2f V > %.2f" % (sag, float(cfg["bat_sag_v"])))
        if why:
            tm = float(t[load][np.argmin(b[load])]) if load.any() else 0.0
            self._other("battery", tm, None, [dict(sig="battery", **self.battery)], "battery: " + "; ".join(why))

    def d13_truth(self):
        tr = self.truth
        self.truth_contacts = []
        if tr is None:
            return
        c = tr["contacts"]
        for i in np.nonzero(np.diff(c) > 0)[0]:
            self.truth_contacts.append(float(tr["t"][i + 1]))

    # ---------------------------------------------------------------------------------------------- fusion
    def _fuse(self):
        cfg = self.cfg
        ms = float(cfg["merge_s"])
        pts = sorted([c for c in self.cands if not c["span"]], key=lambda c: c["t"])
        spans = sorted([c for c in self.cands if c["span"]], key=lambda c: c["t"])
        clusters = []
        for c in pts:
            if clusters and c["t"] - clusters[-1]["t_last"] <= ms:
                cl = clusters[-1]
                cl["c"].append(c)
                cl["t_last"] = max(cl["t_last"], c["t"])
                cl["t_end"] = max(cl["t_end"], c["t_end"])
            else:
                clusters.append(dict(c=[c], t_first=c["t"], t_last=c["t"], t_end=c["t_end"]))
        for s in spans:
            # the span joins the cluster whose points are nearest its start: from merge_s before the first point to
            # 1 s after the last (a hit, then the lens pressed on the wall, then the push), or one whose spans it
            # continues; else it is an incident of its own
            best, best_d = None, None
            for cl in clusters:
                if cl["t_first"] - ms <= s["t"] <= cl["t_last"] + 1.0:
                    d = 0.0 if cl["t_first"] <= s["t"] <= cl["t_last"] else min(abs(s["t"] - cl["t_first"]),
                                                                               abs(s["t"] - cl["t_last"]))
                elif any(c["span"] and c["t"] - ms <= s["t"] <= c["t_end"] + 1.0 for c in cl["c"]):
                    d = 0.5
                else:
                    continue
                if best is None or d < best_d or (d == best_d and self._contactish(cl) and not self._contactish(best)):
                    best, best_d = cl, d
            if best is None:
                clusters.append(dict(c=[s], t_first=s["t"], t_last=s["t"], t_end=s["t_end"]))
                clusters.sort(key=lambda cl: cl["t_first"])
            else:
                best["c"].append(s)
                best["t_end"] = max(best["t_end"], s["t_end"])
        incs = []
        for cl in clusters:
            cs = cl["c"]
            strong = [c for c in cs if c["strong"]]
            weak_dets = {c["det"] for c in cs if not c["strong"] and c["kind"] == "contact" and c["det"] != "D1"}
            d1w = [c for c in cs if c["det"] == "D1" and not c["strong"]]
            if d1w:
                weak_dets.add("D1")
            nears = [c for c in cs if c["kind"] == "near"]
            if strong or len(weak_dets) >= 2 or (weak_dets - {"D1", "D2"} and nears and any(
                    c["det"] != n["det"] for c in cs if c["kind"] == "contact" for n in nears)):
                kind = "contact"
            elif nears or weak_dets - {"D1", "D2", "D4ev"}:
                kind = "near"
            elif weak_dets & {"D1", "D2"}:
                kind = "bump"
            elif weak_dets:
                kind = "near"
            else:
                continue
            # the incident's time: the first strong IMU spike when there is one (the moment of the hit; re-touches of a
            # push chain after it), else the first strong point signal, else the first strong span's start, else the
            # first contact-type signal
            d1s = [c for c in strong if c["det"] == "D1"]
            pts_s = [c for c in strong if not c["span"]]
            key = d1s or pts_s or strong or [c for c in cs if c["kind"] == "contact"] or cs
            t = min(c["t"] for c in key)
            incs.append(dict(kind=kind, sev=SEV[kind], t=t, t_end=max(cl["t_end"], t), cands=cs))
        # near misses within 1 s of a contact, and anything inside a contact's span (the push that follows a hit),
        # merge into it
        out = []
        for inc in incs:
            if inc["kind"] != "contact":
                host = next((o for o in incs if o is not inc and o["kind"] == "contact" and (
                    (inc["kind"] == "near" and o["t"] - 1.0 <= inc["t"] <= o["t_end"] + 1.0) or
                    (o["t"] <= inc["t"] <= o["t_end"]))), None)
                if host is not None:
                    host["cands"] = host["cands"] + inc["cands"]
                    continue
            else:
                host = next((o for o in out if o["kind"] == "contact" and o["t"] < inc["t"] <= o["t_end"]
                             and not any(c["det"] == "D1" and c["strong"] for c in inc["cands"])), None)
                if host is not None:
                    host["cands"] = host["cands"] + inc["cands"]
                    host["t_end"] = max(host["t_end"], inc["t_end"])
                    continue
            out.append(inc)
        for inc in out:
            inc["evidence"] = self._evidence(inc["cands"])
        return out

    @staticmethod
    def _contactish(cl):
        return any(c["strong"] or c["kind"] == "contact" for c in cl["c"])

    @staticmethod
    def _evidence(cands):
        ev = []
        seen = set()
        for c in sorted(cands, key=lambda c: (not c["strong"], c["t"])):
            e = {k: v for k, v in c["ev"].items() if v is not None}
            e["det"] = c["det"]
            e["t"] = _f(c["t"], 2)
            if c["strong"]:
                e["strong"] = True
            key = (e.get("sig"), e.get("t"))
            if key in seen:
                continue
            seen.add(key)
            ev.append(e)
        return ev

    # ---------------------------------------------------------------------------------------------- where / why
    def _where(self, inc):
        t = inc["t"]
        pz = self.pose_at(t)
        w = dict(x=None, y=None, th=None, sxy=None, section=None, object=None, part=None)
        cands = inc.get("cands") or []
        lid = next((c for c in cands if c["det"] == "D5"), None)
        cam = next((c for c in cands if c["det"] == "D6"), None)
        suspect = None
        tr0, pz0 = self.truth_why(t), self.pose_at(t)
        truth_ok = bool(tr0 and pz0 and math.hypot(tr0[0] - pz0[0], tr0[1] - pz0[1]) <= 60.0)
        if self.suspect(t) and not truth_ok:
            suspect = "pose suspect: %s" % self.suspect(t)
        elif inc.get("_loc_text") and not truth_ok:
            suspect = inc["_loc_text"]
            if len(suspect) > 70:
                suspect = suspect[:67].rsplit(" ", 1)[0] + "..."
        if pz is None:
            w["object"] = "unknown (no pose)"
            if lid is not None and lid["ev"].get("bearing") is not None:
                w["part"] = _bearing_part(lid["ev"]["bearing"])
            elif cam is not None:
                w["part"] = "front"
            return w
        x, y, th, sxy = pz
        w.update(x=_f(x, 0), y=_f(y, 0), th=_f(th, 1), sxy=_f(sxy, 0), section=_sec_name(x, y))
        clr, idx = self.clearance([x], [y], [th])
        clr, idx = float(clr[0]), int(idx[0])
        name = self.element_name(idx, x, y)
        inc["_obj"] = (idx, clr, name)
        bad = float(self.cfg["sxy_bad_mm"])
        unsure = (not np.isfinite(sxy)) or sxy >= bad or bool(suspect)
        slack = 60.0 + 2.0 * (sxy if np.isfinite(sxy) else 0.0) + abs(self.v_at(t)) * 250.0     # as UNKNOWN_OBSTACLE
        if inc["kind"] == "contact" and not unsure and clr > slack:
            name = "unknown (not in the map)"
        if unsure:
            tag = suspect or ("pose +-%.0f mm" % sxy if np.isfinite(sxy) else "pose uncertain")
            name = "%s? (%s)" % (name, tag)
        w["object"] = name
        if lid is not None and lid["ev"].get("bearing") is not None and (unsure or inc["kind"] != "contact"):
            w["part"] = _bearing_part(lid["ev"]["bearing"])
        elif not unsure:
            w["part"] = self.nearest_part(idx, x, y, th)
        elif cam is not None:
            w["part"] = "front"
        return w

    def _loc_trigger(self, t):
        """(trigger text, sub list) when the pose was off at t (LOC_POSE_OFF), else (None, [])."""
        if self.pose is None:
            return None, []
        cfg = self.cfg
        bad = float(cfg["sxy_bad_mm"])
        jump_mm = float(cfg["pose_jump_mm"])
        reloc_max = float(self.run.params.get("reloc_max", 0.35) or 0.35)
        tr0, pz0 = self.truth_why(t), self.pose_at(t)
        if tr0 and pz0 and math.hypot(tr0[0] - pz0[0], tr0[1] - pz0[1]) <= 60.0:
            return None, []                   # (an.why_truth 1 only) the simulator KNOWS the pose was right
        parts = []
        for t2, x in self.ev(("lost", "reloc_rejected", "reloc_heading"), t - 3.0, t):
            parts.append("%s at %.1f s%s" % (x["ev"], t2, " (fit %s)" % x.get("fit") if x.get("fit") is not None else ""))
        for t2, d, cost in self.reloc_jumps:
            if t - 3.0 <= t2 <= t and d > jump_mm:
                parts.append("relocalised at %.1f s at cost %s%s, the pose jumped %.0f mm" % (
                    t2, cost, " (> %.2f)" % reloc_max if isinstance(cost, (int, float)) and cost > reloc_max else "", d))
            if t < t2 <= t + 3.0 and d > jump_mm:
                parts.append("relocalised %.0f mm away at %.1f s (cost %s)" % (d, t2, cost))
        po = [i for i in self.others if i["kind"] == "pose_out" and i["t"] - 3.0 <= t <= i["t_end"]]
        if po and po[0]["t"] <= t:
            parts.append("the pose was inside a wall")
        pz = self.pose_at(t)
        if pz and np.isfinite(pz[3]) and pz[3] > bad:
            parts.append("sxy %.0f mm" % pz[3])
        # held while the pose drove on: a push (the program's stuck events, D4's spans) in the 5 s before -- the pose
        # integrates the command through the wall (sim a1_f_wall: 225 mm off at the first contact, then outside the
        # field; without the truth this read UNKNOWN_OBSTACLE and TURN_RADIUS)
        held = [(t2, x["ev"]) for t2, x in self.ev(("stuck", "stuck_gyro"), t - 5.0, t)
                if (self.state_at(t2) or "GO") == "GO"]
        held += [(a, "push") for a, b, _h, _i in getattr(self, "pushes", []) if a < t - 0.3 and b >= t - 5.0]
        if held:
            parts.append("the car was held (%s at %.1f s) while the pose drove on" % (held[0][1], held[0][0]))
        tr = self.truth_why(t)
        if tr and pz and math.hypot(tr[0] - pz[0], tr[1] - pz[1]) > 60.0:
            parts.append("%.0f mm off the truth" % math.hypot(tr[0] - pz[0], tr[1] - pz[1]))
        if not parts:
            return None, []
        after = [(t2, x) for t2, x in self.ev("lost", t, t + 3.0)]
        if after:
            parts.append("lost again at %.1f s (fit %s)" % (after[0][0], after[0][1].get("fit")))
        # sub-evidence: the camera pitch, accepted poor relocalisations, the start
        sub = []
        errs = [x.get("err") for _t, x in self.ev("seat", t - 30.0, t) if isinstance(x.get("err"), (int, float))][-4:]
        pitch0 = _get(self.run.rp, "camera.pitch_deg")
        pc = [x for _t, x in self.ev(("pitch_selfcal", "pitch_final"))]
        pitch_off = [abs(float(x.get("fitted", x.get("pitch", pitch0))) - float(pitch0)) for x in pc
                     if isinstance(pitch0, (int, float)) and isinstance(x.get("fitted", x.get("pitch")), (int, float))]
        if errs and float(np.median(errs)) > 60.0:
            sub.append("camera pitch: seat errors %s mm" % " / ".join(str(int(e)) for e in errs[:6]))
        elif pitch_off and max(pitch_off) > 0.5:
            sub.append("camera pitch: the fit moved it %.1f deg from params" % max(pitch_off))
        if not self._camera_verified():
            sub.append("camera NOT verified this session")
        acc = [x.get("cost") for _t, x in self.ev("relocalised") if isinstance(x.get("cost"), (int, float))
               and x["cost"] > reloc_max]
        if acc:
            sub.append("reloc accepted: %s > %.2f" % (", ".join("%.2f" % c for c in acc), reloc_max))
        ss = self.ev(("start_search", "fix"))
        if ss:
            x = ss[0][1]
            c, h = x.get("cost"), x.get("hypotheses", len(x.get("hyps") or []) or None)
            if (isinstance(c, (int, float)) and c > 0.3) or (isinstance(h, int) and h > 1):
                sub.append("start: %s hypotheses, cost %s" % (h, c))
        return "the pose was off: " + "; ".join(parts), sub

    def _camera_verified(self):
        vt = _get(self.run.rp, "camera.verified_t")
        ag = next((c.get("agent_started") for _t, c in self.run.clocks if c.get("agent_started")), None)
        return bool(isinstance(vt, (int, float)) and vt > 0 and (ag is None or vt >= float(ag)))

    def _cause(self, inc):
        t = inc["t"]
        cfg = self.cfg
        codes, texts = [], {}
        # the camera stopped before it (a frozen grabber, a dead USB): the pose then ran on dead reckoning
        cs_ = [iv for iv in getattr(self, "cam_stale", []) if iv[0] - 0.2 <= t <= iv[1] + 3.0]
        if cs_:
            codes.append("CAMERA_STALE")
            texts["CAMERA_STALE"] = "the camera stopped %.1f s before (%s): the pose ran on dead reckoning" % (
                max(0.0, t - cs_[0][0]), cs_[0][2])
        loc_text, sub = self._loc_trigger(t)
        if loc_text:
            codes.append("LOC_POSE_OFF")
            texts["LOC_POSE_OFF"] = loc_text
        pe_ = self._pitch_evidence(t)
        if pe_:
            codes.append("CAMERA_PITCH")
            texts["CAMERA_PITCH"] = "the camera view stopped matching the map: %s -- a moved camera (pitch) or a "                                     "wrong pose" % pe_
        # held BEFORE it (>= 0.3 s) while map-localised (GO): the pose drove on through what held the car.  The push
        # that follows a hit is its consequence (M5 6.05 s), and lapcam has no pose (its pushes are TURN_RADIUS's)
        held_ = []
        if self.run.program != "lapcam":
            held_ = [(t2, x["ev"]) for t2, x in self.ev(("stuck", "stuck_gyro"), t - 3.0, t - 0.3)
                     if (self.state_at(t2) or "GO") == "GO"]
            held_ += [(a, "push %.1f s" % (b - a)) for a, b, _h, _i in getattr(self, "pushes", [])
                      if a <= t - 0.3 <= b + 0.5 and (self.state_at(a) or "GO") == "GO"]
        if held_:
            codes.append("PUSH")
            texts["PUSH"] = "commanded motion that did not happen: %s at %.1f s -- the car pressed on something" % (
                held_[0][1], held_[0][0])
        v, ve = self.v_at(t), self.vest_at(t)
        lid = next((c for c in inc["cands"] if c["det"] == "D5"), None)
        blind = False
        if lid is not None and lid["ev"].get("bearing") is not None:
            li = self.run.rp.get("lidar") or {}
            b = lid["ev"]["bearing"] % 360.0
            for lo, hi in (li.get("block_deg") or ((130.0, 230.0),)):
                lo, hi = lo % 360.0, hi % 360.0
                blind = blind or ((lo <= b <= hi) if lo <= hi else (b >= lo or b <= hi))
        if (v < -0.02 or ve < -0.02) and (not self.lidar_loc or blind):
            codes.append("REVERSE_BLIND")
            texts["REVERSE_BLIND"] = ("reversing blind (%s)" % ("the A1 sees nothing behind" if not self.lidar_loc
                                                                 else "the WLtoys hides its rear sector"))
        obj = inc.get("_obj")
        pz = self.pose_at(t)
        # "nothing in the map within 60 mm" -- plus what the pose and the moment can be off by: 2 sxy and 0.25 s of
        # travel (SIM 2026-09-25: a sign clipped at 0.26 m/s read 73 mm clear at the IMU's instant with a 17 mm pose)
        slack = 60.0 + (2.0 * pz[3] if pz and np.isfinite(pz[3]) else 0.0) + abs(v) * 250.0
        if inc["kind"] == "contact" and obj and not loc_text and pz and np.isfinite(pz[3]) and pz[3] < float(cfg["sxy_bad_mm"]) \
                and obj[1] > slack:
            codes.append("UNKNOWN_OBSTACLE")
            texts["UNKNOWN_OBSTACLE"] = ("something not in the map at (%.0f, %.0f): a sign off its seat, a non-standard "
                                         "field, an object on the mat" % (pz[0], pz[1]))
            pin = [b - a for a, b, _h, _i in getattr(self, "pushes", []) if t - 0.5 <= a <= t + 2.0]
            po = [i for i in self.others if i["kind"] == "pose_out" and t <= i["t"] <= t + 120.0]
            if (pin and max(pin) >= 5.0) or po:
                texts["UNKNOWN_OBSTACLE"] += " -- OR the pose was off: %s" % (
                    "the car stayed pinned %.0f s after it" % max(pin) if pin and max(pin) >= 5.0
                    else "the believed pose left the field at %.1f s" % po[0]["t"])
        if obj and obj[0] >= 0 and self.boxes[obj[0]][4] == "sign":
            s = self._sign_of(obj[0])
            why = []
            if s is not None:
                if s[3] > t - 1.0:
                    why.append("colour known only %.1f s before" % (t - s[3]) if s[3] <= t else "colour unknown then")
                if s[4] is not None and t - 1.0 <= s[4] <= t + 1.0:
                    why.append("colour changed at %.1f s" % s[4])
                if pz and s[2] in ("red", "green"):
                    c, sn = math.cos(math.radians(pz[2])), math.sin(math.radians(pz[2]))
                    Xc, Yc = c * (s[0] - pz[0]) + sn * (s[1] - pz[1]), -sn * (s[0] - pz[0]) + c * (s[1] - pz[1])
                    if -self.car[0] - 50 <= Xc <= self.car[1] + 50 and ((s[2] == "red" and Yc < 0) or (s[2] == "green" and Yc > 0)):
                        why.append("passed on the wrong side")
            if why:
                codes.append("SIGN_SIDE")
                texts["SIGN_SIDE"] = "%s: %s" % (self.boxes[obj[0]][5], ", ".join(why))
        # full lock
        mx = self.steer_max
        fl = self._full_lock_for(t)
        st = self.state_at(t) or ""
        corner = pz is not None and F.where(pz[0], pz[1])[0] == "corner"
        guard_tight = [x for _t, x in self.ev("guard", t - 1.0, t)
                       if isinstance(x.get("clr"), (int, float)) and x["clr"] < float(self.run.params.get("guard_mm", 40) or 40)]
        # full lock into what is AHEAD also counts outside the corners: a dodge in a straight that R_eff cannot close
        # (SIM 2026-09-25, stock A1 at R 505: the nose into the N / W outer wall after passing a sign, both at full lock)
        front_hit = str((inc.get("where") or {}).get("part") or "").startswith("front")
        if (fl >= 0.4 and (corner or st in ("TURN", "CORNER") or self.run.program == "lapcam" or front_hit)
                and st not in PARK_STATES) or guard_tight:
            r_eff = self.L / max(math.tan(math.radians(mx)), 1e-6)
            codes.append("TURN_RADIUS")
            texts["TURN_RADIUS"] = ("turning at full lock for %.1f s (R_eff %.0f mm by chassis.wheelbase_m %.3f) and "
                                    "still not clear" % (fl, r_eff, self.L / 1000.0)
                                    if fl >= 0.4 else "the guard found only %s mm" % guard_tight[-1].get("clr"))
        sh = [x for _t, x in self.ev("shield", t - 0.5, t) if isinstance(x.get("need"), (int, float))
              and isinstance(x.get("free"), (int, float)) and x["need"] > x["free"]]
        vc_lim = self.run.params.get("v_corner", self.run.params.get("v_turn"))
        if sh:
            codes.append("TOO_FAST")
            texts["TOO_FAST"] = "v %.2f m/s needs %s mm to stop, %s mm were free" % (abs(v), sh[-1]["need"], sh[-1]["free"])
        elif isinstance(vc_lim, (int, float)) and (corner or st in ("TURN", "CORNER")) and abs(v) > float(vc_lim) + 0.02:
            codes.append("TOO_FAST")
            texts["TOO_FAST"] = "v %.2f m/s in the corner above v_corner %.2f" % (abs(v), float(vc_lim))
        stall = [i for i in self.others if i["kind"] == "overrun" and i["evidence"] and
                 i["evidence"][0].get("sig") in ("loop_gap", "deadman") and t - 0.5 <= i["t"] <= t]
        if stall:
            codes.append("LOOP_STALL")
            texts["LOOP_STALL"] = stall[0]["text"]
        gs = [x for _t, x in self.ev("guard", t - 1.0, t) if isinstance(x.get("clr"), (int, float))
              and isinstance(x.get("clr_cmd"), (int, float)) and x["clr"] < 0 and x["clr_cmd"] < 0]
        if gs:
            codes.append("GUARD_SATURATED")
            texts["GUARD_SATURATED"] = "the guard found no free steering (clearance %s mm): the plan passes too close here" % gs[-1]["clr"]
        if st in PARK_STATES:
            codes.append("PARK_LEG")
            pk = [x for _t, x in self.ev(("approach", "park_lane", "parked", "exited"), -1e9, t)]
            texts["PARK_LEG"] = "park leg (%s)%s" % (st, (": " + json.dumps(pk[-1])[:80]) if pk else "")
        cb = self._camera_blind_before(t)
        stale = self.ev("camera_stale", t - 3.0, t)
        if cb >= 1.0 or stale:
            codes.append("CAMERA_BLIND")
            texts["CAMERA_BLIND"] = "the camera saw no wall base for %.1f s" % cb if cb >= 1.0 else "camera_stale"
        sl = [i for i in self.others if i["kind"] in ("sensor", "error") and t - 1.0 <= i["t"] <= t]
        if sl:
            codes.append("SENSOR_LOSS")
            texts["SENSOR_LOSS"] = sl[0]["text"]
        if not codes:
            codes.append("UNEXPLAINED")
            texts["UNEXPLAINED"] = "no rule explains it: look at the frames and the timeline"
        step = next((c["ev"].get("after_command_step") for c in inc["cands"] if c["det"] == "D1"
                     and c["ev"].get("after_command_step")), None)
        if step and inc["kind"] == "bump":
            # [MAT 2026-09-23 M5] the same shape 0.05 s after both back-off -> forward switches: 0.54 g at 9.80 s,
            # 0.29 g at 13.62 s, lateral, decaying over 0.2 s, the heading free right after -- a command transient
            for c_ in codes:
                texts[c_] = texts[c_] + "; or the transient of the %s 0.3 s before it" % step
        code = codes[0]
        sub = list(sub) if code == "LOC_POSE_OFF" else []
        # the heading turning against the command with nothing touched, shortly before (D4's wrong-way anomaly)
        for a, b, info in getattr(self, "heading_wrong", []):
            if a - 0.5 <= t <= b + 2.0:
                e, g = info.get("exp_dps") or 0.0, info.get("gz_dps") or 0.0
                sub.append("the heading turned %s at %.0f deg/s while the command asked %.0f deg/s %s (%.1f-%.1f s): "
                           "the steering or a wheel did not follow" % ("left" if g > 0 else "right", abs(g), abs(e),
                                                                       "left" if e > 0 else "right", a, b))
        inc["cause"] = dict(code=code, text=texts[code], also=codes[1:], sub=sub)
        inc["fix_hint"] = self._fix_hint(code, inc, sub)

    def _sign_of(self, bi):
        cx, cy = self.boxes[bi][0], self.boxes[bi][1]
        return next((s for s in self.signs if abs(s[0] - cx) < 1 and abs(s[1] - cy) < 1), None)

    def _full_lock_for(self, t):
        """Seconds the steering stood at full lock (>= max - 1 deg) up to t."""
        if not len(self.cmd_t):
            return 0.0
        i = int(np.searchsorted(self.cmd_t, t, side="right")) - 1
        if i < 0 or abs(self.cmd_s[i]) < self.steer_max - 1.0:
            return 0.0
        sgn = np.sign(self.cmd_s[i])
        j = i
        while j > 0 and abs(self.cmd_s[j - 1]) >= self.steer_max - 1.0 and np.sign(self.cmd_s[j - 1]) == sgn:
            j -= 1
        return float(t - self.cmd_t[j])

    def _camera_blind_before(self, t):
        fs = self.frame_stats
        if fs is None:
            return 0.0
        ft = fs["t"]
        i = int(np.searchsorted(ft, t, side="right")) - 1
        if i < 0 or fs["seen_wall"][i] > 0:
            return 0.0
        j = i
        while j > 0 and fs["seen_wall"][j - 1] == 0:
            j -= 1
        return float(t - ft[j])

    def _fix_hint(self, code, inc, sub):
        if code == "LOC_POSE_OFF":
            h = []
            if any(s.startswith("camera pitch") for s in sub):
                h.append("bw preflight pitch --field (%s say the pitch is off)" % next(
                    s for s in sub if s.startswith("camera pitch")).split(": ", 1)[1])
            elif any(s.startswith("camera NOT") for s in sub):
                h.append("bw preflight pitch --field (the camera was not verified this session)")
            if any(s.startswith("reloc accepted") for s in sub):
                h.append("reloc_max 0.35 gates relocalisations since 2026-09-23: check the program is the new one")
            if any(s.startswith("start") for s in sub):
                h.append("start from a taped mark with expect= (bw mat) so a two-way start search cannot pick the twin")
            return "; ".join(h) or "the pose was off: check the camera (bw preflight pitch --field) and the lost / relocalised events"
        r_eff = self.L / max(math.tan(math.radians(self.steer_max)), 1e-6)
        hints = {
            "CAMERA_STALE": "the camera stopped: its cable / USB, `bw sys restart_camera`, the camera's fps in bw status; "
                            "wro_next stands after camera_stale_s and ends camera_dead after camera_dead_s",
            "CAMERA_PITCH": "bw preflight pitch --field, then plans/a1_check.json on the mark (SUNDAY steps 2 + 3): the "
                            "camera may have moved -- or the pose was off (the lost / relocalised events)",
            "PUSH": "the car pressed on something the pose did not show: the frames before it, the pose (lost / "
                    "relocalised), bump_lost / push_img / repeat_mm",
            "REVERSE_BLIND": "never reverse blind on the A1: a shorter back-off (backoff_mm) or a three-point back-off",
            "UNKNOWN_OBSTACLE": "something not in the map: check the mat at the report's (x, y) (a sign off its seat, "
                                "a non-standard field, an object)",
            "SIGN_SIDE": "the sign's colour came late or flipped: check the HSV at that distance (TESTS -> camera) or "
                         "slow down before the sign (v)",
            "TURN_RADIUS": "the turn needs more room than R_eff %.0f mm: start it earlier or wider (%s), and check "
                           "chassis.wheelbase_m (TESTS -> turn_radius)%s" % (
                               r_eff, "r_turn / the turn trigger" if self.run.program == "lapcam" else "r_corner / lane",
                               "; the stock A1 measured 0.28 effective on the mat (R ~505 mm, 2026-09-23), not %.3f"
                               % (self.L / 1000.0) if (not self.wltoys and self.L < 200.0) else ""),
            "TOO_FAST": "slow down here (v_corner / v_turn), or give the shield more room",
            "LOOP_STALL": "the control loop stood: read the cpu event (share, loop p99) and stop what else ran",
            "GUARD_SATURATED": "the plan passes too close here: lane_pass / pass_half / guard_mm",
            "PARK_LEG": "the park leg touched: park_margin / the lot fit (lot events)",
            "CAMERA_BLIND": "the camera saw no wall base: the pitch (bw preflight pitch --field), the exposure, the lens",
            "SENSOR_LOSS": "a sensor dropped out: cables, the lidar's power, bw sys",
            "UNEXPLAINED": "look at the frames in the report and the timeline around t",
        }
        return hints.get(code, "")

    def _decision(self, inc):
        t = inc["t"]
        st = self.state_at(t) or "?"
        v, s = self.v_at(t), self.s_at(t)
        prog = self.run.program or ""
        mx = self.steer_max
        head = st if prog != "lapcam" else "%s (lapcam)" % st
        what = {"GO": "pure pursuit", "CREEP": "creeping to see", "LOST": "backing off / searching",
                "APPROACH": "park approach leg", "PARK": "park leg", "EXIT": "lot exit leg", "SETTLE": "settling",
                "LOCATE": "locating (never moves)", "LOOK": "a LOOK move"}.get(st, "")
        parts = [("%s: %s" % (head, what)) if what else head + ":"]
        parts.append("%s %s at %.2f m/s" % ("reversing" if v < 0 else "driving", _steer_text(s, mx), abs(v))
                     if abs(v) > 1e-3 else "stopped, " + _steer_text(s, mx))
        g = self.ev("guard", t - 1.0, t + 0.2)
        if g:
            x = g[-1][1]
            parts.append("the guard moved %s -> %s deg (clearance %s mm)" % (x.get("cmd"), x.get("steer"), x.get("clr")))
        sh = self.ev("shield", t - 1.0, t + 0.2)
        if sh:
            x = sh[-1][1]
            parts.append("the shield: %s (free %s mm, need %s mm)" % (x.get("act"), x.get("free"), x.get("need")))
        if prog == "lapcam":
            c = self.ev("corner", t - 15.0, t)
            if c:
                x = c[-1][1]
                parts.append("corner %s triggered at front %s mm" % (x.get("n"), x.get("front")))
        fs = self.frame_stats
        if fs is not None:
            i = int(np.searchsorted(fs["t"], t, side="right")) - 1
            if 0 <= i < len(fs["t"]) and np.isfinite(fs["front"][i]):
                parts.append("camera wall ahead at %.0f mm" % fs["front"][i])
        return dict(state=st, v=_f(v, 2), steer=_f(s, 1), text="; ".join(parts[:1]) + " " + ", ".join(parts[1:]))

    # ---------------------------------------------------------------------------------------------- run it
    def analyze(self) -> dict:
        t_start = time.perf_counter()
        run, cfg = self.run, self.cfg
        self.progress(0.1, "clock")
        self._clock()
        self._signals()
        self.T = np.round(np.arange(0.0, self.dur + 1e-9, 0.1), 2)
        if not len(self.T):
            self.T = np.zeros(1)
        self._map()
        self.progress(0.2, "frames")
        self._frames()
        self.progress(0.72, "scans")
        self._scans()
        self.progress(0.8, "detect")
        self.pushes = []
        self._spikes()
        self._suspect_intervals()
        self.d6_camera()
        self.d5_lidar()
        self.d4_push()
        # while the car is pinned (a push, the lens pressed on a wall) the believed pose integrates the command and
        # drives on through the wall: not evidence of anything
        for c in self.cands:
            if c["span"] and c["strong"]:
                self.suspect_iv.append((c["t"], c["t_end"], "the car was pinned (%s)" % c["ev"].get("sig")))
        self.d7_map()
        self.d3_stall()
        self.d1_acc()
        self.d2_gyro()
        self.d8_loc()
        self.d9_interventions()
        self.d10_overruns()
        self.d11_sensors()
        self.d11b_camera_stale()
        self.d12_battery()
        self.d13_truth()
        self.progress(0.88, "fuse")
        fused = self._fuse()
        for inc in fused:
            loc_text, _sub = self._loc_trigger(inc["t"])
            if loc_text:
                inc["_loc_text"] = "pose suspect: " + loc_text.split(": ", 1)[1].split(";")[0]
            inc["where"] = self._where(inc)
            inc["state"] = self.state_at(inc["t"])
            inc["decision"] = self._decision(inc)
            self._cause(inc)
        refused = (run.end or {}).get("reason") in REFUSALS
        for inc in self.others:
            inc["where"] = self._where(dict(inc, cands=[]))
            inc["state"] = self.state_at(inc["t"])
            inc["decision"] = self._decision(inc)
            inc["cause"] = dict(code=inc["kind"].upper(), text=inc.get("text", ""), also=[], sub=[])
            if refused and inc["kind"] == "loc_loss" and any(e.get("sig") == "fix" for e in inc["evidence"]):
                # the designed answer at a mark it cannot resolve (boxed_in at mark C): nothing to fix on the camera
                inc["fix_hint"] = ("an honest refusal: the car did not move (%s).  Not a fault -- another mark, or "
                                   "survey / expect=; the camera only if EVERY mark refuses" % run.end.get("reason"))
                continue
            if inc["kind"] == "loc_loss":
                pe_ = self._pitch_evidence(inc["t"])
                if pe_:                                  # losses by the fit after a clean start: name the camera
                    inc["cause"].update(also=["CAMERA_PITCH"], sub=["the camera view stopped matching the map: " + pe_])
                    inc["fix_hint"] = self._fix_hint("CAMERA_PITCH", inc, [])
                    continue
            inc["fix_hint"] = {"loc_loss": "see the contact's cause, the camera (bw preflight pitch --field)",
                               "pose_out": "the pose left the field: a wrong relocalisation (reloc_max) or a blocked camera",
                               "battery": "charge the battery (rest >= 7.5 V)",
                               "overrun": "the loop / the camera / the IMU ran slow: the cpu event, what else ran",
                               "sensor": "check the sensor's cable and power, bw sys",
                               "error": "the program raised: read the traceback in the run log",
                               "start_mismatch": "the car is not on the mark, or the fix is wrong: re-place it",
                               "intervention": ""}.get(inc["kind"], "")
        incs = fused + self.others
        # keep the most important first when there are too many, then time order
        mx = int(cfg["max_incidents"])
        if len(incs) > mx:
            incs.sort(key=lambda i: (-i["sev"], i["t"]))
            incs = incs[:mx]
        incs.sort(key=lambda i: i["t"])
        counts = {k: 0 for k in COUNT_KEYS}
        for inc in incs:
            k = "sensor" if inc["kind"] == "error" else inc["kind"]
            counts[k] = counts.get(k, 0) + 1
        self.progress(0.92, "report")
        report = self._report(incs, counts)
        self._thumbs(report, incs)
        report["analyzer"] = dict(version=VERSION, ms=int(round((time.perf_counter() - t_start) * 1000)))
        return report

    def _verdict(self, counts):
        end = self.run.end
        err = bool(self.run.errors) or (end or {}).get("reason") == "error"
        why_end = (end or {}).get("reason")
        # nothing ran: the START button never came (or the series stopped during the wait) -- was shown CLEAN
        if why_end == "no_button" or (self.run.waited and self.run.button_t is None and end is not None):
            return "NOT_RUN"
        if counts["contact"]:
            return "CONTACT"
        # ended by a dead sensor, the battery or a failed park: never CLEAN (a camera_dead run had no other incident)
        if end is None or err or why_end in ("camera_dead", "lidar_dead", "board_lost", "battery", "park_failed"):
            return "INCOMPLETE"
        if self.run.program == "field_check":
            return "CLEAN" if why_end == "standard" else "NOT_STANDARD"
        # an honest refusal (the car never moved): not LOST with a camera hint (review 2026-09-25)
        if why_end in REFUSALS:
            return "REFUSED"
        if counts["near"]:
            return "NEAR"
        if counts["loc_loss"] or counts["pose_out"]:
            return "LOST"
        return "CLEAN"

    def _report(self, incs, counts):
        run, cfg = self.run, self.cfg
        rp = run.rp
        body = rp.get("body") or {}
        out_incs = []
        for n, inc in enumerate(incs, 1):
            ev = inc.get("evidence") or []
            dur = inc["t_end"] - inc["t"]
            o = dict(id=n, kind=inc["kind"], sev=inc["sev"], t=_f(inc["t"], 2), t_end=_f(inc["t_end"], 2),
                     dur_s=_f(dur, 1) if dur >= 0.05 else None, where=inc["where"], state=inc.get("state"),
                     decision=inc["decision"], evidence=ev, cause=inc.get("cause"), fix_hint=inc.get("fix_hint"),
                     frames=[], truth=None)
            if inc["kind"] in ("contact", "near", "bump") and self.truth_contacts:
                m = [tc for tc in self.truth_contacts if inc["t"] - 0.5 <= tc <= inc["t_end"] + 0.5]
                o["truth"] = dict(matched=bool(m), t=_f(m[0], 2) if m else None)
            out_incs.append(o)
        # score
        mc = None
        if getattr(self, "map_clear", None) is not None and getattr(self, "map_use", None) is not None:
            ok = self.map_use
            if ok.any():
                mc = float(np.min(self.map_clear[ok]))
        if self.scan_stats is not None:
            ss = self.scan_stats
            mv = np.array([self.moving(t - 0.3, t) for t in ss["t"]])
            if mv.any():
                lc = float(np.min(ss["clr"][mv]))
                mc = lc if mc is None else min(mc, lc)
        pe = None
        if self.truth is not None and self.pose is not None:
            errs = []
            for t in self.T[::5]:
                pz, tr = self.pose_at(t), self.truth_at(t)
                if pz and tr and (self.state_at(t) in ("GO", "CREEP")):
                    errs.append(math.hypot(pz[0] - tr[0], pz[1] - tr[1]))
            pe = float(np.median(errs)) if errs else None
        guard_n = len(self.ev("guard"))
        sh_st = next((x for _t, x in reversed(self.ev("shield_stats"))), None)
        brakes = (sh_st or {}).get("n_brake")
        if brakes is None:
            brakes = sum(1 for _t, x in self.ev("shield") if x.get("act") == "brake")
        score = dict(min_clearance_mm=_f(mc, 0), pose_err_med_mm=_f(pe, 0),
                     push_s=_f(sum(s[1] - s[0] for s in self.pushes), 1), guard_n=guard_n, shield_brakes=brakes)
        rec = self.rec
        fs = self.frame_stats
        sources = dict(imu=self.imu_src, clock=self.clock_src or ("run file" if not rec else "none"),
                       recording=("rec/" + os.path.basename(rec.dir)) if rec else None,
                       frames=int(len(rec.frames_n)) if rec else 0, frames_analysed=int(fs["ok"].sum()) if fs else 0,
                       scans=len(rec.scans) if rec else 0, pose=self.pose["src"] if self.pose else None,
                       truth=self.truth is not None, camera_verified=self._camera_verified(), notes=self.notes)
        # timeline
        T = self.T
        tl = dict(hz=10, t0=0.0, t=[_f(t, 1) for t in T])
        if self.pose is not None and getattr(self, "pose_T", None) is not None:
            x, y, th, sxy = self.pose_T
        else:
            x = y = th = sxy = np.full(len(T), np.nan)
        tl["x"] = [_f(v, 0) for v in x]
        tl["y"] = [_f(v, 0) for v in y]
        tl["th"] = [_f(v, 1) for v in th]
        tl["sxy"] = [_f(v, 0) for v in sxy]
        tl["v"] = [_f(v, 2) for v in _hold(self.cmd_t, self.cmd_v, T)]
        tl["steer"] = [_f(v, 1) for v in _hold(self.cmd_t, self.cmd_s, T)]
        vm = np.full(len(T), np.nan)
        if np.isfinite(self.v_lidar).any():
            vm = _hold(self.tel_t[np.isfinite(self.v_lidar)], self.v_lidar[np.isfinite(self.v_lidar)], T)
        tl["vm"] = [_f(v, 2) for v in vm]
        tl["img"] = [_f(v, 1) for v in (_hold(fs["t"][np.isfinite(fs["diff"])], fs["diff"][np.isfinite(fs["diff"])], T)
                                         if fs is not None and np.isfinite(fs["diff"]).any() else np.full(len(T), np.nan))]
        sts = [self.state_at(t) for t in T]
        states = []
        for s in sts:
            if s is not None and s not in states:
                states.append(s)
        tl["states"] = states
        tl["state"] = [states.index(s) if s is not None else None for s in sts]
        clr = np.full(len(T), np.nan)
        if self.scan_stats is not None:
            clr = _hold(self.scan_stats["t"], self.scan_stats["clr"], T)
        elif getattr(self, "map_clear", None) is not None and getattr(self, "map_use", None) is not None:
            clr = np.where(self.map_use, self.map_clear, np.nan)
        tl["clear"] = [_f(min(v, 999.0), 0) if np.isfinite(v) else None for v in clr]
        acc = np.full(len(T), np.nan)
        if len(self.dev):
            k = np.clip(np.floor(self.acc_t / 0.1 + 0.5).astype(int), 0, len(T) - 1)
            acc = np.full(len(T), np.nan)
            for kk, d in zip(k, self.dev):
                if not (acc[kk] >= d):
                    acc[kk] = d
        tl["acc"] = [_f(v, 2) for v in acc]
        loop = np.full(len(T), np.nan)
        if rec is not None and len(self.step_gaps_ms):
            tr = self.mono_rel(rec.steps["t"][1:])
            k = np.clip(np.floor(tr / 0.1 + 0.5).astype(int), 0, len(T) - 1)
            for kk, g in zip(k, self.step_gaps_ms):
                if not (loop[kk] >= g):
                    loop[kk] = g
        tl["loop"] = [_f(v, 0) for v in loop]
        tl["bat"] = [_f(v, 2) for v in (_hold(self.tel_t, self.bat, T) if len(self.bat) else np.full(len(T), np.nan))]
        tl["yaw"] = [_f(v, 1) for v in (_hold(self.tel_t, self.yaw, T) if len(self.yaw) else np.full(len(T), np.nan))]
        if self.truth is not None:
            tl["truth_x"] = [_f(v, 0) for v in _hold(self.truth["t"], self.truth["x"], T)]
            tl["truth_y"] = [_f(v, 0) for v in _hold(self.truth["t"], self.truth["y"], T)]
        # stats
        dv = self.dev
        # p999: the 50 Hz noise floor that sets an.acc_contact_g after the first clean laps (SUNDAY, BRAIN4_SPEC 15:
        # max(0.45, 1.5 x p999) of the three a1_laps_A reports)
        stats = dict(acc_dev_g=dict(p50=_f(np.percentile(dv, 50), 3), p95=_f(np.percentile(dv, 95), 3),
                                    p99=_f(np.percentile(dv, 99), 3), p999=_f(np.percentile(dv, 99.9), 3),
                                    max=_f(np.max(dv), 3), src=self.imu_src) if len(dv) else None,
                     loop_ms=dict(p50=_f(np.percentile(self.step_gaps_ms, 50), 0),
                                  p99=_f(np.percentile(self.step_gaps_ms, 99), 0),
                                  max=_f(np.max(self.step_gaps_ms), 0)) if len(self.step_gaps_ms) else None,
                     cpu=None, battery=self.battery)
        # the drive's own start / brake jolts D1 left out (an.jolt_g): what they were, so a real hit taken for one can
        # be found (at: the first 20)
        stats["jolts"] = dict(n=len(self.jolts), max_g=max((j["g"] for j in self.jolts), default=None),
                              ceiling_g=_f(self.jolt_g, 2), src=self.jolt_src,
                              windows_s=[_f(cfg["jolt_start_s"], 2), _f(cfg["jolt_brake_s"], 2)],
                              at=self.jolts[:20]) if self.jolt_g > 0.0 else None
        cp = next((x for _t, x in reversed(self.ev("cpu"))), None)
        if cp:
            stats["cpu"] = {k: cp.get(k) for k in ("share", "process_share", "loop_p50_ms", "loop_p99_ms")}
        cg = next((x for _t, x in reversed(self.ev("camera_gate"))), None)
        if cg:
            stats["camera_gate"] = {k: cg.get(k) for k in ("skipped", "frames")}
        # map
        mp = dict(layout=self.layout, island=[_f(v, 0) for v in self.island],
                  signs=[[_f(s[0], 0), _f(s[1], 0), s[2], _f(s[3], 1)] for s in self.signs],
                  lot=[_f(v, 0) for v in self.lot] if self.lot else None,
                  obstacles=[[_f(o[0], 0), _f(o[1], 0)] for o in self.obstacles],
                  car=[_f(v, 1) for v in self.car], direction=self.direction)
        if self.truth is not None:
            sc = _get(run.meta, "sim.scene", None)
            if isinstance(sc, dict):
                mp["truth_signs"] = [[p[0], p[1], p[2]] for p in sc.get("pillars") or [] if isinstance(p, list) and len(p) >= 3]
        # detector block (sim)
        det = None
        if self.truth is not None and (self.want_truth or True):
            tc = self.truth_contacts
            cont = [i for i in out_incs if i["kind"] == "contact"]
            matched_t = [c for c in tc if any(i["t"] - 0.5 <= c <= (i["t_end"] or i["t"]) + 0.5 for i in cont)]
            tp = [i for i in cont if any(i["t"] - 0.5 <= c <= (i["t_end"] or i["t"]) + 0.5 for c in tc)]
            det = dict(truth_contacts=[_f(c, 2) for c in tc], matched=len(matched_t),
                       recall=_f(len(matched_t) / len(tc), 3) if tc else None,
                       precision=_f(len(tp) / len(cont), 3) if cont else None,
                       false_contacts=[i["t"] for i in cont if i not in tp],
                       missed=[_f(c, 2) for c in tc if c not in matched_t])
        hints = []
        for i in out_incs:
            h = i.get("fix_hint")
            if h and h not in hints and i["kind"] in ("contact", "near", "loc_loss", "pose_out", "battery", "sensor", "error"):
                hints.append(h)
        end = run.end or {}
        return {
            "schema": SCHEMA, "id": run.id, "program": run.program,
            "profile": body.get("profile") or None,
            "body": dict(expect=body.get("expect") or None, shell=body.get("shell") or None,
                         car="wltoys" if self.wltoys else "stock_a1"),
            "t0": _f(run.t0, 3), "duration_s": _f(self.dur, 1),
            "end": dict(reason=end.get("reason"), laps=end.get("laps"), seconds=end.get("seconds")) if run.end else None,
            "verdict": self._verdict(counts), "counts": counts, "score": score, "sources": sources,
            "thresholds": {k: cfg[k] for k in ("acc_contact_g", "acc_bump_g", "gyro_jerk_dps", "push_s", "push_ratio",
                                               "touch_mm", "near_mm", "merge_s", "loop_gap_ms", "pose_jump_mm",
                                               "sxy_bad_mm", "bat_rest_v", "bat_min_v", "bat_sag_v")},
            "params": {k: run.params.get(k) for k in ("v", "v_corner", "v_turn", "r_corner", "r_turn", "laps", "park",
                                                      "lot_start", "start_any", "cam_shield") if k in run.params},
            "incidents": out_incs, "timeline": tl, "stats": stats, "map": mp, "fix_hints": hints, "detector": det,
            "anomalies": [dict(t=_f(a, 2), t_end=_f(b, 2), kind="heading_wrong",
                               text="heading %+.0f deg/s against a commanded %+.0f deg/s, nothing touched" % (
                                   info.get("gz_dps") or 0.0, info.get("exp_dps") or 0.0))
                          for a, b, info in getattr(self, "heading_wrong", [])],
        }

    def _thumbs(self, report, incs):
        """an.thumbs frames per incident (at t, t - 0.5, t - 1.0 s), 160 x 120 JPEG q70; the incident's own frame gets
        the camera's wall bases drawn (seen: camera orange, clipped: red)."""
        rec = self.rec
        report["_thumbs"] = {}
        if rec is None or not len(rec.frames_n) or self.offset is None:
            return
        try:
            import cv2
            from . import vision as V
        except Exception:
            return
        ft = self.mono_rel(rec.frames_t)
        k = int(self.cfg["thumbs"])
        fs = self.frame_stats
        for o in report["incidents"]:
            if o["kind"] in ("intervention",) and len(report["incidents"]) > 12:
                continue
            nums = []
            for j in range(k):
                t = (o["t"] or 0.0) - 0.5 * j
                i = int(np.searchsorted(ft, t, side="right")) - 1
                if i >= 0 and (not nums or int(rec.frames_n[i]) not in nums):
                    nums.append(int(rec.frames_n[i]))
            o["frames"] = nums
            for jj, n in enumerate(nums):
                if n in report["_thumbs"]:
                    continue
                im = cv2.imread(rec.frame_path(n), cv2.IMREAD_REDUCED_COLOR_4)
                if im is None:
                    continue
                if im.shape[1] != 160:
                    im = cv2.resize(im, (160, 120), interpolation=cv2.INTER_AREA)
                if jj == 0 and fs is not None:
                    try:
                        g = V.Ground(fs["cam"], im.shape[1], im.shape[0])
                        vps = dict(fs["vp"], K=max(2, int(round(fs["vp"]["K"] / 4))), col_step=1)
                        e = V.edges(im, g, vps)
                        for u, vv, c, cl in zip(e.u, e.vv, e.cls, e.clip):
                            if c == V.WALL and np.isfinite(vv) and vv < im.shape[0] - 0.5:
                                colr = (60, 60, 230) if cl else (6, 119, 217)
                                cv2.circle(im, (int(u), int(min(vv, im.shape[0] - 1))), 1, colr, -1)
                    except Exception:
                        pass
                ok, buf = cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    report["_thumbs"][n] = buf.tobytes()


# ================================================================================================== API
def _cfg(run: Run | None, params: dict | None) -> dict:
    cfg = dict(DEFAULTS)
    if run is not None:
        for k, v in (run.rp.get("an") or {}).items():
            if k in cfg and isinstance(v, (int, float)):
                cfg[k] = v
    for k, v in (params or {}).items():
        k = k[3:] if k.startswith("an.") else k
        if k in cfg:
            cfg[k] = v
    return cfg


def analyze(run_path: str, rec_dir: str | None = "auto", params: dict | None = None, progress=None,
            truth: bool = False) -> dict:
    """The report dict (schema bw.report/1) of one run file.  `rec_dir`: a recording directory, "auto" (found from
    the run) or None / "none".  `params`: an.* overrides (with or without the prefix).  `progress(p, stage)`.  The
    thumbnails travel in report["_thumbs"] ({frame: JPEG bytes}); write() takes them out of the JSON."""
    run = Run(run_path)
    cfg = _cfg(run, params)
    rp = find_rec(run, rec_dir if rec_dir is not None else "none")
    rec = None
    if rp:
        try:
            rec = Rec(rp)
        except OSError:
            rec = None
    return Analysis(run, rec, cfg, truth=truth, progress=progress).analyze()


def _atomic(path: str, data: bytes):
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def write(report: dict, out_dir: str, html: bool = True) -> tuple:
    """runs/<id>.report.json (+ .report.html): atomic writes.  Returns (json_path, html_path | None)."""
    os.makedirs(out_dir, exist_ok=True)
    thumbs = report.pop("_thumbs", {}) or {}
    jp = os.path.join(out_dir, "%s.report.json" % report.get("id", "run"))
    _atomic(jp, json.dumps(report, separators=(",", ":"), default=str).encode("utf-8"))
    hp = None
    if html:
        from . import report_html
        hp = os.path.join(out_dir, "%s.report.html" % report.get("id", "run"))
        _atomic(hp, report_html.render(report, thumbs).encode("utf-8"))
    return jp, hp


def _fmt_where(i) -> str:
    """object (x,y) part -- the object without its long suspicion note (the cause line carries the why)."""
    w = i.get("where") or {}
    obj = (w.get("object") or "").split(" (")[0]
    if w.get("x") is None:
        s = obj or "-"
    else:
        s = "%s (%s,%s)" % (obj or w.get("section") or "", w["x"], w["y"])
    if w.get("part"):
        s += " " + w["part"]
    return s


def table(report: dict) -> str:
    """The text block `bw analyze` prints (BRAIN4_SPEC 6.9): one header, the sources, one line per incident, the
    battery and the loop, the fix hints."""
    r = report
    end = r.get("end") or {}
    src = r.get("sources") or {}
    lines = ["%s  %s  %s  %s s  end %s  laps %s   %s" % (
        r.get("id"), r.get("program"), r.get("profile") or (r.get("body") or {}).get("car") or "-",
        r.get("duration_s"), end.get("reason"), end.get("laps"), r.get("verdict"))]
    lines.append("sources: imu %s | clock %s | rec %s frames, %s scans | camera %s" % (
        src.get("imu"), src.get("clock"), src.get("frames"), src.get("scans"),
        "verified" if src.get("camera_verified") else "NOT verified this session"))
    if r.get("error"):
        lines.append("ANALYZER ERROR: %s" % r["error"])
    lines.append(" #  t       kind            sev  where                                          state    decision / cause")
    for i in r.get("incidents") or []:
        if i["kind"] == "intervention" and len(r["incidents"]) > 15:
            continue
        c = i.get("cause") or {}
        dur = " %.0f s" % i["dur_s"] if i.get("dur_s") and i["dur_s"] >= 1.0 else ""
        lines.append("%2d  %-6s  %-14s  %d    %-46s %-8s %s" % (
            i["id"], "%.2f" % i["t"] if i.get("t") is not None else "-", i["kind"] + dur, i["sev"],
            _fmt_where(i)[:46], (i.get("state") or "-")[:8], "%s: %s" % (c.get("code"), c.get("text") or "")))
        if i["kind"] in ("contact", "near") and (i.get("decision") or {}).get("text"):
            lines.append("        doing: %s" % i["decision"]["text"][:160])
            ev = ", ".join("%s%s" % (e.get("sig"), (" %s" % e["val"]) if e.get("val") is not None else
                                     (" %s s" % e["dur_s"]) if e.get("dur_s") is not None else "")
                           for e in (i.get("evidence") or [])[:6])
            if ev:
                lines.append("        evidence: %s" % ev)
            for s in c.get("sub") or []:
                lines.append("        - %s" % s[:160])
            if c.get("also"):
                lines.append("        also: %s" % ", ".join(c["also"]))
    b = (r.get("stats") or {}).get("battery") or {}
    lp = (r.get("stats") or {}).get("loop_ms") or {}
    lines.append("battery rest %s min %s end %s (sag %s V)   loop p99 %s ms" % (
        b.get("rest_v"), b.get("min_v"), b.get("end_v"), b.get("sag_v"), lp.get("p99")))
    jo = (r.get("stats") or {}).get("jolts") or {}
    if jo.get("n"):
        lines.append("jolts: %d start / brake jolts of the drive left out (max %s g < %s g, %s)" % (
            jo["n"], jo.get("max_g"), jo.get("ceiling_g"), jo.get("src")))
    if r.get("detector"):
        d = r["detector"]
        lines.append("truth: %d contacts, matched %s, recall %s, precision %s, missed %s" % (
            len(d.get("truth_contacts") or []), d.get("matched"), d.get("recall"), d.get("precision"), d.get("missed")))
    for h in r.get("fix_hints") or []:
        lines.append("fix: %s" % h)
    return "\n".join(lines)


def summary_line(report: dict) -> str:
    c = report.get("counts") or {}
    first = next((i for i in report.get("incidents") or [] if i["kind"] in ("contact", "near")), None)
    s = "%s: %s contact %s, near %s, lost %s" % (report.get("id"), report.get("verdict"), c.get("contact", 0),
                                                 c.get("near", 0), c.get("loc_loss", 0))
    if first:
        s += " -- first %.1f s %s %s" % (first["t"], (first.get("where") or {}).get("object") or "",
                                         (first.get("cause") or {}).get("code") or "")
    return s


def _minimal(run_path: str, err: str) -> dict:
    base = os.path.basename(run_path)
    return {"schema": SCHEMA, "id": base[:-6] if base.endswith(".jsonl") else base, "program": None, "profile": None,
            "verdict": "INCOMPLETE", "counts": {k: 0 for k in COUNT_KEYS}, "incidents": [], "error": err,
            "analyzer": dict(version=VERSION, ms=0)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m bluewave.analyze", description=__doc__.split("\n\n")[0])
    ap.add_argument("run")
    ap.add_argument("--rec", default="auto", help="a recording directory, auto (found from the run) or none")
    ap.add_argument("--out-dir", default=None, help="where the report goes (default: the run file's directory)")
    ap.add_argument("--no-html", action="store_true")
    ap.add_argument("--json-progress", action="store_true")
    ap.add_argument("--set", action="append", default=[], help="an.KEY=VALUE threshold overrides")
    ap.add_argument("--truth", action="store_true", help="a simulator run: score the detectors against the truth")
    ap.add_argument("--print", action="store_true", help="print the table")
    a = ap.parse_args(argv)
    params = {}
    for s in a.set:
        k, _, v = s.partition("=")
        try:
            params[k.strip()] = json.loads(v)
        except ValueError:
            params[k.strip()] = v
    out_dir = a.out_dir or os.path.dirname(os.path.abspath(a.run))

    def prog(p, stage):
        if a.json_progress:
            print(json.dumps({"progress": round(float(p), 3), "stage": stage}), flush=True)
    try:
        Run(a.run)
    except (OSError, ValueError) as e:
        msg = "cannot read %s: %s" % (a.run, e)
        if a.json_progress:
            print(json.dumps({"error": msg}), flush=True)
        else:
            print(msg, file=sys.stderr)
        return 2
    try:
        rep = analyze(a.run, a.rec, params, progress=prog, truth=a.truth)
        jp, hp = write(rep, out_dir, html=not a.no_html)
        code = 0
    except Exception:
        tb = traceback.format_exc()
        last = [l for l in tb.strip().splitlines() if l.strip()][-1]
        rep = _minimal(a.run, last)
        rep["traceback"] = tb[-2000:]
        try:
            jp, hp = write(rep, out_dir, html=False)
        except OSError:
            jp, hp = None, None
        print(tb, file=sys.stderr)
        code = 3
    if a.json_progress:
        print(json.dumps({"result": {"report": jp, "html": hp, "verdict": rep.get("verdict"), "counts": rep.get("counts"),
                                     "error": rep.get("error")}}), flush=True)
    elif a.print or code:
        print(table(rep))
    else:
        print(summary_line(rep))
        print("report: %s" % (hp or jp))
    return code


if __name__ == "__main__":
    sys.exit(main())
