"""params.json with a memory: a history of every agent write, one-step undo, restore, diffs and profiles.

Every change is applied IN PLACE to the live params dict (robot.p).  hw.Robot._house holds a reference to
robot.p["imu"] taken when its thread starts, so replacing a sub-dict object would silently keep the old gyro bias --
a restore, a profile and a nested PUT all update the existing dicts instead.

    params_history/params-<hid>.json + index.json    beside params.json, the newest 30, hid = YYYYmmdd-HHMMSS-fff
    profiles.json                                    custom profiles {name: {desc, patch}}; the built-ins are fixed
    mentorpi/profiles/<name>.json                    built-in profiles that ship with the code ({desc, patch, notes};
                                                     wltoys.json is written by tools/profile_wltoys.py)
"""
from __future__ import annotations

import copy
import json
import os
import threading
import time

from . import params as P

KEEP = 30
_LOCK = threading.RLock()

# The stock A1's wro_next numbers: GEOMETRY for its full-lock radius (R_eff 505 mm, twice what the programs' DEFAULTS
# were tuned on) and the push stop -- no speed (the Sunday rungs stay in plans/*.json; the mat confirms a rung first).
# [SIM 2026-09-25, docs/SPEED_LADDER.md "Evaluation"]  ramp 700 / lookahead 400 / guard_horizon 350: an interleaved A/B,
# seeds 1-6 both ways, 3 laps, lot, 0.22 / 0.20, simq TRUSTED: wall contacts 14 -> 0, 0-contact runs 8 -> 10 of 12 (wrong-side
# sign passes 28 -> 31 of ~200: NOT reduced; CLEAN now needs 0 of them), touch 21 -> 10
# s, 3 laps 117.6 -> 114.9 s, program CPU share 0.141 -> 0.143; the remaining contacts are a parking limitation driven
# into with park 0 (both arms).  The kinematic screen (perfect pose, every sign known, 80 scenes): wall contacts 20 -> 6.
# 1 lap seeds 1-3 both ways at 0.22 / 0.30 / 0.40: CLEAN 3 / 2 / 3 -> 4 / 4 / 4 of 6, walls 11 -> 0, err_med 2-130 -> 2-6.
# bump_lost 1 (W22): an IMU hit is LOST at once -- pushes > 60 s (the mat's M4 pushed 85 s) without it: 3 of 24 runs at
# pitch +-1 deg, 5 of 30 at nominal pitch (109-124 s), 1 of 6 at +0.5; with it 0 of 39 (wall_stuck: 205 s -> 0.1 s).
# [DAY1] the 50 Hz IMU floor on the first clean laps (BRAIN4_SPEC 15): a false hit costs one LOST back-off.
# [REVIEW 2026-09-25] bump_lost's evidence above was CIRCULAR: the mock gave every contact a 1.0 g shake with an 80 ms
# tail (~8 samples >= 0.28 g at its then 100 Hz), so the two-sample rule always passed in the sim.  Replayed on the
# mat's accelerometer the pair rule fires on 3 of the 5 real contacts: bump_lost is UNPROVEN until the first mat laps
# record 50 Hz contacts.  push_img 1 (the image push detector) and bump_single (a single spike confirmed by the camera,
# the program's default) are the camera-only car's push stops that need no IMU tail.
STOCK_A1_WRO_NEXT = {"ramp": 700.0, "lookahead": 400.0, "guard_horizon": 350.0, "bump_lost": 1, "push_img": 1}
# lapcam: the turn trigger is r_turn + W_next / 2, and W_next read 600 (w_default) at every corner of the 1000 mm
# standard field in the SIM.  With 300 / 600 the R 505 turn started ~400 mm late, ended beside the outer wall, and that
# wall then read as a wall AHEAD: a second corner at once, into the island (both ways: 32-38 contacts per run, 0.1 true
# laps while the program counted 3; r_turn 500 alone the same).  r_turn 500 (the mat's R 505; plans/a1_lapcam.json has
# it) + w_default 1000: 3 true laps, 0 contacts, stopped in the start straight, 122 s, both ways [SIM 2026-09-25].
# Open Challenge (random 600 / 1000 mm corridors), seeds 1-3 both ways: 0 of 6 with 300 / 600 or 500 / 600 -> 4 of 6 with
# these (the two others end `blocked` after 2-5 contacts, clockwise with 600 mm corridors).
STOCK_A1_LAPCAM = {"r_turn": 500.0, "w_default": 1000.0}

# The stock A1's own measured values.  Its profile patch (stock_a1_patch) is EVERY key a shipped profile (profiles/
# wltoys.json) changes, at its DEFAULTS value, and then these: a hand list once missed odo.speed_scale (0.92 -> 1.0 left
# the stock car dead-reckoning 8.7 % long), lidar.offset_deg (180: front / back swapped) and the camera pose.
STOCK_A1 = {"lidar.loc": 0, "lidar.pos_mm": [0.0, 0.0], "car.rear_mm": 34.0, "car.front_mm": 179.0,
            "car.half_w_mm": 80.5, "car.lot_len_mm": 320.0, "camera.x_mm": 150.0,
            # the camera as the MAT fitted it [MAT 2026-09-23, calib fit of rec 20260923-185306: fit 98 -> 20 mm]:
            # the last KNOWN values, not a promise -- the mount moves (~10 deg in one day), so `bw preflight pitch
            # --field` proves them each session (camera.verified_t).  Switching wltoys -> stock_a1 used to put back
            # the 2026-09-22 floor fit (pitch -5.58, roll -1.34, h 123) and keep the card's latency (BRAIN4_SPEC G9)
            "camera.h_mm": 127.0, "camera.pitch_deg": 4.7, "camera.roll_deg": 0.4, "camera.latency_s": 0.09,
            "car.park_margin_mm": 13.0, "lidar.lot_map": 0,
            # which chassis these params are for (bodyid.py): the strap pin 29 reads open (1) on the A1
            "body.expect": "stock_a1", "body.shell": "a1", "body.id_gpio": 5, "mat.cooldown_s": 20.0,
            # back from the WLtoys build (profiles/wltoys.json): the encoder motors, the vendor chassis and servo
            "drive.backend": "rrc", "drive.rrc_motor": 1, "drive.wheel_d_m": 0.067, "drive.max_mps": 1.0,
            "drive.ay_max": 0.0, "drive.v_tight": 0.0, "drive.stall_s": 0.0, "drive.speed_loop.on": 0,
            "odo.source": "cmd", "odo.speed_scale": 0.92,
            # the EFFECTIVE wheelbase the mat measured (R ~510 mm at 29 deg, 2026-09-23), not the vendor's 0.145:
            # switching profiles used to lose it (BRAIN_SPEC G5)
            "step.on": 0, "buttons.start_gpio": -1, "chassis.wheelbase_m": 0.28, "chassis.wheelbase_geom_m": 0.145,
            "chassis.track_m": 0.133, "lidar.h_mm": 145.7, "shield.on": -1,
            "steer.center_us": 1500, "steer.us_per_deg": 2000 / 180, "steer.max_deg": 29.0, "steer.min_us": 0,
            "steer.max_us": 0,
            # the programs' DEFAULTS again (WLtoys: prog.*) + the A1's geometry and push stop (STOCK_A1_WRO_NEXT,
            # STOCK_A1_LAPCAM)
            "prog.wro_next": dict(STOCK_A1_WRO_NEXT), "prog.lapcam": dict(STOCK_A1_LAPCAM)}

BUILTIN = {
    "stock_a1": dict(
        desc="Stock MentorPi A1: camera + gyro; the LD19 scans above the walls, so it does not localise. "
             "Every key the WLtoys profile changes goes back; the camera gets the mat's fit (h 127, pitch 4.7, "
             "roll 0.4, latency 0.09) and the effective wheelbase 0.28 (R ~505 mm at full lock). Then restart the "
             "agent (SYSTEM; bw body does it), then `bw preflight pitch --field` (the camera moves), and TESTS -> "
             "turn_radius when the steering was touched.",
        patch=STOCK_A1),
    "bluewave_body": dict(
        desc="BLUE WAVE body: 50 mm nose lidar localises, camera on the 20 deg wedge "
             "(3d/bluewave_body/design/LAYOUT.py via tools/sim_run.py NEW_BODY*).",
        patch={"car.rear_mm": 34.2, "car.front_mm": 262.6, "car.half_w_mm": 80.5, "car.lot_len_mm": 445.2,
               "camera.x_mm": 241.8, "camera.h_mm": 120.0, "camera.pitch_deg": 20.0, "camera.roll_deg": 0.0,
               "lidar.loc": 1, "lidar.pos_mm": [232.3, 0.0], "lidar.block_deg": [[130.7, 229.35]],
               "lidar.scan_lag_s": 0.05}),
}


def _dir() -> str:
    return os.path.dirname(os.path.abspath(P.path()))


def hist_dir() -> str:
    return os.path.join(_dir(), "params_history")


def _index() -> dict:
    try:
        with open(os.path.join(hist_dir(), "index.json"), encoding="utf-8") as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _atomic(path: str, obj) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)
    os.replace(tmp, path)


# ---------------------------------------------------------------------------------------------------------- history
def backup(current: dict, reason: str) -> str:
    """Copy the current params.json (or, before the first save, the live dict) into the history; returns its hid."""
    with _LOCK:
        d = hist_dir()
        os.makedirs(d, exist_ok=True)
        t = time.time()
        hid = time.strftime("%Y%m%d-%H%M%S", time.localtime(t)) + "-%03d" % int((t % 1) * 1000)
        idx = _index()
        base, k = hid, 1
        while hid in idx:
            hid = "%s.%d" % (base, k)
            k += 1
        dst = os.path.join(d, "params-%s.json" % hid)
        try:
            with open(P.path(), "rb") as f:
                data = f.read()
            with open(dst, "wb") as f:
                f.write(data)
        except FileNotFoundError:
            with open(dst, "w", encoding="utf-8") as f:
                json.dump(current, f, indent=2)
        idx[hid] = dict(t=round(t, 3), reason=str(reason))
        for old in sorted(idx)[:-KEEP]:
            try:
                os.remove(os.path.join(d, "params-%s.json" % old))
            except OSError:
                pass
            del idx[old]
        _atomic(os.path.join(d, "index.json"), idx)
        return hid


def history() -> list:
    """[{hid, t, reason, bytes}], newest first."""
    with _LOCK:
        idx = _index()
    out = []
    for hid in sorted(idx, reverse=True):
        path = os.path.join(hist_dir(), "params-%s.json" % hid)
        if os.path.isfile(path):
            out.append(dict(hid=hid, t=idx[hid].get("t"), reason=idx[hid].get("reason"), bytes=os.path.getsize(path)))
    return out


def entry(hid: str) -> dict | None:
    if not hid or "/" in hid or "\\" in hid or ".." in hid:
        return None
    try:
        with open(os.path.join(hist_dir(), "params-%s.json" % hid), encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def replace_in_place(dst: dict, src: dict) -> None:
    """dst becomes equal to src, keeping every existing sub-dict OBJECT (recursive update + delete)."""
    for k in [k for k in dst if k not in src]:
        del dst[k]
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            replace_in_place(dst[k], v)
        else:
            dst[k] = copy.deepcopy(v)


def restore(p: dict, hid: str, reason: str | None = None) -> dict:
    """Back the current file up, then make the params (live and on disk) equal to history entry `hid`."""
    with _LOCK:
        e = entry(hid)
        if e is None:
            raise KeyError(hid)
        backup(p, reason or "restore:%s" % hid)
        replace_in_place(p, P._merge(P.DEFAULTS, e))
        P.save(p)
        return p


def undo(p: dict):
    """Restore the newest history entry: one step that works both ways (pressing it again returns)."""
    with _LOCK:
        h = history()
        if not h:
            raise KeyError("no history yet")
        hid = h[0]["hid"]
        restore(p, hid, "undo")
        return p, hid


# ---------------------------------------------------------------------------------------------------------- changes
def leaves(d: dict, prefix: str = "") -> dict:
    out = {}
    for k, v in (d or {}).items():
        key = prefix + str(k)
        if isinstance(v, dict) and v:
            out.update(leaves(v, key + "."))
        else:
            out[key] = v
    return out


def diff(a: dict, b: dict) -> list:
    """[{key, old, new}] over dotted leaf paths; lists compare as values."""
    la, lb = leaves(a), leaves(b)
    out = []
    for k in sorted(set(la) | set(lb)):
        if la.get(k, None) != lb.get(k, None) or (k in la) != (k in lb):
            out.append(dict(key=k, old=la.get(k), new=lb.get(k)))
    return out


def patch(p: dict, change: dict) -> None:
    """Apply dotted keys {"steer.center_us": 1512} or nested dicts {"imu": {"gyro_bias": 0.1}} IN PLACE."""
    def merge(dst, src):
        for k, v in src.items():
            if isinstance(v, dict) and isinstance(dst.get(k), dict):
                merge(dst[k], v)
            else:
                dst[k] = copy.deepcopy(v)
    for k, v in (change or {}).items():
        if "." in k:
            node = p
            parts = k.split(".")
            for part in parts[:-1]:
                if not isinstance(node.get(part), dict):
                    node[part] = {}
                node = node[part]
            node[parts[-1]] = copy.deepcopy(v)
        elif isinstance(v, dict) and isinstance(p.get(k), dict):
            merge(p[k], v)
        else:
            p[k] = copy.deepcopy(v)


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v == v and abs(v) != float("inf")


# the keys the dead-man, the gyro and the link watchdog read in a loop: one bad value there once killed the
# housekeeping thread (imu.gyro_axis=9) or let a REST command drive for 2 s (safety.deadman_ms=2000)
SAFETY_KEYS = {
    "imu.gyro_axis": (lambda v: _num(v) and int(v) == v and 0 <= v <= 5, "an integer 0-5 (the board sends ax ay az gx gy gz)"),
    "imu.gyro_sign": (lambda v: _num(v) and abs(v) == 1, "1 or -1"),
    "imu.gyro_bias": (lambda v: _num(v) and abs(v) < 20, "a number of deg/s under 20"),
    "safety.deadman_ms": (lambda v: _num(v) and 100 <= v <= 1000, "100-1000 ms"),
    "safety.link_timeout_ms": (lambda v: _num(v) and (v == 0 or 1000 <= v <= 60000), "0 (off) or 1000-60000 ms"),
    "safety.prog_ttl_ms": (lambda v: _num(v) and (v == 0 or 100 <= v <= 1000), "0 (off) or 100-1000 ms"),
    # the lidar shield's margins (BRAIN_SPEC 2.2): a typo here would either never brake or brake on every planned pass
    "shield.margin_mm": (lambda v: _num(v) and 0 <= v <= 150, "0-150 mm"),
    "shield.side_mm": (lambda v: _num(v) and 0 <= v <= 40, "0-40 mm (the planner passes signs 85 mm off)"),
    "shield.park_margin_mm": (lambda v: _num(v) and 0 <= v <= 40, "0-40 mm"),
    "shield.stop_mm": (lambda v: _num(v) and 0 <= v <= 100, "0-100 mm"),
    "shield.latency_s": (lambda v: _num(v) and 0 <= v <= 0.5, "0-0.5 s"),
    "shield.horizon_mm": (lambda v: _num(v) and 200 <= v <= 3000, "200-3000 mm"),
    "shield.v_floor": (lambda v: _num(v) and 0.05 <= v <= 0.3, "0.05-0.3 m/s"),
    # dynamics_wl.md S-1 on tight arcs (hw.Robot.tight_cap): a negative or huge value would switch it off unseen
    "drive.v_tight": (lambda v: _num(v) and (v == 0 or 0.1 <= v <= 0.6), "0 (off) or 0.1-0.6 m/s"),
    "drive.r_tight_mm": (lambda v: _num(v) and 50 <= v <= 1000, "50-1000 mm"),
    "drive.r_free_mm": (lambda v: _num(v) and 50 <= v <= 2000, "50-2000 mm"),
    # BRAIN4_SPEC 2.1: LOOK moves the car before it knows where it is (S14); the camshield brakes it; the analyzer's
    # contact threshold decides what stops a mat series; the body strap pin is claimed as an input; the button wait
    "look.v": (lambda v: _num(v) and 0.05 <= v <= 0.25, "0.05-0.25 m/s"),
    "look.max_mm": (lambda v: _num(v) and 0 <= v <= 2000, "0-2000 mm"),
    "camshield.margin_mm": (lambda v: _num(v) and 0 <= v <= 100, "0-100 mm"),
    "camshield.latency_s": (lambda v: _num(v) and 0 <= v <= 0.6, "0-0.6 s"),
    "an.acc_contact_g": (lambda v: _num(v) and 0.1 <= v <= 5, "0.1-5 g"),
    "body.id_gpio": (lambda v: _num(v) and int(v) == v and (v == -1 or 2 <= v <= 27), "-1 (none) or a GPIO 2-27"),
    "mat.wait_s": (lambda v: _num(v) and 5 <= v <= 600, "5-600 s"),
}


def validate(p: dict) -> list:
    """[(key, 'key = value: must be ...')] for every safety-critical value in `p` out of range (absent keys pass)."""
    lv = leaves(p)
    return [(k, "%s = %s: must be %s" % (k, json.dumps(lv[k]), why)) for k, (ok, why) in SAFETY_KEYS.items()
            if k in lv and not ok(lv[k])]


def change_text(changes: list, cap: int = 6) -> str:
    def f(v):
        s = json.dumps(v)
        return s if len(s) <= 24 else s[:21] + "..."
    parts = ["%s %s -> %s" % (c["key"], f(c["old"]), f(c["new"])) for c in changes[:cap]]
    more = " (+%d more)" % (len(changes) - cap) if len(changes) > cap else ""
    return "%d change%s: %s%s" % (len(changes), "" if len(changes) == 1 else "s", ", ".join(parts), more)


# ---------------------------------------------------------------------------------------------------------- profiles
def _profiles_path() -> str:
    return os.path.join(_dir(), "profiles.json")


PROFILES_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "profiles")


def file_profiles() -> dict:
    """The profiles that ship with the code: mentorpi/profiles/<name>.json = {desc, patch[, notes]} (read-only)."""
    out = {}
    try:
        names = sorted(f for f in os.listdir(PROFILES_DIR) if f.endswith(".json"))
    except OSError:
        return out
    for fn in names:
        try:
            with open(os.path.join(PROFILES_DIR, fn), encoding="utf-8") as f:
                d = json.load(f)
        except (OSError, ValueError):
            continue
        if isinstance(d, dict) and isinstance(d.get("patch"), dict):
            out[fn[:-5]] = dict(desc=str(d.get("desc", "")), patch=dict(d["patch"]))
    return out


_MISSING = object()


def _default_of(key: str):
    node = P.DEFAULTS
    for part in key.split("."):
        if not isinstance(node, dict) or part not in node:
            return _MISSING
        node = node[part]
    return copy.deepcopy(node)


def stock_a1_patch() -> dict:
    """Every dotted key a shipped profile patches, at its DEFAULTS value (keys DEFAULTS lacks, mock.*, are left
    alone), then STOCK_A1's measured values: applying wltoys and then stock_a1 leaves nothing of the WLtoys behind."""
    out = {}
    for prof in file_profiles().values():
        for k in prof["patch"]:
            v = _default_of(k)
            if v is not _MISSING:
                out[k] = v
    out.update(copy.deepcopy(STOCK_A1))
    return out


# keys the agent reads only when it builds the Robot (hw.Robot.__init__: the drive backend, the motor's pins, the
# lidar's port and angle convention, the camera and depth streams, the board, the start button, the speed estimator):
# a save changes them in params.json at once, the car only after `bw sys restart_agent` (PARAMS showed nothing:
# console review 2026-09-24, the WLtoys profile applied, tel.drive still null)
RESTART_KEYS = ("drive.backend", "drive.pwm.", "lidar.device", "lidar.baud", "lidar.res_deg", "lidar.cw",
                "lidar.offset_deg", "camera.device", "camera.width", "camera.height", "camera.fps", "depth.on",
                "depth.host", "depth.port", "board.", "buttons.", "odo.est.", "mock.")


def needs_restart(changes) -> list:
    """The dotted keys in `changes` ([{key, ...}] from diff(), or key strings) the running Robot does not re-read."""
    keys = [c["key"] if isinstance(c, dict) else str(c) for c in (changes or [])]
    return sorted(k for k in keys if any(k == r or (r.endswith(".") and k.startswith(r)) for r in RESTART_KEYS))


def profiles() -> dict:
    out = {k: dict(v, builtin=True) for k, v in BUILTIN.items()}
    out["stock_a1"]["patch"] = stock_a1_patch()
    for k, v in file_profiles().items():
        if k not in out:
            out[k] = dict(v, builtin=True)
    try:
        with open(_profiles_path(), encoding="utf-8") as f:
            custom = json.load(f)
        for k, v in (custom if isinstance(custom, dict) else {}).items():
            if k not in out and isinstance(v, dict):
                out[k] = dict(desc=str(v.get("desc", "")), patch=dict(v.get("patch") or {}), builtin=False)
    except (OSError, ValueError):
        pass
    return out


def profile_put(name: str, desc: str, change: dict) -> None:
    if name in BUILTIN or name in file_profiles():
        raise PermissionError(name)
    with _LOCK:
        try:
            with open(_profiles_path(), encoding="utf-8") as f:
                custom = json.load(f)
        except (OSError, ValueError):
            custom = {}
        custom[name] = dict(desc=desc or "", patch=dict(change or {}))
        _atomic(_profiles_path(), custom)


def profile_apply(p: dict, name: str) -> list:
    """Back up, apply the profile's patch in place, stamp body.profile = name (bodyid.py and `bw mat` read which
    profile is on the car: BRAIN4_SPEC 8.3, S15), save.  Returns the changes."""
    prof = profiles().get(name)
    if prof is None:
        raise KeyError(name)
    with _LOCK:
        before = copy.deepcopy(p)
        backup(p, "profile:%s" % name)
        patch(p, prof["patch"])
        patch(p, {"body.profile": name})
        P.save(p)
        return diff(before, p)


# ---------------------------------------------------------------------------------------------------------- meta (P2)
def meta() -> dict:
    """{dotted_key: comment} from params.py's DEFAULTS comments -- the [VENDOR] / [DAY1] / [RULE] provenance tags."""
    import ast
    from . import progmeta
    path = P.__file__
    try:
        with open(path, encoding="utf-8") as f:
            src = f.read()
        tree = ast.parse(src)
    except (OSError, SyntaxError):
        return {}
    com, only = progmeta.comments(src)
    node = None
    for st in tree.body:
        if isinstance(st, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "DEFAULTS" for t in st.targets):
            node = st.value
    out = {}

    def walk(d, prefix):
        if not isinstance(d, ast.Dict):
            return
        for k, v in zip(d.keys, d.values):
            if not (isinstance(k, ast.Constant) and isinstance(k.value, str)):
                continue
            key = prefix + k.value
            if isinstance(v, ast.Dict):
                h = progmeta.help_for(k.lineno, com, only) if v.lineno == k.lineno and v.end_lineno != k.lineno else ""
                if h:
                    out[key] = h
                walk(v, key + ".")
            else:
                h = progmeta.help_for(v.end_lineno, com, only)
                if h:
                    out[key] = h

    walk(node, "")
    return out

