"""bw -- the laptop's handle on the robot: what the team uses to deploy, run, test and read it back.

Stdlib only (urllib, json, subprocess), so it runs from any shell on the dev PC.  Robot address from BW_HOST
(default bluewave.local), agent port BW_PORT (8000), SSH user BW_USER (bluewave), token BW_TOKEN (optional).

    bw status                      one-line health: link, battery, IMU Hz, lidar rpm, camera fps, program
    bw watch [seconds]             live telemetry lines (Ctrl-C to stop)
    bw estop | arm | beep
    bw drive V STEER [MS]          one manual command, dead-man MS (default 600)
    bw servo DEG | --us PULSE
    bw tests                       list the component tests
    bw test NAME [k=v ...]         run one; prints the measurement and the params it suggests.  bw run / test / mat
                                   hold the link themselves (a GET every 0.5 s < the hub's 3 s link_timeout_ms): no
                                   console page needed; bw killed or off the network = the car stops 3 s later
                                   WLtoys drive (gpio_pwm), wall straight ahead, in this order:
                                     duty_sweep  breakaway + duty -> speed line   (drive.duty.*)
                                     coast       braked stop distance             (drive.stop_decel)
                                     step_table  the park's kicks, measured       (step.table)
                                   HP60C depth (depth.on=1 + bw sys restart_agent; pillar 500 mm ahead of the bumper):
                                     depth       depth vs the tape, fill, rate    (depth.scale)
    bw apply KEY=VALUE ...         save params (dotted keys: steer.center_us=1512)
    bw apply --profile NAME        merge a profile into params.json (wltoys | wltoys_bw2 | stock_a1 | bluewave_body | custom);
                                   bw params-undo takes it back.  profiles/wltoys.json <- tools/profile_wltoys.py
                                   (wltoys_bw2.json <- --auto-bw2).  To switch bodies use `bw body NAME` (below)
    bw body [NAME [--yes] | fingerprint [--save]]   which chassis this is vs the applied profile (strap pin 29-30,
                                   lidar fingerprint); NAME switches: apply + stamp + restart the agent + what is due
    bw mat PLAN.json [--from N] [--dry] [--yes]   a scripted mat series: the car's own program from marked starts,
                                   the START button, a report per run, stop rules (plans/*.json; runs/mat/ summary)
    bw analyze RUN_ID|latest [--redo] [--json] [--open] | analyze --local FILE.jsonl   the crash report of a run
    bw params [KEY]                print params (or one dotted key)
    bw run PROGRAM [@PRESET] [k=v ...] [--wait-button [SECONDS]]   start a program, follow its log until it ends
                                   (Ctrl-C = stop it); --wait-button: it starts on the car's START button (hands off)
    bw stop
    bw run watch                   WATCH mode: the car localises while pushed / driven by hand (never drives)
    bw live [SECONDS] [--json]     what the running program believes: pose, state, lap, signs, lot
    bw log [-f] [KIND ...]         the current or last run's log (KIND: ev error pose text data)
    bw sys [--json] | sys ACTION [--yes]   robot health / restart_camera restart_agent reboot shutdown sync_clock
    bw presets PROGRAM [save NAME k=v ... | rm NAME]
    bw replay RUN_ID [--json]      a run's pose, signs, lot and event timeline
    bw params-history | params-undo | params-restore HID
    bw profile [NAME [--apply]]    stock_a1 / bluewave_body / custom: the diff, then apply
    bw run slam                    SLAM: map the room / the field while you drive by hand (never drives itself)
    bw map [OUT.png] [--overlay] [--json]   the served map (SLAM's or the known-pose one) as a PNG for review
    bw map save NAME | map clear --yes | maps   keep / forget / list maps (runs/maps/ on the robot)
    bw cloud OUT.ply [--png OUT.png [--view top|side]]   the HP60C depth cloud (depth.on=1, the 3D view open)
    bw perc [SECONDS] [--json]     what the lidar sees now: segments, signs, limitations, free run ahead / behind
    bw tf                          the sensor tree (xyz / rpy) with rates and ages
    bw recs                        the recordings (runs/rec) a camera fit can use
    bw calib fit [REC|latest] [--kind replay|lidar] [--apply]   fit the camera from a recorded drive (robot-side
                                   process; --apply saves the diff, reason calib:fit) | calib status | calib cancel
    bw runs                        recorded runs on the robot
    bw pull RUN_ID [DIR]           download a run's .jsonl (default mentorpi/runs/)
    bw summary RUN_ID              pull + summarise: duration, min front/side distance, max steer, yaw turned
    bw scan [FILE]                 the latest lidar revolution as JSON
    bw snap FILE.jpg [--overlay]   a camera frame
    bw preflight [pitch] [--field] [--program NAME]   GO / NO-GO: battery (< 7.4 V NO), board, gyro bias, camera
                                   feed (NO where the program needs it, WARN for the WLtoys' lidar programs), race
                                   program, body (the chassis vs the profile), profile stamp, lidar cw / offset (NO) /
                                   block (WARN) vs the stamped profile, camera verified this session
                                   (pitch: from the pillar 500 mm ahead; --field: a still field_check must fit the map)
    bw deploy [--force]            copy bluewave/ + programs/ to the robot and restart the agent (refused while a
                                   program drives the car); restarts bluewave-camera when the depth bridge changed
    bw ssh CMD ...                 run a shell command on the robot
    bw mode dev|race               which service the robot boots into (race = radios OFF, start button)
"""
from __future__ import annotations

import io
import json
import os
import platform
import subprocess
import sys
import tarfile
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
_CFG = {}
try:
    _CFG = json.load(open(os.path.join(ROOT, ".robot.json"), encoding="utf-8"))   # written by join_team_wifi.py
except (OSError, ValueError):
    pass
HOST = os.environ.get("BW_HOST", _CFG.get("ip") or "bluewave.local")
PORT = int(os.environ.get("BW_PORT", "8000"))
USER = os.environ.get("BW_USER", _CFG.get("user") or "bluewave")
KEY = os.path.expanduser("~/.ssh/bluewave_ed25519")
SSH = (["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=accept-new"]
       + (["-i", KEY] if os.path.isfile(KEY) else []))
TOKEN = os.environ.get("BW_TOKEN", "")
BASE = "http://%s:%d" % (HOST, PORT)


def req(method: str, path: str, body=None, timeout: float = 30.0, raw: bool = False):
    data = None if body is None else json.dumps(body).encode()
    r = urllib.request.Request(BASE + path, data=data, method=method)
    r.add_header("Content-Type", "application/json")
    r.add_header("X-BW-Client", "bw/" + (platform.node() or "laptop"))
    if TOKEN:
        r.add_header("X-BW-Token", TOKEN)
    try:
        with urllib.request.urlopen(r, timeout=timeout) as resp:
            b = resp.read()
            return b if raw else json.loads(b or b"null")
    except urllib.error.HTTPError as e:
        msg = e.read().decode(errors="replace")
        sys.exit("robot says %d: %s" % (e.code, msg))
    except urllib.error.URLError as e:
        sys.exit("cannot reach the agent at %s (%s).  Same network?  `bw ssh systemctl status bluewave-agent`"
                 % (BASE, e.reason))


TEST_TIMEOUT_S = 180.0      # bw test's POST: a moving test (duty_sweep ~15 s, coast, step_table) runs while the
#                             keepalive holds the link; 60 s cut the longer ones off on the laptop side only


def keepalive():
    """A STARTED bw_keepalive.KeepAlive: GET /api/status through this module's `req` (looked up per ping: the tests
    swap bw.req) every 0.5 s until .stop() -- bw run / bw test hold the link themselves, no console page needed."""
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import bw_keepalive as K
    return K.KeepAlive(K.status_ping(lambda *a, **k: globals()["req"](*a, **k))).start()


def ireq(*a, **k):
    """req() waited on so that Ctrl-C lands at once, even inside a long request on Windows (bw_keepalive.interruptible):
    with the keepalive running, a Ctrl-C held back until bw test's POST returned would let the test drive on."""
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import bw_keepalive as K
    return K.interruptible(lambda: globals()["req"](*a, **k))


def kv(args) -> dict:
    out = {}
    for a in args:
        k, _, v = a.partition("=")
        try:
            out[k] = json.loads(v)
        except json.JSONDecodeError:
            out[k] = v
    return out


def ssh(cmd: str, check: bool = True) -> int:
    rc = subprocess.call(SSH + ["%s@%s" % (USER, HOST), cmd])
    if check and rc:
        sys.exit("ssh failed (%d) -- is the key installed?  see mentorpi/README.md, 'first connection'" % rc)
    return rc


def line(s: dict) -> str:
    r = s.get("robot", s)
    dp = r.get("depth")                                   # only when depth.on = 1
    dtxt = "" if not isinstance(dp, dict) else (" | depth %s fps" % dp.get("fps") if dp.get("ok") else
                                                " | depth DOWN (%s)" % (dp.get("error") or "no frames"))
    return ("bat %s V | imu %s Hz (age %s ms) | lidar %s rpm | cam %s fps%s | front %s left %s right %s | yaw %s | "
            "v %s steer %s%s" % (r.get("battery_v"), r.get("imu_hz"), r.get("imu_age_ms"), r.get("lidar_rpm"),
                                 r.get("cam_fps"), dtxt, r.get("front_mm"), r.get("left_mm"), r.get("right_mm"),
                                 r.get("yaw"), r.get("v"), r.get("steer"), " | E-STOP" if r.get("estop") else ""))


def summarise(path: str) -> dict:
    tel, logs, meta = [], [], None
    with open(path, encoding="utf-8") as f:
        for l in f:
            r = json.loads(l)
            if "tel" in r:
                tel.append(r["tel"])
            elif "log" in r:
                logs.append(r["log"])
            elif "meta" in r:
                meta = r["meta"]
    if not tel:
        return dict(program=(meta or {}).get("program"), samples=0)
    def col(k):
        return [t[k] for t in tel if t.get(k) is not None]
    yaw = col("yaw")
    turned = 0.0
    for a, b in zip(yaw, yaw[1:]):
        turned += ((b - a + 180) % 360) - 180
    out = dict(program=(meta or {}).get("program"), params=(meta or {}).get("params"), samples=len(tel),
               seconds=round(tel[-1]["t"] - tel[0]["t"], 2), min_front_mm=min(col("front_mm"), default=None),
               min_left_mm=min(col("left_mm"), default=None), min_right_mm=min(col("right_mm"), default=None),
               max_abs_steer=max((abs(x) for x in col("steer")), default=None), yaw_turned_deg=round(turned, 1),
               errors=[l for l in logs if isinstance(l, dict) and "error" in l][:3])
    evs = [l for l in logs if isinstance(l, dict) and "ev" in l]
    out["events"] = {k: sum(1 for l in evs if l["ev"] == k) for k in sorted({l["ev"] for l in evs})}
    end = next((l for l in reversed(evs) if l["ev"] == "end"), None)
    if end:
        out["end"] = {k: end.get(k) for k in ("reason", "laps", "seconds")}
    cpu = next((l for l in reversed(evs) if l["ev"] == "cpu"), None)
    if cpu:
        # the program thread's share and the whole process's (OpenCV's workers): on the Pi, share > 0.25 = NO-GO for
        # the 0.55 / 0.50 step; loop p99 > 100 ms = the shield's latency_s is too short (BRAIN_SPEC 13)
        out["cpu"] = {k: cpu.get(k) for k in ("share", "process_share", "loop_p50_ms", "loop_p99_ms", "pillars_skipped")}
    if tel[-1].get("sim"):
        out["sim_contacts"] = tel[-1]["sim"]["contacts"]
    return out


MAP_PROGRAMS = ("wro_next", "survey")   # they localise on the camera: an unverified camera is a NO for them (8.5)
# the programs the camera is load-bearing for: no frames = NO for them.  The WLtoys' lidar programs (lapstop, and
# open_final's colour cross-check) drive without it: a missing camera is a WARN there ([MAT 2026-09-30]: the camera
# gives colours only on that car)
CAMERA_PROGRAMS = ("wro_next", "lapcam", "survey", "field_check", "wro", "watch")
RACE_PROGRAMS = ("wro_next", "lapcam")   # the race row's answer when the robot's program list cannot be read
# the lidar's angle convention, checked against the stamped profile: cw / offset wrong = a MIRRORED or turned scan --
# [MAT 2026-09-29/30] cw false centred the car into the walls, braked on phantoms and drew a pentagon: NO.  The
# blocked sector only narrows or widens what is used: WARN
LIDAR_HARD = ("lidar.cw", "lidar.offset_deg")
LIDAR_SOFT = ("lidar.block_deg",)


def profile_patch(name: str, req=req) -> tuple:
    """(patch, source) of profile `name`: the laptop's mentorpi/profiles/<name>.json (the repo's numbers, the mat's
    included) first, else the robot's /api/profiles entry (the built-in stock_a1), else (None, why)."""
    if not name or "/" in name or "\\" in name:
        return None, "no profile stamp"
    path = os.path.join(ROOT, "profiles", name + ".json")
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        if isinstance(d.get("patch"), dict):
            return d["patch"], "profiles/%s.json" % name
    except (OSError, ValueError, AttributeError):
        pass
    try:
        pr = req("GET", "/api/profiles") or {}
    except (SystemExit, Exception):
        pr = {}
    if isinstance(pr.get(name), dict) and isinstance(pr[name].get("patch"), dict):
        return pr[name]["patch"], "the robot's profile %s" % name
    return None, "no profile %s here or on the robot" % name


def _dotted(d: dict, key: str):
    for part in key.split("."):
        if not isinstance(d, dict) or part not in d:
            return None
        d = d[part]
    return d


def _same(key: str, have, want) -> bool:
    """A robot value equal to the profile's: booleans as booleans, angles within 0.5 deg (the offset mod 360), the
    blocked sectors pairwise within 0.5 deg."""
    if isinstance(want, bool) or key.endswith(".cw"):
        return have is not None and bool(have) == bool(want)
    if isinstance(want, (int, float)) and isinstance(have, (int, float)):
        d = float(have) - float(want)
        if key.endswith("offset_deg"):
            d = (d + 180.0) % 360.0 - 180.0
        return abs(d) <= 0.5
    if isinstance(want, list) and isinstance(have, list):
        if len(want) != len(have):
            return False
        return all(_same(key, h, w) for h, w in zip(have, want))
    return have == want


def lidar_check(p: dict, name: str, req=req) -> tuple | None:
    """("OK" | "WARN" | "NO", text) of the robot's lidar.cw / offset_deg / block_deg against the stamped profile's, or
    None without a stamp.  The text of a mismatch carries the command that fixes it."""
    if not name:
        return None
    patch, src = profile_patch(name, req)
    if patch is None:
        return "WARN", "cannot compare the lidar convention: %s" % src
    keys = [k for k in LIDAR_HARD + LIDAR_SOFT if k in patch]
    if not keys:
        return None
    diffs = [(k, _dotted(p, k), patch[k]) for k in keys if not _same(k, _dotted(p, k), patch[k])]
    if not diffs:
        return "OK", "%s as %s" % (", ".join("%s %s" % (k.split(".", 1)[1], json.dumps(patch[k])) for k in keys), src)
    hard = [d for d in diffs if d[0] in LIDAR_HARD]
    fix = "bw apply %s%s" % (" ".join("'%s=%s'" % (k, json.dumps(w, separators=(",", ":"))) for k, _h, w in diffs),
                             " then bw sys restart_agent --yes" if hard else "")
    text = "; ".join("%s is %s, %s says %s" % (k, json.dumps(h), src, json.dumps(w)) for k, h, w in diffs)
    if hard:
        text += " -- a MIRRORED / turned scan steers every lidar program into the walls [MAT 2026-09-30]"
    return ("NO" if hard else "WARN"), "%s  (fix: %s)" % (text, fix)


def follow(prog: str, prm: dict, req=req, timeout: float = 90.0) -> tuple:
    """Start `prog` (a STILL program here: field_check) and follow its log until it ends: (run_id, [records])."""
    r = req("POST", "/api/programs/%s/run" % prog, prm or {})
    rid, since, recs, end, last_n = r["run_id"], 0.0, [], time.time() + timeout, 0
    while True:
        lg = req("GET", "/api/programs/log?since=%f" % since)
        mine = lg.get("run_id") == rid
        for l in lg["lines"]:
            since = max(since, l["t"])
            if mine and l.get("n", last_n + 1) > last_n:  # "%f" rounds `since` down: never a record twice
                last_n = l.get("n", last_n + 1)
                recs.append(l["log"])
        if mine and not lg["busy"]:
            return rid, recs
        if time.time() > end:
            req("POST", "/api/programs/stop", {})
            return rid, recs + [{"ev": "end", "reason": "timeout"}]
        time.sleep(0.3)


def field_verdict(recs: list, good: float) -> dict:
    """A still field_check's answer: {standard, verdict, cost, pose} (its `field_check` event; a `fix` event's cost
    when globloc wrote one, BRAIN4_SPEC 4.11 -- the verdict words stay)."""
    ev = next((x for x in reversed(recs) if isinstance(x, dict) and x.get("ev") == "field_check"), None) or {}
    fix = next((x for x in reversed(recs) if isinstance(x, dict) and x.get("ev") == "fix"), None) or {}
    cost = ev.get("cost_per_column", fix.get("cost"))
    verdict = str(ev.get("verdict") or ("no verdict: " + json.dumps(recs[-1:])[:80] if recs else "no log"))
    pose = ev.get("pose") or ([fix["x"], fix["y"], fix["th"]] if {"x", "y", "th"} <= set(fix) else None)
    ok = verdict.startswith("STANDARD") and isinstance(cost, (int, float)) and cost <= good
    return dict(standard=bool(ok), verdict=verdict, cost=cost, pose=pose)


def mark_pose(name: str, profile: str):
    """The pose [x, y, deg] of start mark `name` from the first plans/*.json of this profile (any profile's when none
    names it), or None."""
    import glob
    best = None
    for f in sorted(glob.glob(os.path.join(ROOT, "plans", "*.json"))):
        try:
            with open(f, encoding="utf-8") as fh:
                d = json.load(fh)
        except (OSError, ValueError):
            continue
        st = (d.get("starts") or {}).get(name)
        if not st or not isinstance(st.get("pose"), list):
            continue
        if d.get("profile") == profile:
            return [float(v) for v in st["pose"]]
        best = best or [float(v) for v in st["pose"]]
    return best


def preflight(a, req=req, out=print) -> tuple:
    """GO / NO-GO before any run: battery, board, camera feed, gyro bias (applied), [pitch from one pillar centred 500
    mm in front of the bumper, applied], race program, [--field: a STILL field_check must fit the standard map], and
    BRAIN4_SPEC 8.5's rows: body (the chassis vs the profile: MISMATCH = NO, unknown = WARN), profile (no stamp =
    WARN), camera (not verified since the agent started = NO for the map programs -- the stock mount moves).
    [MAT 2026-09-29/30] rows: lidar (cw / offset_deg vs the stamped profile = NO: the robot ran cw false against the
    profile's true, a mirrored scan; block_deg = WARN), cam_feed without frames = WARN for a program that drives
    without the camera (CAMERA_PROGRAMS need it), the race program any the robot has (lapstop was a NO).
    `--program NAME` names the program the camera rows judge for (default: the race program).
    Returns (go, [(row, "OK" | "NO" | "WARN", text)])."""
    want_pitch, want_field = "pitch" in a, "--field" in a
    prog = a[a.index("--program") + 1] if "--program" in a and a.index("--program") + 1 < len(a) else None
    rows, state = [], dict(go=True)

    def row(name, ok, text, warn=False):
        level = "OK" if ok else ("WARN" if warn else "NO")
        if level == "NO":
            state["go"] = False
        rows.append((name, level, text))
        out("%-10s %-4s %s" % (name, level, text))

    st = req("GET", "/api/status")
    s = st["robot"]
    p = req("GET", "/api/params")
    race = p.get("race", {})
    rprog = race.get("program") or race.get("programs", {}).get(race.get("challenge", "obstacle"))
    prog = prog or rprog
    bat = s.get("battery_v")
    # ONE rule, the runbook's (SUNDAY.md): at rest < 7.4 V = charge (NO); 7.4-7.5 = WARN; a series also stops between
    # runs below its plan's stop_on.battery_v (7.2, read right after a run, under the sag).  This said NO below 7.5
    # while SUNDAY said carry on to 7.4, so every series refused from 7.49 V (review 2026-09-25)
    if not isinstance(bat, (int, float)) or bat <= 0.0:
        row("battery", False, "no battery reading (the RRC board reports it: bw sys, the board row below)")
    else:
        row("battery", bat >= 7.5, "%.2f V (%s)" % (
            bat, "charge it: below 7.4 V at rest" if bat < 7.4 else "low: charge soon (7.4 = NO)" if bat < 7.5 else
            "charge below 7.4"), warn=7.4 <= bat < 7.5)
    for t, label in (("board", "board"), ("camera", "cam_feed")):
        r = req("POST", "/api/tests/" + t, {}, timeout=60)
        if label == "cam_feed" and not r["ok"]:
            # NO only for a program the camera is load-bearing for; the WLtoys' lidar programs drive without it.  No
            # frame after a reboot: the agent (hw.RetryCamera) retries until the container serves it
            need = prog in CAMERA_PROGRAMS
            row(label, False, "%s -- %s; after a Pi reboot the camera container starts late: wait ~30 s (the agent "
                "retries by itself), else bw sys restart_camera --yes" % (
                    r["summary"], "%s needs it" % prog if need else "%s drives without it" % (prog or "the program")),
                warn=not need)
            continue
        row(label, r["ok"], r["summary"])
    r = req("POST", "/api/tests/imu_bias", {}, timeout=60)
    row("gyro", r["ok"], r["summary"])
    if r["ok"] and r["suggest"]:
        req("PUT", "/api/params?reason=preflight:gyro", r["suggest"])
    if want_pitch:
        r = req("POST", "/api/tests/cam_pitch", {}, timeout=60)
        row("cam_pitch", r["ok"], r["summary"])
        if r["ok"] and r["suggest"]:
            req("PUT", "/api/params?reason=preflight:pitch", r["suggest"])      # stamps camera.verified_t (agent)
    p = req("GET", "/api/params")
    # a race program the robot HAS (lapstop / open_final on the WLtoys were a NO here: only wro_next / lapcam passed)
    try:
        known = [m.get("name") for m in req("GET", "/api/programs") or [] if isinstance(m, dict)]
    except (SystemExit, Exception):
        known = list(RACE_PROGRAMS)
    row("race", rprog in known, "challenge %s -> program %s%s" % (
        race.get("challenge"), rprog, "" if rprog in known else "  (not on the robot: bw deploy, or fix: bw apply "
                                                                "race.program=)"))
    if want_field:
        good = float((p.get("loc") or {}).get("good", 0.30))
        _rid, recs = follow("field_check", {}, req=req)
        fv = field_verdict(recs, good)
        # the pose must be ON the mark (up to the field's symmetry, a1_check's 150 mm / 15 deg): a wrong pitch applied
        # just before (a far sign taken for the test pillar) still fits the walls as STANDARD, 513 mm off the mark
        # (review 2026-09-25) -- so a STANDARD off the mark is NO and the camera is NOT stamped verified
        mk_name = a[a.index("--mark") + 1] if "--mark" in a and a.index("--mark") + 1 < len(a) else "A"
        mk = mark_pose(mk_name, (p.get("body") or {}).get("profile") or "")
        me = None
        if fv["pose"] and mk is not None:
            sys.path.insert(0, HERE)
            import bw_mat
            me = bw_mat.mark_error(fv["pose"], mk)
        on_mark = me is None or (me[0] <= bw_mat.MARK_TOL_MM and me[1] <= bw_mat.MARK_TOL_DEG)
        if fv["standard"] and on_mark:
            req("PUT", "/api/params?reason=preflight:field", {"camera.verified_t": 1})   # the robot's clock stamps it
        row("field", fv["standard"], "%s (cost %s, good <= %.2f)%s" % (
            fv["verdict"], fv["cost"], good, "  pose %s" % fv["pose"] if fv["pose"] else ""))
        if mk is None:
            row("mark", False, "no mark %s in a plan for this profile: the pose is not checked" % mk_name, warn=True)
        elif me is not None:
            row("mark", on_mark, "%d mm / %.1f deg from mark %s %s%s" % (
                me[0], me[1], mk_name, mk, "" if on_mark else " -- OFF THE MARK: the car is not on the mark, or the "
                "pitch just applied is wrong (the test pillar missing / at the wrong distance?): fix it, redo step 2"))
        p = req("GET", "/api/params")
    try:
        body = req("GET", "/api/body?fresh=1")
    except (SystemExit, Exception):                      # an agent older than BRAIN4 answers 404
        body = None
    if body is None:
        row("body", False, "the agent has no /api/body: bw deploy", warn=True)
    else:
        ok = body.get("ok")
        row("body", ok is True, ("MISMATCH -- " if ok is False else "") + str(body.get("why")), warn=ok is None)
    bp = (p.get("body") or {}).get("profile") or ""
    row("profile", bool(bp), bp + ((" (shell %s)" % p["body"].get("shell")) if bp and p["body"].get("shell") else "")
        if bp else "no profile stamp: bw body NAME (stock_a1 | wltoys_bw2) --yes", warn=not bp)
    lc = lidar_check(p, bp, req=req)                     # [MAT 2026-09-29/30]: the robot ran cw false, the profile true
    if lc is not None:
        row("lidar", lc[0] == "OK", lc[1], warn=lc[0] == "WARN")
    cam = p["camera"]
    vt, t0 = float(cam.get("verified_t", 0.0) or 0.0), (st.get("host") or {}).get("agent_started")
    geo = "pitch %.2f deg, height %.1f mm" % (cam["pitch_deg"], cam["h_mm"])
    if t0 is None:
        row("camera", False, "%s; the agent is older than BRAIN4 (no agent_started): bw deploy" % geo, warn=True)
    elif vt >= float(t0):
        row("camera", True, "%s, verified %s" % (geo, time.strftime("%H:%M:%S", time.localtime(vt))))
    else:
        row("camera", False, "%s, NOT verified since the agent started -- the camera may have moved: bw preflight "
                             "pitch or --field" % geo, warn=prog not in MAP_PROGRAMS)
    st = req("GET", "/api/status")
    wp = st.get("exit_plans") or {}
    row("plans", wp.get("state") in ("ready", "idle"), "wro_next exit plans: %s%s" % (
        wp.get("state"), "" if wp.get("state") != "computing" else " (wait: a start now would compute them)"))
    if st.get("restart"):
        row("restart", False, "saved but not loaded: %s -> bw sys restart_agent --yes" % ", ".join(st["restart"]))
    return state["go"], rows


def main(argv) -> int:
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__)
        return 0
    c, a = argv[0], argv[1:]
    if c == "status":
        s = req("GET", "/api/status")
        print(line(s))
        p = s["program"]
        print("program: %s%s" % (p["name"] or "none", " (running)" if p["busy"] else ""), "| host", s.get("host"))
    elif c == "watch":
        end = time.time() + (float(a[0]) if a else 1e9)
        try:
            while time.time() < end:
                print(line(req("GET", "/api/status")))
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
    elif c in ("estop", "arm", "beep"):
        print(req("POST", "/api/" + c, {}))
    elif c == "drive":
        print(req("POST", "/api/drive", dict(v=float(a[0]), steer=float(a[1]), ttl_ms=int(a[2]) if len(a) > 2 else 600)))
    elif c == "servo":
        print(req("POST", "/api/servo", {"us": int(a[1])} if a[0] == "--us" else {"deg": float(a[0])}))
    elif c == "tests":
        for t in req("GET", "/api/tests"):
            print("%-16s %s %s" % (t["name"], "[moves]" if t["moves"] else "       ", t["how"]))
    elif c == "test":
        # ONE blocking POST: without the keepalive the hub stopped a moving test 3 s in unless a console was open
        # (safety.link_timeout_ms, [MAT 2026-09-30]).  Ctrl-C stops the test on the robot, not only here
        try:
            with keepalive():
                r = ireq("POST", "/api/tests/" + a[0], kv(a[1:]), timeout=TEST_TIMEOUT_S)
        except KeyboardInterrupt:
            print(req("POST", "/api/tests/stop", {}))
            return 1
        print(("PASS  " if r["ok"] else "CHECK ") + r["summary"])
        print(json.dumps(r["data"], indent=1))
        if r["suggest"]:
            print("suggest:", " ".join("%s=%s" % kvp for kvp in r["suggest"].items()), " -> bw apply ...")
    elif c == "apply" and a and a[0] == "--profile":
        if len(a) < 2:
            sys.exit("bw apply --profile NAME   (bw profile lists them)")
        sys.path.insert(0, HERE)
        import bw_console
        return bw_console.main("profile", [a[1], "--apply"], req)
    elif c == "apply":
        req("PUT", "/api/params", kv(a))
        print("saved", kv(a))
    elif c == "params":
        p = req("GET", "/api/params")
        if a:
            for part in a[0].split("."):
                p = p[part]
        print(json.dumps(p, indent=2))
    elif c == "run":
        rest, wait_s = list(a[1:]), None
        if "--wait-button" in rest:                        # BRAIN4_SPEC 7.5: it starts on the car's START button
            i = rest.index("--wait-button")
            rest.pop(i)
            if i < len(rest) and rest[i].replace(".", "", 1).isdigit():
                wait_s = float(rest.pop(i))
            else:
                wait_s = 0.0                               # 0 = the robot's mat.wait_s
        pre = [x[1:] for x in rest if x.startswith("@")]
        q = ["preset=" + pre[0]] if pre else []
        if wait_s is not None:
            q += ["wait=button"] + (["wait_s=%g" % wait_s] if wait_s else [])
        r = req("POST", "/api/programs/%s/run%s" % (a[0], "?" + "&".join(q) if q else ""),
                kv([x for x in rest if not x.startswith("@")]))
        print("run", r["run_id"])
        if wait_s is not None:
            print("waiting for the START button on the car: hands off, then press it (Ctrl-C = cancel)")
        since, last_n = 0.0, 0
        ka = keepalive()                                   # the link renewed < 1 s apart while it runs: no console
        try:
            while True:
                lg = ireq("GET", "/api/programs/log?since=%f" % since)
                for l in lg["lines"]:
                    since = max(since, l["t"])
                    if lg.get("run_id") == r["run_id"] and l.get("n", last_n + 1) <= last_n:
                        continue                           # "%f" rounds `since` down: never print a record twice
                    last_n = l.get("n", last_n + 1) if lg.get("run_id") == r["run_id"] else last_n
                    print(l["log"] if isinstance(l["log"], str) else json.dumps(l["log"]))
                if not lg["busy"]:
                    break
                time.sleep(0.3)
        except KeyboardInterrupt:
            req("POST", "/api/programs/stop", {})
            print("stopped")
        finally:
            ka.stop()
        if ka.fails or ka.max_gap_s > 1.0:
            print("link: %d of %d keepalive requests failed, longest gap %.1f s%s" % (
                ka.fails, ka.sent, ka.max_gap_s, " (%s)" % ka.last_error[:80] if ka.last_error else ""))
        print("run id:", r["run_id"], " -> bw summary", r["run_id"])
    elif c == "stop":
        print(req("POST", "/api/programs/stop", {}))
    elif c == "runs":
        for r in req("GET", "/api/runs"):
            print("%-40s %8.1f kB" % (r["id"], r["bytes"] / 1024))
    elif c in ("pull", "summary"):
        d = a[1] if len(a) > 1 else os.path.join(ROOT, "runs")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, a[0] + ".jsonl")
        with open(path, "wb") as f:
            f.write(req("GET", "/api/runs/" + a[0], raw=True))
        print(path) if c == "pull" else print(json.dumps(summarise(path), indent=1))
    elif c == "scan":
        s = req("GET", "/api/scan")
        if a:
            json.dump(s, open(a[0], "w"))
            print(a[0])
        else:
            print(json.dumps({k: s[k] for k in s if k not in ("dist", "conf")}))
    elif c == "snap":
        ov = "--overlay" in a
        out = [x for x in a if not x.startswith("--")][0]
        open(out, "wb").write(req("GET", "/api/camera.jpg?overlay=%d" % ov, raw=True))
        print(out)
    elif c == "deploy":
        # never under a moving car: the restart kills the program mid-command (the WLtoys PWM then relies on the
        # unit's ExecStopPost alone).  An unreachable agent is not a reason to refuse: the robot may be booting
        try:
            st = json.loads(urllib.request.urlopen(BASE + "/api/status", timeout=3).read() or b"null") or {}
        except Exception:
            st = {}
        pr = st.get("program") or {}
        if pr.get("busy") and pr.get("drives", True) and "--force" not in a:
            sys.exit("%s is running and drives the car: bw stop first (or bw deploy --force)" % pr.get("name"))
        buf = io.BytesIO()
        with tarfile.open(fileobj=buf, mode="w:gz") as tf:
            for sub in ("bluewave", "programs", "race", "profiles"):
                p = os.path.join(ROOT, sub)
                if os.path.isdir(p):
                    tf.add(p, arcname=sub, filter=lambda ti: None if "__pycache__" in ti.name else ti)
        data = buf.getvalue()
        # the depth bridge runs INSIDE Hiwonder's container, copied in by bluewave-camera's ExecStartPre: a changed
        # bridge reaches it only when that service restarts (the stock card; the BlueWave OS card has no such unit)
        br = "~/bluewave/bluewave/depth_bridge.py"
        p = subprocess.run(SSH + ["%s@%s" % (USER, HOST),
                            "old=$(sha256sum %s 2>/dev/null | cut -c1-64); mkdir -p ~/bluewave && tar -xzf - -C "
                            "~/bluewave && sudo systemctl restart bluewave-agent && echo deployed; "
                            "new=$(sha256sum %s 2>/dev/null | cut -c1-64); if [ \"$old\" != \"$new\" ] && "
                            "systemctl cat bluewave-camera >/dev/null 2>&1; then sudo systemctl restart bluewave-camera "
                            "&& echo 'depth bridge changed: bluewave-camera restarted'; fi" % (br, br)], input=data)
        return p.returncode
    elif c == "preflight":
        go, rows = preflight(a)
        warns = [r[0] for r in rows if r[1] == "WARN"]
        print("\n%s" % (("GO" + ("  (warnings: %s)" % ", ".join(warns) if warns else "")) if go else
                        "NO-GO -- fix the NO lines first"))
        return 0 if go else 1
    elif c == "body":
        sys.path.insert(0, HERE)
        import bw_body                                     # the chassis check and the body switch (BRAIN4_SPEC 8.4)
        return bw_body.main(a, req)
    elif c == "mat":
        sys.path.insert(0, HERE)
        import bw_mat                                      # scripted mat series (BRAIN4_SPEC 7)
        return bw_mat.main(a)
    elif c == "analyze":
        sys.path.insert(0, HERE)
        try:
            import bw_analyze                              # the crash report (BRAIN4_SPEC 6.9, builder B3)
        except ImportError as e:
            sys.exit("bw analyze is not installed here yet (tools/bw_analyze.py): %s" % e)
        return bw_analyze.main(a, req)
    elif c in ("map", "maps", "cloud"):
        sys.path.insert(0, HERE)
        import bw_map                                      # the Map page's map and cloud (docs/BRAIN_SPEC.md 8.5)
        return bw_map.main(c, a, req)
    elif c in ("perc", "tf", "recs", "calib"):
        sys.path.insert(0, HERE)
        import bw_brain                                    # lidar perception, TF, recordings, the camera fit (8.5, 12.4)
        return bw_brain.main(c, a, req)
    elif c in ("sys", "live", "log", "presets", "replay", "params-history", "params-undo", "params-restore", "profile"):
        sys.path.insert(0, HERE)
        import bw_console                                  # the console v2 commands (docs/CONSOLE_SPEC.md 8)
        return bw_console.main(c, a, req)
    elif c == "ssh":
        return ssh(" ".join(a), check=False)
    elif c == "mode":
        if a[0] not in ("dev", "race"):
            sys.exit("bw mode dev|race")
        return ssh("echo %s | sudo tee /boot/firmware/bluewave.mode >/dev/null && echo 'next boot: %s'" % (a[0], a[0]))
    else:
        sys.exit("unknown command %s (bw help)" % c)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
