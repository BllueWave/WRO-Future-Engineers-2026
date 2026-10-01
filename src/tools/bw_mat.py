"""bw mat -- a scripted series of real runs on the mat, one run at a time (BRAIN4_SPEC 7).

    bw mat PLAN.json [--from N] [--dry] [--yes] [--out DIR]

Each run is the robot's OWN program started from a marked pose with the car's START button (the link was too slow to
save the car from a wall: nothing here steers it).  While a run lives bw mat holds the link itself (bw_keepalive: a
GET every 0.5 s; the hub stops a drives program 3 s without one): no console page needed.  After each run bw mat waits
for the robot's crash report, prints one row, checks the plan's stop rules, cools down, and names the next mark.  It
only starts, stops and reads (S17): a run starts only after the previous one ended, its stop rules passed and the
cooldown ran; an E-STOP ends the series.

    --dry        check the plan (program, marks, profile) and print every run with its tape line; start nothing
    --from N     start at run N (after a stop: the summary names it)
    --yes        no "start the series?" question
    --out DIR    reports and the summary go to DIR and DIR/mat (default mentorpi/runs, mentorpi/runs/mat)

The plan (plans/*.json; the keys and their meaning: BRAIN4_SPEC 7.2):
    {"name", "profile", "program", "desc", "params", "starts": {MARK: {"pose": [x, y, deg], "tape": "..."}},
     "runs": [{"start": MARK, "repeat": N, "params": {...}}], "precheck": ["preflight", "field_check", "body"],
     "start_by": "button" | "enter" | "auto", "countdown_s": 5, "expect": true, "cooldown_s": null,
     "run_timeout_s": 200, "report_timeout_s": 90, "stop_on": {...}, "ledger": "old" | "new" | null}

Marks are field-frame poses of the REAR AXLE's centre: x right, y up (north), mm from the field's centre; tape = x + 1500
from the west wall's inner face, y + 1500 from the south wall's inner face.  Output: one row per run (the table of 7.4),
`runs/<id>.report.{json,html}`, and `runs/mat/<plan>-<time>.json` = {plan, started, rows, stopped_by, rungs} -- what
the team reads to fix one thing before the next series.  Stdlib only.
"""
from __future__ import annotations

import copy
import json
import math
import os
import platform
import subprocess
import sys
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WRO = os.path.dirname(ROOT)
OUT_DIR = os.path.join(ROOT, "runs")

FIELD_HALF_MM = 1500.0          # [RULE] the outer walls' inner faces at +-1500 (bluewave/field.py HALF)
ISLAND_HALF_MM = 500.0          # [RULE] the standard island's faces at +-500 (field.py ISL)
MARK_WALL_MM = 200.0            # [BRAIN4_SPEC 7.3] a mark >= 200 mm from every wall
MARK_TOL_MM, MARK_TOL_DEG = 150.0, 15.0   # a field_check pose "at the mark" (7.6 a1_check; loc.expect_mm / _deg)
GATE_NEAR_MM = 20.0             # [BRAIN4_SPEC 9.4] a rung passes with every run CLEAN or NEAR >= 20 mm ...
GATE_LOOP_P99_MS = 100.0        # ... loop p99 <= 100 ms ...
GATE_BAT_MIN_V = 7.0            # ... and battery min >= 7.0 V
COMPACT = ("fix", "start_check", "direction", "lost", "shield", "end", "wait_button", "button", "locate", "look",
           "not_standard", "join_blocked", "handover", "segment_end", "survey_saved", "field_check", "warn",
           "camera_stale", "camera_dead", "repeat_contact", "park_failed", "park_blocked", "bump")
ALWAYS_STOP = ("timeout", "no_button", "refused", "estop", "stopped")   # nothing sensible follows these: the series
#                                                                         stops (a Stop pressed means the team's hand)
DEFAULT_STOP = dict(contact=True, bump=False, near=False, loc_loss=False, estop=True,
                    end_reasons=["stuck", "lost", "lidar_dead", "board_lost", "park_blocked", "park_failed", "camera_dead", "repeat_contact", "battery", "error",
                                 "start_mismatch", "not_standard"],
                    battery_v=7.2, analysis_failed=True, not_standard=True, off_mark=True)
PLAN_KEYS = {"name", "profile", "program", "desc", "params", "starts", "runs", "precheck", "start_by", "countdown_s",
             "expect", "cooldown_s", "run_timeout_s", "report_timeout_s", "stop_on", "ledger", "notes"}
COMPASS = {0: "east", 45: "north-east", 90: "north", 135: "north-west", 180: "west", 225: "south-west",
           270: "south", 315: "south-east"}
LOCAL_ANALYZER = True           # False: a robot without the report route gets the run file's own summary only (tests)


class HttpError(Exception):
    def __init__(self, code: int, text: str):
        super().__init__("%s: %s" % (code, text))
        self.code, self.text = int(code), str(text)


def make_req():
    """tools/bw.py's robot address and token, but an error RAISES (HttpError) instead of exiting: a 404 / 409 from
    the report route is an answer here, not a failure."""
    sys.path.insert(0, HERE)
    import bw

    def req(method: str, path: str, body=None, timeout: float = 30.0, raw: bool = False):
        data = None if body is None else json.dumps(body).encode()
        r = urllib.request.Request(bw.BASE + path, data=data, method=method)
        r.add_header("Content-Type", "application/json")
        r.add_header("X-BW-Client", "bw-mat/" + (platform.node() or "laptop"))
        if bw.TOKEN:
            r.add_header("X-BW-Token", bw.TOKEN)
        try:
            with urllib.request.urlopen(r, timeout=timeout) as resp:
                b = resp.read()
                return b if raw else json.loads(b or b"null")
        except urllib.error.HTTPError as e:
            raise HttpError(e.code, e.read().decode(errors="replace"))
        except (urllib.error.URLError, OSError) as e:
            raise HttpError(0, "cannot reach the agent at %s (%s)" % (bw.BASE, getattr(e, "reason", e)))
    return req


def _detail(e: Exception) -> str:
    t = getattr(e, "text", None) or str(e)
    try:
        return str(json.loads(t).get("detail", t))
    except (ValueError, AttributeError):
        return t


# ---------------------------------------------------------------------------------------------------------- the plan
def find_plan(path: str) -> str | None:
    for cand in (path, os.path.join(ROOT, path), os.path.join(ROOT, "plans", path),
                 os.path.join(ROOT, "plans", path + ".json")):
        if os.path.isfile(cand):
            return cand
    return None


def facing(deg: float) -> str:
    d = float(deg) % 360.0
    k = int(round(d / 45.0)) * 45 % 360
    if abs(((d - k + 180.0) % 360.0) - 180.0) < 1.0:
        return COMPASS[k]
    return "%g deg (0 = east, 90 = north)" % round(d, 1)


def mark_problem(pose) -> str | None:
    """Why a mark is not a start the rules allow (inside the corridor ring, >= 200 mm from every wall), or None."""
    try:
        x, y, deg = (float(v) for v in pose)
    except (TypeError, ValueError):
        return "pose must be [x, y, deg]"
    if not all(math.isfinite(v) for v in (x, y, deg)):
        return "pose must be finite"
    d_out = FIELD_HALF_MM - max(abs(x), abs(y))
    if d_out < MARK_WALL_MM:
        return "%.0f mm from an outer wall (>= %.0f)" % (d_out, MARK_WALL_MM)
    d_isl = math.hypot(max(abs(x) - ISLAND_HALF_MM, 0.0), max(abs(y) - ISLAND_HALF_MM, 0.0))
    if d_isl < MARK_WALL_MM:
        return "%.0f mm from the island (>= %.0f)" % (d_isl, MARK_WALL_MM)
    return None


def tape_line(name: str, mark: dict) -> str:
    x, y, deg = (float(v) for v in mark["pose"])
    head = "mark %s (%.0f, %.0f, %g deg)" % (name, x, y, deg)
    if mark.get("tape"):
        return "%s = %s" % (head, mark["tape"])
    return ("%s = %.0f mm from the west wall, %.0f mm from the south wall, facing %s (the rear axle's centre on the "
            "mark).  Nothing within 550 mm ahead or 250 mm beside it." % (
                head, x + FIELD_HALF_MM, y + FIELD_HALF_MM, facing(deg)))


def validate(plan: dict) -> list:
    """Static problems of a plan (nothing asked of the robot)."""
    bad = []
    if not isinstance(plan, dict):
        return ["the plan is not a JSON object"]
    for k in ("name", "profile", "program", "starts", "runs"):
        if not plan.get(k):
            bad.append("missing %s" % k)
    extra = set(plan) - PLAN_KEYS
    if extra:
        bad.append("unknown keys %s (typo?)" % ", ".join(sorted(extra)))
    starts = plan.get("starts") or {}
    if not isinstance(starts, dict):
        bad.append("starts must be {MARK: {pose: [x, y, deg]}}")
        starts = {}
    for name, m in starts.items():
        why = mark_problem((m or {}).get("pose")) if isinstance(m, dict) else "not {pose: [...]}"
        if why:
            bad.append("mark %s: %s" % (name, why))
    for i, r in enumerate(plan.get("runs") or [], 1):
        if not isinstance(r, dict) or r.get("start") not in starts:
            bad.append("run %d: start %r is not one of the marks %s" % (i, (r or {}).get("start"), sorted(starts)))
            continue
        rep = r.get("repeat", 1)
        if not isinstance(rep, int) or isinstance(rep, bool) or not 1 <= rep <= 50:
            bad.append("run %d: repeat must be 1-50" % i)
        if not isinstance(r.get("params", {}), dict):
            bad.append("run %d: params must be an object" % i)
    if plan.get("start_by", "button") not in ("button", "enter", "auto"):
        bad.append("start_by must be button, enter or auto")
    for pc in plan.get("precheck") or []:
        if pc not in ("preflight", "field_check", "body"):
            bad.append("precheck %r (one of preflight, field_check, body)" % pc)
    if plan.get("ledger") not in (None, "old", "new"):
        bad.append("ledger must be old, new or null")
    so = plan.get("stop_on") or {}
    unk = set(so) - set(DEFAULT_STOP)
    if unk:
        bad.append("stop_on: unknown %s" % ", ".join(sorted(unk)))
    if not isinstance(plan.get("params", {}), dict):
        bad.append("params must be an object")
    return bad


def expand(plan: dict) -> list:
    """[{n, start, pose, params}] in order; a run's params over the plan's (the mark's pose as `expect` is added
    later, when the program takes one)."""
    out, n = [], 0
    for r in plan["runs"]:
        for _ in range(int(r.get("repeat", 1))):
            n += 1
            out.append(dict(n=n, start=r["start"], pose=list(plan["starts"][r["start"]]["pose"]),
                            params=dict(copy.deepcopy(plan.get("params") or {}), **copy.deepcopy(r.get("params") or {}))))
    return out


def stop_rules(plan: dict) -> dict:
    so = dict(DEFAULT_STOP)
    so.update(plan.get("stop_on") or {})
    return so


# ---------------------------------------------------------------------------------------------------------- one run
def compact(t: float, x) -> str:
    if isinstance(x, dict) and "error" in x:
        return "  %6.1f  ERROR %s" % (t, str(x["error"]).strip().splitlines()[-1][:110])
    if not isinstance(x, dict):
        return "  %6.1f  %s" % (t, str(x)[:110])
    ev = x.get("ev")
    parts = []
    for k, v in x.items():
        if k in ("ev", "t", "params", "hyps", "robot_params"):
            continue
        if isinstance(v, float):
            v = round(v, 3)
        s = json.dumps(v) if not isinstance(v, str) else v
        parts.append("%s=%s" % (k, s if len(s) <= 40 else s[:37] + "..."))
    return ("  %6.1f  %-12s %s" % (t, ev, " ".join(parts)))[:150]


def follow_run(req, rid: str, run_timeout_s: float, wait_mode: bool, out, t_post: float) -> dict:
    """Poll the log until the run ends; its END event, the button, E-STOP, a timeout (then bw stop)."""
    since, recs = 0.0, []
    end_ev, pressed = None, not wait_mode
    t_go = time.time() if pressed else None
    estop = timed_out = False
    last_status, stop_t, lost_t = 0.0, None, None
    t_ref = None                                         # the run's first record, ROBOT clock (the Pi's may be off)
    last_n = 0                                           # "%f" rounds `since` down: a record can come twice -- by n
    while True:
        lg = req("GET", "/api/programs/log?since=%f" % since)
        mine = lg.get("run_id") == rid
        if mine:
            lost_t = None
            for l in lg["lines"]:
                since = max(since, l["t"])
                if l.get("n", last_n + 1) <= last_n:
                    continue
                last_n = l.get("n", last_n + 1)
                t_ref = l["t"] if t_ref is None else t_ref
                x = l["log"]
                recs.append(x)
                if isinstance(x, dict):
                    ev = x.get("ev")
                    if ev == "button":
                        pressed, t_go = True, time.time()
                    elif ev == "end":
                        end_ev = x
                    if ev in COMPACT or "error" in x:
                        out(compact(l["t"] - t_ref, x))
            if not lg["busy"]:
                break
        else:
            lost_t = lost_t or time.time()
            if time.time() - lost_t > 5.0:               # another run replaced ours: never follow a stranger
                out("  the robot's current run is %s, not %s: stopped following" % (lg.get("run_id"), rid))
                break
        now = time.time()
        if now - last_status >= 1.0:
            last_status = now
            try:
                if (req("GET", "/api/status").get("robot") or {}).get("estop"):
                    if not estop:
                        out("  E-STOP pressed: the series ends after this run")
                    estop = True
            except HttpError:
                pass
        if pressed and not timed_out and t_go is not None and now - t_go > run_timeout_s:
            out("  run_timeout_s %.0f passed: bw stop" % run_timeout_s)
            try:
                req("POST", "/api/programs/stop", {})
            except HttpError:
                pass
            timed_out, stop_t = True, now
        if timed_out and now - stop_t > 15.0:
            out("  the program did not end 15 s after the stop: press E-STOP")
            break
        time.sleep(0.3)
    return dict(end=end_ev, pressed=pressed, estop=estop, timed_out=timed_out, recs=recs,
                errors=[x for x in recs if isinstance(x, dict) and "error" in x])


def save_bytes(path: str, data: bytes) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)


def save_json(path: str, obj) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, indent=1, default=str)
        f.write("\n")
    os.replace(tmp, path)


def runfile_report(path: str) -> dict:
    """The least a run file says without the analyzer: the end, laps, time, the battery; in a simulator run the
    TRUE contacts (tel.sim.contacts).  Never a verdict it cannot back: without truth the verdict is UNANALYSED."""
    tel, end, errors, t0 = [], None, 0, None
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            t0 = t0 if t0 is not None else r.get("t")
            if "tel" in r:
                tel.append(r["tel"])
            elif isinstance(r.get("log"), dict):
                if r["log"].get("ev") == "end":
                    end = r["log"]
                if "error" in r["log"]:
                    errors += 1
    sim = [t["sim"]["contacts"] for t in tel if isinstance(t.get("sim"), dict) and "contacts" in t["sim"]]
    truth = (max(sim) - sim[0]) if sim else None
    bats = [t["battery_v"] for t in tel if isinstance(t.get("battery_v"), (int, float))]
    verdict = "INCOMPLETE" if end is None or errors else ("CONTACT" if truth else "CLEAN") if truth is not None \
        else "UNANALYSED"
    return {"schema": "bw.mat-runfile/1", "verdict": verdict,
            "end": {k: (end or {}).get(k) for k in ("reason", "laps", "seconds")},
            "duration_s": round(tel[-1]["t"] - tel[0]["t"], 1) if len(tel) > 1 else None,
            "counts": {"contact": truth, "bump": None, "near": None, "loc_loss": None, "intervention": None},
            "score": {"min_clearance_mm": None}, "incidents": [],
            "stats": {"loop_ms": None, "battery": {"min_v": min(bats) if bats else None,
                                                   "end_v": bats[-1] if bats else None}},
            "sources": {"truth": truth is not None, "analyzer": None}}


def local_report(req, rid: str, out_dir: str, out) -> tuple:
    """The robot has no report route (an agent before BRAIN4 6.9): pull the run file and analyse it HERE with
    bluewave.analyze (no recording: IMU and tel only), else the run file's own summary."""
    path = os.path.join(out_dir, rid + ".jsonl")
    try:
        save_bytes(path, req("GET", "/api/runs/" + rid, raw=True, timeout=60))
    except HttpError as e:
        return None, "none", "failed: cannot pull the run file (%s)" % _detail(e)
    if LOCAL_ANALYZER and os.path.isfile(os.path.join(ROOT, "bluewave", "analyze.py")):
        env = {k: v for k, v in os.environ.items() if k != "BW_MOCK"}
        try:
            p = subprocess.run([sys.executable, "-m", "bluewave.analyze", path, "--rec", "none", "--out-dir", out_dir,
                                "--json-progress"], cwd=ROOT, env=env, capture_output=True, text=True, timeout=180)
            rp = os.path.join(out_dir, rid + ".report.json")
            if os.path.isfile(rp):
                with open(rp, encoding="utf-8") as f:
                    rep = json.load(f)
                return rep, "local", "failed: " + str(rep["error"])[:80] if rep.get("error") else "ready"
            out("  local analysis wrote no report (exit %s): %s" % (p.returncode, (p.stderr or p.stdout)[-200:]))
        except (OSError, subprocess.SubprocessError, ValueError) as e:
            out("  local analysis failed: %s" % e)
    rep = runfile_report(path)
    return rep, "runfile", "truth" if rep["sources"]["truth"] else "none"


def fetch_report(req, rid: str, timeout_s: float, out_dir: str, out) -> tuple:
    """(report | None, src "robot" | "local" | "runfile" | "none", state "ready" | "truth" | "none" | "failed: ..." |
    "timeout").  Polls GET /api/runs/<id>/report (409 = analysing) up to `timeout_s`; asks for the analysis once when
    the robot has not queued it (an.auto 0); saves the JSON and HTML beside the other reports."""
    deadline, asked, route, said = time.time() + timeout_s, False, None, 0.0
    while True:
        try:
            rep = req("GET", "/api/runs/%s/report" % rid, timeout=30)
            save_json(os.path.join(out_dir, rid + ".report.json"), rep)
            try:
                save_bytes(os.path.join(out_dir, rid + ".report.html"),
                           req("GET", "/api/runs/%s/report.html" % rid, raw=True, timeout=60))
            except HttpError:
                pass
            return rep, "robot", ("failed: " + str(rep["error"])[:80]) if rep.get("error") else "ready"
        except HttpError as e:
            if e.code == 409:
                if time.time() - said > 10.0:
                    said = time.time()
                    out("  analysing on the robot ... %s" % _detail(e)[:80])
            elif e.code == 404:
                if route is None:
                    try:
                        job = req("GET", "/api/analyze")
                        route = True
                    except HttpError as e2:
                        route = e2.code != 404
                        job = None
                    if not route:
                        out("  the robot has no report route (an older agent): analysing here")
                        return local_report(req, rid, out_dir, out)
                    queued = job and (job.get("run_id") == rid or rid in (job.get("queue") or []))
                    try:
                        st = json.loads(_detail(e))                  # runs_api: {"state", "error"} of this run
                    except (ValueError, TypeError):
                        st = {}
                    if isinstance(st, dict) and st.get("state") == "failed" and asked:
                        return None, "robot", "failed: %s" % str(st.get("error"))[:100]
                    if not queued and not asked:
                        asked = True
                        try:
                            req("POST", "/api/runs/%s/analyze" % rid, {"redo": False})
                        except HttpError as e3:
                            out("  could not ask for the analysis: %s" % _detail(e3)[:100])
            else:
                return None, "robot", "failed: %s" % _detail(e)[:100]
        if time.time() > deadline:
            return None, "robot", "timeout"
        time.sleep(1.0)


def first_incident(rep: dict | None) -> str:
    incs = (rep or {}).get("incidents") or []
    pick = next((i for i in incs if i.get("kind") == "contact"), None) or \
        next((i for i in incs if (i.get("sev") or 0) >= 2), None)
    if not pick:
        return ""
    w = pick.get("where") or {}
    cause = (pick.get("cause") or {}).get("code") or ""
    return " ".join(str(v) for v in ("%.1f s" % float(pick.get("t") or 0.0), pick.get("kind"), w.get("object"),
                                     w.get("part"), pick.get("state"), cause) if v)


def mark_error(pose, mark) -> tuple:
    """(mm, deg) from a field_check pose to the mark, the smallest over the field's 4-fold symmetry."""
    best = (math.inf, math.inf)
    x, y, th = (float(v) for v in pose)
    mx, my, md = (float(v) for v in mark)
    for k in range(4):
        c, s = math.cos(k * math.pi / 2), math.sin(k * math.pi / 2)
        rx, ry, rth = x * c - y * s, x * s + y * c, th + 90.0 * k
        d = math.hypot(rx - mx, ry - my)
        a = abs(((rth - md + 180.0) % 360.0) - 180.0)
        if (d, a) < best:
            best = (d, a)
    return best


def make_row(run: dict, rid: str, fol: dict, rep, src: str, state: str, rest_v, drives: bool) -> dict:
    end = fol.get("end") or {}
    reason = "timeout" if fol.get("timed_out") else end.get("reason") or (rep or {}).get("end", {}).get("reason") or \
        ("error" if fol.get("errors") else "?")
    counts = dict((rep or {}).get("counts") or {})
    stats = (rep or {}).get("stats") or {}
    loop = stats.get("loop_ms") or {}
    bat = stats.get("battery") or {}
    row = dict(n=run["n"], mark=run["start"], run_id=rid, end=reason, laps=end.get("laps"),
               secs=end.get("seconds") or (rep or {}).get("duration_s"), verdict=(rep or {}).get("verdict") or "-",
               counts=counts, min_clearance_mm=((rep or {}).get("score") or {}).get("min_clearance_mm"),
               loop_p99_ms=loop.get("p99") if isinstance(loop, dict) else None,
               battery_min_v=bat.get("min_v"), battery_rest_v=rest_v, first_incident=first_incident(rep),
               report=None, src=src, analysis=state, estop=bool(fol.get("estop")), params=run["params"])
    if not drives:
        row["analysis"] = "still"
    fc = next((x for x in reversed(fol.get("recs") or []) if isinstance(x, dict) and x.get("ev") == "field_check"),
              None)
    if fc is not None:                                   # a field_check run: the verdict and the pose vs the mark
        v = str(fc.get("verdict") or "")
        row["verdict"] = ("STANDARD" if v.startswith("STANDARD") else "NOT" if v.startswith("NOT") else
                          "MOVING" if "moving" in v else "UNCLEAR")
        row["field_cost"] = fc.get("cost_per_column")
        if fc.get("pose"):
            d, a = mark_error(fc["pose"], run["pose"])
            row["mark_err"] = [round(d), round(a, 1)]
            row["first_incident"] = "pose %s: %.0f mm / %.1f deg from the mark%s" % (
                fc["pose"], d, a, "" if d <= MARK_TOL_MM and a <= MARK_TOL_DEG else "  OFF THE MARK")
    return row


def row_text(r: dict) -> str:
    c = r.get("counts") or {}

    def f(v, fmt="%s"):
        return "-" if v is None else fmt % v
    return "%2d  %-4s  %-13s %5s  %5s  %4s  %4s  %6s  %4s  %6s  %5s  %7s  %-9s %s" % (
        r["n"], r["mark"], str(r["end"])[:13], f(r.get("laps"), "%.2f"), f(r.get("secs"), "%.1f"), f(c.get("contact")),
        f(c.get("near")), f(r.get("min_clearance_mm"), "%.0f"), f(c.get("loc_loss")), f(c.get("intervention")),
        f(r.get("loop_p99_ms"), "%.0f"), f(r.get("battery_min_v"), "%.2f"), r.get("verdict"),
        r.get("first_incident") or "")


HEADER = " #  mark  end            laps   secs  cont  near  minclr  lost  interv  p99ms  bat_min  verdict   first incident"


def stop_reason(r: dict, so: dict, program_is_check: bool) -> str | None:
    if r.get("estop"):
        return "E-STOP (the series ends: S17)"
    if r["end"] in ALWAYS_STOP:
        return "the run ended %s" % r["end"]
    if r["end"] in (so.get("end_reasons") or []):
        return "end reason %s (stop_on.end_reasons)" % r["end"]
    c = r.get("counts") or {}
    for key in ("contact", "bump", "near", "loc_loss"):
        if so.get(key) and (c.get(key) or 0) > 0:
            return "%s %s (stop_on.%s)" % (c.get(key), key, key)
    if so.get("analysis_failed") and r.get("analysis") not in ("ready", "truth", "still"):
        return "no crash report: %s (stop_on.analysis_failed)" % r.get("analysis")
    bv = so.get("battery_v")
    if bv and isinstance(r.get("battery_rest_v"), (int, float)) and r["battery_rest_v"] < float(bv):
        return "battery %.2f V at rest < %.1f (stop_on.battery_v): charge" % (r["battery_rest_v"], float(bv))
    if program_is_check:
        if so.get("not_standard") and r.get("verdict") != "STANDARD":
            return "field_check says %s (stop_on.not_standard): camera or field, before any driving" % r.get("verdict")
        me = r.get("mark_err")
        if so.get("off_mark") and me and (me[0] > MARK_TOL_MM or me[1] > MARK_TOL_DEG):
            return "the pose is %d mm / %.1f deg off the mark (stop_on.off_mark): camera or field" % (me[0], me[1])
    return None


def rungs(rows: list, runs: list) -> list:
    """BRAIN4_SPEC 9.4 per distinct params set: every run CLEAN or NEAR >= 20 mm, 0 contacts, 0 loc losses, loop p99
    <= 100 ms, battery min >= 7.0 V; `complete` when every planned run of the rung ran."""
    out, order = [], []
    for r in runs:
        k = json.dumps(r["params"], sort_keys=True)
        if k not in order:
            order.append(k)
    for k in order:
        planned = [r for r in runs if json.dumps(r["params"], sort_keys=True) == k]
        done = [r for r in rows if json.dumps(r.get("params") or {}, sort_keys=True) == k]
        why = []
        for r in done:
            c = r.get("counts") or {}
            clean = r.get("verdict") in ("CLEAN", "REFUSED") or (r.get("verdict") == "NEAR" and
                                                     (r.get("min_clearance_mm") or -1) >= GATE_NEAR_MM)
            if not clean:
                why.append("run %d %s" % (r["n"], r.get("verdict")))
            if (c.get("contact") or 0) or (c.get("loc_loss") or 0):
                why.append("run %d contact %s loc_loss %s" % (r["n"], c.get("contact"), c.get("loc_loss")))
            if (r.get("loop_p99_ms") or 0) > GATE_LOOP_P99_MS:
                why.append("run %d loop p99 %s ms" % (r["n"], r.get("loop_p99_ms")))
            if isinstance(r.get("battery_min_v"), (int, float)) and r["battery_min_v"] < GATE_BAT_MIN_V:
                why.append("run %d battery min %.2f V" % (r["n"], r["battery_min_v"]))
        out.append(dict(params=json.loads(k), planned=len(planned), done=len(done),
                        complete=len(done) == len(planned), passed=bool(done) and len(done) == len(planned) and not why,
                        why=why))
    return out


def enter_pressed() -> bool:
    """A non-blocking check for Enter (skips the rest of a cooldown)."""
    try:
        if os.name == "nt":
            import msvcrt
            hit = False
            while msvcrt.kbhit():
                hit = msvcrt.getwch() in "\r\n" or hit
            return hit
        import select
        if not sys.stdin.isatty():
            return False
        r, _w, _x = select.select([sys.stdin], [], [], 0)
        if r:
            sys.stdin.readline()
            return True
    except Exception:
        return False
    return False


# ---------------------------------------------------------------------------------------------------------- series
class Series:
    def __init__(self, plan: dict, plan_path: str, req, out, inp, out_dir: str, keys=enter_pressed,
                 sleep=time.sleep):
        self.plan, self.plan_path, self.req, self.out, self.inp = plan, plan_path, req, out, inp
        self.out_dir, self.keys, self.sleep = out_dir, keys, sleep
        self.runs = expand(plan)
        self.so = stop_rules(plan)
        self.rows, self.stopped_by, self.active = [], None, None
        self.started = time.strftime("%Y-%m-%d %H:%M:%S")
        self.summary_path = os.path.join(out_dir, "mat", "%s-%s.json" % (plan["name"], time.strftime("%Y%m%d-%H%M%S")))
        self.prog = None                                  # the program's /api/programs entry
        self.robot_p = None
        self.mock = False                                 # the agent drives the simulator (/api/status robot.mock)

    # ---- checks
    def robot_checks(self) -> list:
        """Program exists and knows every plan param, the applied profile is the plan's, the body is not a mismatch."""
        bad = []
        plan = self.plan
        progs = self.req("GET", "/api/programs")
        self.prog = next((m for m in progs if m.get("name") == plan["program"]), None)
        if self.prog is None:
            bad.append("the robot has no program %s (bw deploy?)" % plan["program"])
        else:
            keys = {q["key"] for q in self.prog.get("params") or []}
            used = set()
            for r in self.runs:
                used |= set(r["params"])
            unknown = sorted(used - keys)
            if unknown:
                bad.append("%s on the robot has no param %s: the plan cannot mean what it says (bw deploy the code "
                           "that has it)" % (plan["program"], ", ".join(unknown)))
        self.robot_p = self.req("GET", "/api/params")
        try:
            self.mock = bool(((self.req("GET", "/api/status") or {}).get("robot") or {}).get("mock"))
        except (HttpError, AttributeError):
            self.mock = False
        applied = str((self.robot_p.get("body") or {}).get("profile") or "")
        if applied != plan["profile"]:
            bad.append("the robot's profile is %s, the plan is for %s: bw body %s --yes (S15)" % (
                applied or "(no stamp)", plan["profile"], plan["profile"]))
        try:
            chk = self.req("GET", "/api/body?fresh=1")
            if chk.get("ok") is False:
                bad.append("body %s (S15)" % chk.get("why"))
        except HttpError as e:
            bad.append("no body check on the robot (%s): bw deploy" % _detail(e)[:60])
        return bad

    def prechecks(self, confirm: bool = False) -> list:
        bad, field_done = [], False
        sys.path.insert(0, HERE)
        import bw
        good = float(((self.robot_p or {}).get("loc") or {}).get("good", 0.30))
        pcs = self.plan.get("precheck") or []
        mk = self.runs[0]["start"] if self.runs else "A"
        if any(pc in ("preflight", "field_check") for pc in pcs):
            # the prechecks' field check needs the car STILL on a known mark: the first run's (its pose is checked
            # against it -- a STANDARD fit off the mark means a wrong pitch or a car off its mark, review 2026-09-25)
            self.out("prechecks: the car on %s, hands off" % tape_line(mk, self.plan["starts"][mk]))
            if confirm:
                ans = self.inp("  Enter = check now (q = stop): ")
                if ans.strip().lower() == "q":
                    return ["stopped by the operator before the prechecks"]
        for pc in pcs:
            self.out("precheck %s" % pc)
            try:
                if pc == "preflight":
                    go, rows = bw.preflight(["--field", "--program", self.plan["program"], "--mark", mk], req=self.req,
                                            out=lambda s: self.out("  " + s))
                    field_done = any(r[0] == "field" and r[1] == "OK" for r in rows)
                    if not go:
                        bad.append("preflight: NO on %s" % ", ".join(r[0] for r in rows if r[1] == "NO"))
                elif pc == "field_check" and not field_done:
                    _rid, recs = bw.follow("field_check", {}, req=self.req)
                    fv = bw.field_verdict(recs, good)
                    self.out("  %s (cost %s)" % (fv["verdict"], fv["cost"]))
                    if not fv["standard"]:
                        bad.append("field_check: %s" % fv["verdict"])
                elif pc == "body":
                    chk = self.req("GET", "/api/body?fresh=1")
                    # MISMATCH (ok False) refuses; UNKNOWN (ok None: the strap unreadable, no informative fingerprint --
                    # the stock A1's lidar never sees its own body) is a WARN, as bw preflight and BRAIN4_SPEC 8.3 say.
                    # It refused every A1 series when the strap did not read (review 2026-09-25)
                    if chk.get("ok") is False:
                        self.out("  MISMATCH %s" % chk.get("why"))
                        bad.append("body: %s" % chk.get("why"))
                    elif chk.get("ok") is None:
                        self.out("  WARN  UNKNOWN (%s) -- carrying on: the profile's stamp decides" % chk.get("why"))
                    else:
                        self.out("  %s" % chk.get("why"))
            except HttpError as e:
                bad.append("%s: the robot said %s" % (pc, _detail(e)[:120]))
        return bad

    # ---- output
    def write_summary(self) -> None:
        save_json(self.summary_path, dict(
            plan=dict(name=self.plan["name"], path=self.plan_path, profile=self.plan["profile"],
                      program=self.plan["program"], desc=self.plan.get("desc")),
            started=self.started, rows=self.rows, stopped_by=self.stopped_by, rungs=rungs(self.rows, self.runs)))

    # ---- one run
    def start_run(self, run: dict, k_of: str) -> tuple:
        plan = self.plan
        prm = dict(run["params"])
        if plan.get("expect", False) and self.prog and "expect" in {q["key"] for q in self.prog.get("params") or []}:
            prm["expect"] = list(run["pose"])
        how = plan.get("start_by", "button")
        self.out("")
        self.out("run %s  %s" % (k_of, tape_line(run["start"], plan["starts"][run["start"]])))
        q = ""
        if how == "button":
            wait_s = float(((self.robot_p or {}).get("mat") or {}).get("wait_s", 120.0))
            q = "?wait=button&wait_s=%g" % wait_s
            self.out("  Place the car, hands off, press START on the car (it waits %.0f s)." % wait_s)
        elif how == "enter":
            ans = self.inp("  Place the car, hands off; Enter = start (q = stop the series): ")
            if ans.strip().lower() == "q":
                return None, "operator"
        else:
            cd = float(plan.get("countdown_s", 5))
            self.out("  Place the car, hands off: it starts in %.0f s." % cd)
            self.sleep(cd)
        t_post = time.time()
        r = self.req("POST", "/api/programs/%s/run%s" % (plan["program"], q), prm)
        return (r["run_id"], t_post, how == "button"), None

    def one(self, run: dict, k_of: str) -> str | None:
        """Run `run`; returns the stop reason or None."""
        plan = self.plan
        try:
            got, why = self.start_run(run, k_of)
        except HttpError as e:
            row = dict(n=run["n"], mark=run["start"], run_id=None, end="refused", laps=None, secs=None, verdict="-",
                       counts={}, first_incident=_detail(e)[:120], src="none", analysis="none", estop=False,
                       params=run["params"])
            self.rows.append(row)
            self.out(row_text(row))
            self.write_summary()
            return "the robot refused the start: %s" % _detail(e)[:160]
        if got is None:
            return "stopped by the operator (%s)" % why
        rid, t_post, wait_mode = got
        self.active = rid
        self.out("  run id %s" % rid)
        # the link held by bw mat itself while the run lives (the wait for START included): a log poll hung on a Wi-Fi
        # hiccup (30 s timeout) no longer lets the hub's 3 s link_timeout_ms stop the run.  Ctrl-C / bw killed = the
        # pings stop and the watchdog stops the car as before
        if HERE not in sys.path:
            sys.path.insert(0, HERE)
        import bw_keepalive as K
        ka = K.KeepAlive(K.status_ping(self.req)).start()
        # every poll waited on interruptibly: a Ctrl-C (Windows holds it until a socket read returns) must reach
        # run()'s stop at once, not after a hung poll's 30 s while the keepalive holds the link
        ireq = lambda *a_, **k_: K.interruptible(lambda: self.req(*a_, **k_))     # noqa: E731
        try:
            fol = follow_run(ireq, rid, float(plan.get("run_timeout_s", 200)), wait_mode, self.out, t_post)
        finally:
            ka.stop()
        if ka.fails and ka.fails * 2 >= ka.sent:
            self.out("  link: %d of %d keepalive requests failed (%s)" % (ka.fails, ka.sent, ka.last_error[:80]))
        self.active = None
        drives = bool((self.prog or {}).get("drives", True))
        ended_before = (fol.get("end") or {}).get("reason") in ("no_button", "stopped", "estop") and not fol["pressed"]
        if drives and not ended_before:
            rep, src, state = fetch_report(self.req, rid, float(plan.get("report_timeout_s", 90)), self.out_dir,
                                           self.out)
        else:
            rep, src, state = None, "none", "still" if not drives else "none"
        try:
            rest_v = (self.req("GET", "/api/status").get("robot") or {}).get("battery_v")
        except HttpError:
            rest_v = None
        row = make_row(run, rid, fol, rep, src, state, rest_v, drives)
        if ended_before:
            row["analysis"] = "still"                    # nothing ran: nothing to analyse
        rp = os.path.join(self.out_dir, rid + ".report.html")
        rj = os.path.join(self.out_dir, rid + ".report.json")
        row["report"] = rp if os.path.isfile(rp) else rj if os.path.isfile(rj) else None
        self.rows.append(row)
        self.out(HEADER)
        self.out(row_text(row))
        if row["report"]:
            self.out("  report: %s" % os.path.relpath(row["report"], WRO) if not os.path.relpath(
                row["report"], WRO).startswith("..") else "  report: %s" % row["report"])
        self.ledger(rid)
        self.write_summary()
        return stop_reason(row, self.so, plan["program"] == "field_check")

    def ledger(self, rid: str) -> None:
        body = self.plan.get("ledger")
        if body not in ("old", "new"):
            return
        path = os.path.join(self.out_dir, rid + ".jsonl")
        led = os.path.join(WRO, "devtools", "ledger.py")
        try:
            if not os.path.isfile(path):
                save_bytes(path, self.req("GET", "/api/runs/" + rid, raw=True, timeout=60))
            # the simulator's runs are not mat runs: B4's acceptance of a1_laps_A on the mock wrote 3 rows "where=mat"
            # into the team's ledger on a day the robot was off (review 2026-09-25)
            sim = bool(self.mock)
            if not sim:
                try:
                    with open(path, encoding="utf-8") as fh:
                        for _i, line in zip(range(40), fh):
                            if '"sim":' in line and '"tel"' in line:
                                sim = True
                                break
                except OSError:
                    pass
            if os.path.isfile(led):
                p = subprocess.run([sys.executable, led, "import", path, "--body", "sim" if sim else body,
                                    "--where", "sim" if sim else "mat"],
                                   capture_output=True, text=True, timeout=60)
                if p.returncode:
                    self.out("  ledger: %s" % (p.stderr or p.stdout)[-160:].strip())
        except (HttpError, OSError, subprocess.SubprocessError) as e:
            self.out("  ledger: %s" % e)

    def cooldown(self, secs: float) -> str | None:
        end = time.time() + secs
        last = 0.0
        while time.time() < end:
            if self.keys():
                self.out("  cooldown skipped")
                return None
            if time.time() - last >= 5.0:
                last = time.time()
                try:
                    st = self.req("GET", "/api/status")
                    if (st.get("robot") or {}).get("estop"):
                        return "E-STOP (the series ends: S17)"
                    self.out("  cooldown %3.0f s   battery %s V   cpu %s C   (Enter skips)" % (
                        end - time.time(), (st.get("robot") or {}).get("battery_v"),
                        (st.get("host") or {}).get("cpu_c")))
                except HttpError:
                    pass
            self.sleep(0.2)
        return None

    def run(self, first: int = 1) -> int:
        todo = [r for r in self.runs if r["n"] >= first]
        cd = self.plan.get("cooldown_s")
        if cd is None:
            cd = float(((self.robot_p or {}).get("mat") or {}).get("cooldown_s", 20.0))
        try:
            for i, run in enumerate(todo):
                why = self.one(run, "%d/%d" % (run["n"], len(self.runs)))
                if why:
                    self.stopped_by = dict(after_run=run["n"], why=why, resume="--from %d" % (run["n"] + 1))
                    self.out("STOPPED after run %d: %s" % (run["n"], why))
                    break
                if i + 1 < len(todo) and cd > 0:
                    why = self.cooldown(float(cd))
                    if why:
                        self.stopped_by = dict(after_run=run["n"], why=why, resume="--from %d" % (run["n"] + 1))
                        self.out("STOPPED after run %d: %s" % (run["n"], why))
                        break
        except KeyboardInterrupt:
            if self.active:
                try:
                    self.req("POST", "/api/programs/stop", {})
                    self.out("\nCtrl-C: the program was stopped")
                except HttpError:
                    self.out("\nCtrl-C: could not stop the program -- press E-STOP")
            self.stopped_by = dict(after_run=self.rows[-1]["n"] if self.rows else 0, why="ctrl-c",
                                   resume="--from %d" % ((self.rows[-1]["n"] + 1) if self.rows else first))
        self.write_summary()
        for g in (rungs(self.rows, self.runs) if (self.prog or {}).get("drives", True) else []):
            if g["done"]:
                self.out("rung %s: %s %d/%d%s" % (
                    json.dumps(g["params"], sort_keys=True), "PASS" if g["passed"] else
                    "not passed" if g["complete"] or g["why"] else "incomplete", g["done"], g["planned"],
                    ("  (%s)" % "; ".join(g["why"][:3])) if g["why"] else ""))
        rel = os.path.relpath(self.summary_path, WRO)
        self.out("summary: %s      reports: %s" % (rel if not rel.startswith("..") else self.summary_path,
                                                   os.path.join(os.path.relpath(self.out_dir, WRO)
                                                                if not os.path.relpath(self.out_dir, WRO).startswith("..")
                                                                else self.out_dir, "*.report.html")))
        return 1 if self.stopped_by else 0


def main(argv: list, req=None, out=print, inp=input, keys=enter_pressed, sleep=time.sleep) -> int:
    a = list(argv)
    if not a or a[0] in ("-h", "--help", "help"):
        out(__doc__)
        return 0
    dry, yes = "--dry" in a, "--yes" in a
    first, out_dir = 1, OUT_DIR
    if "--from" in a:
        i = a.index("--from")
        try:
            first = int(a[i + 1])
        except (IndexError, ValueError):
            out("--from N (a run number)")
            return 2
    if "--out" in a:
        i = a.index("--out")
        try:
            out_dir = os.path.abspath(a[i + 1])
        except IndexError:
            out("--out DIR")
            return 2
    skip = {"--from", "--out"}
    pos = [x for i, x in enumerate(a) if not x.startswith("--") and not (i and a[i - 1] in skip)]
    if not pos:
        out("bw mat PLAN.json [--from N] [--dry] [--yes] [--out DIR]")
        return 2
    path = find_plan(pos[0])
    if path is None:
        out("no plan %s (plans/: %s)" % (pos[0], ", ".join(sorted(f for f in os.listdir(os.path.join(ROOT, "plans"))
                                                                   if f.endswith(".json")))
                                         if os.path.isdir(os.path.join(ROOT, "plans")) else "none"))
        return 2
    try:
        with open(path, encoding="utf-8") as f:
            plan = json.load(f)
    except (OSError, ValueError) as e:
        out("cannot read %s: %s" % (path, e))
        return 2
    bad = validate(plan)
    if bad:
        out("plan %s refused:\n  %s" % (path, "\n  ".join(bad)))
        return 2
    req = req or make_req()
    s = Series(plan, path, req, out, inp, out_dir, keys=keys, sleep=sleep)
    out("plan %s: %s -- %s, %d run%s, profile %s, start by %s" % (
        plan["name"], plan.get("desc") or "", plan["program"], len(s.runs), "" if len(s.runs) == 1 else "s",
        plan["profile"], plan.get("start_by", "button")))
    try:
        rb = s.robot_checks()
    except HttpError as e:
        rb = ["the robot is not reachable: %s" % _detail(e)[:120]]
    if dry:
        for r in s.runs:
            out("run %d/%d  %s  params %s" % (r["n"], len(s.runs), tape_line(r["start"], plan["starts"][r["start"]]),
                                              json.dumps(r["params"], sort_keys=True)))
        out("prechecks (not run by --dry): %s" % (", ".join(plan.get("precheck") or []) or "none"))
        if rb:
            out("the robot would refuse this plan:\n  %s" % "\n  ".join(rb))
            return 2
        out("dry run: nothing started")
        return 0
    if rb:
        out("refused before the first run:\n  %s" % "\n  ".join(rb))
        return 2
    pcb = s.prechecks(confirm=not yes)
    if pcb:
        out("precheck failed -- nothing started:\n  %s" % "\n  ".join(pcb))
        return 2
    if not yes:
        ans = inp("%d run%s from run %d.  Start the series? [y/N] " % (
            len([r for r in s.runs if r["n"] >= first]), "" if len(s.runs) == 1 else "s", first))
        if ans.strip().lower() not in ("y", "yes"):
            out("nothing started")
            return 1
    return s.run(first)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
