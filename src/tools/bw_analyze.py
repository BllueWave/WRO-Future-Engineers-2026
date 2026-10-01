"""bw analyze -- the crash report of a run (BRAIN4_SPEC 6.9): WHERE and WHY the car touched something.

    bw analyze RUN_ID|latest [--redo] [--json] [--open] [--out DIR]
        the robot's report (made by the agent after every run; --redo makes it again) ->
        mentorpi/runs/<id>.report.json + .report.html, and the table below
    bw analyze --local FILE.jsonl [FILE ...] [--rec DIR|auto|none] [--truth] [--set an.KEY=V ...] [--out DIR] [--json] [--open]
        bluewave.analyze on the laptop: pulled mat runs, simulator run files (sim_run --runfile), the deployed copies'
        runs (read in place; the report goes to --out, default mentorpi/runs/)

It prints, for the team, one header line, the sources, one line per incident (time, kind, where, state,
the cause), the decision and the cause's evidence under each contact, battery and loop, and the fix hints:

    20260923-185306.346-wro_next  wro_next  stock_a1  120.1 s  end time  laps 0.56   CONTACT
    sources: imu tel 20 Hz | clock estimated +-0.1 s | rec 1467 frames, 0 scans | camera NOT verified this session
     #  t       kind     sev  where                         state    decision / cause
     4  12.80   contact  3    island face E? (pose suspect: ...)  GO  LOC_POSE_OFF: the pose was off: relocalised ...
    ...
    fix: bw preflight pitch --field (seat errors 92 / 137 mm say the pitch is off); ...
    report: mentorpi/runs/20260923-185306.346-wro_next.report.html

Stdlib + the repo's bluewave (numpy, OpenCV for the frames) -- nothing new.  It never drives the car (S17).
"""
from __future__ import annotations

import json
import os
import sys
import time
import urllib.error
import urllib.request
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
RUNS = os.path.join(ROOT, "runs")
WAIT_S = 120.0                         # a 3-minute run takes <= 30 s on the Pi (6.6); the queue may hold 3 more


def _http(method: str, path: str, body=None, timeout: float = 30.0):
    """(status, bytes) -- never exits: 404 / 409 are answers here, not failures."""
    import bw as B
    data = None if body is None else json.dumps(body).encode()
    r = urllib.request.Request(B.BASE + path, data=data, method=method)
    r.add_header("Content-Type", "application/json")
    r.add_header("X-BW-Client", "bw/analyze")
    if B.TOKEN:
        r.add_header("X-BW-Token", B.TOKEN)
    try:
        with urllib.request.urlopen(r, timeout=timeout) as resp:
            return resp.status, resp.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()
    except urllib.error.URLError as e:
        sys.exit("cannot reach the agent at %s (%s)" % (B.BASE, e.reason))


def _jl(b: bytes):
    try:
        return json.loads(b or b"null")
    except ValueError:
        return None


def _robot(rid: str, redo: bool, out: str, as_json: bool, open_: bool, req) -> int:
    from bluewave import analyze as A
    if rid == "latest":
        rows = req("GET", "/api/runs")
        rows = [r for r in rows if not str(r.get("id", "")).endswith(".report")]
        if not rows:
            sys.exit("no runs on the robot")
        rid = rows[0]["id"]
    if redo:
        st, b = _http("POST", "/api/runs/%s/analyze" % rid, {"redo": True})
        if st >= 400:
            sys.exit("robot says %d: %s" % (st, b.decode(errors="replace")[:300]))
    t_end = time.time() + WAIT_S
    asked = redo
    shown = None
    while True:
        st, b = _http("GET", "/api/runs/%s/report" % rid)
        if st == 200:
            rep = _jl(b)
            break
        if st == 404 and not asked:
            s2, b2 = _http("POST", "/api/runs/%s/analyze" % rid, {"redo": False})
            if s2 >= 400:
                sys.exit("no report for %s and the robot will not make one now (%d): %s" % (
                    rid, s2, b2.decode(errors="replace")[:300]))
            asked = True
        elif st == 404:
            d = _jl(b) or {}
            det = d.get("detail") if isinstance(d, dict) else None
            info = _jl(det.encode()) if isinstance(det, str) else None
            if isinstance(info, dict) and info.get("state") in ("failed", "cancelled"):
                sys.exit("the analysis of %s %s: %s" % (rid, info["state"], info.get("error")))
        elif st == 409:
            d = _jl(b) or {}
            line = "analysing %s: %s %s" % (rid, d.get("state"), "%.0f %%" % (100 * float(d.get("progress") or 0))
                                           + (" (%s)" % d["stage"] if d.get("stage") else ""))
            if line != shown and not as_json:
                print(line, flush=True)
                shown = line
        else:
            sys.exit("robot says %d: %s" % (st, b.decode(errors="replace")[:300]))
        if time.time() > t_end:
            sys.exit("no report for %s after %.0f s (bw analyze %s again later; GET /api/analyze shows the queue)"
                     % (rid, WAIT_S, rid))
        time.sleep(1.0)
    os.makedirs(out, exist_ok=True)
    jp = os.path.join(out, rid + ".report.json")
    with open(jp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(rep, f, separators=(",", ":"))
    hp = None
    st, b = _http("GET", "/api/runs/%s/report.html" % rid, timeout=60.0)
    if st == 200:
        hp = os.path.join(out, rid + ".report.html")
        with open(hp, "wb") as f:
            f.write(b)
    _show(rep, jp, hp, as_json, open_, A)
    return 0


def _show(rep, jp, hp, as_json, open_, A):
    if as_json:
        print(json.dumps(rep, indent=1))
    else:
        print(A.table(rep))
        p = os.path.abspath(hp or jp)
        rel = os.path.relpath(p, os.path.dirname(ROOT)) if os.path.splitdrive(p)[0] == os.path.splitdrive(ROOT)[0] else p
        print("report: %s" % (p if rel.startswith("..") else rel))
    if open_ and (hp or jp):
        webbrowser.open("file:///" + os.path.abspath(hp or jp).replace("\\", "/"))


def _local(files, rec, truth, sets, out, as_json, open_) -> int:
    from bluewave import analyze as A
    params = {}
    for s in sets:
        k, _, v = s.partition("=")
        try:
            params[k.strip()] = json.loads(v)
        except ValueError:
            params[k.strip()] = v
    code = 0
    for fn in files:
        if not os.path.isfile(fn):
            print("no file %s" % fn, file=sys.stderr)
            code = 2
            continue
        rep = A.analyze(fn, rec, params, truth=truth)
        jp, hp = A.write(rep, out, html=True)
        _show(rep, jp, hp, as_json, open_, A)
        if len(files) > 1:
            print()
    return code


def main(argv, req=None) -> int:
    a = list(argv)
    if not a or a[0] in ("-h", "--help", "help"):
        print(__doc__)
        return 0

    def opt(name, default=None):
        if name in a:
            i = a.index(name)
            v = a[i + 1] if i + 1 < len(a) else default
            del a[i:i + 2]
            return v
        return default

    def flag(name):
        if name in a:
            a.remove(name)
            return True
        return False
    out = opt("--out", RUNS)
    as_json, open_ = flag("--json"), flag("--open")
    if flag("--local"):
        rec = opt("--rec", "auto")
        truth = flag("--truth")
        sets = []
        while "--set" in a:
            sets.append(opt("--set"))
        files = [x for x in a if not x.startswith("--")]
        if not files:
            sys.exit("bw analyze --local FILE.jsonl [...]")
        return _local(files, rec, truth, sets, out, as_json, open_)
    redo = flag("--redo")
    ids = [x for x in a if not x.startswith("--")]
    if not ids:
        sys.exit("bw analyze RUN_ID|latest [--redo] [--json] [--open]")
    if req is None:
        import bw as B
        req = B.req
    return _robot(ids[0], redo, out, as_json, open_, req)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
