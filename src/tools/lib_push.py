"""Push the seed's library fields (programs/_library.json) to a running robot, so the console's PROGRAMS list on the
robot shows the team's order: PUT /api/library/<name> for every seeded program the robot has.

    py -3 tools/lib_push.py                 rank / rating / status / challenge always; a note or tags only where the
                                            robot has none
    py -3 tools/lib_push.py --dry-run       say what would change, change nothing
    py -3 tools/lib_push.py --notes         also replace notes and tags the team typed on the robot
    py -3 tools/lib_push.py --ip <robot-address>

Why: the robot's library.json (saved from the console) wins over the seed field by field, so a status or rating typed
on the robot earlier hides the seed's new order.  The ordering fields are pushed always; a note typed on the mat is the
team's own record, so it is replaced only with --notes.

Robot address as bw.py finds it: BW_HOST, else .robot.json's ip, else bluewave.local; port BW_PORT (8000); token
BW_TOKEN.  An agent older than the rank field answers 400 "unknown field rank": the other fields are pushed without it
and the list sorts by rating until the robot has this code (bw.py deploy).  Stdlib only; prints ASCII only (the Arabic
notes are never printed: a Windows console cannot show them).

Exit 0 = pushed (or nothing to push), 1 = robot not reached or a save refused, 2 = the seed does not read.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import urllib.error
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SEED = os.path.join(ROOT, "programs", "_library.json")
ORDER_FIELDS = ("rank", "rating", "status", "challenge")       # pushed always: they decide the list's order
TEXT_FIELDS = ("note", "tags")                                 # pushed where the robot has none (or --notes)


def load_seed(path: str = SEED) -> dict:
    """{program: entry} of the seed, in the list's order (rank, then name); ValueError when it does not read."""
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError) as e:
        raise ValueError("cannot read %s: %s" % (path, e))
    if not isinstance(d, dict):
        raise ValueError("%s is not a JSON object" % path)
    items = [(k, v) for k, v in d.items() if isinstance(k, str) and not k.startswith("_") and isinstance(v, dict)]

    def order(kv):
        r = kv[1].get("rank")
        ok = isinstance(r, int) and not isinstance(r, bool) and r >= 1
        return (0 if ok else 1, r if ok else 0, kv[0].lower())
    return dict(sorted(items, key=order))


def _same(field: str, want, row: dict) -> bool:
    """The robot's row already shows the seed's value (then nothing is sent for it)."""
    if field == "rank":
        return "rank" in row and row.get("rank") == want      # an agent without rank in its rows: always sent
    if field == "challenge":
        return bool(row.get("challenge_set")) and row.get("challenge") == want
    if field == "rating":
        return int(row.get("rating") or 0) == int(want or 0)
    if field == "status":
        return (row.get("status") or "") == (want or "")
    if field == "tags":
        return [str(t).lower() for t in row.get("tags") or []] == [str(t).lower() for t in want or []]
    return (row.get(field) or "") == (want or "")


def plan(seed: dict, rows: dict, notes: bool = False) -> list:
    """[(name, patch or None, why)]: patch None = nothing sent (why says: not on the robot / already matches)."""
    out = []
    for name, e in seed.items():
        row = rows.get(name)
        if row is None:
            out.append((name, None, "not on the robot"))
            continue
        patch, kept = {}, []
        for f in ORDER_FIELDS:
            if f in e and not _same(f, e[f], row):
                patch[f] = e[f]
        for f in TEXT_FIELDS:
            if f not in e or _same(f, e[f], row):
                continue
            if notes or not row.get(f):
                patch[f] = e[f]
            else:
                kept.append(f)
        why = ("kept the robot's " + " + ".join(kept)) if kept else ""
        out.append((name, patch or None, why or ("already matches" if not patch else "")))
    return out


def push(send, steps: list, dry: bool = False, say=print) -> dict:
    """Send each patch with send(method, path, body) -> (status, body).  Returns {updated, same, missing, refused,
    no_rank, sent: {name: patch}}.  A 400 naming the unknown field rank (an older agent) resends without rank."""
    res = dict(updated=0, same=0, missing=[], refused=[], no_rank=False, sent={})
    for name, patch, why in steps:
        if patch is None:
            if why == "not on the robot":
                res["missing"].append(name)
            else:
                res["same"] += 1
            continue
        if res["no_rank"]:
            patch = {k: v for k, v in patch.items() if k != "rank"}
            if not patch:
                res["same"] += 1
                continue
        line = "  %-14s %s%s" % (name, " ".join("%s=%s" % (k, _short(v)) for k, v in patch.items()),
                                 ("   (" + why + ")") if why else "")
        if dry:
            say(line + "   [dry run]")
            res["sent"][name] = patch
            continue
        code, body = send("PUT", "/api/library/%s" % urllib.parse.quote(name), patch)
        if code == 400 and "rank" in patch and "unknown field rank" in _text(body):
            res["no_rank"] = True
            say("  the robot's agent has no rank field yet: pushing without it (the list sorts by rating until "
                "`bw.py deploy`)")
            patch = {k: v for k, v in patch.items() if k != "rank"}
            if not patch:
                res["same"] += 1
                continue
            line = "  %-14s %s" % (name, " ".join("%s=%s" % (k, _short(v)) for k, v in patch.items()))
            code, body = send("PUT", "/api/library/%s" % urllib.parse.quote(name), patch)
        if code == 200:
            res["updated"] += 1
            res["sent"][name] = patch
            say(line)
        else:
            res["refused"].append(name)
            say("  %-14s REFUSED %s: %s" % (name, code, _text(body)[:160]))
    return res


def _short(v) -> str:
    if isinstance(v, str) and (len(v) > 12 or not v.isascii()):
        return "<%d chars>" % len(v)
    if isinstance(v, list):
        return ",".join(str(x) for x in v)
    return json.dumps(v) if v is None or isinstance(v, str) and not v else str(v)


def _text(body) -> str:
    if isinstance(body, dict):
        d = body.get("detail", body)
        return d if isinstance(d, str) else json.dumps(d, ensure_ascii=True)
    return str(body)


# ---------------------------------------------------------------------------------------------------------- robot
def robot_base(ip: str = "") -> str:
    cfg = {}
    try:
        with open(os.path.join(ROOT, ".robot.json"), encoding="utf-8") as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        pass
    host = ip or os.environ.get("BW_HOST") or cfg.get("ip") or "bluewave.local"
    if host.count(":") == 1:                             # --ip 127.0.0.1:8765 (the laptop's mock agent)
        return "http://" + host
    return "http://%s:%d" % (host, int(os.environ.get("BW_PORT", "8000")))


def http_sender(base: str, timeout: float = 6.0):
    token = os.environ.get("BW_TOKEN", "")

    def send(method: str, path: str, body=None):
        data = None if body is None else json.dumps(body).encode()
        r = urllib.request.Request(base + path, data=data, method=method)
        r.add_header("Content-Type", "application/json")
        r.add_header("X-BW-Client", "lib_push/" + (platform.node() or "laptop"))
        if token:
            r.add_header("X-BW-Token", token)
        try:
            with urllib.request.urlopen(r, timeout=timeout) as resp:
                raw = resp.read()
                return resp.status, json.loads(raw or b"null")
        except urllib.error.HTTPError as e:
            raw = e.read().decode(errors="replace")
            try:
                return e.code, json.loads(raw)
            except ValueError:
                return e.code, raw
    return send


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--ip", default="", help="robot address (default: BW_HOST, .robot.json, bluewave.local)")
    ap.add_argument("--dry-run", action="store_true", help="say what would change, change nothing")
    ap.add_argument("--notes", action="store_true", help="also replace notes / tags the team typed on the robot")
    ap.add_argument("--seed", default=SEED, help="the seed file (default programs/_library.json)")
    a = ap.parse_args(argv)
    try:
        seed = load_seed(a.seed)
    except ValueError as e:
        print("lib_push: %s" % e)
        return 2
    base = robot_base(a.ip)
    send = http_sender(base)
    try:
        code, lib = send("GET", "/api/library")
    except (urllib.error.URLError, OSError) as e:
        print("lib_push: robot not reached at %s (%s) -- nothing pushed" % (base, getattr(e, "reason", e)))
        return 1
    if code != 200 or not isinstance(lib, dict):
        print("lib_push: %s/api/library answered %s: %s" % (base, code, _text(lib)[:160]))
        return 1
    rows = {r.get("name"): r for r in lib.get("programs") or [] if isinstance(r, dict)}
    steps = plan(seed, rows, notes=a.notes)
    print("lib_push -> %s  (%d seeded, %d on the robot)%s" % (base, len(seed), sum(1 for s in steps if s[2] !=
          "not on the robot"), "  DRY RUN" if a.dry_run else ""))
    try:
        res = push(send, steps, dry=a.dry_run)
    except (urllib.error.URLError, OSError) as e:
        print("lib_push: the link dropped (%s) -- run it again" % getattr(e, "reason", e))
        return 1
    print("lib_push: %d %s, %d already matched, %d not on the robot%s, %d refused%s" % (
        res["updated"] if not a.dry_run else len(res["sent"]), "would change" if a.dry_run else "updated",
        res["same"], len(res["missing"]), (" (" + ", ".join(res["missing"]) + ")") if res["missing"] else "",
        len(res["refused"]), "; NO RANK on this robot's agent" if res["no_rank"] else ""))
    return 1 if res["refused"] else 0


if __name__ == "__main__":
    sys.exit(main())
