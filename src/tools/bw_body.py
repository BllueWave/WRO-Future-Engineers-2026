"""bw body -- which chassis is this, and switching the car between its two configurations (BRAIN4_SPEC 8.4).

    bw body                          the applied profile, the detected chassis and the evidence; exit 1 on a mismatch
    bw body NAME [--yes]             switch to profile NAME (stock_a1 | wltoys_bw2 | wltoys): refused while a program
                                     runs; shows the changes; applies (history reason profile:NAME, body.profile
                                     stamped); restarts the agent (drive.backend, the lidar angle and the start button
                                     load only then); waits for it (<= 40 s); prints the check and what is still due
    bw body fingerprint [--save] [--yes]   the lidar's near-return fingerprint now (car still, nothing within 30 cm);
                                     --save records it as the applied profile's reference (once per body, on the mat)

Stdlib only; the same `req` as tools/bw.py.  Nothing here moves the car.
"""
from __future__ import annotations

import copy
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RESTART_WAIT_S = 40.0


def _try(req, *a, **k):
    """req(...) or None when the agent is unreachable / refuses (bw.req exits; a test's req raises)."""
    try:
        return req(*a, **k)
    except (SystemExit, Exception):
        return None


def _leaves(d, prefix=""):
    out = {}
    for k, v in (d or {}).items():
        key = prefix + str(k)
        if isinstance(v, dict) and v:
            out.update(_leaves(v, key + "."))
        else:
            out[key] = v
    return out


def _apply(p: dict, patch: dict) -> dict:
    """paramstore.patch's rules on a copy: dotted keys set a leaf, nested dicts merge."""
    q = copy.deepcopy(p)

    def merge(dst, src):
        for k, v in src.items():
            if isinstance(v, dict) and isinstance(dst.get(k), dict):
                merge(dst[k], v)
            else:
                dst[k] = copy.deepcopy(v)
    for k, v in (patch or {}).items():
        if "." in k:
            node = q
            parts = k.split(".")
            for part in parts[:-1]:
                if not isinstance(node.get(part), dict):
                    node[part] = {}
                node = node[part]
            node[parts[-1]] = copy.deepcopy(v)
        elif isinstance(v, dict) and isinstance(q.get(k), dict):
            merge(q[k], v)
        else:
            q[k] = copy.deepcopy(v)
    return q


def changes(p: dict, patch: dict) -> list:
    la, lb = _leaves(p), _leaves(_apply(p, patch))
    return [(k, la.get(k), lb.get(k)) for k in sorted(set(la) | set(lb)) if la.get(k) != lb.get(k)]


def check_lines(chk: dict) -> list:
    ok = chk.get("ok")
    ev = chk.get("evidence") or {}
    fp = ev.get("fp") or {}
    out = ["body      %s" % ("OK" if ok else "MISMATCH" if ok is False else "UNKNOWN"),
           "  %s" % chk.get("why"),
           "  applied %s | expects %s (%s) | shell %s | detected %s by %s" % (
               chk.get("applied") or "(no stamp)", chk.get("expect"), ev.get("expect_by"), chk.get("shell") or "-",
               chk.get("detected"), chk.get("by")),
           "  strap GPIO%s: %s | fingerprint: %s" % (
               ev.get("id_gpio"), {0: "fitted (WLtoys)", 1: "open (A1)"}.get(ev.get("strap"), "no reading"),
               ("score %s vs %s" % (fp.get("score"), fp.get("ref"))) if fp.get("score") is not None
               else (fp.get("why") or "none recorded"))]
    return out


def still_due(req, name: str) -> list:
    """[(what, how, why)]: the camera (verified_t older than the agent), turn_radius (no test:turn_radius applied
    while `name` was the profile, per the params history), the profile's [DAY1] notes."""
    due = []
    st = _try(req, "GET", "/api/status") or {}
    p = _try(req, "GET", "/api/params") or {}
    vt = float(((p.get("camera") or {}).get("verified_t")) or 0.0)
    t0 = (st.get("host") or {}).get("agent_started")
    if t0 is None or vt < float(t0):
        due.append(("camera", "bw preflight pitch --field", "not verified since the agent started (the mount moves)"))
    hist = _try(req, "GET", "/api/params/history") or []
    cur, measured = None, None
    for e in sorted(hist, key=lambda e: e.get("t") or 0.0):              # oldest first
        reason = str(e.get("reason") or "")
        if reason.startswith("profile:"):
            cur = reason.split(":", 1)[1]
        elif reason == "test:turn_radius" and cur == name:
            measured = e.get("t")
    if measured is None:
        wb = (p.get("chassis") or {}).get("wheelbase_m")
        due.append(("turn_radius", "TESTS -> turn_radius (Apply)",
                    "never measured with %s in the params history (chassis.wheelbase_m %s); needed when the steering "
                    "was touched" % (name, wb)))
    try:
        with open(os.path.join(ROOT, "profiles", name + ".json"), encoding="utf-8") as f:
            notes = json.load(f).get("notes") or {}
    except (OSError, ValueError):
        notes = {}
    for k, v in notes.items():
        if "[DAY1" in str(v):
            due.append((k if len(k) <= 40 else k[:37] + "...", "", str(v)[:110]))
    return due


def cmd_show(req, out) -> int:
    chk = req("GET", "/api/body?fresh=1")
    for l in check_lines(chk):
        out(l)
    return 1 if chk.get("ok") is False else 0


def cmd_fingerprint(a, req, out, inp) -> int:
    if "--save" in a:
        if "--yes" not in a:
            ans = inp("record the lidar fingerprint as the applied profile's reference?  The car still, nothing within "
                      "30 cm of it. [y/N] ")
            if ans.strip().lower() not in ("y", "yes"):
                out("not recorded")
                return 1
        r = req("POST", "/api/body/fingerprint", {"confirm": "fingerprint"})
    else:
        r = req("GET", "/api/body/fingerprint")
    fp = r.get("fp") or []
    bar = "".join(" .:-=+*#%@"[min(9, int(round(v * 9)))] for v in fp)
    out("fingerprint (36 x 10 deg from -180, share of bins 20-200 mm): |%s|" % bar)
    out("profile %s | informative %s | 20-30 cm ring bins %s | score vs the recorded reference %s%s" % (
        r.get("profile") or "(no stamp)", r.get("informative"), r.get("clear_ring_bins"), r.get("ref_score"),
        " | SAVED" if r.get("saved") else ""))
    if not r.get("informative"):
        out("the lidar sees none of the car's own parts: the fingerprint cannot tell the bodies apart (the strap does)")
    return 0


def cmd_switch(name: str, a, req, out, inp, sleep) -> int:
    st = req("GET", "/api/status")
    pr = st.get("program") or {}
    if pr.get("busy"):
        out("refused: %s is running -- bw stop first" % pr.get("name"))
        return 2
    profs = req("GET", "/api/profiles")
    if name not in profs:
        out("no profile %s (one of %s)" % (name, ", ".join(sorted(profs))))
        return 2
    p = req("GET", "/api/params")
    ch = changes(p, dict(profs[name].get("patch") or {}, **{"body.profile": name}))
    out("%s: %s" % (name, profs[name].get("desc", "")))
    out("%d change%s%s" % (len(ch), "" if len(ch) == 1 else "s",
                           ": " + ", ".join("%s %s -> %s" % (k, json.dumps(o), json.dumps(n)) for k, o, n in ch[:8])
                           + (" (+%d more)" % (len(ch) - 8) if len(ch) > 8 else "") if ch else ""))
    if "--yes" not in a:
        ans = inp("apply %s and restart the agent? [y/N] " % name)
        if ans.strip().lower() not in ("y", "yes"):
            out("not applied")
            return 1
    req("POST", "/api/profiles/%s/apply" % name, {})
    out("applied %s (history reason profile:%s; bw params-undo takes it back)" % (name, name))
    mock = bool((st.get("robot") or {}).get("mock"))
    if mock:
        out("simulator: the agent is not restarted (BW_MOCK)")
    else:
        t_old = (st.get("host") or {}).get("agent_started")
        r = _try(req, "POST", "/api/sys/action", {"action": "restart_agent", "confirm": "restart_agent"})
        out("restarting the agent: %s" % ((r or {}).get("detail") or "no answer"))
        end = time.time() + RESTART_WAIT_S
        back = False
        sleep(2.0)
        while time.time() < end:
            s2 = _try(req, "GET", "/api/status")
            if s2 and (t_old is None or ((s2.get("host") or {}).get("agent_started") or 0) > t_old):
                back = True
                break
            sleep(1.0)
        if not back:
            out("the agent did not come back within %.0f s: bw status, then bw body" % RESTART_WAIT_S)
            return 1
        out("agent back")
    chk = _try(req, "GET", "/api/body?fresh=1")
    if chk:
        for l in check_lines(chk):
            out(l)
    due = still_due(req, name)
    if due:
        out("still due:")
        for what, how, why in due:
            out("  %-24s %s%s" % (what, how + "  " if how else "", why))
    return 1 if chk and chk.get("ok") is False else 0


def main(a: list, req, out=print, inp=input, sleep=time.sleep) -> int:
    if not a:
        return cmd_show(req, out)
    if a[0] in ("-h", "--help", "help"):
        out(__doc__)
        return 0
    if a[0] == "fingerprint":
        return cmd_fingerprint(a[1:], req, out, inp)
    return cmd_switch(a[0], a[1:], req, out, inp, sleep)


if __name__ == "__main__":
    sys.path.insert(0, HERE)
    import bw
    sys.exit(main(sys.argv[1:], bw.req))
