"""The console v2 commands of `bw` (docs/CONSOLE_SPEC.md section 8): the same data the browser console shows, as text
for the team.  tools/bw.py dispatches here; `req` is its HTTP helper (exits with the agent's message on
an error status).

    bw sys [--json]                    host, CPU per core, temperature, throttling, memory / disk, Wi-Fi, services,
                                       camera container + stream, clock skew
    bw sys ACTION [--yes]              restart_camera | restart_agent | reboot | shutdown | sync_clock | race_mode
    bw live [SECONDS] [--json]         what the running program believes, twice a second (--json: one full payload)
    bw log [-f] [KIND ...]             the current or last run's log; -f follows it; KIND = ev error pose text data
    bw presets PROGRAM [save NAME k=v ... | rm NAME]
    bw replay RUN_ID [--json]          duration, pose source, first / last pose, signs, lot, the event timeline
    bw params-history | params-undo | params-restore HID
    bw profile [NAME [--apply]]        list the profiles; NAME shows its diff against the current params
"""
from __future__ import annotations

import copy
import json
import math
import sys
import time

COMMANDS = ("sys", "live", "log", "presets", "replay", "params-history", "params-undo", "params-restore", "profile")
KINDS = ("ev", "error", "pose", "text", "data")


def _v(x, fmt="%s", unit=""):
    return "-" if x is None else (fmt % x) + unit


def _kv(d: dict, skip=()) -> str:
    out = []
    for k, v in d.items():
        if k in skip:
            continue
        s = json.dumps(v, separators=(",", ":")) if isinstance(v, (list, dict)) else str(v)
        out.append("%s=%s" % (k, s if len(s) <= 40 else s[:37] + "..."))
    return " ".join(out)


def log_line(l: dict, t0: float) -> str:
    x = l.get("log")
    kind = l.get("kind") or "data"
    if isinstance(x, str):
        body = x
    elif isinstance(x, dict) and "error" in x:
        body = str(x["error"]).strip().splitlines()[-1] if str(x["error"]).strip() else "error"
    elif isinstance(x, dict) and "ev" in x:
        body = "%-12s %s" % (x["ev"], _kv(x, skip=("ev", "params", "legs")))
    elif isinstance(x, dict):
        body = _kv(x)
    else:
        body = str(x)
    return "%+8.1f s  %-5s %s" % (l["t"] - t0, kind.upper(), body)


# ---------------------------------------------------------------------------------------------------------- sys
def sys_lines(s: dict, rtt: float = 0.0) -> list:
    h, c, pw, m, d, n = s["host"], s["cpu"], s["power"], s.get("mem") or {}, s.get("disk") or {}, s["net"]
    up = h.get("uptime_s")
    lines = ["host     %s  %s  card %s  mode %s  up %s  agent %s s (pid %s, %s MB, %s %% CPU)" % (
        h.get("hostname"), h.get("model") or "-", h.get("card"), h.get("mode"),
        "-" if up is None else "%dh%02dm" % (up // 3600, up // 60 % 60), h.get("agent_uptime_s"), h.get("agent_pid"),
        _v(h.get("agent_rss_mb"), "%.0f"), _v(h.get("agent_cpu_pct"), "%.0f")),
        "os       %s  kernel %s  python %s" % (h.get("os") or "-", h.get("kernel") or "-", h.get("python")),
        "cpu      %s %%  cores %s  %s  load %s  %s MHz  %s" % (
            _v(c.get("pct"), "%.0f"), " ".join(_v(x, "%.0f") for x in (c.get("cores") or [])) or "-",
            _v(c.get("temp_c"), "%.1f", " C"), " ".join("%.2f" % x for x in (c.get("load") or [])) or "-",
            "/".join(str(x) for x in (c.get("mhz") or [])) or "-", c.get("governor") or "-")]
    flags = pw.get("flags") or {}
    set_ = [k.replace("_now", " (now)").replace("_seen", " (since boot)") for k, v in flags.items() if v]
    lines.append("power    battery %s  throttled %s%s  EXT5V %s" % (
        _v(pw.get("battery_v"), "%.2f", " V"), pw.get("throttled") or "-",
        ("  [" + ", ".join(set_) + "]") if set_ else "",
        _v(pw.get("ext5v_v"), "%.2f", " V")))
    lines.append("memory   %s / %s MB (%s %%)  swap %s MB   disk %s GB free of %s (%s %% used)  runs %s MB" % (
        _v(m.get("used_mb")), _v(m.get("total_mb")), _v(m.get("pct"), "%.0f"), _v(m.get("swap_used_mb")),
        _v(d.get("free_gb")), _v(d.get("total_gb")), _v(d.get("pct"), "%.0f"), _v(d.get("runs_mb"))))
    lines.append("wifi     %s  %s  %s  RSSI %s  quality %s/70  %s MHz  rx %s tx %s kB/s" % (
        n.get("iface") or "-", n.get("ssid") or "-", n.get("ip") or "-", _v(n.get("rssi_dbm"), "%d", " dBm"),
        _v(n.get("quality")), _v(n.get("freq_mhz")), _v(n.get("rx_kBps")), _v(n.get("tx_kBps"))))
    sv = s.get("services") or {}
    lines.append("services " + ("  ".join("%s %s" % kv for kv in sv.items()) if sv else "-"))
    if sv.get("start_node") == "active":
        lines.append("         WARNING: Hiwonder's stack is running -- it holds the board and the lidar")
    cam = s.get("camera") or {}
    lines.append("camera   container %s %s (restarts %s, since %s)  stream %s" % (
        cam.get("container") or "-", cam.get("state") or "-", _v(cam.get("restarts")), cam.get("started") or "-",
        {True: "ok", False: "NO", None: "-"}[cam.get("stream_ok")]))
    ck = s.get("clock") or {}
    skew = time.time() - rtt / 2.0 - ck["unix"] if ck.get("unix") else None
    lines.append("clock    skew %s (laptop - robot)  NTP %s  %s" % (
        _v(skew, "%+.1f", " s"), {True: "yes", False: "no", None: "-"}[ck.get("ntp_synced")], ck.get("tz") or "-"))
    return lines


def cmd_sys(a, req):
    names = [x for x in a if not x.startswith("--")]
    if not names:
        t = time.time()
        s = req("GET", "/api/sys")
        rtt = time.time() - t
        print(json.dumps(s, indent=1) if "--json" in a else "\n".join(sys_lines(s, rtt)))
        return 0
    name = names[0]
    if "--yes" not in a:
        typed = input("type %s to confirm: " % name).strip()
        if typed != name:
            print("not confirmed")
            return 1
    body = dict(action=name, confirm=name)
    if name == "sync_clock":
        body["epoch_s"] = time.time()
    r = req("POST", "/api/sys/action", body, timeout=60)
    print(("OK    " if r["ok"] else "NOT   ") + r["detail"])
    for st in r.get("steps") or []:
        print("  $ %s  -> rc %s  %s" % (st["cmd"], st["rc"], (st.get("out") or "")[-120:]))
    return 0 if r["ok"] else 1


# ---------------------------------------------------------------------------------------------------------- live
def live_line(d: dict) -> str:
    if not d.get("src"):
        return "no program has published (start one, or `bw run watch`)"
    p = d.get("pose")
    parts = [d["src"], str(d.get("state") or "-")]
    if d.get("laps") is not None:
        parts.append("lap %.2f" % d["laps"])
    if d.get("section"):
        parts.append(d["section"])
    s = " ".join(parts)
    if p:
        s += "  x %d y %d th %.1f  ±%s mm ±%s°  fit %s mm" % (p["x"], p["y"], p["th"], _v(p.get("sxy")),
                                                                        _v(p.get("sth")), _v(p.get("fit")))
    else:
        s += "  no pose"
    if d.get("pillars") is not None:
        s += "  pillars %d" % len(d["pillars"])
    if d.get("lot"):
        s += "  lot %d..%d" % (d["lot"][0], d["lot"][2])
    if d.get("prog_fps") is not None:
        s += "  %.1f fps" % d["prog_fps"]
    if d.get("extra"):
        s += "  " + _kv(d["extra"])
    if not d.get("running"):
        s += "  (ended)"
    return s


def cmd_live(a, req):
    if "--json" in a:
        print(json.dumps(req("GET", "/api/live?pts=1&path=1"), indent=1))
        return 0
    nums = [x for x in a if not x.startswith("--")]
    end = time.time() + (float(nums[0]) if nums else 1e9)
    try:
        while time.time() < end:
            print(live_line(req("GET", "/api/live?pts=0&path=0")))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    return 0


# ---------------------------------------------------------------------------------------------------------- log
def cmd_log(a, req):
    follow = "-f" in a
    kinds = [k for k in a if k in KINDS]
    q = "&kinds=" + ",".join(kinds) if kinds else ""
    since, t0 = 0.0, None
    try:
        while True:
            lg = req("GET", "/api/programs/log?since=%f%s" % (since, q))
            if t0 is None:
                print("run %s%s" % (lg.get("run_id") or "none", " (running)" if lg.get("busy") else ""))
                t0 = lg["lines"][0]["t"] if lg["lines"] else time.time()
            for l in lg["lines"]:
                since = max(since, l["t"])
                print(log_line(l, t0))
            if not follow or not lg["busy"]:
                break
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    return 0


# ---------------------------------------------------------------------------------------------------------- presets
def _kvargs(args) -> dict:
    out = {}
    for x in args:
        k, _, v = x.partition("=")
        try:
            out[k] = json.loads(v)
        except ValueError:
            out[k] = v
    return out


def cmd_presets(a, req):
    if not a:
        sys.exit("bw presets PROGRAM [save NAME k=v ... | rm NAME]")
    prog = a[0]
    if len(a) >= 3 and a[1] == "save":
        print(req("PUT", "/api/programs/%s/presets/%s" % (prog, a[2]), _kvargs(a[3:])))
    elif len(a) >= 3 and a[1] == "rm":
        print(req("DELETE", "/api/programs/%s/presets/%s" % (prog, a[2])))
    else:
        ps = req("GET", "/api/programs/%s/presets" % prog)
        if not ps:
            print("no presets for %s  (bw presets %s save NAME k=v ...)" % (prog, prog))
        for name, vals in ps.items():
            print("@%-16s %s" % (name, _kv(vals)))
    return 0


# ---------------------------------------------------------------------------------------------------------- replay
def cmd_replay(a, req):
    if not a:
        sys.exit("bw replay RUN_ID [--json]")
    r = req("GET", "/api/runs/%s/replay" % a[0])
    if "--json" in a:
        print(json.dumps(r))
        return 0
    print("%s  %s  %.1f s" % (r["id"], r.get("program"), r.get("duration_s") or 0.0))
    p = r.get("pose")
    if p:
        print("pose     %s, %d samples; first x %s y %s th %s; last x %s y %s th %s" % (
            p["src"], len(p["t"]), p["x"][0], p["y"][0], p["th"][0], p["x"][-1], p["y"][-1], p["th"][-1]))
    else:
        print("pose     none in this run")
    tr = r.get("truth")
    if tr and p:
        err = []
        for t, x, y in zip(p["t"], p["x"], p["y"]):
            i = min(range(len(tr["t"])), key=lambda k: abs(tr["t"][k] - t))
            err.append(math.hypot(x - tr["x"][i], y - tr["y"][i]))
        err.sort()
        print("truth    pose error median %.0f mm, p90 %.0f mm (simulator)" % (err[len(err) // 2],
                                                                             err[int(len(err) * 0.9)]))
    pil = r.get("pillars") or []
    print("signs    %s" % ("  ".join("%s %d,%d @%.1fs" % (q["c"], q["x"], q["y"], q["t"]) for q in pil) or "-"))
    lot = r.get("lot")
    print("lot      %s" % ("x %d..%d (known at %.1f s)" % (lot["rect"][0], lot["rect"][2], lot["t"]) if lot else "-"))
    ev = r.get("events") or []
    print("events   %d" % len(ev))
    for e in ev[:40]:
        print("  %7.1f s  %s" % (e["t"], e["text"]))
    if len(ev) > 40:
        print("  ... %d more (--json for all)" % (len(ev) - 40))
    return 0


# ---------------------------------------------------------------------------------------------------------- params
def _dotted(d, prefix=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict) and v:
            out.update(_dotted(v, prefix + k + "."))
        else:
            out[prefix + k] = v
    return out


def _apply(p, patch):
    q = copy.deepcopy(p)
    for k, v in patch.items():
        node = q
        parts = k.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = v
    return q


def _diff(a, b):
    la, lb = _dotted(a), _dotted(b)
    return [(k, la.get(k), lb.get(k)) for k in sorted(set(la) | set(lb)) if la.get(k) != lb.get(k)]


def cmd_params(c, a, req):
    if c == "params-history":
        for h in req("GET", "/api/params/history"):
            print("%-22s %s  %-28s %6d B" % (h["hid"], time.strftime("%H:%M:%S", time.localtime(h["t"] or 0)),
                                             h["reason"], h["bytes"]))
        return 0
    before = req("GET", "/api/params")
    if c == "params-undo":
        after = req("POST", "/api/params/undo", {})
        print("undone (one step: run it again to return)")
    else:
        if not a:
            sys.exit("bw params-restore HID   (bw params-history lists them)")
        after = req("POST", "/api/params/restore/" + a[0], {})
        print("restored", a[0])
    for k, o, n in _diff(before, after):
        print("  %-28s %s -> %s" % (k, json.dumps(o), json.dumps(n)))
    return 0


def cmd_profile(a, req):
    profs = req("GET", "/api/profiles")
    names = [x for x in a if not x.startswith("--")]
    if not names:
        for n, pr in profs.items():
            print("%-16s %s  %s" % (n, "[built-in]" if pr.get("builtin") else "[custom]  ", pr.get("desc", "")))
        return 0
    name = names[0]
    if name not in profs:
        sys.exit("no profile %s (one of %s)" % (name, ", ".join(profs)))
    cur = req("GET", "/api/params")
    if "--apply" in a:
        after = req("POST", "/api/profiles/%s/apply" % name, {})
        print("applied %s (bw params-undo to go back)" % name)
        ch = _diff(cur, after)
    else:
        ch = _diff(cur, _apply(cur, profs[name]["patch"]))
        print("%s: %s" % (name, profs[name].get("desc", "")))
    for k, o, n in ch:
        print("  %-28s %s -> %s" % (k, json.dumps(o), json.dumps(n)))
    if not ch:
        print("  (no difference from the current params)")
    elif "--apply" not in a:
        print("bw profile %s --apply   to apply it" % name)
    return 0


def main(c: str, a: list, req) -> int:
    if c == "sys":
        return cmd_sys(a, req)
    if c == "live":
        return cmd_live(a, req)
    if c == "log":
        return cmd_log(a, req)
    if c == "presets":
        return cmd_presets(a, req)
    if c == "replay":
        return cmd_replay(a, req)
    if c in ("params-history", "params-undo", "params-restore"):
        return cmd_params(c, a, req)
    if c == "profile":
        return cmd_profile(a, req)
    sys.exit("unknown command %s" % c)
