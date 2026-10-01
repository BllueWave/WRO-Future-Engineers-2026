"""Which chassis is this?  The applied params profile against the car it runs on (BRAIN4_SPEC 8.3, S15).

The stock A1 and the WLtoys build share the Pi, the RRC board, the camera and the LD19, so nothing in software notices
the wrong profile on the wrong car -- and the wrong one is dangerous: the WLtoys servo limits (853-2194 us) on the A1's
servo, the A1's RRC motor ports on a car whose motor sits on the MD13S (G10).  Two sources, in this order:

  strap        one 2.54 mm jumper cap on header pins 29 (GPIO5) and 30 (GND), part of the WLtoys harness.  Read with
               the internal pull-up (lgpio through motor.open_chip, as hw.StartButton): 0 = fitted = the WLtoys,
               1 = open = the stock A1, None = no lgpio / body.id_gpio -1 / an error / a floating read.  [DAY1] pins
               29-30 must be free on the A1 (`bw ssh "pinctrl get 5"` reads `ip pu | hi` with nothing fitted).
  fingerprint  the lidar's near returns (20 < d < 200 mm: the car's own parts) in 36 x 10 deg sectors, recorded once
               per profile with the car still and nothing within 30 cm (`bw body fingerprint --save`), kept in
               body_fingerprints.json beside params.json.  score >= 0.7 consistent, <= 0.4 mismatch, else unknown.
               A reference with no sector above 0.1 says nothing (the lidar sees none of its own body) and is not used;
               a scan that matches the other chassis' reference as well is ambiguous (unknown).

`check` compares the detected chassis with `body.expect` (the profile's chassis; an unstamped card falls back to what
drive.backend implies: gpio_pwm = the WLtoys).  ok None (unknown) is a preflight WARN, never a silent pass.  Only the
STRAP proves a mismatch strongly enough to refuse a start (S15, agent.Runner.start); a fingerprint mismatch is a
preflight NO and a warning note.

Cost: one strap read = open the chip, claim, 3 reads 1 ms apart, free (~3 ms on the Pi); `check` caches 5 s.
"""
from __future__ import annotations

import json
import os
import threading
import time

import numpy as np

from . import params as P

CHASSIS = ("stock_a1", "wltoys")
SECTORS = 36                         # 10 deg each, lidar frame (bin k of lidar.Scan covers -180 + k res)
NEAR_MIN_MM, NEAR_MAX_MM = 20.0, 200.0
FP_OK, FP_BAD = 0.7, 0.4             # [EST 2026-09-25] fp_match: consistent / mismatch; between = unknown
FP_SIG = 0.1                         # a sector counts when either fingerprint is above this share
CLEAR_MIN_MM, CLEAR_MAX_MM = 200.0, 300.0   # "nothing within 30 cm" while recording: returns in this ring
CLEAR_BINS = 3                       # ...more bins than this there = something near: not recorded
CACHE_S = 5.0
FILE = "body_fingerprints.json"

_FAKE_LG = None                      # tests: a fake lgpio module (strap reads through it on any robot)
_lock = threading.Lock()
_cache: dict = {}                    # key -> (t_mono, result)
STRAP_S = 60.0                       # a real strap READ is kept until clear_cache (bw body); no reading: retried
#                                      after STRAP_S.
_strap_cache: dict = {}              # gpio -> (t_mono, value).  Opening + closing the gpiochip on the event loop
#                                      froze the WHOLE agent 0.5 s every 5.4 s (lgpio.gpiochip_close; stallwatch,
#                                      mat 2026-09-30): corners ran blind and the yaw lost ~20 deg


def set_fake_lgpio(lg) -> None:
    """Tests: read the strap through `lg` (None = the real lgpio again); clears the check cache."""
    global _FAKE_LG
    _FAKE_LG = lg
    clear_cache()


def clear_cache() -> None:
    with _lock:
        _cache.clear()
        _strap_cache.clear()


def other(chassis: str) -> str:
    return "wltoys" if chassis == "stock_a1" else "stock_a1"


def profile_for(chassis: str) -> str:
    """The profile to apply on this chassis (the words of the refusal and of `bw body`)."""
    return "stock_a1" if chassis == "stock_a1" else "wltoys_bw2 (or wltoys)"


def pin_text(gpio) -> str:
    return "pin 29-30" if int(gpio) == 5 else "GPIO%d-GND" % int(gpio)


# ---------------------------------------------------------------------------------------------------------- strap
def strap(p: dict, lg=None, mock: bool = False):
    """0 = the strap is fitted (the WLtoys harness), 1 = open, None = no reading.  On the simulator (no lgpio) the
    environment BW_MOCK_STRAP=0|1 stands in for the jumper; a fake lgpio (set_fake_lgpio / lg) wins over both."""
    try:
        gpio = int((p.get("body") or {}).get("id_gpio", -1))
    except (TypeError, ValueError):
        return None
    if gpio < 0:
        return None
    real = lg is None and _FAKE_LG is None and not mock
    if real:
        hit = _strap_cache.get(gpio)
        if hit and (hit[1] is not None or time.monotonic() - hit[0] < STRAP_S):
            return hit[1]
        v = _strap_read(gpio)
        _strap_cache[gpio] = (time.monotonic(), v)
        return v
    return _strap_read(gpio, lg or _FAKE_LG, mock)


def _strap_read(gpio: int, lg=None, mock: bool = False):
    if lg is None:
        if mock:
            v = os.environ.get("BW_MOCK_STRAP", "").strip()
            return int(v) if v in ("0", "1") else None
        try:
            import lgpio as lg                      # python3-lgpio (the WLtoys step installed it)
        except Exception:
            return None
    from . import motor as M
    h = None
    try:
        h = M.open_chip(lg)
        lg.gpio_claim_input(h, gpio, getattr(lg, "SET_PULL_UP", 32))
        vals = []
        for _ in range(3):
            time.sleep(0.001)                       # the pull-up charges the line (~us); three agreeing reads
            vals.append(int(lg.gpio_read(h, gpio)))
        try:
            lg.gpio_free(h, gpio)
        except Exception:
            pass
        if len(set(vals)) != 1:
            return None                             # a floating or bouncing line proves nothing
        return 0 if vals[0] == 0 else 1
    except Exception:
        return None
    finally:
        if h is not None:
            try:
                lg.gpiochip_close(h)
            except Exception:
                pass


# ---------------------------------------------------------------------------------------------------------- lidar
def fingerprint(scan) -> list:
    """36 x 10 deg sectors (the scan's frame): the share of raw bins with 20 < d < 200 mm -- the body's own parts."""
    d = np.asarray(scan.dist, dtype=float)
    res = float(scan.res)
    a = -180.0 + (np.arange(len(d)) + 0.5) * res
    k = (np.floor((a + 180.0) / 10.0).astype(int)) % SECTORS
    near = ((d > NEAR_MIN_MM) & (d < NEAR_MAX_MM)).astype(float)
    tot = np.bincount(k, minlength=SECTORS).astype(float)
    hit = np.bincount(k, weights=near, minlength=SECTORS)
    return [round(float(h / t), 3) if t else 0.0 for h, t in zip(hit, tot)]


def fp_match(fp, ref) -> float:
    """1 - mean |fp - ref| over the sectors where either is above 0.1 (0..1); 1.0 when neither has one."""
    f, r = np.asarray(fp, dtype=float), np.asarray(ref, dtype=float)
    if f.shape != r.shape:
        return 0.0
    m = (f > FP_SIG) | (r > FP_SIG)
    if not m.any():
        return 1.0
    return round(1.0 - float(np.mean(np.abs(f[m] - r[m]))), 3)


def informative(fp) -> bool:
    return bool(np.any(np.asarray(fp, dtype=float) > FP_SIG))


def store_path() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(P.path())), FILE)


def load_store() -> dict:
    try:
        with open(store_path(), encoding="utf-8") as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def save_fingerprint(profile: str, fp: list, expect: str, shell: str = "") -> dict:
    """Atomically record `fp` as `profile`'s reference; returns the stored entry."""
    with _lock:
        d = load_store()
        e = dict(fp=[round(float(v), 3) for v in fp], t=round(time.time(), 3), expect=str(expect or ""),
                 shell=str(shell or ""))
        d[str(profile)] = e
        tmp = store_path() + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(d, f, indent=1)
        os.replace(tmp, store_path())
        _cache.clear()
        return e


def fresh_scan(robot, max_age_s: float = 1.5):
    sc = getattr(robot, "scan", None)
    if sc is None:
        return None
    try:
        if time.monotonic() - float(sc.t) > max_age_s:
            return None
    except (TypeError, ValueError):
        return None
    return sc


def clear_ring(scan) -> int:
    """Bins with a return 200-300 mm away: something within 30 cm of the lidar besides the car itself."""
    d = np.asarray(scan.dist, dtype=float)
    return int(np.count_nonzero((d > CLEAR_MIN_MM) & (d < CLEAR_MAX_MM)))


def _fp_evidence(robot, applied: str, expect: str):
    """{"score", "ref", "chassis", "other_score", "why"} from the scan now against the recorded references, or None
    when there is no scan or no usable reference for the applied profile."""
    store = load_store()
    ref = store.get(applied) if applied else None
    if not ref or not isinstance(ref.get("fp"), list) or not informative(ref["fp"]):
        return None
    sc = fresh_scan(robot)
    if sc is None:
        return None
    fp = fingerprint(sc)
    s = fp_match(fp, ref["fp"])
    ref_chassis = str(ref.get("expect") or expect or "")
    # the best reference of the OTHER chassis, when one was recorded: a scan that fits both says nothing
    best_other, other_name = None, None
    for name, e in store.items():
        if name == applied or not isinstance(e, dict) or str(e.get("expect") or "") == ref_chassis:
            continue
        if isinstance(e.get("fp"), list) and informative(e["fp"]):
            so = fp_match(fp, e["fp"])
            if best_other is None or so > best_other:
                best_other, other_name = so, name
    out = dict(score=s, ref=applied, other_score=best_other, other_ref=other_name, chassis=None, why="")
    if ref_chassis not in CHASSIS:
        out["why"] = "the reference names no chassis"
    elif s >= FP_OK and not (best_other is not None and best_other >= FP_OK):
        out["chassis"], out["why"] = ref_chassis, "matches %s's fingerprint (%.2f)" % (applied, s)
    elif s <= FP_BAD:
        out["chassis"] = other(ref_chassis)
        out["why"] = "does not match %s's fingerprint (%.2f <= %.1f)" % (applied, s, FP_BAD)
    else:
        out["why"] = "fingerprint score %.2f%s: unclear" % (
            s, "" if best_other is None else " (the other chassis' %.2f)" % best_other)
    return out


# ---------------------------------------------------------------------------------------------------------- verdict
def detect(robot) -> dict:
    """{"chassis": "wltoys" | "stock_a1" | None, "strap": 0 | 1 | None, "fp": {...} | None, "by": "strap" |
    "fingerprint" | None}.  The strap decides when it reads; otherwise a usable fingerprint."""
    p = robot.p
    b = p.get("body") or {}
    s = strap(p, mock=bool(getattr(robot, "mock", False)))
    out = dict(chassis=None, strap=s, fp=None, by=None)
    if s is not None:
        out.update(chassis="wltoys" if s == 0 else "stock_a1", by="strap")
    try:
        exp, _eb = expected(p)
        fpe = _fp_evidence(robot, str(b.get("profile") or ""), exp)
    except Exception as e:                           # a broken store never breaks the check
        fpe = dict(score=None, ref=None, chassis=None, why="fingerprint failed: %s" % e)
    out["fp"] = fpe
    if out["by"] is None and fpe and fpe.get("chassis"):
        out.update(chassis=fpe["chassis"], by="fingerprint")
    return out


def expected(p: dict):
    """(chassis the params are for, where that came from): body.expect, else what drive.backend implies."""
    e = str((p.get("body") or {}).get("expect") or "")
    if e in CHASSIS:
        return e, "body.expect"
    backend = str((p.get("drive") or {}).get("backend", "rrc"))
    return ("wltoys" if backend == "gpio_pwm" else "stock_a1"), "drive.backend"


def check(robot, force: bool = False) -> dict:
    """{"ok": True | False | None, "applied", "expect", "detected", "by", "why", "evidence", "shell", "t"}; cached 5 s
    per (applied profile, expect, id_gpio)."""
    p = robot.p
    b = p.get("body") or {}
    applied = str(b.get("profile") or "")
    expect, exp_by = expected(p)
    key = (applied, expect, b.get("id_gpio", -1), str(b.get("shell") or ""))
    now = time.monotonic()
    with _lock:
        hit = _cache.get(key)
        if hit and not force and now - hit[0] < CACHE_S:
            return dict(hit[1])
    d = detect(robot)
    ok = None if d["chassis"] is None else d["chassis"] == expect
    name = applied or "(unstamped %s params)" % expect
    try:
        gpio = int(b.get("id_gpio", -1))
    except (TypeError, ValueError):
        gpio = -1
    strap_txt = ("strap %s %s" % (pin_text(gpio), "fitted" if d["strap"] == 0 else "open")
                 if d["strap"] is not None else "")
    if ok is True:
        why = "%s profile on a %s chassis (%s)" % (name, d["chassis"],
                                                  strap_txt or (d["fp"] or {}).get("why") or d["by"])
    elif ok is False:
        why = "body mismatch: %s profile on a %s chassis (%s) -- bw body %s" % (
            name, d["chassis"], strap_txt or (d["fp"] or {}).get("why") or d["by"], profile_for(d["chassis"]))
        if d["by"] == "strap" and d["strap"] == 0:      # [DAY1] pin 29 not free on this car: the way out, named
            why += " (if this IS the stock A1 and something else holds %s low: bw apply body.id_gpio=-1)" % (
                pin_text(gpio))
        elif d["by"] == "strap":                         # the WLtoys harness without its jumper cap
            why += " (if this IS the WLtoys: fit the jumper cap on %s, or bw apply body.id_gpio=-1)" % pin_text(gpio)
    else:
        parts = []
        if gpio < 0:
            parts.append("no strap pin (body.id_gpio -1)")
        else:
            parts.append("strap %s unreadable (lgpio missing or the line floats)" % pin_text(gpio))
        fpe = d["fp"]
        parts.append(fpe["why"] if fpe and fpe.get("why") else
                     "no usable fingerprint recorded for %s (bw body fingerprint --save, car still)" % name)
        why = "chassis unknown: " + "; ".join(parts)
    out = dict(ok=ok, applied=applied, expect=expect, detected=d["chassis"], by=d["by"], why=why,
               shell=str(b.get("shell") or ""), t=round(time.time(), 3),
               evidence=dict(strap=d["strap"], id_gpio=gpio, fp=d["fp"], expect_by=exp_by))
    with _lock:
        _cache[key] = (now, out)
    return dict(out)


def refuses(chk: dict, p: dict) -> bool:
    """S15: a DRIVES start is refused only when the STRAP proves the other chassis and body.enforce is on."""
    try:
        enforce = int((p.get("body") or {}).get("enforce", 1))
    except (TypeError, ValueError):
        enforce = 1
    return bool(enforce) and chk.get("ok") is False and chk.get("by") == "strap"


def cfg_block(chk: dict) -> dict:
    """hub.cfg()'s `body` block (8.6)."""
    return dict(profile=chk.get("applied") or "", expect=chk.get("expect") or "", shell=chk.get("shell") or "",
                detected=chk.get("detected"), ok=chk.get("ok"), by=chk.get("by"), why=chk.get("why"))
