"""Fit the simulator's WLtoys plant to the MAT's run logs (2026-09-30 / 10-01) and save profiles/plant_mat1001.json.

    py -3 tools/fit_plant.py                 # every unique run log: ../mentorpi_deploy_10*/runs, ../references/*/
    py -3 tools/fit_plant.py --no-save       # print the fit and its residuals only
    py -3 tools/fit_plant.py --runs A.jsonl B.jsonl

What it fits (each printed with its residuals; bluewave/mock.py plant keys in brackets):
  1. the drive -- the true speed for a duty at a pack voltage.  The logged duty (tel drive.duty) and battery_v drive a
     copy of mock._wl_speed forward in time; its speed is compared with every NEW lidar speed (tel drive.v_lidar while
     lidar_fresh, the car going straight), taken as the mean over the estimator's window [t - 0.35, t - 0.05] s.
     [slope, intercept, vbat_exp, tau_s, brake_decel]  -- vbat_exp: the duty acts as duty x (V / v_ref)^vbat_exp
     (1 = the old model; the mat: the drive's bat_comp over-corrects, the car ran FASTER on a low pack).
     duty 0 on the MD13S = PWM low = the shorted motor brakes (brake_decel, constant) -- vs the coast (a_fric).
  2. the steering bias -- straight-ahead in the servo's degrees: yaw rate / speed against the servo angle sent (tel
     steer, through the servo's 85.6 deg/s slew) near straight: the zero crossing.  [steer_bias_deg]
  3. the effective wheelbase -- R x tan(true angle) in the corners against the fitted true speed: a piecewise-linear
     curve [[v, mm], ...] (flat outside), knots at 0.6 / 1.0 / 1.4 / 1.7 m/s fitted on the yaw rate; the slow end is the
     turn test's 137 mm at 0.2 m/s (profile chassis.wheelbase_m; the logs hold no slow corner).  [wb_curve]
  The bias is fitted twice: on the straights (yaw rate / speed vs the servo near 0) and on the corners (the bias that
  makes left and right R x tan agree); --bias-from picks (the corners' = the mat's "straight at servo -4").
The mat's robot params the logs show and the repo profile does not (steer.max_deg 26, camera.latency_s 0.09) go into
"robot_set": sim_run --mat1001 applies them with the plant ("scene": the rendered camera's age = what the car believes).

Every number here is a fit to the MAT; the simulator it feeds is still not the judge (team rule).
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # mentorpi/
REPO = os.path.dirname(ROOT)                       # WRO/
OUT = os.path.join(ROOT, "profiles", "plant_mat1001.json")
PROFILE = os.path.join(ROOT, "profiles", "wltoys_bw2.json")

V_REF = 8.0
BREAKAWAY = 0.1362             # [MAT 2026-09-30] duty_sweep: from rest (kept; the logs start every run with a kick)
SERVO_DPS = 85.6               # [MAT 2026-09-30] 0.35 s centre to lock (profile mock.plant servo_dps) -- kept
WB_SLOW_MM, WB_V_SLOW = 137.0, 0.2   # [MAT 2026-09-30] tests.turn_radius at 0.2 m/s (chassis.wheelbase_m 0.137)


# ------------------------------------------------------------------------------------------------------------ loading
def run_files(paths=None):
    if paths:
        cand = list(paths)
    else:
        cand = sorted(glob.glob(os.path.join(REPO, "mentorpi_deploy_10*", "runs", "*.jsonl")))
        cand += sorted(glob.glob(os.path.join(REPO, "references", "*", "*.jsonl")))
    seen, out = set(), []
    for f in cand:
        with open(f, "rb") as fh:
            h = hashlib.md5(fh.read()).hexdigest()
        if h in seen:
            continue
        seen.add(h)
        out.append(f)
    return out


def load(path):
    tel, logs, meta = [], [], None
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                d = json.loads(line)
            except ValueError:
                continue
            if "tel" in d and isinstance(d["tel"], dict):
                tel.append(d["tel"])
            elif "log" in d and isinstance(d["log"], dict):
                x = dict(d["log"])
                x["_t"] = float(d["t"])
                logs.append(x)
            elif "meta" in d:
                meta = d["meta"]
    if len(tel) < 50:
        return None

    def col(fn, default=np.nan):
        out = np.empty(len(tel))
        for i, x in enumerate(tel):
            try:
                v = fn(x)
                out[i] = default if v is None else float(v)
            except (TypeError, ValueError, KeyError):
                out[i] = default
        return out
    r = dict(name=os.path.basename(path), path=path, program=(meta or {}).get("program"))
    r["t"] = col(lambda x: x["t"])
    r["v_cmd"] = col(lambda x: x["v"], 0.0)
    r["steer"] = col(lambda x: x["steer"], 0.0)
    r["yaw"] = col(lambda x: x["yaw"])
    r["vbat"] = col(lambda x: x["battery_v"])
    r["front"] = col(lambda x: x["front_mm"])
    r["duty"] = col(lambda x: (x.get("drive") or {}).get("duty"), 0.0)
    r["v_lidar"] = col(lambda x: (x.get("drive") or {}).get("v_lidar"))
    r["fresh"] = col(lambda x: 1.0 if (x.get("drive") or {}).get("lidar_fresh") else 0.0, 0.0) > 0.5
    vb = r["vbat"]
    good = np.isfinite(vb) & (vb > 5.0) & (vb < 9.5)
    r["vbat"] = np.where(good, vb, np.nanmedian(vb[good]) if good.any() else 7.4)
    y = r["yaw"]
    if not np.isfinite(y).all():
        idx = np.arange(len(y))
        ok = np.isfinite(y)
        y = np.interp(idx, idx[ok], y[ok])
    r["yaw_u"] = np.degrees(np.unwrap(np.radians(y)))
    d = next((q for q in logs if q.get("ev") == "direction"), None)
    r["dir"] = None if d is None else ("ccw" if str(d.get("dir")).startswith("ccw") else "cw")
    # the servo that stopped applying commands (mat fact 6: steer_dead / turn_blocked): its wheels hold an angle the
    # log does not show -- no steering sample from 2 s before such an event to 1 s after it
    bad = [float(q["_t"]) for q in logs if q.get("ev") in ("steer_dead", "turn_blocked", "steer_wrong")]
    tt = np.array([float(x["t"]) for x in tel])
    r["steer_ok"] = np.ones(len(tel), bool)
    for tb in bad:
        r["steer_ok"] &= ~((tt >= tb - 2.0) & (tt <= tb + 1.0))
    r["n_bad"] = len(bad)
    st = next((q for q in logs if q.get("ev") == "start"), None)
    r["trim"] = ((st or {}).get("params") or {}).get("steer_trim")
    r["logs"] = logs
    return r


# ------------------------------------------------------------------------------------------------ 1. the drive's speed
def simulate_speed(r, prm):
    """A copy of mock._wl_speed over the logged duty / battery (sampled at the tel times, zero-order hold), exact per
    step: the first-order lag toward the duty's steady speed, the constant brake at duty 0, static friction."""
    slope, icpt, p, tau, bd = prm
    t, duty, vb = r["t"], r["duty"], r["vbat"]
    v = np.zeros(len(t))
    cur = 0.0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0.0 or dt > 0.5:                           # a gap in the log: start again from the lidar or rest
            vl = r["v_lidar"][i]
            cur = float(vl) if r["fresh"][i] and np.isfinite(vl) else 0.0
            v[i] = cur
            continue
        a = duty[i - 1] * (vb[i - 1] / V_REF) ** p
        if abs(cur) < 0.003 and a < BREAKAWAY:
            cur = 0.0
        elif a > icpt:
            vss = (a - icpt) * slope
            nv = cur + (vss - cur) * (1.0 - math.exp(-dt / tau))
            if vss < cur and bd > 0.0:                      # slowing: never faster than the brake
                nv = max(nv, cur - bd * dt)
            cur = nv
        else:
            cur = max(0.0, cur - bd * dt)
        v[i] = cur
    return v


def speed_obs(r):
    """(index, measured m/s): every NEW lidar speed while fresh, the car straight (|yaw rate| < 25 deg/s)."""
    t, vl, fr = r["t"], r["v_lidar"], r["fresh"]
    w = np.gradient(r["yaw_u"], t)
    out = []
    for i in range(1, len(t)):
        if not fr[i] or not np.isfinite(vl[i]) or vl[i] == vl[i - 1]:
            continue
        if abs(w[i]) > 25.0 or not (0.08 < vl[i] < 2.8):
            continue
        out.append((i, float(vl[i])))
    return out


def window_mean(t, v, i, a=0.35, b=0.05):
    m = (t >= t[i] - a) & (t <= t[i] - b)
    return float(v[m].mean()) if m.any() else float(v[i])


def speed_residuals(runs, obs, prm):
    res = []
    for r, ob in zip(runs, obs):
        v = simulate_speed(r, prm)
        for i, vm in ob:
            res.append(window_mean(r["t"], v, i) - vm)
    return np.array(res)


def fit_speed(runs, obs):
    from scipy.optimize import least_squares

    def f(x):
        return speed_residuals(runs, obs, x)
    x0 = np.array([4.49, 0.1187, 1.0, 0.3, 1.6])
    lo, hi = [2.0, 0.05, 0.0, 0.05, 0.3], [9.0, 0.2, 2.0, 1.2, 8.0]      # vbat_exp >= 0: physical
    sol = least_squares(f, x0, bounds=(lo, hi), loss="soft_l1", f_scale=0.1, x_scale=[1.0, 0.02, 0.5, 0.1, 0.5],
                        diff_step=1e-3, max_nfev=200)
    return sol.x, f(sol.x), f(x0)


# ------------------------------------------------------------------------------------ 2-3. the steering: bias, wheelbase
def servo_lagged(r, delay_s=0.0):
    """The wheel's servo angle (deg): tel steer through the 85.6 deg/s slew, `delay_s` late."""
    t, s = r["t"], r["steer"]
    out = np.empty(len(t))
    cur = s[0]
    for i in range(len(t)):
        dt = t[i] - t[i - 1] if i else 0.0
        if dt < 0.0 or dt > 0.5:
            cur = s[i]
        else:
            step = SERVO_DPS * dt
            cur = cur + max(-step, min(step, s[i - 1] - cur)) if i else cur
        out[i] = cur
    if delay_s > 0.0:
        out = np.interp(t - delay_s, t, out)
    return out


def steer_samples(runs, speeds, delay_s):
    """Per sample: (v true m/s, yaw rate rad/s, servo deg, servo settled?, run index)."""
    rows = []
    for k, (r, v) in enumerate(zip(runs, speeds)):
        t = r["t"]
        w = np.radians(np.gradient(r["yaw_u"], t))
        # the yaw rate averaged like the servo (a 20 Hz difference of a 50 Hz gyro integral: smooth over 3 samples)
        w = np.convolve(w, np.ones(3) / 3.0, mode="same")
        s = servo_lagged(r, delay_s)
        settled = np.abs(s - np.interp(t - delay_s, t, r["steer"])) < 0.5
        dt = np.diff(t, prepend=t[0])
        ok = (dt < 0.2) & np.isfinite(w) & r["steer_ok"]
        for i in np.nonzero(ok)[0]:
            rows.append((v[i], w[i], s[i], bool(settled[i]), k))
    return np.array(rows, dtype=float)


def fit_bias(S):
    """w / v = (s + b) / L near straight: the zero crossing of a line through (servo rad, w / v)."""
    m = (S[:, 0] > 0.4) & (np.abs(S[:, 2]) < 12.0) & (S[:, 3] > 0.5)
    x, y = np.radians(S[m, 2]), S[m, 1] / S[m, 0]
    A = np.vstack([x, np.ones_like(x)]).T
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    for _ in range(3):                                       # robust: drop the 5 % worst and refit
        e = y - A @ coef
        keep = np.abs(e) < np.quantile(np.abs(e), 0.95)
        coef, *_ = np.linalg.lstsq(A[keep], y[keep], rcond=None)
    a, c = coef
    b = math.degrees(c / a)
    e = y - A @ coef
    return b, 1000.0 / a, float(np.sqrt(np.mean(e[np.abs(e) < np.quantile(np.abs(e), 0.95)] ** 2))), int(m.sum())


def corner_bias(S, lo=0.6, hi=1.2):
    """The bias that makes R x tan(true angle) the same for left and right corners at lo..hi m/s (the mat's '210 BOTH
    ways'): a second estimate of the bias, from the corners instead of the straights."""
    best = None
    for b in np.arange(1.0, 8.01, 0.05):
        true = S[:, 2] + b
        m = (np.abs(true) >= 10.0) & (S[:, 3] > 0.5) & (S[:, 0] >= lo) & (S[:, 0] < hi) & (np.abs(S[:, 1]) > 0.4) &             (np.sign(S[:, 1]) == np.sign(true))
        L = S[m, 0] * np.tan(np.radians(np.abs(true[m]))) / np.abs(S[m, 1]) * 1000.0
        sd = np.sign(true[m])
        if (sd > 0).sum() < 20 or (sd < 0).sum() < 20:
            continue
        lft, rgt = float(np.median(L[sd > 0])), float(np.median(L[sd < 0]))
        if best is None or abs(lft - rgt) < abs(best[1] - best[2]):
            best = (float(b), lft, rgt, int((sd > 0).sum()), int((sd < 0).sum()))
    return best


WB_KNOTS = (0.6, 1.0, 1.4, 1.7)       # m/s: the curve's free knots (the slow end is the turn test's)


def wb_curve(v, knots):
    """R x tan(true angle), mm, at |v| m/s: piecewise linear through [[v, mm], ...], flat outside (mock.World)."""
    ks = sorted(knots)
    return np.interp(np.abs(np.asarray(v, float)), [k[0] for k in ks], [k[1] for k in ks])


def fit_wheelbase(S, bias, v_min=0.5):
    """R x tan(true angle) in the corners at >= v_min m/s (the fitted speed is least sure at a run's slow start): the
    curve's knots at WB_KNOTS, fitted on the yaw rate; the slow end [WB_V_SLOW, WB_SLOW_MM] is the turn test's."""
    from scipy.optimize import least_squares
    true = S[:, 2] + bias
    m = (np.abs(true) >= 10.0) & (S[:, 3] > 0.5) & (S[:, 0] > 0.3) & (np.abs(S[:, 1]) > 0.4) &         (np.sign(S[:, 1]) == np.sign(true))
    v, w, d = S[m, 0], S[m, 1], np.radians(true[m])
    L = v * np.tan(np.abs(d)) / np.abs(w) * 1000.0
    keep = (L > 60.0) & (L < 500.0)
    v, w, d, L, side = v[keep], w[keep], d[keep], L[keep], np.sign(true[m][keep])
    f = v >= v_min

    def knots(x):
        return [[WB_V_SLOW, WB_SLOW_MM]] + [[kv, float(xm)] for kv, xm in zip(WB_KNOTS, x)]

    def res(x):
        return v[f] * np.tan(np.abs(d[f])) / (wb_curve(v[f], knots(x)) / 1000.0) - np.abs(w[f])
    sol = least_squares(res, [220.0] * len(WB_KNOTS), bounds=([100.0] * len(WB_KNOTS), [450.0] * len(WB_KNOTS)),
                        loss="soft_l1", f_scale=0.2)
    bins = []
    for lo in np.arange(0.3, 1.8, 0.1):
        k = (v >= lo) & (v < lo + 0.1)
        if k.sum() >= 5:
            bins.append((round(lo + 0.05, 2), int(k.sum()), round(float(np.median(L[k]))),
                         round(float(np.median(L[k & (side > 0)]))) if (k & (side > 0)).sum() >= 3 else None,
                         round(float(np.median(L[k & (side < 0)]))) if (k & (side < 0)).sum() >= 3 else None))
    kn = [[a_, round(b_, 1)] for a_, b_ in knots(sol.x)]
    return kn, res(sol.x), bins, int(f.sum())


# ------------------------------------------------------------------------------------------------------------ report
def rms(x):
    x = np.asarray(x)
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="*", default=None)
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--bias", type=float, default=None, help="use this steering bias (deg) instead of a fitted one")
    ap.add_argument("--bias-from", default="straights", choices=("straights", "corners", "mean"),
                    help="which bias estimate the plant takes (both are printed)")
    a = ap.parse_args()
    files = run_files(a.runs)
    runs = [r for r in (load(f) for f in files) if r is not None]
    print("runs: %d unique (%s)" % (len(runs), ", ".join(r["name"][:24] for r in runs)))

    # 1. the drive
    obs = [speed_obs(r) for r in runs]
    print("\n1. DRIVE: %d new lidar speeds, straight" % sum(len(o) for o in obs))
    x, res, res0 = fit_speed(runs, obs)
    slope, icpt, p, tau, bd = (float(v) for v in x)
    print("   fit  slope %.3f m/s per duty, intercept %.4f, vbat_exp %.3f, tau_s %.3f, brake_decel %.2f m/s^2"
          % (slope, icpt, p, tau, bd))
    print("   residual (model - lidar) m/s: rms %.3f  median %+.3f  p90|e| %.3f   [the profile's line 4.49 / 0.1187, "
          "vbat_exp 1, tau 0.3, brake 1.6: rms %.3f median %+.3f]"
          % (rms(res), float(np.median(res)), float(np.quantile(np.abs(res), 0.9)), rms(res0), float(np.median(res0))))
    print("   the steady speed the fit gives (m/s):  duty  " + "  ".join("%4.2f" % d for d in (0.25, 0.30, 0.35, 0.40, 0.48)))
    for vb in (6.4, 7.1, 7.3, 8.0):
        row = ["%5.2f" % max(0.0, (d * (vb / V_REF) ** p - icpt) * slope) for d in (0.25, 0.30, 0.35, 0.40, 0.48)]
        print("       pack %.1f V                           %s" % (vb, "  ".join(row)))
    per_run = []
    speeds = []
    for r, ob in zip(runs, obs):
        v = simulate_speed(r, x)
        speeds.append(v)
        e = [window_mean(r["t"], v, i) - vm for i, vm in ob]
        vm_ = [vm for _i, vm in ob]
        # what the car's own duty model believed (commanded) against the lidar: the mat's 1.24-1.53 x
        ratio = [vm / max(r["v_cmd"][i], 0.05) for i, vm in ob if r["v_cmd"][i] > 0.3]
        per_run.append(dict(run=r["name"][:32], n=len(ob), bat=round(float(np.median(r["vbat"])), 2),
                            rms=round(rms(e), 3), bias=round(float(np.median(e)), 3) if e else None,
                            v_lidar_med=round(float(np.median(vm_)), 2) if vm_ else None,
                            lidar_over_cmd=round(float(np.median(ratio)), 2) if ratio else None))
    for q in per_run:
        print("   %-32s n %3d  pack %.2f V  rms %.3f  bias %+.3f  lidar/cmd %s" % (q["run"], q["n"], q["bat"], q["rms"],
                                                                                q["bias"] or 0.0, q["lidar_over_cmd"]))

    # 2. the steering bias (the servo -> yaw delay first: the one that explains the yaw rate best)
    best = None
    for dl in (0.0, 0.05, 0.1, 0.15):
        S = steer_samples(runs, speeds, dl)
        b, L0, e, n = fit_bias(S)
        if best is None or e < best[2]:
            best = (b, L0, e, n, dl, S)
    bias, L0, e_b, n_b, delay, S = best
    print("\n2. STEER BIAS: servo -> yaw delay %.2f s; %d near-straight samples: true = servo %+.2f deg "
          "(straight at servo %+.2f), small-angle wheelbase %.0f mm, residual rms %.4f rad/m"
          % (delay, n_b, bias, -bias, L0, e_b))
    per_bias = []
    for k, r in enumerate(runs):
        Sk = S[S[:, 4] == k]
        try:
            bk, _L, _e, nk = fit_bias(Sk)
        except Exception:
            continue
        if nk >= 40:
            per_bias.append((r["name"][:32], r["dir"], r["trim"], round(bk, 2), nk))
    for q in per_bias:
        print("   %-32s %-4s trim %-5s  bias %+.2f deg  (n %d)" % q)

    cb = corner_bias(S)
    if cb:
        print("   corners (0.6-1.2 m/s): left and right R x tan agree at bias %+.2f deg (L left %.0f / right %.0f mm; "
              "n %d / %d)" % cb)
    bias_str = bias
    if a.bias is not None:
        bias = float(a.bias)
        print("   USED: %+.2f deg (--bias)" % bias)
    elif a.bias_from == "corners" and cb:
        bias = cb[0]
        print("   USED: %+.2f deg (the corners' balance)" % bias)
    elif a.bias_from == "mean" and cb:
        bias = 0.5 * (bias_str + cb[0])
        print("   USED: %+.2f deg (the mean of the straights' %+.2f and the corners' %+.2f)" % (bias, bias_str, cb[0]))
    else:
        print("   USED: %+.2f deg (the straights)" % bias)

    # 3. the effective wheelbase
    kn, res_w, bins, n_w = fit_wheelbase(S, bias)
    print("\n3. WHEELBASE: %d corner samples >= 0.5 m/s (|true| >= 10 deg, servo settled): R x tan(true) curve %s "
          "(v m/s, mm; flat outside); yaw-rate residual rms %.3f rad/s" % (n_w, kn, rms(res_w)))
    print("   v bin   n   L median (left, right) mm")
    for b_ in bins:
        print("   %.2f  %4d   %4d  (%s, %s)" % b_)

    prof = json.load(open(PROFILE, encoding="utf-8"))
    base = dict(prof["patch"]["mock.plant"])
    plant = dict(base)
    plant.update(slope=round(slope, 3), intercept=round(icpt, 4), vbat_exp=round(p, 3), tau_s=round(tau, 3),
                 brake_decel=round(bd, 2), brake_tau_s=0.0, duty_meas_max=0.0, slope_hi_share=1.0,
                 steer_bias_deg=round(bias, 2), wb_curve=kn, servo_dps=SERVO_DPS, vbat=7.1)
    out = dict(
        desc="The WLtoys car's simulator truth fitted to the MAT 2026-09-30/10-01 run logs by tools/fit_plant.py "
             "(sim_run --mat1001).  Over the profile's mock.plant: the drive (slope / intercept at v_ref, vbat_exp, "
             "tau_s, the PWM-low brake_decel), the steering bias, the speed-dependent effective wheelbase; vbat 7.1 "
             "= the pack of the 18.7-18.8 s BEST_OPEN runs.  robot_set = the mat's robot params the repo profile "
             "lacks.  SIM, not the judge.",
        source=dict(runs=[os.path.relpath(r["path"], REPO).replace("\\", "/") for r in runs],
                    generator="mentorpi/tools/fit_plant.py", profile="profiles/wltoys_bw2.json mock.plant"),
        plant=plant,
        robot_set={"steer.max_deg": 26, "camera.latency_s": 0.09},
        scene={"latency_s": 0.09},
        fit=dict(drive=dict(n=int(sum(len(o) for o in obs)), rms_mps=round(rms(res), 3),
                            median_mps=round(float(np.median(res)), 3), rms_old_line_mps=round(rms(res0), 3),
                            per_run=per_run),
                 steer=dict(n=n_b, delay_s=delay, bias_deg=round(bias, 2), bias_straights=round(bias_str, 2),
                            bias_corners=round(cb[0], 2) if cb else None, bias_from=a.bias if a.bias is not None
                            else a.bias_from, small_angle_wb_mm=round(L0), rms_rad_per_m=round(e_b, 4),
                            per_run=[dict(run=q[0], dir=q[1], trim=q[2], bias=q[3], n=q[4]) for q in per_bias]),
                 wheelbase=dict(n=n_w, rms_rad_s=round(rms(res_w), 3), curve=kn,
                                bins=[dict(v=b_[0], n=b_[1], L=b_[2], left=b_[3], right=b_[4]) for b_ in bins])))
    if not a.no_save:
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            json.dump(out, f, indent=1)
            f.write("\n")
        print("\nsaved %s" % a.out)
    print(json.dumps({k: plant[k] for k in ("slope", "intercept", "vbat_exp", "tau_s", "brake_decel", "steer_bias_deg",
                                           "wb_curve", "servo_dps", "vbat")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
