"""Speed without an encoder: what makes the WLtoys build's open-loop 130 motor go where it is told.

The car sees its own speed only through the LIDAR POSE: map-matched at 10 Hz, 0.15-0.25 s old by the time a
3-5 scan slope is taken (research/dynamics_wl.md 6).  The motor gets only a duty cycle.  Four small pieces, each
behind its own param so one can be changed per mat test:

    DutyModel       commanded m/s -> duty: the kinetic intercept `deadband` (a rolling car stops below it) + m/s per
                    duty, times v_ref / V_bat (bat_comp), and a KICK above the breakaway to start from rest.
                    [DAY1] tests.duty_sweep fits it on the mat.                                   params drive.duty
    SpeedEstimator  the speed the lidar poses show (least-squares slope over the last n scans) against a first-order
                    model of the command over the SAME window: the difference is a bias carried to "now" (a Smith
                    predictor), so a 0.2 s old measurement corrects the present speed without putting 0.2 s of lag
                    in the loop.  It is also the travel estimate programs dead-reckon on.        params odo.source
    SpeedLoop       a PI on (commanded - estimated) added to the feed-forward, about 1 Hz, integrating only while
                    the lidar speed is fresh.                                              params drive.speed_loop
    StepTable       the park's PULSES: measured (travel mm, duty, ms) kicks; pick() returns the biggest that fits.
                    [DAY1] tests.step_table measures it.                                         params step.table
"""
from __future__ import annotations

import math
import threading
from collections import deque


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class DutyModel:
    """Reads its numbers from the live params dict every call (a `bw apply` takes effect at once)."""

    def __init__(self, cfg: dict):
        self.c = cfg

    def comp(self, vbat) -> float:
        """v_ref / V_bat: the same duty on a pack at 7.4 V drives ~12 % slower than at 8.4 V, which halves 0.15 m/s
        (dynamics_wl.md 5.3).  1.0 when off or when the voltage reading is implausible."""
        c = self.c
        if not int(c.get("bat_comp", 1)) or not vbat or not (5.0 < float(vbat) < 9.5):
            return 1.0
        return float(c.get("v_ref", 8.0)) / float(vbat)

    def ff(self, v: float, vbat=None) -> float:
        """Signed duty for a steady `v` m/s: deadband + |v| / mps_per_duty, battery-compensated, capped at `max`."""
        c = self.c
        if abs(v) < float(c.get("v_zero", 0.01)):
            return 0.0
        d = (float(c["deadband"]) + abs(v) / float(c["mps_per_duty"])) * self.comp(vbat)
        return math.copysign(min(d, float(c["max"])), v)

    def kick(self, sign: float, vbat=None) -> float:
        c = self.c
        return math.copysign(min(float(c["kick"]) * self.comp(vbat), float(c["max"])), sign)

    def speed(self, duty: float, vbat=None) -> float:
        """The steady speed the model predicts for a signed duty (the inverse of ff), m/s."""
        c = self.c
        a = abs(duty) / self.comp(vbat)
        if a <= float(c["deadband"]):
            return 0.0
        return math.copysign((a - float(c["deadband"])) * float(c["mps_per_duty"]), duty)


class SpeedEstimator:
    """Thread-safe: the program feeds poses, the robot's housekeeping steps the model, both read v()."""

    def __init__(self, n: int = 4, tau_s: float = 0.24, gain: float = 0.6, max_age_s: float = 0.6,
                 jump_mm: float = 150.0):
        self.n, self.tau, self.gain, self.max_age, self.jump = n, tau_s, gain, max_age_s, jump_mm
        self.lock = threading.Lock()
        self.vm = 0.0                    # the model's speed now, m/s
        self.sm = 0.0                    # the model's travel, mm (integral of vm)
        self.hist = deque(maxlen=400)    # (t, sm): the model's travel over time, for any past window
        self.poses = deque(maxlen=n)     # (t, s mm): the lidar's travel along the car's heading
        self.s = 0.0
        self.last = None                 # (t, x, y, th) of the last pose fed
        self.bias = 0.0                  # measured - model, m/s
        self.v_meas, self.t_meas = 0.0, 0.0
        self.t_now = 0.0
        self.resets = 0

    def configure(self, cfg: dict):
        self.n = max(3, int(cfg.get("n_scans", self.n)))
        self.poses = deque(self.poses, maxlen=self.n)
        self.tau = float(cfg.get("tau_s", self.tau))
        self.gain = float(cfg.get("gain", self.gain))
        self.max_age = float(cfg.get("max_age_s", self.max_age))

    def step(self, now: float, v_target: float, dt: float, decel: float | None = None):
        """The model: a first-order lag toward v_target (m/s, already scaled).  With `decel` (m/s^2, the braked stop
        drive.stop_decel) a zero target BRAKES at that rate instead: a lag alone would let a 0.2 m/s park pulse coast
        ~50 mm where the braked car stops in ~7, and the filter's prediction would run ahead of the lidar."""
        with self.lock:
            self.t_now = now
            if v_target == 0.0 and decel:
                self.vm = math.copysign(max(0.0, abs(self.vm) - decel * dt), self.vm)
            else:
                self.vm += (v_target - self.vm) * min(1.0, dt / max(self.tau, 1e-3))
            self.sm += self.vm * 1000.0 * dt
            self.hist.append((now, self.sm))
            if self.t_meas and now - self.t_meas > self.max_age:
                self.bias *= math.exp(-dt / 0.5)              # stale: forget the correction over ~0.5 s
            if v_target == 0.0 and abs(self.vm) < 0.01:
                self.bias *= math.exp(-dt / 0.2)              # stopped and told to stop: nothing to correct

    def _model_travel(self, t: float) -> float:
        h = self.hist
        if not h:
            return 0.0
        if t <= h[0][0]:
            return h[0][1]
        if t >= h[-1][0]:
            return h[-1][1] + self.vm * 1000.0 * (t - h[-1][0])
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (t0, s0), (t1, s1) = h[lo], h[hi]
        return s0 + (s1 - s0) * (t - t0) / max(t1 - t0, 1e-9)

    def add_pose(self, t: float, x: float, y: float, th: float):
        """A lidar-corrected rear-axle pose (mm, rad) and the time it holds for (the scan's time, not the time the
        filter finished).  A jump beyond `jump_mm` in one step or a gap over 0.5 s (a relocalisation, a lost scan)
        restarts the window instead of reading as speed."""
        with self.lock:
            if self.last is not None:
                t0, x0, y0, th0 = self.last
                dx, dy = x - x0, y - y0
                d = math.hypot(dx, dy)
                if t <= t0:
                    return
                if d > self.jump or t - t0 > 0.5:
                    self.poses.clear()
                    self.resets += 1
                else:
                    hm = math.atan2(math.sin(th) + math.sin(th0), math.cos(th) + math.cos(th0))
                    self.s += d if dx * math.cos(hm) + dy * math.sin(hm) >= 0.0 else -d
            self.last = (t, x, y, th)
            self.poses.append((t, self.s))
            if len(self.poses) < 3:
                return
            ts = [p[0] for p in self.poses]
            ss = [p[1] for p in self.poses]
            tm, sm_ = sum(ts) / len(ts), sum(ss) / len(ss)
            den = sum((a - tm) ** 2 for a in ts)
            if den <= 1e-9:
                return
            v = sum((a - tm) * (b - sm_) for a, b in zip(ts, ss)) / den / 1000.0      # m/s
            vmod = (self._model_travel(ts[-1]) - self._model_travel(ts[0])) / 1000.0 / max(ts[-1] - ts[0], 1e-6)
            self.bias += self.gain * ((v - vmod) - self.bias)
            self.v_meas, self.t_meas = v, t

    def v(self, now: float | None = None) -> float:
        """The estimated speed NOW, m/s: the model carried forward plus the lidar's bias."""
        with self.lock:
            return self.vm + self.bias

    def fresh(self, now: float) -> bool:
        return bool(self.t_meas) and now - self.t_meas < self.max_age


class SpeedLoop:
    """PI on the estimated speed, in duty units.  Resets on stop and on a direction change; integrates only while the
    measurement is fresh (otherwise it holds what it learnt)."""

    def __init__(self, cfg: dict):
        self.c = cfg
        self.i = 0.0
        self.sign = 0

    def reset(self):
        self.i, self.sign = 0.0, 0

    def out(self, v_cmd: float, v_hat: float, dt: float, fresh: bool) -> float:
        c = self.c
        if not int(c.get("on", 0)) or v_cmd == 0.0:
            self.reset()
            return 0.0
        sgn = 1 if v_cmd > 0 else -1
        if sgn != self.sign:
            self.i, self.sign = 0.0, sgn
        if not fresh:
            return self.i
        e = v_cmd - v_hat
        im = float(c.get("i_max", 0.04))
        self.i = _clamp(self.i + float(c.get("ki", 0.09)) * e * dt, -im, im)
        return float(c.get("kp", 0.03)) * e + self.i


class StepTable:
    """rows [[travel_mm, duty, ms], ...] as measured on the mat (battery-normalised duty)."""

    def __init__(self, rows):
        self.rows = sorted(([float(r[0]), float(r[1]), float(r[2])] for r in rows), key=lambda r: r[0])

    def pick(self, mm: float):
        """The biggest step not longer than `mm` (the smallest when even that is longer): (mm, duty, ms)."""
        if not self.rows:
            raise ValueError("empty step table")
        best = self.rows[0]
        for r in self.rows:
            if r[0] <= mm:
                best = r
        return tuple(best)

    @property
    def smallest(self) -> float:
        return self.rows[0][0] if self.rows else 0.0
