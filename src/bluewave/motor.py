"""The WLtoys build's drive motor: one brushed 130 motor on a PWM + DIR driver (the team's Cytron MD13S) wired to the
Pi 5's GPIO.  The RRC Lite cannot run it: its firmware has no duty command, and its encoder speed PID ramps to 100 %
and never stops without an encoder (3d/bluewave_body/wl/research/electronics_wl.md 2.1).  The RRC Lite keeps the
steering servo, the IMU, the keys and the buzzer.

    SysfsPwm    hardware PWM through /sys/class/pwm.  The RP1 PWM0 chip is found by its DEVICE PATH (it is pwmchip2
                on kernel 6.6 and pwmchip0 on 6.12); channel 0 = GPIO12 (pin 32) with
                    dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4        under [pi5] in /boot/firmware/config.txt
    LgpioPwm    lgpio's software PWM on the same pin: the fallback when the overlay or the sysfs permission is missing
    Pins        output pins through lgpio: the direction pin (MD13S DIR = GPIO16, pin 36), or two for an IN1/IN2 driver
    PwmMotor    a signed duty -> PWM + direction, with the safety rules of electronics_wl.md 2.3:
                  * duty 0 for `dir_dwell_ms` before the direction flips
                  * stop = BRAKE (MD13S: PWM low brakes, and DIR goes low after it so a PWM/DIR pair swapped at the
                    Grove joint still stops; a two-pin driver: both high) or COAST (two-pin: both low)
                  * a watchdog thread: duty 0 when nothing refreshed it for `watchdog_ms`
                  * a GUARDIAN process: a hardware PWM keeps running after its program dies, so a child process that
                    only reads a heartbeat pipe zeroes it when the pipe closes (crash, kill -9) or stays silent
                  * atexit, and `python -m bluewave.motor --off` for systemd's ExecStopPost
                  * an optional PLUG BRAKE (PlugBrake, armed by hw.Robot when params drive.brake.active = 1): a slow-
                    down to a lower speed the same way reverses the duty for a bounded time -- off by default
    PlugBrake   active braking toward a lower speed: reverse duty of bounded size and length, >= 50 ms at duty 0
                before each DIR flip, stopped by its own speed prediction or the lidar's
    open_motor  params drive.pwm -> a PwmMotor on the real hardware
    probe()     `python -m bluewave.motor --probe`: every pwmchip with its device path, which one matches, the gpiochips

Everything takes a sysfs `root` and an `lgpio` module, so the unit tests run on a fake tree and a fake lgpio.
"""
from __future__ import annotations

import atexit
import glob
import math
import os
import subprocess
import sys
import threading
import time

RP1_PWM0 = "1f00098000.pwm"          # [3RD, verify with --probe] RP1 PWM0; 1f0009c000.pwm (PWM1) drives the fan
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------------------------------------------------- sysfs
def _write(path: str, value) -> None:
    with open(path, "w") as f:
        f.write(str(value))


def _read(path: str, default: str = "") -> str:
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def find_chip(root: str = "/sys", match: str = RP1_PWM0):
    """The pwmchip directory whose device path contains `match`, or None.  /sys/class/pwm/pwmchipN are symlinks into
    /sys/devices/.../<match>/pwm/pwmchipN; the number changes between kernels, the device path does not."""
    for c in sorted(glob.glob(os.path.join(root, "class", "pwm", "pwmchip*"))):
        if match in os.path.realpath(c).replace("\\", "/"):
            return c
    for depth in range(1, 4):                       # platform/axi/1000120000.pcie/1f00098000.pwm on the Pi 5
        pat = os.path.join(root, "devices", "platform", *(["*"] * depth), match, "pwm", "pwmchip*")
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


class SysfsPwm:
    def __init__(self, chip_dir: str, channel: int = 0, freq_hz: float = 490.0, wait_s: float = 1.5):
        self.chip, self.ch = chip_dir, int(channel)
        self.dir = os.path.join(chip_dir, "pwm%d" % self.ch)
        if not os.path.isdir(self.dir):
            _write(os.path.join(chip_dir, "export"), self.ch)
        duty = os.path.join(self.dir, "duty_cycle")
        end = time.monotonic() + wait_s              # udev sets the group a moment after the export
        while not (os.path.isfile(duty) and os.access(duty, os.W_OK)):
            if time.monotonic() > end:
                raise OSError("%s did not become writable (the user in group gpio? the udev rule?)" % duty)
            time.sleep(0.02)
        self.period = int(round(1e9 / float(freq_hz)))
        self._w("duty_cycle", 0)                     # before the period: a period below the old duty is refused
        self._w("period", self.period)
        self._w("duty_cycle", 0)
        try:
            self._w("polarity", "normal")            # only writable while disabled; an enabled channel keeps its own
        except OSError:
            pass
        self._w("enable", 1)
        self.how = "sysfs %s ch%d %.0f Hz" % (os.path.basename(chip_dir), self.ch, 1e9 / self.period)

    def _w(self, name: str, value):
        _write(os.path.join(self.dir, name), value)

    def set(self, duty: float):
        """0..1.  Written every call (no cache): the guardian may have zeroed it behind our back."""
        self._w("duty_cycle", int(round(self.period * min(1.0, max(0.0, float(duty))))))

    def off(self):
        self.set(0.0)

    def close(self):
        self.off()                                   # stays ENABLED at 0 %: the pin is driven low = MD13S brake


# ---------------------------------------------------------------------------------------------------------- lgpio
def open_chip(lg, label: str = "rp1"):
    """An lgpio handle on the gpiochip whose name or label contains `label` (the Pi 5's RP1 is gpiochip4 on kernel 6.6,
    gpiochip0 on 6.12), or chip 0 when none matches."""
    first = None
    for n in range(8):
        try:
            h = lg.gpiochip_open(n)
        except Exception:
            continue
        try:
            info = lg.gpio_get_chip_info(h)
            text = " ".join(str(x) for x in info[2:4]).lower()
        except Exception:
            text = ""
        if label and label.lower() in text:
            if first is not None:
                lg.gpiochip_close(first)
            return h
        if first is None:
            first = h
        else:
            lg.gpiochip_close(h)
    if first is None:
        raise OSError("no gpiochip could be opened")
    return first


class LgpioPwm:
    def __init__(self, lg, handle, gpio: int, freq_hz: float = 490.0):
        self.lg, self.h, self.gpio, self.freq = lg, handle, int(gpio), float(freq_hz)
        lg.gpio_claim_output(handle, self.gpio, 0)
        self.how = "lgpio software PWM on GPIO%d %.0f Hz" % (self.gpio, self.freq)

    def set(self, duty: float):
        self.lg.tx_pwm(self.h, self.gpio, self.freq, 100.0 * min(1.0, max(0.0, float(duty))))

    def off(self):
        self.lg.tx_pwm(self.h, self.gpio, self.freq, 0.0)
        self.lg.gpio_write(self.h, self.gpio, 0)

    def close(self):
        self.off()


class Pins:
    def __init__(self, lg, handle, pins):
        self.lg, self.h, self.pins = lg, handle, [int(p) for p in pins]
        for p in self.pins:
            lg.gpio_claim_output(handle, p, 0)
        self.levels = [0] * len(self.pins)

    def write(self, levels):
        for p, v in zip(self.pins, levels):
            self.lg.gpio_write(self.h, p, int(v))
        self.levels = [int(v) for v in levels]

    def close(self):
        self.write([0] * len(self.pins))


# ---------------------------------------------------------------------------------------------------------- guardian
class Guardian:
    """The child process that stops the motor when this process can no longer: its stdin is a heartbeat pipe."""

    def __init__(self, args: list):
        env = dict(os.environ, PYTHONPATH=ROOT + os.pathsep + os.environ.get("PYTHONPATH", ""))
        self.p = subprocess.Popen([sys.executable, "-m", "bluewave.motor", "--guard"] + [str(a) for a in args],
                                  stdin=subprocess.PIPE, cwd=ROOT, env=env)
        self.ok = True

    def beat(self):
        if not self.ok:
            return
        try:
            self.p.stdin.write(b".")
            self.p.stdin.flush()
        except (OSError, ValueError):
            self.ok = False                          # the guardian is gone: the watchdog thread still guards

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(1.5)
        except Exception:
            pass


def guard_main(argv) -> int:
    """`--guard --pwm-dir DIR | --lgpio-label L --gpio G  [--dir-pins 16,..] [--silence-ms 800]`: zero the motor when the
    heartbeat pipe closes (the parent died) or stays silent for silence-ms (the parent hung)."""
    a = dict(zip(argv[::2], argv[1::2]))
    silence = float(a.get("--silence-ms", 800)) / 1000.0
    state = dict(t=time.monotonic(), eof=False)

    def on_term(_sig, _frm):
        # systemd stops the whole cgroup at once (KillMode=control-group): the guardian gets SIGTERM together with the
        # agent, and without a handler it exited WITHOUT zeroing -- a hung agent's PWM then ran on
        _zero(a)
        raise SystemExit(0)

    try:
        import signal
        signal.signal(signal.SIGTERM, on_term)
    except (ImportError, ValueError, OSError):
        pass

    def reader():
        while True:
            b = sys.stdin.buffer.read(1)
            if not b:
                state["eof"] = True
                return
            state["t"] = time.monotonic()

    threading.Thread(target=reader, daemon=True).start()
    zeroed = False
    while True:
        time.sleep(0.02)
        quiet = time.monotonic() - state["t"] > silence
        if state["eof"] or (quiet and not zeroed):
            _zero(a)
            zeroed = True
            if state["eof"]:
                return 0
        elif not quiet:
            zeroed = False


def _claim_low(label: str, pins: list) -> int:
    """Claim `pins` as outputs driven low through lgpio; returns how many were claimed.  A pin the parent still holds
    refuses the claim (the parent is alive and hung): skipped, the next call tries again.  lgpio's claims die with the
    process that made them, so the pins stay claimed only while this one lives."""
    try:
        import lgpio
        h = open_chip(lgpio, label)
    except Exception:
        return 0
    n = 0
    for p in pins:
        try:
            lgpio.gpio_claim_output(h, p, 0)
            n += 1
        except Exception:
            pass
    if not n:
        try:
            lgpio.gpiochip_close(h)                  # nothing claimed: do not leak a handle on every quiet spell
        except Exception:
            pass
    return n


def _zero(a: dict):
    dirs = [int(p) for p in str(a.get("--dir-pins", "")).split(",") if p.strip()]
    label = a.get("--lgpio-label", "rp1")
    if a.get("--pwm-dir"):
        try:
            _write(os.path.join(a["--pwm-dir"], "duty_cycle"), 0)
        except OSError:
            pass
        if dirs:
            _claim_low(label, dirs)                  # the DIR pin low too (see PwmMotor._stop: a swapped PWM/DIR joint)
    elif a.get("--gpio"):                            # lgpio: the parent's claim is released when it dies
        _claim_low(label, [int(a["--gpio"])] + dirs)


# ---------------------------------------------------------------------------------------------------------- plug brake
PLUG_DWELL_MIN_S = 0.050    # [3RD MD13S] duty 0 at least this long before a DIR flip.  The plug brake holds it on its
#                             own, whatever drive.pwm.dir_dwell_ms says (a floor under the scale of a safety action)


class PlugBrake:
    """Active braking toward a LOWER speed in the SAME direction (params drive.brake; hw.Robot arms it).

    The mechanism: on the MD13S (sign-magnitude) duty 0 = PWM low = the motor shorted, which slows the car on the
    motor's own time constant -- dv/dt = -v / tau, tau 0.52 s measured from 0.25 m/s [MAT 2026-09-30].  From 1.7 m/s
    that is 0.9 m of travel to stop and 0.3 m to lose 0.5 m/s.  A REVERSE duty d drives toward the reverse speed of that
    duty instead: dv/dt = -(v + v_ss(d)) / tau, a stronger pull the whole way.

      pre    duty 0 (the motor's own brake) for >= PLUG_DWELL_MIN_S: the MD13S must never see DIR flip under duty
      plug   reverse duty = the largest that keeps the predicted deceleration <= ax_max (the grip / nose-over limit over
             a safety factor), capped at `duty` and the driver's max; at most max_ms long.  Ends when the speed --
             its own prediction, or the lidar's when that is fresh and lower -- reaches the target (plus the margin the
             post phase still takes off), or falls to v_floor (a plug near rest would reverse the car)
      post   duty 0 again for >= PLUG_DWELL_MIN_S before the caller's forward duty comes back

    It never runs through a stop: a zero duty (stop, E-STOP, the dead-man, the stall cut), a reversal, or the watchdog
    end it at once (PwmMotor), so every safety path stays the plain PWM-low brake the mat measured.  The deceleration
    at a reverse duty is [EST] (the plant law above with the mat's brake tau); the mat test is in the profile's notes.
    """

    def __init__(self, v0: float, target: float, cfg: dict, line: dict, meas=None, now: float | None = None):
        now = time.monotonic() if now is None else now
        self.sign = 1.0 if v0 >= 0.0 else -1.0
        self.v = abs(float(v0))                     # the predicted speed along the run direction, m/s
        self.target = abs(float(target))
        c = cfg or {}
        self.duty_cap = max(0.0, min(float(c.get("duty", 0.30)), float(line.get("max_duty", 0.5))))
        self.max_s = max(0.0, float(c.get("max_ms", 250))) / 1000.0
        self.dwell = max(PLUG_DWELL_MIN_S, float(line.get("dwell_ms", 50) or 0.0) / 1000.0)
        self.margin = max(0.0, float(c.get("v_margin", 0.05)))
        self.v_floor = max(0.10, float(c.get("v_floor", 0.25)))          # a floor: never plug a car near rest
        self.ax_max = max(0.0, float(c.get("ax_max", 3.0)))
        self.tau = max(0.05, float(c.get("tau_s", 0.52)))
        self.intercept, self.slope = float(line["deadband"]), max(1e-3, float(line["mps_per_duty"]))
        self.comp = float(line.get("comp", 1.0) or 1.0)
        self.meas = meas                            # () -> signed m/s (the lidar's, fresh) or None
        self.t0 = self.t_last = self.t_phase = now
        self.phase, self.t_plug, self.why = "pre", None, ""
        self.peak = 0.0                             # the largest reverse duty it asked for
        self.v_start = self.v

    def retarget(self, target: float):
        self.target = min(self.target, abs(float(target))) if self.phase != "post" else self.target

    def release(self, why: str = "released", now: float | None = None):
        """End the plug now but keep the post phase: >= PLUG_DWELL_MIN_S at duty 0 before the forward duty returns."""
        if self.phase != "post":
            self.why, self.phase = self.why or why, "post"
            self.t_phase = time.monotonic() if now is None else now

    def v_stop(self) -> float:
        """The speed to end the plug at: the post phase's PWM-low dwell still takes v x (1 - e^(-dwell/tau)) off."""
        return (self.target + self.margin) * math.exp(self.dwell / self.tau)

    def law(self, v: float) -> float:
        """The reverse duty (> 0, at the pin) whose predicted deceleration (v + v_ss(d)) / tau is ax_max at speed v,
        capped; 0 = the motor's own brake is already at ax_max (or a reverse duty under the intercept adds nothing)."""
        x = self.ax_max * self.tau - v
        if x <= 0.0:
            return 0.0
        d = min(self.duty_cap, (self.intercept + x / self.slope) * self.comp)
        return d if d > self.intercept * self.comp else 0.0

    def _vss(self, duty_along: float) -> float:
        """The steady speed along the run direction for a duty at the pin (negative = the reverse pull)."""
        a = abs(duty_along) / self.comp
        if a <= self.intercept:
            return 0.0
        return math.copysign((a - self.intercept) * self.slope, duty_along)

    def step(self, now: float, applied: float):
        """The duty for now (signed, at the pin), or None when the brake is over.  `applied` = the signed duty the
        driver had since the last call: the prediction integrates it."""
        dt = max(0.0, now - self.t_last)
        self.t_last = now
        self.v = max(0.0, self.v + (self._vss(applied * self.sign) - self.v) * min(1.0, dt / self.tau))
        if now - self.t0 > 2.0 * self.dwell + self.max_s + 0.1:
            self.why = self.why or "deadline"
            return None
        if self.phase == "pre":
            if now - self.t_phase < self.dwell:
                return 0.0
            self.phase, self.t_phase = "plug", now
        if self.phase == "plug":
            m = None
            if self.meas is not None:
                try:
                    m = self.meas()
                except Exception:
                    m = None
            v_seen = self.v if m is None or m != m else min(self.v, max(0.0, float(m) * self.sign))
            end = ("target" if v_seen <= self.v_stop() else "v_floor" if v_seen <= self.v_floor else
                   "max_ms" if self.t_plug is not None and now - self.t_plug >= self.max_s else "")
            if not end:
                d = self.law(v_seen)
                if d > 0.0 and applied * self.sign < 0.0 and self.t_plug is None:
                    self.t_plug = now                # the driver is reversed now: max_ms counts from here
                self.peak = max(self.peak, d)
                return -self.sign * d if d > 0.0 else 0.0
            self.why, self.phase, self.t_phase = end, "post", now
        if now - self.t_phase < self.dwell:
            return 0.0
        return None

    def summary(self) -> dict:
        return dict(v_start=round(self.v_start, 3), target=round(self.target, 3), v_pred=round(self.v, 3),
                    why=self.why, peak_duty=round(self.peak, 3), secs=round(self.t_last - self.t0, 3))


# ---------------------------------------------------------------------------------------------------------- motor
class PwmMotor:
    """set(signed duty): + = forward.  cfg is params drive.pwm (read live): dir_forward_high, dir_dwell_ms,
    stop_mode ("brake" | "coast"), watchdog_ms, max_duty.  brake_to(PlugBrake) arms a plug brake (see PlugBrake)."""

    def __init__(self, pwm, pins, cfg: dict, guardian: Guardian | None = None):
        self.pwm, self.pins, self.c, self.guardian = pwm, pins, cfg, guardian
        self.how = getattr(pwm, "how", type(pwm).__name__)
        self.lock = threading.Lock()
        self.duty = 0.0                     # the signed duty the driver has now
        self.run_dir = 0                    # direction of the last non-zero duty (0 = none yet)
        self.pin_dir = None                 # what the pins show: +1 / -1, "brake", "coast", None = unknown
        self.zero_t = 0.0                   # when the duty last went to 0
        self.fed = time.monotonic()
        self.trips = 0
        self.plug = None                    # an armed PlugBrake, or None (the default: never)
        self.plugs, self.plug_last = 0, None   # how many were armed; the summary of the last one that ended
        self._run = True
        with self.lock:
            self._stop(time.monotonic())
        threading.Thread(target=self._watch, name="motor-watchdog", daemon=True).start()
        atexit.register(self.close)

    # ------------------------------------------------------------------ output
    def _dir_levels(self, d: int):
        fwd_high = bool(self.c.get("dir_forward_high", True))
        if len(self.pins.pins) == 1:
            return [1 if (d > 0) == fwd_high else 0]
        a, b = (1, 0) if (d > 0) == fwd_high else (0, 1)
        return [a, b]

    def _stop(self, now: float):
        if self.duty != 0.0 or not self.zero_t:
            self.zero_t = now
        two = len(self.pins.pins) >= 2
        coast = str(self.c.get("stop_mode", "brake")) == "coast"
        if two and coast:
            self.pwm.set(0.0)
            if self.pin_dir != "coast":
                self.pins.write([0, 0])
                self.pin_dir = "coast"
        elif two:
            if self.pin_dir != "brake":
                self.pwm.set(0.0)
                self.pins.write([1, 1])                   # IN1 = IN2 = high: short brake whatever the PWM
                self.pin_dir = "brake"
            self.pwm.set(1.0)
        else:
            self.pwm.set(0.0)                            # sign-magnitude (MD13S): PWM low = brake; it cannot coast
            if self.pin_dir is not None:
                # DIR low too, after the PWM (still a brake: the MD13S ignores DIR while PWM is low).  With PWM and DIR
                # swapped at the hand-made Grove <-> Dupont joint, a DIR left high is the driver's PWM input held at
                # 100 %, and every stop path (E-STOP, watchdog, stall cut) would leave the motor running.
                self.pins.write([0])
                self.pin_dir = None
        self.duty = 0.0

    def _set(self, duty: float, now: float):
        mx = float(self.c.get("max_duty", 1.0))
        duty = max(-mx, min(mx, float(duty)))
        if abs(duty) < 1e-4:
            self._stop(now)
            return
        want = 1 if duty > 0 else -1
        if self.run_dir and want != self.run_dir:
            if self.duty != 0.0:
                self._stop(now)                          # first zero, then wait the dwell, then flip
                return
            if now - self.zero_t < float(self.c.get("dir_dwell_ms", 50)) / 1000.0:
                self._stop(now)
                return
        if self.pin_dir != want:
            self.pwm.set(0.0)
            self.pins.write(self._dir_levels(want))
            self.pin_dir = want
        self.pwm.set(abs(duty))
        self.duty, self.run_dir = duty, want

    def set(self, duty: float):
        with self.lock:
            self.fed = time.monotonic()
            try:
                self._set(self._plug_filter(float(duty), self.fed), self.fed)
            except Exception:
                self.plug = None
                self._safe_off()
                raise

    # ------------------------------------------------------------------ plug brake
    def brake_to(self, plug: PlugBrake | None) -> bool:
        """Arm a plug brake (True), or with None release the armed one (its post dwell still runs: duty 0 >= 50 ms
        before the forward duty).  Refused (False) unless the motor runs the way the brake slows it: a plug never
        starts from rest or against the running direction."""
        with self.lock:
            if plug is None:
                if self.plug is not None:
                    self.plug.release("cancelled")
                return False
            if self.run_dir == 0 or self.duty == 0.0 or (plug.sign > 0) != (self.run_dir > 0):
                return False
            self.plug = plug
            self.plugs += 1
            return True

    def _plug_end(self):
        if self.plug is not None:
            self.plug.why = self.plug.why or "cancelled"
            self.plug_last = self.plug.summary()
        self.plug = None

    def _plug_filter(self, duty: float, now: float) -> float:
        """Under the lock: the duty to write while a plug brake is armed.  A zero duty (stop, E-STOP, the dead-man, the
        stall cut, a failed board write) or a reversal ends the brake and passes unchanged: those paths stay exactly
        the plain PWM-low brake."""
        pb = self.plug
        if pb is None:
            return duty
        if abs(duty) < 1e-4 or (duty > 0) != (pb.sign > 0):
            self._plug_end()
            return duty
        out = pb.step(now, self.duty)
        if out is None:
            self._plug_end()
            return duty
        return out

    def _safe_off(self):
        try:
            self.pwm.set(0.0)
        except Exception:
            pass
        self.duty = 0.0

    # ------------------------------------------------------------------ safety
    def _watch(self):
        while self._run:
            time.sleep(0.02)
            if self.guardian is not None:
                self.guardian.beat()
            with self.lock:
                if self.duty != 0.0 and time.monotonic() - self.fed > float(self.c.get("watchdog_ms", 200)) / 1000.0:
                    self.trips += 1
                    self._plug_end()                 # a silent loop never gets its reverse duty back either
                    try:
                        self._stop(time.monotonic())
                    except Exception:
                        self._safe_off()

    def close(self):
        if not self._run:
            return
        self._run = False
        with self.lock:
            self._safe_off()
            for d in (self.pwm, self.pins):
                try:
                    d.close()
                except Exception:
                    pass
        if self.guardian is not None:
            self.guardian.close()


def open_motor(cfg: dict, root: str | None = None, lg=None) -> PwmMotor:
    """params drive.pwm -> a PwmMotor: hardware PWM if the RP1 chip is found and writable, else (lgpio_fallback) lgpio
    software PWM; the direction pin(s) through lgpio; a guardian process unless guardian=0."""
    root = root or cfg.get("sysfs_root", "/sys")
    if lg is None:
        try:
            import lgpio as lg
        except ImportError as e:
            raise OSError("python3-lgpio is missing: the direction pin needs it (sudo apt install python3-lgpio)") from e
    label = str(cfg.get("gpio_chip_label", "rp1"))
    h = open_chip(lg, label)
    dir_pins = [int(p) for p in cfg.get("dir_pins", [16])]
    pins = Pins(lg, h, dir_pins)
    pwm, why = None, ""
    if int(cfg.get("hw_pwm", 1)):
        chip = find_chip(root, str(cfg.get("chip_match", RP1_PWM0)))
        if chip:
            try:
                pwm = SysfsPwm(chip, int(cfg.get("channel", 0)), float(cfg.get("freq_hz", 490.0)))
            except OSError as e:
                why = str(e)
        else:
            why = "no pwmchip matches %s (the dtoverlay line?)" % cfg.get("chip_match", RP1_PWM0)
    if pwm is None:
        if not int(cfg.get("lgpio_fallback", 1)):
            raise OSError("hardware PWM unavailable: " + why)
        print("motor: hardware PWM unavailable (%s) -> lgpio software PWM" % why, flush=True)
        pwm = LgpioPwm(lg, h, int(cfg.get("gpio", 12)), float(cfg.get("freq_hz", 490.0)))
    guardian = None
    if int(cfg.get("guardian", 1)):
        args = (["--pwm-dir", pwm.dir] if isinstance(pwm, SysfsPwm) else ["--gpio", int(cfg.get("gpio", 12))])
        args += ["--lgpio-label", label, "--dir-pins", ",".join(str(p) for p in dir_pins),
                 "--silence-ms", int(cfg.get("guardian_silence_ms", 800))]
        try:
            guardian = Guardian(args)
        except OSError as e:
            print("motor: no guardian process (%s)" % e, flush=True)
    return PwmMotor(pwm, pins, cfg, guardian)


# ---------------------------------------------------------------------------------------------------------- CLI
def probe(root: str = "/sys", match: str = RP1_PWM0) -> list:
    out = []
    for c in sorted(glob.glob(os.path.join(root, "class", "pwm", "pwmchip*"))):
        out.append("%s -> %s  npwm=%s%s" % (os.path.basename(c), os.path.realpath(c), _read(os.path.join(c, "npwm"), "?"),
                                            "   <- the motor's chip" if match in os.path.realpath(c) else ""))
    if not out:
        out.append("no /sys/class/pwm/pwmchip*: add the dtoverlay line and reboot")
    try:
        import lgpio
        for n in range(8):
            try:
                h = lgpio.gpiochip_open(n)
            except Exception:
                continue
            out.append("gpiochip%d %s" % (n, lgpio.gpio_get_chip_info(h)[2:4]))
            lgpio.gpiochip_close(h)
    except ImportError:
        out.append("python3-lgpio is not installed")
    return out


def off_main() -> int:
    """systemd ExecStopPost: whatever happened to the program, the motor's PWM goes to 0 (sysfs; lgpio's claim died
    with the process) and the DIR pin(s) are claimed low."""
    sys.path.insert(0, ROOT)
    from bluewave import params as P
    p = P.load()
    d = p.get("drive", {})
    if d.get("backend", "rrc") != "gpio_pwm":
        print("drive.backend is %s: nothing to do" % d.get("backend", "rrc"))
        return 0
    cfg = d.get("pwm", {})
    chip = find_chip(cfg.get("sysfs_root", "/sys"), str(cfg.get("chip_match", RP1_PWM0)))
    pwm_dir = chip and os.path.join(chip, "pwm%d" % int(cfg.get("channel", 0)))
    lg = {"--lgpio-label": cfg.get("gpio_chip_label", "rp1"),
          "--dir-pins": ",".join(str(x) for x in cfg.get("dir_pins", [16]))}
    if pwm_dir and os.path.isdir(pwm_dir):
        _zero(dict(lg, **{"--pwm-dir": pwm_dir}))
        print("duty 0 on", pwm_dir, "+ DIR GPIO%s low" % lg["--dir-pins"])
    else:
        _zero(dict(lg, **{"--gpio": cfg.get("gpio", 12)}))
        print("GPIO%s + DIR GPIO%s driven low" % (cfg.get("gpio", 12), lg["--dir-pins"]))
    return 0


if __name__ == "__main__":
    if "--guard" in sys.argv:
        raise SystemExit(guard_main([x for x in sys.argv[1:] if x != "--guard"]))
    if "--off" in sys.argv:
        raise SystemExit(off_main())
    print("\n".join(probe()))
