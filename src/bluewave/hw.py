"""The one object every program, test and the dev agent drives: `Robot`.  Real hardware or the simulator (BW_MOCK=1),
same methods.

    robot.drive(v_mps, steer_deg)   # steer + = LEFT (CCW), v + = forward; clamped to params
    robot.stop()                    # motors 0, wheels straight
    robot.estop() / robot.arm()     # latched emergency stop: every drive() is refused until arm()
    robot.yaw                       # deg, CCW +, integrated gyro since reset_yaw()
    robot.scan                      # latest lidar revolution (lidar.Scan) or None
    robot.frame()                   # (bgr, t) latest camera frame
    robot.depth()                   # (uint16 mm, t_exposure) latest HP60C depth frame, (None, 0.0) unless depth.on = 1
    robot.button()                  # next start-button event or None
    robot.snapshot()                # dict for telemetry
    robot.v_odo()                   # m/s to dead-reckon on (odo.source: the command, or the lidar-corrected estimate)
    robot.feed_pose(t, x, y, th)    # a lidar-corrected pose for the speed estimator (programs that localise call it)
    robot.pulse(sign, steer, mm)    # the park's step mode: one measured kick, then brake; pulse_busy() while it runs

Two drive backends (params drive.backend):
    rrc       the RRC Lite's encoder motors hold the commanded speed (stock A1)
    gpio_pwm  one encoderless motor on a PWM + DIR driver on Pi 5 GPIO (the WLtoys build, bluewave/motor.py): a 200 Hz
              motor loop turns the commanded speed into a duty -- feed-forward (speed.DutyModel), a kick from rest, a
              PI on the lidar-measured speed (drive.speed_loop), a stall cut (drive.stall_s) -- and the motor's own
              watchdog and guardian process stop it when this loop or this process dies

A 100 Hz housekeeping thread integrates the gyro and enforces the dead-man: a drive command with a ttl stops the car
when it is not renewed -- the manual-drive safety net over Wi-Fi, and (safety.prog_ttl_ms) the race programs' net
against a stalled program thread: a 1.0 s stall at 0.50 m/s drove the WLtoys ~0.5 m unguarded (sim, wro_next seed 1).

The RRC Lite is a USB serial device the 130 motor's EMI, a loose cable or a board reset can take away.  A board write
that fails never leaves the old speed running: every command sets its speed, and zeroes the gpio_pwm motor (the Pi's
own GPIO), BEFORE the board is written; a failed write is recorded (board_error), a failed steering write stops the
gpio_pwm motor (it would drive on a steering nobody holds), and the house thread re-sends until the board takes it.
Before this, a raising servo write skipped the motor zero and the v_cmd update, and the 200 Hz motor loop drove on at
the last command for good: Stop, E-STOP, the dead-man and the program's end all raised the same way (sim: 1033 mm
into the island at 0.50 m/s after the serial failed).
"""
from __future__ import annotations

import math
import os
import threading
import time

from . import params as P
from . import speed as S
from .rrc import IMU_PERIOD                  # the board's IMU sample period: the yaw integrates every sample (house)


class Robot:
    def __init__(self, params: dict | None = None, mock: bool | None = None):
        self.p = params or P.load()
        self.mock = bool(int(os.environ.get("BW_MOCK", "0"))) if mock is None else mock
        self.sim = None
        if self.mock:
            from .mock import Sim
            self.sim = Sim(self.p)
            self.board, self.lidar, self.camera = self.sim.board, self.sim.lidar, self.sim.camera
        else:
            from .rrc import Board
            b = self.p["board"]
            self.board = Board(b["device"], b["baud"])
            self.lidar = self._try(self._open_lidar)
            self.camera = self._try(self._open_camera)
        # the HP60C depth stream only when asked for (params depth.on): off = no socket, no thread, nothing rendered
        self.depth_cam = None
        if int(self.p.get("depth", {}).get("on", 0)):
            self.depth_cam = self.sim.depth if self.mock else self._try(self._open_depth)
        self.yaw = 0.0
        self.v_cmd, self.steer_cmd = 0.0, 0.0
        self.cmd_until = 0.0                      # monotonic deadline of the last ttl command (0 = no dead-man)
        self.deadman_trips = 0                    # ttl commands the house thread stopped (a stalled program / link)
        self.board_error, self.board_errors = "", 0   # the last failed RRC write, and how many
        self._board_dirty, self._board_err_t = False, 0.0   # a write failed: the house thread re-sends the command
        self._board_bad_since = None              # the first of the servo / motor writes failing since the last one
        self._servo_want = None
        self._servo_last, self._servo_t = None, 0.0   # the last pulse sent: an unchanged pulse is re-sent only
        #                                           every 0.1 s (40 Hz of identical frames froze the RRC's servo
        #                                           output on the mat 2026-09-30; a port reopen reset it)
        #                                           that went through (board_down_s)
        self.estopped = False
        self.house_t = time.monotonic()           # heartbeat of the housekeeping thread (the dead-man)
        self._key_seen = 0
        self._last_imu_t = 0.0
        self._imu_src, self._imu_last = None, None     # the board state the yaw integrates from + its last imu_int
        self._lock = threading.Lock()
        self._run = True
        # ---- the drive: speed estimate (every backend), and the gpio_pwm motor loop
        d = self.p["drive"]
        self.backend = str(d.get("backend", "rrc"))
        self.speed = S.SpeedEstimator()
        self.speed.configure(self.p.get("odo", {}).get("est", {}))
        self.duty_model = S.DutyModel(d.get("duty", P.DEFAULTS["drive"]["duty"]))
        self.loop = S.SpeedLoop(d.get("speed_loop", P.DEFAULTS["drive"]["speed_loop"]))
        self.motor, self.motor_error = None, ""
        self.duty_out, self._model_v = 0.0, 0.0
        self._pulse = None                        # (end, signed duty) of a step-mode kick (gpio_pwm)
        self._pulse_end, self._pulse_rrc = 0.0, False
        self._raw = None                          # signed duty set by duty_only() (calibration tests)
        self._kick_until, self._rest_since, self._run_sign = 0.0, time.monotonic(), 0
        self._stall_since, self.stalled, self.stalls, self._stall_t = None, False, 0, 0.0
        self.kick_gain, self.no_starts = 1.0, 0   # the escalated kick (x the model's) and its re-kicks
        self.kick_hold = False                    # a program's: keep the kick at its present gain (wro_next: the
        #                                           shield sees something inside a park leg -- no harder push into it)
        self._obs = []                            # (scan t, nose range mm, yaw deg): the motion observer
        self._obs_t = None
        self._move_t = 0.0                        # when the present move command began
        if self.backend == "gpio_pwm":
            try:
                from . import motor as M
                cfg = d.get("pwm", P.DEFAULTS["drive"]["pwm"])
                if self.mock:
                    self.motor = M.PwmMotor(self.sim.driver.pwm, self.sim.driver.pins, cfg)
                else:
                    self.motor = M.open_motor(cfg)
            except Exception as e:                # the car must not drive on a half-open driver: no motor at all
                self.motor_error = "%s: %s" % (type(e).__name__, e)
                print("drive motor unavailable:", self.motor_error, flush=True)
        self.button_hw = None
        bcfg = self.p.get("buttons", {})
        if not self.mock and int(bcfg.get("start_gpio", -1)) >= 0:
            self.button_hw = self._try(lambda: StartButton(self.board.state, int(bcfg["start_gpio"]),
                                                           int(bcfg.get("start_key", 1))))
        threading.Thread(target=self._house, name="robot-house", daemon=True).start()
        if self.motor is not None:
            threading.Thread(target=self._motor_loop, name="robot-motor", daemon=True).start()
        self.stop()

    # ------------------------------------------------------------------ devices
    @staticmethod
    def _try(fn):
        try:
            return fn()
        except Exception as e:                    # a missing lidar / camera must not stop the board from working
            print("device unavailable:", e)
            return None

    def _open_lidar(self):
        from .lidar import Lidar
        l = self.p["lidar"]
        return Lidar(l["device"], l["baud"], res=l["res_deg"], cw=l["cw"], offset_deg=l["offset_deg"])

    def _open_camera(self):
        # RetryCamera, not camera.Camera: after a Pi reboot this agent starts before the camera container (the RGB is
        # its MJPEG on :8080), Camera's one open failed, and its reader retried the dead handle for good -- 'no
        # camera' until the agent restarted [MAT 2026-09-29/30].  RetryCamera opens in its thread and reopens
        c = self.p["camera"]
        return RetryCamera(c["device"], c["width"], c["height"], c["fps"])

    def _open_depth(self):
        from .camera import DepthCamera
        d = self.p["depth"]
        return DepthCamera(str(d.get("host", "127.0.0.1")), int(d.get("port", 8091)), float(d.get("latency_s", 0.12)),
                           float(d.get("stamp_lag_s", 0.067)))

    # ------------------------------------------------------------------ actuation
    def _ay_cap(self, v: float, steer: float) -> float:
        """|v| <= the speed whose lateral acceleration AT THE CG is drive.ay_max on this steering's circle."""
        d = self.p["drive"]
        ay = float(d.get("ay_max", 0.0) or 0.0)
        if ay <= 0.0 or abs(steer) < 0.5:
            return v
        R = self.p["chassis"]["wheelbase_m"] / math.tan(math.radians(abs(steer)))
        Rcg = math.hypot(R, float(d.get("cg_x_m", 0.0)))
        vmax = math.sqrt(ay * Rcg) * R / Rcg
        return max(-vmax, min(vmax, v))

    def tight_cap(self, v: float, steer: float) -> float:
        """drive.v_tight > 0 (the WLtoys profile; 0 = off, the stock A1): dynamics_wl.md S-1's tight-arc rule on the
        radius R the rear axle turns on at this steering -- |v| <= v_tight up to r_tight_mm (S-1: <= 0.30 m/s below
        250 mm, the full-lock dodges), rising linearly to max_mps at r_free_mm (300: S-1's corners from R 300 on are
        the program's v_corner).  Not the rollover (ay_max holds that, 3-4x above these speeds): what the sensing keeps
        up with on a tight arc -- the camera colour is paired to a lidar sign by bearing, which drifts omega x latency,
        and a dodge at straight speed is 2.4 rad/s (sim, WLtoys lot seed 1 cw: 0.45 m/s at R 190 beside a sign, 2.4 s
        of such arcs in one run).  A ramp out to 2 x r_tight also capped the lane changes (R 360 at 15 deg): with the
        speed preview (wro_next tight_look_s) 59.2 s for 3 laps against 54.6 s with this one (sim, 0.55 m/s, seeds 1-3
        both ways)."""
        d = self.p["drive"]
        vt = float(d.get("v_tight", 0.0) or 0.0)
        if vt <= 0.0 or abs(steer) < 0.5:
            return v
        rt = float(d.get("r_tight_mm", 250.0))
        rf = max(rt + 1.0, float(d.get("r_free_mm", 300.0)))
        R = self.p["chassis"]["wheelbase_m"] * 1000.0 / math.tan(math.radians(abs(steer)))
        vmax = vt + min(1.0, max(0.0, R - rt) / (rf - rt)) * max(0.0, float(d["max_mps"]) - vt)
        return max(-vmax, min(vmax, v))

    # ------------------------------------------------------------------ a program's speed envelope
    # key -> the params path under drive it sets.  Tighter = smaller for max_mps / duty_max / ay_max, LARGER for
    # v_tight (0 = off, the loosest); brake 1 = the plug brake on
    ENV_KEYS = {"max_mps": ("max_mps",), "duty_max": ("duty", "max"), "ay_max": ("ay_max",),
                "v_tight": ("v_tight",), "brake": ("brake", "active")}

    def speed_envelope(self, **caps):
        """A program's own speed caps for the length of a `with` block, then the params' again:

            with robot.speed_envelope(max_mps=1.7, duty_max=0.5, ay_max=3.7, v_tight=0, brake=1):
                ...

        Each cap may only LOOSEN as far as params drive.env allows (the ceilings the profile computed from the car:
        max_mps / duty_max / ay_max the highest, v_tight the lowest -- 0 = may switch the tight-arc rule off -- and
        brake 1 = the plug brake may be used); without drive.env a cap can only tighten.  duty_max never passes
        drive.pwm.max_duty (the driver's own cap), ay_max is never 0 (that is 'off').  The values are written into the
        live params (every reader -- the motor loop's duty cap, the shield -- sees them) and put back on exit, whatever
        ends the block.  One envelope at a time."""
        return _Envelope(self, caps)

    def _env_values(self, caps: dict) -> dict:
        """The caps as they will be applied: {key: value}, each clamped to drive.env (see speed_envelope)."""
        d = self.p["drive"]
        # the ceilings are the WLtoys duty drive's (its duty line, its tip / grip): the stock A1's encoder drive never
        # loosens by them -- drive.env is not in params.DEFAULTS, so `bw body stock_a1` leaves a WLtoys one behind
        lim = (d.get("env") or {}) if self.backend == "gpio_pwm" else {}
        out = {}
        for key, val in caps.items():
            if val is None:
                continue
            if key not in self.ENV_KEYS:
                raise ValueError("speed_envelope: unknown cap %r (%s)" % (key, ", ".join(self.ENV_KEYS)))
            path = self.ENV_KEYS[key]
            node = d
            for part in path[:-1]:
                node = node.get(part) or {}
            cur = node.get(path[-1], 0)
            val = float(val)
            if key == "brake":
                out[key] = 1 if (int(val) and int(lim.get("brake", 0) or 0)) else 0
            elif key == "v_tight":
                loosest = float(lim.get("v_tight", cur) or 0.0)
                out[key] = max(0.0, val, loosest) if (val > 0.0 or loosest > 0.0) else 0.0
            elif key == "ay_max":
                if val <= 0.0:
                    raise ValueError("speed_envelope: ay_max %g would switch the rollover cap off" % val)
                ceil = float(lim.get("ay_max") or (cur if float(cur or 0.0) > 0.0 else val))
                out[key] = min(val, ceil)
            else:
                ceil = float(lim.get(key) if lim.get(key) is not None else cur)
                if key == "duty_max":
                    ceil = min(ceil, float(d["pwm"].get("max_duty", 1.0)))
                out[key] = max(0.0, min(val, ceil))
        return out

    def _brake_arm(self, v: float):
        """(under the lock, from _apply) params drive.brake.active: a command to a speed lower than the estimate by
        more than brake.dv_min, the same way, arms the motor's plug brake (motor.PlugBrake) -- 'brake hard to v'; a
        lower command while it brakes lowers its target, a higher one or a reversal ends it.  A zero command never arms
        one (a stop stays the motor's own PWM-low brake, and the motor ends a running plug on the zero duty)."""
        m = self.motor
        if m is None or not hasattr(m, "brake_to"):
            return
        d = self.p["drive"]
        br = d.get("brake") or {}
        pb = getattr(m, "plug", None)
        if not int(br.get("active", 0) or 0):
            if pb is not None:
                m.brake_to(None)
            return
        if v == 0.0:
            return
        if pb is not None:
            if (v > 0) == (pb.sign > 0) and abs(v) < pb.v:
                pb.retarget(v)
            elif pb.phase != "post":
                m.brake_to(None)                              # released: its post dwell still runs
            return
        now = time.monotonic()
        v_now = self.speed.v(now)
        if v_now * v <= 0.0 or abs(v_now) <= float(br.get("v_floor", 0.25)) \
                or abs(v_now) - abs(v) < float(br.get("dv_min", 0.30)):
            return
        from . import motor as M
        du, pw = d["duty"], d["pwm"]
        line = dict(deadband=float(du["deadband"]), mps_per_duty=float(du["mps_per_duty"]),
                    comp=self.duty_model.comp(self.battery_v()),
                    max_duty=min(float(pw.get("max_duty", 1.0)), float(du["max"])),
                    dwell_ms=float(pw.get("dir_dwell_ms", 50)))
        sp = self.speed

        def meas():
            return sp.v_meas if sp.fresh(time.monotonic()) else None
        m.brake_to(M.PlugBrake(v_now, v, br, line, meas, now))

    def _board_try(self, what: str, fn, *a) -> bool:
        """One RRC write; False (recorded, printed at most every 5 s, re-sent by the house thread) when it raised."""
        try:
            fn(*a)
            if what in ("servo", "motors"):
                self._board_bad_since = None
            return True
        except Exception as e:
            if what in ("servo", "motors") and self._board_bad_since is None:
                self._board_bad_since = time.monotonic()
            self.board_error, self.board_errors = "%s: %s: %s" % (what, type(e).__name__, e), self.board_errors + 1
            self._board_dirty = self._board_dirty or what in ("servo", "motors")
            self._board_err_t = _house_error(self._board_err_t, "board " + what, e)
            return False

    def _apply(self, v: float, steer: float) -> bool:
        """Under the lock.  The speed first -- v_cmd, and the gpio_pwm motor zeroed when v is 0 -- then the board: a
        board write that raises must never leave the old command running (module docstring).  False = a write failed."""
        s, d = self.p["steer"], self.p["drive"]
        steer = max(-s["max_deg"], min(s["max_deg"], steer))
        v = max(-d["max_mps"], min(d["max_mps"], v))
        v = self.tight_cap(self._ay_cap(v, steer), steer)
        if self.backend == "gpio_pwm":
            self._brake_arm(v)                                # drive.brake.active: 'brake hard to v' (off by default)
        if v != 0.0 and (self.v_cmd == 0.0 or (v > 0) != (self.v_cmd > 0)):
            self._move_t = time.monotonic()               # a move begins (standing() waits out the observer's window)
        self.v_cmd, self.steer_cmd = v, steer
        if self.backend == "gpio_pwm":
            self._pulse, self._raw = None, None               # a drive command ends a pulse / a raw duty
            if v == 0.0:
                self._motor_zero()                            # stop NOW, not at the next motor tick
        pulse = s["center_us"] + (-1 if s["invert"] else 1) * steer * s["us_per_deg"]
        lo, hi = float(s.get("min_us", 0) or 0), float(s.get("max_us", 0) or 0)
        if 0 < lo < hi:
            pulse = max(lo, min(hi, pulse))
        pi, now_s = int(round(pulse)), time.monotonic()
        self._servo_want = pi                             # a change held back by the rate cap: house sends it
        gap_s = now_s - self._servo_t
        if self._servo_last is None or gap_s >= 0.1 or (abs(pi - self._servo_last) >= 2 and gap_s >= 0.04):
            ok = self._board_try("servo", self.board.set_pwm_servo, s["servo_id"], pi, 0.02)
            if ok:
                self._servo_last, self._servo_t = pi, now_s
        else:
            ok = True
        if self.backend == "gpio_pwm":
            if not ok and v != 0.0:
                self.v_cmd = 0.0                              # a steering nobody holds: the motor does not drive on it
                self._motor_zero()
            return ok
        rps = v * d["rps_scale"] / (3.141592653589793 * d["wheel_d_m"])
        if not int(d.get("rrc_motor", 1)):
            rps = 0.0
        return self._board_try("motors", self.board.set_motors_rps,
                               {int(m): rps * float(sign) for m, sign in d["motors"].items()}) and ok

    def imu_age_s(self) -> float:
        """s since the board's last IMU sample: the RRC streams it at 50 Hz, so > 0.3 s = the board stopped talking."""
        try:
            return float(self.board.state.imu.age())
        except Exception:
            return 0.0

    def board_reopen(self) -> bool:
        """Close and reopen the RRC port (opening it pulses DTR/RTS, which resets the board): the recovery for a
        servo output that stopped following commands while the link itself still worked (mat 2026-09-30)."""
        if self.mock:
            return False
        from .rrc import Board
        self.v_cmd = 0.0
        self._motor_zero()                                    # the port is shut ~0.4 s: nobody drives blind on it
        with self._lock:
            try:
                self.board.close()
            except Exception:
                pass
            b = self.p["board"]
            self.board = Board(b["device"], b["baud"])
            self._servo_last = None
            if getattr(self.button_hw, "state", None) is not None:
                self.button_hw.state = self.board.state       # START reads the new board's keys
        return True

    def _motor_zero(self):
        if self.motor is None:
            return
        try:
            self._motor_write(0.0)
        except Exception as e:                                # PwmMotor.set already forced the PWM off
            self.motor_error = "%s: %s" % (type(e).__name__, e)

    def board_down_s(self) -> float:
        """How long every servo / motor write has been failing (0 = the board takes them): the race programs end
        after ~1 s of it (the car already stands: a steering nobody holds stops the gpio_pwm motor)."""
        t = self._board_bad_since
        return 0.0 if t is None else time.monotonic() - t

    def prog_ttl_ms(self) -> int | None:
        """params safety.prog_ttl_ms: the dead-man the race programs put on every motion command (0 = none)."""
        try:
            v = int((self.p.get("safety") or {}).get("prog_ttl_ms", 250))
        except (TypeError, ValueError):
            v = 250
        return max(100, v) if v > 0 else None

    def drive(self, v_mps: float, steer_deg: float, ttl_ms: int | None = None) -> bool:
        with self._lock:
            if self.estopped:
                return False
            self._apply(v_mps, steer_deg)
            self.cmd_until = time.monotonic() + ttl_ms / 1000.0 if ttl_ms else 0.0
            return True

    def steer_only(self, steer_deg: float) -> bool:
        with self._lock:
            if self.estopped:                     # the latch holds the servo too (a sweep used to go on after E-STOP)
                return False
            self._apply(0.0, steer_deg)
            return True

    def motor_only(self, motor_id: int, rps: float) -> bool:
        """One wheel, raw rps, for calibration: the latch is checked at the moment of the write.  On the gpio_pwm
        drive the one motor turns at the duty the model gives for that wheel speed (v_cmd stays 0)."""
        with self._lock:
            if self.estopped:
                return False
            if self.backend == "gpio_pwm":
                if self.motor is None:
                    return False
                v = float(rps) * math.pi * self.p["drive"]["wheel_d_m"]
                self._raw = self.duty_model.ff(v, self.battery_v())
                return True
            if not int(self.p["drive"].get("rrc_motor", 1)):
                return False
            self.board.set_motors_rps({int(motor_id): float(rps)})
            return True

    def motor_stop(self, motor_id: int):
        """The end of a motor_only(): that motor (RRC) or the PWM motor to 0."""
        if self.backend == "gpio_pwm":
            with self._lock:
                self._raw = None
                if self.motor is not None and self.v_cmd == 0.0:
                    self._motor_write(0.0)
            return
        self.board.set_motors_rps({int(motor_id): 0.0})

    def duty_only(self, duty: float, steer_deg: float = 0.0) -> bool:
        """A raw signed duty with no model, loop or kick (tests.duty_sweep); ended by stop() / drive() / E-STOP."""
        with self._lock:
            if self.estopped or self.motor is None:
                return False
            if not self._apply(0.0, steer_deg):
                return False                              # the steering did not take the command: no duty on it
            self._raw = float(duty)
            self._motor_write(self._raw)                  # at once: no 5 ms brake blip between two held duties
            return True

    def pulse(self, sign: int, steer_deg: float, mm: float) -> float:
        """One step of the park's step mode, `mm` still to go: the step table's biggest kick that fits (gpio_pwm), or
        a timed drive at step.v_rrc (rrc), then brake.  Returns the travel it is expected to give (0 = refused)."""
        st = self.p.get("step", P.DEFAULTS["step"])
        with self._lock:
            if self.estopped:
                return 0.0
            now = time.monotonic()
            if self.backend == "gpio_pwm":
                if self.motor is None:
                    return 0.0
                row_mm, duty, ms = S.StepTable(st["table"]).pick(abs(mm))
                if not self._apply(0.0, steer_deg):
                    return 0.0                            # the steering did not take the command: no kick on it
                duty = math.copysign(min(duty * self.duty_model.comp(self.battery_v()),
                                         float(self.p["drive"]["duty"]["max"])), sign)
                self._pulse = (now + ms / 1000.0, duty)
                self._pulse_end = now + ms / 1000.0
                self._motor_write(duty)                   # the kick starts now, not at the next motor tick
                return row_mm
            v = float(st.get("v_rrc", 0.08))
            self._apply(math.copysign(v, sign), steer_deg)
            self._pulse_end, self._pulse_rrc = now + abs(mm) / (v * 1000.0), True
            return abs(mm)

    def pulse_busy(self) -> bool:
        return time.monotonic() < self._pulse_end

    def stop(self):
        with self._lock:
            self.cmd_until = 0.0
            self._pulse_rrc = False
            self._apply(0.0, 0.0)

    def estop(self):
        """The latch first and the Pi's own motor to 0, whatever the board then does (a dead RRC serial must not keep
        the latch from being set: the E-STOP used to raise before it latched)."""
        with self._lock:
            self.estopped = True
            self.cmd_until = 0.0
            self._pulse_rrc = False
            self._pulse, self._raw = None, None
            self._motor_zero()
            self._apply(0.0, 0.0)
        self._board_try("buzzer", self.board.buzzer, 2600, 0.15, 0.05, 2)

    def arm(self):
        with self._lock:
            self.estopped = False

    def beep(self, freq: int = 1900, on_s: float = 0.06):
        self._board_try("buzzer", self.board.buzzer, freq, on_s, 0.01, 1)

    # ------------------------------------------------------------------ the gpio_pwm motor loop
    def _motor_write(self, duty: float):
        self.motor.set(duty)
        self.duty_out = self.motor.duty

    def _motor_step(self, now: float, dt: float) -> float:
        """The signed duty for now.  Called under the lock."""
        d, dm = self.p["drive"], self.duty_model
        vbat = self.battery_v()
        if self.estopped:
            return 0.0
        if self._pulse is not None:
            end, duty = self._pulse
            if now < end:
                return duty
            self._pulse, self._rest_since = None, now
            return 0.0
        if self._raw is not None:
            return self._raw
        v = self.v_cmd
        if v == 0.0:
            if self._rest_since is None:
                self._rest_since = now
            self.loop.reset()
            self._stall_since, self.stalled = None, False
            return 0.0
        sgn = 1 if v > 0 else -1
        c = d["duty"]
        kick_s = float(c.get("kick_ms", 60)) / 1000.0
        rested = self._rest_since is not None and now - self._rest_since >= float(c.get("rest_ms", 150)) / 1000.0
        fresh = self.speed.fresh(now)
        seen = self.motion_seen(now)
        if seen is True or (fresh and abs(self.speed.v_meas) > 0.05):
            self.kick_gain = 1.0                          # it moves: the model's kick is enough again
        # standing although told to move: the scans themselves say so (motion_seen: the nose range and the gyro flat),
        # or the lidar speed does.  The scans come first: the lidar speed is the slope of the program's POSES, and a
        # program that dead-reckons on this drive's model moves its pose while the car stands -- the speed then read
        # 0.3 m/s at rest and no kick came (sim, breakaway +8 %: the pose slid 650 mm, LOST four times).  Only the
        # lidar speed used to count, so a WLtoys lapcam (no poses) held for a moment never moved again either
        still = seen is False or (fresh and abs(self.speed.v_meas) < 0.02 and seen is not True)
        if rested or sgn != self._run_sign:
            # a reversal first waits out the driver's dwell at duty 0 (motor.PwmMotor), so the kick starts after it
            dwell = float(d["pwm"].get("dir_dwell_ms", 50)) / 1000.0 if self._run_sign not in (0, sgn) else 0.0
            self._kick_until = now + dwell + kick_s
            self._stall_since, self.stalled = None, False
        elif still and not self.stalled and now > self._kick_until + 0.35:
            # static friction holds it below the breakaway (stick-slip at low speed, a snag, a hand) -- no running duty
            # the loop may reach restarts it, a kick does.  And each kick that did not is followed by one 10 % stronger
            # (up to duty.max): a fixed kick had 6 % over the plant's breakaway, and a pack sagging under load while the
            # board still reports 7.6 V left it under -- the car never left the start (sim, 7.0 V true / 7.6 reported)
            mx_gain = float(c["max"]) / max(float(c["kick"]), 1e-3)
            if not self.kick_hold:
                self.kick_gain = min(self.kick_gain * float(c.get("kick_step", 1.1)), max(1.0, mx_gain))
            self.no_starts += 1
            self._kick_until = now + kick_s
            if self.kick_gain < 1.3:
                self._stall_since = None                  # the first stronger kicks each get their own chance
        self._rest_since, self._run_sign = None, sgn
        # during a kick and 0.3 s after it the lidar speed measures the kick itself: the loop holds what it learnt
        # instead of answering the overshoot by cutting the duty under the kinetic intercept (the car then sticks
        # again -- sim, ki 0.2 at 0.10 m/s: three stops in the first second)
        settled = now > self._kick_until + 0.3
        duty = dm.ff(v, vbat) + self.loop.out(v, self.speed.v(now), dt, fresh and settled)
        if now < self._kick_until:
            duty = sgn * max(abs(duty), abs(dm.kick(sgn, vbat)) * self.kick_gain)
        stall_s = float(d.get("stall_s", 0.0) or 0.0)
        if stall_s > 0.0 and still:
            # told to move, kicked, and the lidar still sees no motion: a wall, a pillar or a limiter holds the car
            # and the 130 turns 20-40 W into heat (dynamics_wl.md 5.5)
            self._stall_since = self._stall_since or now
            if now - self._stall_since > stall_s and not self.stalled:
                self.stalled, self.stalls, self._stall_t = True, self.stalls + 1, now
                print("robot: STALL -- no motion on the lidar for %.1f s at duty %.3f: motor off until the command "
                      "stops or reverses (or drive.stall_retry_s)" % (stall_s, duty), flush=True)
        elif not self.stalled:
            self._stall_since = None
        retry = float(d.get("stall_retry_s", 2.0) or 0.0)
        if self.stalled and retry > 0.0 and now - self._stall_t > retry:
            # try again, kicked: a car held for a moment (a hand, a snag) and a program that keeps its command would
            # otherwise never move again -- a wall that still holds it stalls it again after stall_s (29 % of the time
            # at duty: 0.8 s on, 2 s off)
            self.stalled, self._stall_since, self._kick_until = False, None, now + kick_s
        if self.stalled:
            return 0.0
        mx = float(c["max"])
        return max(-mx, min(mx, duty))

    def _motor_loop(self):
        err_t, last = 0.0, time.monotonic()
        while self._run:
            time.sleep(0.005)
            now = time.monotonic()
            dt, last = min(now - last, 0.05), now
            try:
                with self._lock:
                    self._motor_write(self._motor_step(now, dt))
                    # the estimator's model runs on what the duty does (the model's steady speed for it), kicks and
                    # pulses included -- the lidar's bias then only has the model's error to correct.  At rest, a
                    # duty under the breakaway moves nothing (static friction)
                    vbat, dm = self.battery_v(), self.duty_model
                    vt = dm.speed(self.duty_out, vbat)
                    if abs(self.speed.vm) < 0.003 and abs(self.duty_out) < float(dm.c["breakaway"]) * dm.comp(vbat):
                        vt = 0.0
                    if self.standing(now):
                        vt = 0.0                  # the scans say it stands: so does the model (the odometry with it)
                    self._model_v = vt
            except Exception as e:
                err_t = _house_error(err_t, "motor loop", e)
                try:
                    self.motor.set(0.0)
                except Exception:
                    pass

    # ------------------------------------------------------------------ the motion observer
    def _observe(self):
        """(house thread) one sample per new lidar revolution: (scan time, the range straight ahead, the gyro yaw)."""
        sc = self.scan
        if sc is None or sc.t == self._obs_t:
            return
        self._obs_t = sc.t
        self._obs = (self._obs + [(float(sc.t), float(sc.at(0.0, 8.0)), float(self.yaw))])[-12:]   # swapped whole:
        #                                                                          the motor thread reads it unlocked

    def motion_seen(self, now: float | None = None, window: float = 0.45):
        """Does the car move, from the scans alone (no drive model, no pose)?  True / False / None = cannot tell.  Over
        the last `window` s: the median range of the nose sector (+-8 deg) and the gyro heading.  Standing: both flat
        (< 12 mm, < 1 deg; a still LD19's median holds within a few mm); moving: > 25 mm or > 2 deg (0.06 m/s).  An
        encoderless car with no fresh lidar pose (lapcam, a lost program) has nothing else that says it stands."""
        now = time.monotonic() if now is None else now
        obs = [o for o in self._obs if now - o[0] <= window + 0.05]
        if len(obs) < 3 or any(o[1] != o[1] for o in obs):
            return None
        rs = [o[1] for o in obs]
        span = max(rs) - min(rs)
        yspan = max(abs(((o[2] - obs[0][2] + 180.0) % 360.0) - 180.0) for o in obs)
        if span > 25.0 or yspan > 2.0:
            return True
        if span < 12.0 and yspan < 1.0:
            return False
        return None

    def standing(self, now: float | None = None) -> bool:
        """Told to move for longer than the observer's window, and the scans say the car stands (motion_seen False).
        The window must be waited out: at a start the scans of the car at rest are still in it, and zeroing the
        odometry on them froze the drive model at 0 for good (sim: the pose stood while the car drove, LOST)."""
        now = time.monotonic() if now is None else now
        return self.v_cmd != 0.0 and now - self._move_t > 0.5 and self.motion_seen(now) is False

    # ------------------------------------------------------------------ sensing
    @property
    def scan(self):
        return self.lidar.latest if self.lidar else None

    def frame(self):
        return self.camera.read() if self.camera else (None, 0.0)

    def depth(self):
        """(uint16 mm H x W, 0 = no return; the host monotonic time of its EXPOSURE) or (None, 0.0)."""
        return self.depth_cam.read() if self.depth_cam else (None, 0.0)

    def imu_raw(self):
        return self.board.state.imu.value

    def reset_yaw(self, value: float = 0.0):
        self.yaw = value

    def button(self):
        """The next key event since the last call, as (key_id, event) or None."""
        ks = list(self.board.state.keys)
        for t, kid, ev in ks:
            if t > self._key_seen:
                self._key_seen = t
                return kid, ev
        return None

    def battery_v(self):
        mv = self.board.state.battery_mv.value
        return mv / 1000.0 if mv else None

    def feed_pose(self, t: float, x: float, y: float, th: float):
        """A lidar-corrected rear-axle pose (mm, rad) and the monotonic time it holds for (the scan's)."""
        self.speed.add_pose(t, x, y, th)

    def v_odo(self, scale: float | None = None) -> float:
        """m/s for dead reckoning.  odo.source "cmd" (the stock A1: its board holds the speed on encoders): the
        commanded speed x speed_scale.  "lidar" (the encoderless WLtoys): the estimator -- the command through the
        plant's lag, corrected by the speed the lidar poses measured, carried to now."""
        o = self.p.get("odo", {})
        if o.get("source", "cmd") == "lidar":
            return self.speed.v(time.monotonic())
        return self.v_cmd * (float(o.get("speed_scale", 0.92)) if scale is None else scale)

    # ------------------------------------------------------------------ housekeeping
    def _house(self):
        """The only dead-man.  It must never die: PUT imu.gyro_axis=9 once raised IndexError here and killed the
        thread silently, and the next manual command then drove until the WS closed (measured 2026-09-23).  So the
        gyro and the dead-man each have their own guard, params are re-read every pass, and `house_t` is a
        heartbeat the hub checks before it accepts a drive command."""
        err_t = 0.0
        last = time.monotonic()
        while self._run:
            self.house_t = time.monotonic()
            try:
                im = self.p["imu"]
                bs = self.board.state
                st = bs.imu
                acc = getattr(bs, "imu_int", None)
                if acc is not None:
                    if bs is not self._imu_src:            # a new board (start, board_reopen): count from here
                        self._imu_src, self._imu_last = bs, acc
                    n, sm = acc
                    n0, sm0 = self._imu_last
                    ax = int(im["gyro_axis"])
                    if n > n0 and 0 <= ax < len(sm):
                        self._imu_last = acc
                        d = (sm[ax] - sm0[ax]) * im["gyro_sign"] - im["gyro_bias"] * (n - n0) * IMU_PERIOD
                        self.yaw = (self.yaw + d + 180.0) % 360.0 - 180.0
                        self._last_imu_t = st.t
                elif st.t and st.t != self._last_imu_t:
                    dt = st.t - self._last_imu_t if self._last_imu_t else 0.0
                    self._last_imu_t = st.t
                    ax = int(im["gyro_axis"])
                    if 0 < dt < 0.2 and st.value and 0 <= ax < len(st.value):
                        gz = st.value[ax] * im["gyro_sign"] - im["gyro_bias"]
                        self.yaw = (self.yaw + gz * dt + 180.0) % 360.0 - 180.0
            except Exception as e:
                err_t = _house_error(err_t, "gyro", e)
            try:
                if self.cmd_until and time.monotonic() > self.cmd_until and not self.pulse_busy():
                    # dead-man: the commands stopped arriving (a lost link, a stalled program).  The speed goes to 0
                    # and the steering stays: a step pulse's lock must not swing, and a stopped car does not care
                    with self._lock:
                        if self.cmd_until and time.monotonic() > self.cmd_until:
                            self.cmd_until = 0.0
                            self.deadman_trips += 1
                            self._apply(0.0, self.steer_cmd)
                if self._pulse_rrc and time.monotonic() > self._pulse_end:
                    self.stop()                       # the closed-loop board's step-mode pulse ends here
                sw = self._servo_want
                if (sw is not None and sw != self._servo_last and time.monotonic() - self._servo_t >= 0.04
                        and not self.mock):
                    with self._lock:                  # the rate cap held a change and no command came after it
                        if self._servo_want == sw and sw != self._servo_last and self._board_try(
                                "servo", self.board.set_pwm_servo, self.p["steer"]["servo_id"], sw, 0.02):
                            self._servo_last, self._servo_t = sw, time.monotonic()
                if self._board_dirty and not self.pulse_busy() and self._raw is None:
                    # a board write failed: send the present command again every pass until the board takes it
                    with self._lock:
                        self._board_dirty = False
                        self._apply(self.v_cmd, self.steer_cmd)
            except Exception as e:
                err_t = _house_error(err_t, "dead-man stop", e)
            try:
                self._observe()
            except Exception as e:
                err_t = _house_error(err_t, "motion observer", e)
            try:
                now = time.monotonic()
                dt, last = min(now - last, 0.05), now
                o = self.p.get("odo", {})
                if self.motor is not None:           # the open-loop motor: brakes at the measured rate on duty 0
                    self.speed.step(now, self._model_v * float(o.get("speed_scale", 0.92)), dt,
                                    float(self.p["drive"].get("stop_decel", 2.2)))
                else:
                    self.speed.step(now, self.v_cmd * float(o.get("speed_scale", 0.92)), dt)
            except Exception as e:
                err_t = _house_error(err_t, "speed model", e)
            time.sleep(0.01)

    def snapshot(self) -> dict:
        st = self.board.state
        sc = self.scan
        cam_fps = self.camera.fps() if self.camera else 0.0
        out = dict(t=time.time(), mock=self.mock, estop=self.estopped, v=round(self.v_cmd, 3),
                   steer=round(self.steer_cmd, 1), yaw=round(self.yaw, 2), battery_v=self.battery_v(),
                   imu_hz=round(st.imu_rate_hz(), 1), imu_age_ms=round(min(st.imu.age(), 99) * 1000),
                   lidar=bool(sc), lidar_rpm=round(sc.rpm, 0) if sc else 0, cam_fps=round(cam_fps, 1))
        acc = st.imu.value[:3] if st.imu.value else None
        if acc:
            out["acc"] = [round(float(a), 3) for a in acc]            # body tilt: the camera's pitch rides on it
        if self.camera is not None:                   # the newest frame's age: a frozen grabber shows here (review
            try:                                      # 2026-09-25; the analyzer's CAMERA_STALE reads it)
                _fr, t_fr = self.camera.read()
                if t_fr:
                    out["cam_age_ms"] = int(round(min(time.monotonic() - float(t_fr), 99.0) * 1000))
            except Exception:
                pass
        if sc is not None:
            out.update(front_mm=_r(sc.at(0)), left_mm=_r(sc.at(90)), right_mm=_r(sc.at(-90)), back_mm=_r(sc.at(180)))
        if self.depth_cam is not None:
            dc = self.depth_cam
            out["depth"] = dict(fps=round(dc.fps(), 1), frames=dc.frames, ok=bool(getattr(dc, "connected", True)),
                                age_ms=getattr(dc, "age_ms", None), error=getattr(dc, "error", "") or None)
        if self.board_errors:
            out["board_error"] = dict(n=self.board_errors, last=self.board_error)
        if self.deadman_trips:
            out["deadman_trips"] = self.deadman_trips
        if self.backend != "rrc":
            now = time.monotonic()
            out["drive"] = dict(backend=self.backend, duty=round(self.duty_out, 4), v_est=round(self.speed.v(now), 3),
                                v_lidar=round(self.speed.v_meas, 3), lidar_fresh=self.speed.fresh(now),
                                stall=self.stalled, error=self.motor_error or None,
                                watchdog_trips=getattr(self.motor, "trips", None), kick_gain=round(self.kick_gain, 2),
                                no_starts=self.no_starts)
        if self.sim is not None:
            w = self.sim.world
            out["sim"] = dict(x=round(w.x), y=round(w.y), th=round(w.th, 3), contacts=w.contacts)
        return out

    def close(self):
        self._run = False
        try:
            self.stop()
        finally:                                  # the motor closes (duty 0, guardian released) whatever stop() did
            for d in (self.motor, self.button_hw, self.lidar, self.camera, self.depth_cam, self.board, self.sim):
                if d is not None:
                    try:
                        d.close()
                    except Exception:
                        pass


class _Envelope:
    """Robot.speed_envelope's context manager: writes the clamped caps into the live params under the robot's lock and
    puts the old values back on exit (an exception, the program's end, the stop event -- every way out)."""

    def __init__(self, robot: Robot, caps: dict):
        self.r, self.caps = robot, dict(caps)
        self.saved, self.applied = [], {}

    def __enter__(self):
        r = self.r
        with r._lock:
            if getattr(r, "_envelope", None) is not None:
                raise RuntimeError("speed_envelope: one is active already (%s)" % r._envelope)
            vals = r._env_values(self.caps)
            d = r.p["drive"]
            for key, val in vals.items():
                path = Robot.ENV_KEYS[key]
                node = d
                for part in path[:-1]:
                    if not isinstance(node.get(part), dict):
                        node[part] = {}
                    node = node[part]
                had = path[-1] in node
                self.saved.append((node, path[-1], had, node.get(path[-1])))
                node[path[-1]] = int(val) if key == "brake" else val
            self.applied = vals
            r._envelope = dict(vals)
        return self

    def __exit__(self, *_exc):
        r = self.r
        with r._lock:
            for node, leaf, had, old in reversed(self.saved):
                if had:
                    node[leaf] = old
                else:
                    node.pop(leaf, None)
            self.saved = []
            r._envelope = None
            m = r.motor
            if m is not None and getattr(m, "plug", None) is not None:
                m.brake_to(None)                  # the params' own brake setting again from here
        return False


class RetryCamera:
    """The RGB grabber the agent opens (hw.Robot._open_camera): camera.Camera's read() / fps() / age_s() / frames, but
    the capture is opened IN the reader thread and opened again until frames come.

    The mechanism it fixes ([MAT 2026-09-29/30]): after a Pi reboot bluewave-agent starts before the camera container
    serves its MJPEG (http://127.0.0.1:8080, bluewave-camera.service); camera.Camera opened the stream once in its
    constructor, the open failed, and its reader called read() on that dead handle for good -- 'no camera' until the
    agent restarted.  A stream that dies later (bw sys restart_camera, the container restarting) froze it the same way.

    Here: no capture, or one that is not opened -> a new open every retry_s; an opened capture with no frame for
    stale_s -> released and opened again.  The capture is released only by this thread (releasing it from another
    thread while read() blocks inside OpenCV can crash the process), so close() only asks the thread to end.  The open
    is the one camera.Camera did (the V4L2 backend for a device number, OpenCV's default for a URL), proven on the car.
    `open_cap` replaces cv2 in the tests.  opens / reopens / waiting / error say what it did (the journal gets one line
    when it starts waiting and one when the frames come)."""

    STALE_S = 2.0           # an open stream without a frame this long is dead: 15-30 fps = 30-60 frames missed
    RETRY_S = 2.0           # between failed opens: the container needs ~5-10 s after boot / restart_camera

    def __init__(self, device=0, width: int = 640, height: int = 480, fps: int = 30, open_cap=None,
                 stale_s: float | None = None, retry_s: float | None = None):
        from collections import deque
        self.device, self.width, self.height, self.fps_req = device, int(width), int(height), int(fps)
        self._open_cap = open_cap or self._cv2_open
        self.stale_s = float(self.STALE_S if stale_s is None else stale_s)
        self.retry_s = float(self.RETRY_S if retry_s is None else retry_s)
        self._frame, self._t = None, 0.0
        self._times = deque(maxlen=60)          # the arrival stamps camera.Camera.fps() / age_s() read
        self.frames = 0
        self.opens = self.reopens = self.failed_opens = 0
        self.waiting, self.error = True, ""
        self._run = True
        self._th = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._th.start()

    def _cv2_open(self):
        import cv2
        dev = self.device
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2) if isinstance(dev, int) else cv2.VideoCapture(dev)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps_req)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # the newest frame, not a queue of old ones
        return cap

    @staticmethod
    def _release(cap):
        try:
            cap.release()
        except Exception:
            pass

    def _pause(self, secs: float):
        end = time.monotonic() + secs
        while self._run and time.monotonic() < end:
            time.sleep(0.05)

    def _loop(self):
        cap, since = None, 0.0                  # since: the newest frame, or the open when none came yet
        while self._run:
            if cap is None:
                try:
                    cap = self._open_cap()
                    if cap is not None and not cap.isOpened():
                        self._release(cap)
                        cap, self.error = None, "not opened"
                except Exception as e:
                    cap, self.error = None, "%s: %s" % (type(e).__name__, e)
                if cap is None:
                    self.failed_opens += 1
                    if self.failed_opens == 1:
                        print("camera: %s not there yet (%s) -- retrying every %g s (the camera container starts "
                              "after the agent at boot)" % (self.device, self.error, self.retry_s), flush=True)
                    # the first ~30 s every retry_s (a boot), then every 10 s (a camera that is really missing: OpenCV
                    # warns in the journal on every failed open)
                    self._pause(self.retry_s if self.failed_opens <= 15 else max(self.retry_s, 10.0))
                    continue
                self.opens += 1
                since = time.monotonic()
            try:
                ok, f = cap.read()
            except Exception as e:
                ok, f, self.error = False, None, "%s: %s" % (type(e).__name__, e)
            now = time.monotonic()
            if ok and f is not None:
                if self.waiting:
                    self.waiting, self.error = False, ""
                    print("camera: frames from %s (open %d, %d failed opens before)" % (
                        self.device, self.opens, self.failed_opens), flush=True)
                self._frame, self._t, since = f, now, now
                self._times.append(now)
                self.frames += 1
                continue
            if now - since > self.stale_s:     # opened, but no frame for stale_s: the stream is dead -- open again
                self._release(cap)
                cap = None
                self.reopens += 1
                self.waiting, self.error = True, self.error or "no frame for %.1f s" % (now - since)
                print("camera: no frame for %.1f s -- reopening %s" % (now - since, self.device), flush=True)
                continue
            time.sleep(0.02)
        if cap is not None:
            self._release(cap)

    def read(self):
        return self._frame, self._t

    def fps(self) -> float:
        from .camera import Camera
        return Camera.fps(self)                 # the same measured-up-to-now rate, on this grabber's stamps

    def age_s(self) -> float | None:
        return (time.monotonic() - self._t) if self._t else None

    def close(self):
        self._run = False                       # the reader releases the capture itself (see the class notes)
        self._th.join(1.0)


class StartButton:
    """A start button between a Pi GPIO and GND (the WLtoys build: GPIO17 = pin 11, GND = pin 9), internal pull-up,
    polled every 5 ms: a press held >= 30 ms, then released, is pushed into the board's key stream as
    (key, KEY_CLICK) -- race/race_main.py reads it like the RRC's KEY1 and does not change."""

    def __init__(self, state, gpio: int, key: int = 1, lg=None, poll_s: float = 0.005, debounce_s: float = 0.03):
        from . import motor as M
        from .rrc import KEY_CLICK
        if lg is None:
            import lgpio as lg
        self.lg, self.state, self.gpio, self.key, self.click = lg, state, int(gpio), int(key), KEY_CLICK
        self.h = M.open_chip(lg)
        lg.gpio_claim_input(self.h, self.gpio, getattr(lg, "SET_PULL_UP", 32))
        self.poll, self.deb = poll_s, debounce_s
        self.presses = 0
        self._run = True
        threading.Thread(target=self._loop, name="start-button", daemon=True).start()

    def _loop(self):
        down_t, pressed = None, False
        while self._run:
            try:
                low = self.lg.gpio_read(self.h, self.gpio) == 0
            except Exception:
                time.sleep(0.1)
                continue
            now = time.monotonic()
            if low:
                down_t = down_t or now
                pressed = pressed or now - down_t >= self.deb
            else:
                if pressed:
                    self.presses += 1
                    self.state.keys.append((now, self.key, self.click))
                down_t, pressed = None, False
            time.sleep(self.poll)

    def close(self):
        self._run = False


def _r(x):
    return None if x != x else int(x)          # nan -> None for JSON


def _house_error(last_t: float, what: str, e: Exception) -> float:
    """Print a housekeeping error at most once per 5 s; returns the time of the last print."""
    now = time.monotonic()
    if now - last_t < 5.0:
        return last_t
    print("robot-house: %s failed: %s: %s" % (what, type(e).__name__, e), flush=True)
    return now
