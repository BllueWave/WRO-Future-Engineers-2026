"""RRC Lite controller board (STM32F407) over USB serial -- the MentorPi A1's motors, steering servo, IMU and buttons.

A lean port of Hiwonder's `ros_robot_controller_sdk.py` (vendor/MentorPi-A1/driver/ros_robot_controller/...), the only
documentation of the protocol:

    frame  = AA 55 | func | len | data[len] | crc8(func, len, data)       crc8 = Maxim / Dallas (reflected poly 0x8C)
    funcs  = SYS 0, LED 1, BUZZER 2, MOTOR 3, PWM_SERVO 4, BUS_SERVO 5, KEY 6, IMU 7, GAMEPAD 8, SBUS 9, OLED 10, RGB 11

What changed against the vendor SDK, and why:
  * reception reads every byte that is waiting, not one byte per call -- the SDK's `port.read()` with no size is one
    byte per Python call at 1 Mbaud;
  * every report is kept as the LATEST value with its arrival time, so a caller can tell a stale IMU from a fresh one;
  * writes are serialised by a lock, so the control loop and the dev agent can both command the board;
  * no ROS.  The board closes its own motor speed loop; there is no encoder report in the protocol (the SDK has none).
"""
from __future__ import annotations

import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field

FUNC_SYS, FUNC_LED, FUNC_BUZZER, FUNC_MOTOR, FUNC_PWM_SERVO, FUNC_BUS_SERVO = 0, 1, 2, 3, 4, 5
FUNC_KEY, FUNC_IMU, FUNC_GAMEPAD, FUNC_SBUS, FUNC_OLED, FUNC_RGB, FUNC_NONE = 6, 7, 8, 9, 10, 11, 12

# key events the board reports (vendor PacketReportKeyEvents)
KEY_PRESSED, KEY_LONGPRESS, KEY_LONGPRESS_REPEAT = 0x01, 0x02, 0x04
KEY_RELEASE_FROM_LP, KEY_RELEASE_FROM_SP, KEY_CLICK, KEY_DOUBLE_CLICK, KEY_TRIPLE_CLICK = 0x08, 0x10, 0x20, 0x40, 0x80
IMU_PERIOD = 0.02                        # s: the board streams the IMU at 50 Hz on its own clock
IMU_HIST = 1000                          # IMU samples kept (20 s at the board's 50 Hz; the simulator's is 50 Hz too)


def _crc8_table() -> list:
    """Maxim / Dallas CRC-8 (reflected polynomial 0x8C), the table the vendor SDK hard-codes."""
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
        t.append(c)
    return t


CRC8 = _crc8_table()


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = CRC8[c ^ b]
    return c


def frame(func: int, data: bytes) -> bytes:
    body = bytes([func, len(data)]) + bytes(data)
    return b"\xAA\x55" + body + bytes([crc8(body)])


class Parser:
    """Byte-level frame parser (the vendor state machine), fed with any chunk size.  Yields (func, data)."""

    def __init__(self):
        self.state = 0
        self.buf = bytearray()
        self.need = 0
        self.crc_errors = 0
        self.frames = 0

    def feed(self, chunk: bytes):
        out = []
        for b in chunk:
            s = self.state
            if s == 0:
                self.state = 1 if b == 0xAA else 0
            elif s == 1:
                self.state = 2 if b == 0x55 else (1 if b == 0xAA else 0)
            elif s == 2:
                if b < FUNC_NONE:
                    self.buf = bytearray([b])
                    self.state = 3
                else:
                    self.state = 0
            elif s == 3:
                self.buf.append(b)
                self.need = b
                self.state = 4 if b else 5
            elif s == 4:
                self.buf.append(b)
                if len(self.buf) - 2 >= self.need:
                    self.state = 5
            else:  # s == 5: checksum
                if crc8(bytes(self.buf)) == b:
                    self.frames += 1
                    out.append((self.buf[0], bytes(self.buf[2:])))
                else:
                    self.crc_errors += 1
                self.state = 0
        return out


@dataclass
class Stamped:
    value: object = None
    t: float = 0.0            # time.monotonic() of arrival, 0 = never

    def age(self) -> float:
        return time.monotonic() - self.t if self.t else float("inf")


@dataclass
class BoardState:
    imu: Stamped = field(default_factory=Stamped)       # (ax, ay, az, gx, gy, gz) as the board sends them
    battery_mv: Stamped = field(default_factory=Stamped)
    keys: deque = field(default_factory=lambda: deque(maxlen=32))   # (t, key_id, event)
    imu_times: deque = field(default_factory=lambda: deque(maxlen=200))
    # every IMU sample with its arrival time (t, ax, ay, az, gx, gy, gz): the run file's `imu` records
    # (bluewave/blackbox.py) -- tel samples acc at 20 Hz and misses a one-sample contact spike (BRAIN4_SPEC G5)
    imu_hist: deque = field(default_factory=lambda: deque(maxlen=IMU_HIST))
    # (samples, per-channel sum of value x its period): swapped as ONE tuple, so a reader never sees half an update.
    # The yaw integrates EVERY sample from it -- the latest value x the gap between reads lost a 0.26 s program
    # stall's ~20 deg of a turn (a gap >= 0.2 s was dropped: mat 2026-09-30, corner 6 exited 17 deg off)
    imu_int: tuple = (0, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def add_imu(self, now: float, value: tuple, period: float = IMU_PERIOD):
        self.imu.value, self.imu.t = value, now
        self.imu_times.append(now)
        self.imu_hist.append((now,) + tuple(value))
        n, sm = self.imu_int
        self.imu_int = (n + 1, tuple(a + float(b) * period for a, b in zip(sm, value)))

    def imu_rate_hz(self) -> float:
        ts = self.imu_times
        if len(ts) < 2 or ts[-1] == ts[0]:
            return 0.0
        return (len(ts) - 1) / (ts[-1] - ts[0])


class Board:
    """The RRC Lite on `device` (udev: /dev/rrc -> the CH9102 at 1a86:55d4)."""

    def __init__(self, device: str = "/dev/rrc", baudrate: int = 1_000_000):
        import serial                                    # pyserial; imported here so the mock needs none of it
        # write_timeout: a hung board (it stops reading) must not block the drive loop on write() -- the write raises,
        # the caller zeroes the motor (mat 2026-09-30: 0.3-0.5 s program stalls, dead-man trips, at full lock)
        self.port = serial.Serial(None, baudrate, timeout=0.02)
        self.port.write_timeout = 0.05
        self.port.rts = False                            # the vendor holds RTS / DTR low before open
        self.port.dtr = False
        self.port.port = device
        self.port.open()
        self.parser = Parser()
        self.state = BoardState()
        self._wlock = threading.Lock()
        self._run = True
        time.sleep(0.3)
        self._rx = threading.Thread(target=self._rx_loop, name="rrc-rx", daemon=True)
        self._rx.start()

    # ------------------------------------------------------------------ reception
    def _rx_loop(self):
        while self._run:
            try:
                chunk = self.port.read(max(1, self.port.in_waiting))
            except Exception:                            # unplugged: the watchdog sees the stale stamps
                time.sleep(0.05)
                continue
            if chunk:
                for func, data in self.parser.feed(chunk):
                    self._dispatch(func, data)

    def _dispatch(self, func: int, data: bytes):
        now = time.monotonic()
        st = self.state
        if func == FUNC_IMU and len(data) == 24:
            st.add_imu(now, struct.unpack("<6f", data))
        elif func == FUNC_SYS and len(data) == 3 and data[0] == 0x04:
            st.battery_mv.value, st.battery_mv.t = struct.unpack("<H", data[1:])[0], now
        elif func == FUNC_KEY and len(data) >= 2:
            st.keys.append((now, data[0], data[1]))

    # ------------------------------------------------------------------ commands
    def _write(self, func: int, data: bytes):
        with self._wlock:
            self.port.write(frame(func, data))

    def set_motors_rps(self, speeds: dict):
        """{motor_id (1-4): revolutions per second}.  The board runs the speed loop on its encoders."""
        data = bytearray([0x01, len(speeds)])
        for mid, rps in speeds.items():
            data += struct.pack("<Bf", int(mid) - 1, float(rps))
        self._write(FUNC_MOTOR, bytes(data))

    def set_pwm_servo(self, servo_id: int, pulse_us: int, duration_s: float = 0.02):
        d = int(duration_s * 1000)
        data = bytes([0x01, d & 0xFF, (d >> 8) & 0xFF, 1]) + struct.pack("<BH", int(servo_id), int(pulse_us))
        self._write(FUNC_PWM_SERVO, data)

    def set_pwm_servo_offset(self, servo_id: int, offset: int):
        self._write(FUNC_PWM_SERVO, struct.pack("<BBb", 0x07, int(servo_id), int(offset)))

    def buzzer(self, freq: int = 1900, on_s: float = 0.05, off_s: float = 0.01, repeat: int = 1):
        self._write(FUNC_BUZZER, struct.pack("<HHHH", int(freq), int(on_s * 1000), int(off_s * 1000), int(repeat)))

    def led(self, on_s: float = 0.1, off_s: float = 0.9, repeat: int = 1, led_id: int = 1):
        self._write(FUNC_LED, struct.pack("<BHHH", int(led_id), int(on_s * 1000), int(off_s * 1000), int(repeat)))

    def rgb(self, pixels):
        """[(index 1-based, r, g, b), ...]"""
        data = bytearray([0x01, len(pixels)])
        for i, r, g, b in pixels:
            data += struct.pack("<BBBB", int(i) - 1, int(r), int(g), int(b))
        self._write(FUNC_RGB, bytes(data))

    def close(self):
        self._run = False
        try:
            self.set_motors_rps({1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
        finally:
            self.port.close()
