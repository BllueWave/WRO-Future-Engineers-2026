"""Oradar MS200 (and the LD06 / LD19 family it shares a frame with) over USB serial at 230400 baud.

The frame, from Oradar's own SDK (vendor/oradar_ros1/sdk/src/ordlidar_protocol.h + ord_lidar_driver.cpp), 47 bytes:

    54 2C | speed u16 (deg/s) | start_angle u16 (0.01 deg) | 12 x (distance u16 mm, confidence u8)
          | end_angle u16 (0.01 deg) | timestamp u16 (ms) | crc8 over the 46 bytes before it

crc8 is the table the SDK hard-codes: CRC-8 polynomial 0x4D, MSB first, init 0 (lidar_selftest() checks it against
the SDK's table).  Points are spread evenly from start to end angle; the SDK divides the span with INTEGER division
(`diff / 11`), which drops up to 0.11 deg per point -- this uses the exact division.

Our frame: angle 0 = straight ahead, POSITIVE = counter-clockwise (to the LEFT), degrees -- the convention every
geometry function here uses.  The raw lidar angle is converted with two parameters measured on the car (day-1 test):
`cw` (raw angles grow clockwise) and `offset_deg` (the raw angle of the car's nose).
"""
from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass

import numpy as np

FRAME_LEN = 47
POINTS = 12


def _crc_table() -> list:
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return t


CRC = _crc_table()


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = CRC[(c ^ b) & 0xFF]
    return c


def decode(frame: bytes):
    """One CRC-checked 47-byte frame -> (speed_dps, [(raw_angle_deg, dist_mm, conf) x 12]) or None."""
    if len(frame) != FRAME_LEN or frame[0] != 0x54 or frame[1] != 0x2C or crc8(frame[:46]) != frame[46]:
        return None
    speed, start = struct.unpack_from("<HH", frame, 2)
    end, _ts = struct.unpack_from("<HH", frame, 42)
    span = ((end + 36000 - start) % 36000) / 100.0
    step = span / (POINTS - 1)
    a0 = start / 100.0
    pts = []
    for i in range(POINTS):
        d, c = struct.unpack_from("<HB", frame, 6 + 3 * i)
        pts.append(((a0 + i * step) % 360.0, d, c))
    return speed, pts


class FrameParser:
    """Finds 54 2C frames in a byte stream, any chunk size."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.frames = 0

    def feed(self, chunk: bytes):
        self.buf += chunk
        out = []
        while True:
            i = self.buf.find(b"\x54\x2C")
            if i < 0:
                del self.buf[:-1]
                return out
            if len(self.buf) - i < FRAME_LEN:
                del self.buf[:i]
                return out
            f = bytes(self.buf[i:i + FRAME_LEN])
            r = decode(f)
            if r is None:
                self.crc_errors += 1
                del self.buf[:i + 1]           # resync one byte later: 54 2C can appear inside a payload
            else:
                self.frames += 1
                out.append(r)
                del self.buf[:i + FRAME_LEN]


@dataclass
class Scan:
    """One revolution in OUR frame: bin k covers angle -180 + k*res .. +res (deg, CCW positive).  mm, 0 = no return."""
    t: float
    res: float
    dist: np.ndarray                 # int32 mm per bin
    conf: np.ndarray                 # uint8 per bin
    rpm: float

    def angles(self) -> np.ndarray:
        return -180.0 + (np.arange(len(self.dist)) + 0.5) * self.res

    def at(self, angle_deg: float, half_width: float = 3.0, min_conf: int = 0) -> float:
        """Median distance (mm) of the valid returns within +-half_width of angle_deg; nan when there are none."""
        a = self.angles()
        d = ((a - angle_deg + 180.0) % 360.0) - 180.0
        m = (np.abs(d) <= half_width) & (self.dist > 0) & (self.conf >= min_conf)
        return float(np.median(self.dist[m])) if m.any() else float("nan")

    def xy(self, min_conf: int = 0):
        """Valid returns as (x forward, y left) in mm."""
        a = np.radians(self.angles())
        m = (self.dist > 0) & (self.conf >= min_conf)
        return self.dist[m] * np.cos(a[m]), self.dist[m] * np.sin(a[m])


class ScanAssembler:
    """Bins frames into revolutions; emits a Scan every time the raw angle wraps."""

    def __init__(self, res: float = 1.0, cw: bool = True, offset_deg: float = 0.0, min_mm: int = 50,
                 max_mm: int = 12000):
        self.res, self.cw, self.offset, self.min_mm, self.max_mm = res, cw, offset_deg, min_mm, max_mm
        self.n = int(round(360.0 / res))
        self._new()
        self.last_raw = None
        self.speed = 0.0

    def _new(self):
        self.dist = np.zeros(self.n, dtype=np.int32)
        self.conf = np.zeros(self.n, dtype=np.uint8)

    def to_ours(self, raw: float) -> float:
        a = raw - self.offset
        a = -a if self.cw else a
        return ((a + 180.0) % 360.0) - 180.0

    def add(self, speed: float, pts):
        done = None
        self.speed = speed
        for raw, d, c in pts:
            if self.last_raw is not None and raw < self.last_raw - 180.0:      # wrapped past 360 -> 0
                done = Scan(time.monotonic(), self.res, self.dist, self.conf, self.speed / 6.0)
                self._new()
            self.last_raw = raw
            if self.min_mm <= d <= self.max_mm:
                k = int((self.to_ours(raw) + 180.0) / self.res) % self.n
                self.dist[k], self.conf[k] = d, c
        return done


class Lidar:
    """Reader thread on the lidar's serial port; `latest` is the last complete revolution."""

    def __init__(self, device: str = "/dev/ldlidar", baudrate: int = 230400, **assembler_kw):
        import serial
        self.device, self.baud = device, baudrate
        self.port = serial.Serial(device, baudrate, timeout=0.05)
        self.parser = FrameParser()
        self.asm = ScanAssembler(**assembler_kw)
        self.latest: Scan | None = None
        self.scans = 0
        self.reopens = 0                     # the port reopened after it failed (a USB re-enumeration)
        self._run = True
        self._th = threading.Thread(target=self._loop, name="lidar-rx", daemon=True)
        self._th.start()

    def _reopen(self) -> bool:
        """A read that keeps failing is a port that went away (USB reset, a loose cable): close it and open the same
        device again -- the reader used to retry the dead handle forever, so the lidar never came back."""
        import serial
        try:
            self.port.close()
        except Exception:
            pass
        try:
            self.port = serial.Serial(self.device, self.baud, timeout=0.05)
        except Exception:
            return False
        self.parser = FrameParser()
        self.reopens += 1
        print("lidar: %s reopened" % self.device, flush=True)
        return True

    def _loop(self):
        errs = 0
        while self._run:
            try:
                chunk = self.port.read(max(FRAME_LEN, self.port.in_waiting))
                errs = 0
            except Exception:
                errs += 1
                time.sleep(0.05)
                if errs >= 20 and self._run:         # ~1 s of failed reads: the handle is dead
                    if not self._reopen():
                        time.sleep(0.5)
                    errs = 0
                continue
            for speed, pts in self.parser.feed(chunk):
                s = self.asm.add(speed, pts)
                if s is not None:
                    self.latest = s
                    self.scans += 1

    def close(self):
        self._run = False
        self.port.close()


def capture_raw(device: str = "/dev/ldlidar", baudrate: int = 230400, seconds: float = 2.0) -> bytes:
    """Day-1 check: raw bytes off the port, to confirm the frame format before trusting the parser."""
    import serial
    with serial.Serial(device, baudrate, timeout=0.1) as p:
        t0, out = time.monotonic(), bytearray()
        while time.monotonic() - t0 < seconds:
            out += p.read(4096)
        return bytes(out)


def synth_frame(start_deg: float, end_deg: float, dists, speed: int = 3600, conf: int = 200) -> bytes:
    """A valid frame, for tests and the simulator."""
    body = struct.pack("<BBHH", 0x54, 0x2C, speed, int(round(start_deg * 100)) % 36000)
    for d in dists:
        body += struct.pack("<HB", int(d), conf)
    body += struct.pack("<HH", int(round(end_deg * 100)) % 36000, 0)
    return body + bytes([crc8(body)])
