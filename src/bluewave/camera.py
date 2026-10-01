"""The RGB camera as a background grabber, the HP60C's depth stream (DepthCamera, from bluewave/depth_bridge.py in
the vendor container), plus the colour masks the obstacle challenge needs.

Which /dev/video is the RGB stream is a DAY-1 fact: the Standard / Advanced A1 has an Angstrong Nuwa-HP60C depth
camera (the vendor code reads its RGB through Angstrong's own ROS driver, `/ascamera/camera_publisher/rgb0/image`),
the Starter A1 a UVC GC0308.  This opens a plain V4L2 device; if the HP60C does not expose RGB as UVC, a Pi Camera
Module 3 (CSI) or any UVC camera replaces it and nothing above this file changes.
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from collections import deque

import numpy as np

DEPTH_HDR = struct.Struct("<4sIHHHHqqqI")        # bluewave/depth_bridge.py HDR: the two must stay the same
DEPTH_MAGIC = b"BWD1"


class Camera:
    def __init__(self, device=0, width: int = 640, height: int = 480, fps: int = 30):
        import cv2
        self.cv2 = cv2
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2) if isinstance(device, int) else cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)          # the newest frame, not a queue of old ones
        self._frame, self._t = None, 0.0
        self._times = deque(maxlen=60)
        self.frames = 0
        self._run = True
        threading.Thread(target=self._loop, name="camera", daemon=True).start()

    def _loop(self):
        while self._run:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.02)
                continue
            now = time.monotonic()
            self._frame, self._t = f, now
            self._times.append(now)
            self.frames += 1

    def read(self):
        return self._frame, self._t

    def fps(self) -> float:
        """Frames per second over the stored stamps -- measured up to NOW: once frames stop it decays and reads 0
        after 1 s without one (it used to keep the rate of the last 60 frames forever, so tel.cam_fps could not show
        a frozen grabber: review 2026-09-25)."""
        ts = list(self._times)
        if len(ts) < 2 or ts[-1] <= ts[0]:
            return 0.0
        per = (ts[-1] - ts[0]) / (len(ts) - 1)
        now = time.monotonic()
        stale = now - ts[-1]
        if stale > 2.0 * per:
            return 0.0 if stale > 1.0 else (len(ts) - 1) / (now - ts[0])
        return 1.0 / per

    def age_s(self) -> float | None:
        """Seconds since the newest frame arrived (None before the first)."""
        return (time.monotonic() - self._t) if self._t else None

    def close(self):
        self._run = False
        self.cap.release()


class DepthCamera:
    """The HP60C depth frames from bluewave/depth_bridge.py (inside the vendor container, TCP 127.0.0.1:8091): raw
    uint16 mm, the newest kept by a background thread like Camera.  Opened only when params depth.on = 1.

    read() -> (frame H x W uint16 mm, 0 = no return, or None; t) with t the host monotonic time of the EXPOSURE -- not
    the arrival as Camera.read() gives: the arrival minus the age the bridge measured against the message stamp, minus
    stamp_lag_s (what the stamp does not cover); without a plausible stamp, the arrival minus latency_s.  It reconnects
    by itself; until a frame arrives read() gives (None, 0.0)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8091, latency_s: float = 0.12, stamp_lag_s: float = 0.067,
                 timeout_s: float = 2.0):
        self.host, self.port, self.timeout = host, int(port), float(timeout_s)
        self.latency, self.stamp_lag = float(latency_s), float(stamp_lag_s)
        self._latest = (None, 0.0)
        self._times = deque(maxlen=30)
        self.frames = self.stamped = self.reconnects = 0
        self.age_ms = None                        # the bridge's stamp -> arrival age of the last frame
        self.connected, self.error = False, ""
        self._sock = None
        self._run = True
        threading.Thread(target=self._loop, name="depth", daemon=True).start()

    def _recv(self, s, n: int) -> bytearray:
        buf = bytearray(n)
        view, got = memoryview(buf), 0
        while got < n:
            k = s.recv_into(view[got:], n - got)
            if not k:
                raise ConnectionError("the depth bridge closed the stream")
            got += k
        return buf

    def _loop(self):
        while self._run:
            try:
                s = socket.create_connection((self.host, self.port), timeout=self.timeout)
                s.settimeout(self.timeout)
                self._sock, self.connected, self.error = s, True, ""
                while self._run:
                    magic, _seq, w, h, enc, _fl, stamp, rreal, _rmono, nb = DEPTH_HDR.unpack(self._recv(s, DEPTH_HDR.size))
                    if magic != DEPTH_MAGIC or enc != 1 or nb != w * h * 2:
                        raise ValueError("not a depth frame: %r enc %d, %d bytes for %d x %d" % (magic, enc, nb, w, h))
                    buf = self._recv(s, nb)
                    now = time.monotonic()
                    age = (rreal - stamp) / 1e9
                    if stamp > 0 and 0.0 <= age < 1.0:
                        t_cap = now - age - self.stamp_lag
                        self.stamped += 1
                        self.age_ms = round(age * 1000.0, 1)
                    else:
                        t_cap = now - self.latency
                    self._latest = (np.frombuffer(buf, "<u2").reshape(h, w), t_cap)     # a new buffer every frame
                    self._times.append(now)
                    self.frames += 1
            except (OSError, ValueError, ConnectionError) as e:
                self.connected, self.error = False, "%s: %s" % (type(e).__name__, e)
                self.reconnects += 1
                try:
                    self._sock.close()
                except Exception:
                    pass
                end = time.monotonic() + 1.0
                while self._run and time.monotonic() < end:
                    time.sleep(0.05)

    def read(self):
        return self._latest

    def fps(self) -> float:
        ts = self._times
        return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0.0

    def close(self):
        self._run = False
        try:
            self._sock.close()
        except Exception:
            pass


def masks(bgr: np.ndarray, hsv_ranges: dict) -> dict:
    """{colour: binary mask}.  'red' is the union of red1 and red2 (red wraps around H = 0)."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out = {}
    for name, r in hsv_ranges.items():
        out[name] = cv2.inRange(hsv, np.array(r[:3], np.uint8), np.array(r[3:], np.uint8))
    if "red1" in out and "red2" in out:
        out["red"] = cv2.bitwise_or(out.pop("red1"), out.pop("red2"))
    return out


def blobs(mask: np.ndarray, min_area: int = 80):
    """Connected blobs, largest first: dicts with cx, cy, x, y, w, h, area (pixels)."""
    import cv2
    n, _lab, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if a >= min_area:
            out.append(dict(cx=float(cents[i][0]), cy=float(cents[i][1]), x=int(x), y=int(y), w=int(w), h=int(h),
                            area=int(a)))
    return sorted(out, key=lambda b: -b["area"])


def overlay(bgr: np.ndarray, hsv_ranges: dict, colours=("red", "green", "magenta")) -> np.ndarray:
    """The frame with each colour's blobs boxed -- what the UI's camera tab shows."""
    import cv2
    img = bgr.copy()
    paint = {"red": (40, 40, 230), "green": (60, 200, 60), "magenta": (200, 60, 200), "orange": (0, 140, 255),
             "blue": (220, 120, 30)}
    ms = masks(bgr, hsv_ranges)
    for c in colours:
        for b in blobs(ms.get(c, np.zeros(bgr.shape[:2], np.uint8)))[:4]:
            cv2.rectangle(img, (b["x"], b["y"]), (b["x"] + b["w"], b["y"] + b["h"]), paint.get(c, (255, 255, 255)), 2)
            cv2.putText(img, c, (b["x"], max(12, b["y"] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, paint.get(c), 1)
    return img


def jpeg(bgr: np.ndarray, quality: int = 70) -> bytes:
    import cv2
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else b""
