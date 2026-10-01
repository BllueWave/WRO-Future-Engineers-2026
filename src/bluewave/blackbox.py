"""The IMU black box: every board IMU sample into the run file, so the analyzer sees a contact the 20 Hz tel misses.

The RRC reports (ax, ay, az, gx, gy, gz) at ~50 Hz (acceleration in g, rate in deg/s, exactly as `rrc.Board` unpacks
them); `tel.acc` samples that at 20 Hz, so a one- or two-sample contact spike falls between tel lines (BRAIN4_SPEC G5).
`rrc.BoardState.imu_hist` (and the simulator's board) keeps the last samples with their arrival time; the Runner
(agent.py) and race_main write them every 0.2 s as one `imu` record (BRAIN4_SPEC 2.3):

    {"t": <unix>, "imu": {"t0": <monotonic of sample 0>, "n": 10,
                          "dt": b64 <u2 (t_i - t0) x 1e4,       0.1 ms steps, <= 6.5535 s per record
                          "a":  b64 <i2 n x 3 milli-g,           +-32 g
                          "g":  b64 <i2 n x 3 0.1 deg/s}}        +-3276 deg/s

~110 bytes per record at 50 Hz (10 samples): 0.5 kB/s, next to tel's ~6 kB/s.  Round trip: 1 milli-g, 0.1 deg/s,
0.1 ms (tests_console/test_analyze.py).

    rec, last = take(robot, last)          # the Runner, every 0.2 s: the samples after `last` (None = none new)
    arr = decode(rec["imu"])               # (n, 7): t mono, ax, ay, az (g), gx, gy, gz (deg/s)
    dev = deviation(t, acc)                # the analyzer's D1 signal: |a - median(a over t +- 1 s)|, 3-D, g
    arr = recent(robot, 0.2)               # wro_next bump_lost (W22): the last 0.2 s

Nothing here opens a device or imports hw: the analyzer runs it on the laptop.
"""
from __future__ import annotations

import base64

import numpy as np

DT_SCALE = 1e4                          # (t_i - t0) in 0.1 ms steps
MAX_SPAN_S = 65535 / DT_SCALE           # one record spans at most this (the <u2 limit)
A_SCALE = 1000.0                        # milli-g
G_SCALE = 10.0                          # 0.1 deg/s


def _b64(a: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode("ascii")


def encode(samples) -> dict | None:
    """(t_mono, ax, ay, az, gx, gy, gz) samples, time order -> the `imu` record body; None for no samples.  Samples
    more than MAX_SPAN_S after the first are left out (take() never passes such a batch)."""
    arr = np.asarray(samples, dtype=np.float64).reshape(-1, 7)
    if not len(arr):
        return None
    t0 = float(arr[0, 0])
    arr = arr[(arr[:, 0] - t0) <= MAX_SPAN_S]
    dt = np.clip(np.rint((arr[:, 0] - t0) * DT_SCALE), 0, 65535).astype("<u2")
    a = np.clip(np.rint(arr[:, 1:4] * A_SCALE), -32767, 32767).astype("<i2")
    g = np.clip(np.rint(arr[:, 4:7] * G_SCALE), -32767, 32767).astype("<i2")
    return {"t0": round(t0, 6), "n": int(len(arr)), "dt": _b64(dt), "a": _b64(a), "g": _b64(g)}


def decode(body: dict) -> np.ndarray:
    """The `imu` record body -> float64 (n, 7): t (monotonic s), ax, ay, az (g), gx, gy, gz (deg/s).  A malformed
    record decodes to (0, 7)."""
    try:
        n = int(body["n"])
        dt = np.frombuffer(base64.b64decode(body["dt"]), dtype="<u2").astype(np.float64)
        a = np.frombuffer(base64.b64decode(body["a"]), dtype="<i2").astype(np.float64).reshape(-1, 3)
        g = np.frombuffer(base64.b64decode(body["g"]), dtype="<i2").astype(np.float64).reshape(-1, 3)
        if not (len(dt) == len(a) == len(g) == n):
            return np.zeros((0, 7))
        out = np.empty((n, 7))
        out[:, 0] = float(body["t0"]) + dt / DT_SCALE
        out[:, 1:4] = a / A_SCALE
        out[:, 4:7] = g / G_SCALE
        return out
    except (KeyError, TypeError, ValueError):
        return np.zeros((0, 7))


def _hist(src):
    """The board state's imu_hist deque from a Robot, a board, or a BoardState; None when it has none."""
    for get in (lambda s: s.board.state.imu_hist, lambda s: s.state.imu_hist, lambda s: s.imu_hist):
        try:
            return get(src)
        except AttributeError:
            continue
    return None


def take(src, since_t: float = 0.0):
    """(record body | None, last sample time): the samples that arrived after `since_t` (monotonic), at most
    MAX_SPAN_S of them -- the rest come with the next call.  Thread-safe enough for a deque appended by one thread:
    list() of a deque is atomic under the GIL."""
    h = _hist(src)
    if not h:
        return None, since_t
    new = [s for s in list(h) if s[0] > since_t]
    if not new:
        return None, since_t
    t0 = new[0][0]
    new = [s for s in new if s[0] - t0 <= MAX_SPAN_S]
    return encode(new), float(new[-1][0])


def recent(src, seconds: float = 0.2) -> np.ndarray:
    """The last `seconds` of samples as float64 (n, 7) (the same columns as decode)."""
    h = _hist(src)
    if not h:
        return np.zeros((0, 7))
    arr = np.asarray(list(h), dtype=np.float64).reshape(-1, 7)
    return arr[arr[:, 0] >= arr[-1, 0] - float(seconds)]


def deviation(t, acc, win_s: float = 1.0) -> np.ndarray:
    """|a(t) - median(a over t +- win_s)|, 3-D, in g: the mounting and the body's tilt cancel, a hit stands out
    (BRAIN4_SPEC 6.3 D1).  `t` seconds (n,), `acc` (n, 3).  The window is sized in SAMPLES from the median sample
    period (tel 20 Hz, the board 50 Hz; a gap only widens it), edges padded with the edge value."""
    t = np.asarray(t, dtype=np.float64)
    a = np.asarray(acc, dtype=np.float64).reshape(-1, 3)
    n = len(t)
    if n == 0:
        return np.zeros(0)
    if n < 3:
        return np.linalg.norm(a - np.median(a, axis=0), axis=1)
    per = float(np.median(np.diff(t)))
    k = int(max(1, round(win_s / max(per, 1e-3))))
    k = min(k, n)
    pad = np.pad(a, ((k, k), (0, 0)), mode="edge")
    win = np.lib.stride_tricks.sliding_window_view(pad, 2 * k + 1, axis=0)      # (n, 3, 2k+1)
    med = np.median(win, axis=2)
    return np.linalg.norm(a - med, axis=1)
