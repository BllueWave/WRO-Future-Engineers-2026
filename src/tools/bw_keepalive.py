"""bw_keepalive -- the link keepalive of bw run / bw test / bw mat: no console needed on the mat.

The mechanism it answers ([MAT 2026-09-29/30]): the hub stops a DRIVES program, or a moving test, when no /api request
and no console message arrived for safety.link_timeout_ms (3000).  `bw test duty_sweep` is ONE blocking POST, so the
test was stopped 3 s in unless a console page was open; `bw run` / `bw mat` poll the log every 0.3 s, but a poll that
hangs on a Wi-Fi hiccup (its timeout is 30 s) sends nothing for as long as it hangs.  So the three open a KeepAlive
while the program or test runs: its own thread, its own short timeout, a GET /api/status at least every `period`
(0.5 s) whatever the main loop is doing.

What stays safe: the keepalive lives in the bw process on the laptop.  bw killed, Ctrl-C, the laptop off the network:
the requests stop, and the robot's watchdog stops the car 3 s later exactly as before.  It never sends X-BW-Passive (a
passive request does not renew the link, by design) and never drives -- one read-only GET.

    with KeepAlive(lambda: req("GET", "/api/status", timeout=0.8)):
        ... the blocking call / the follow loop ...
"""
from __future__ import annotations

import threading
import time

PERIOD_S = 0.5          # < 1 s between requests: 6 chances inside the hub's 3 s link_timeout_ms
TIMEOUT_S = 0.8         # one request's own timeout: a hung request never delays the next by more than this


class KeepAlive:
    """Calls `ping()` every `period` s in a daemon thread from start() to stop().  Any exception of ping() -- bw.req's
    SystemExit on an unreachable robot, bw_mat's HttpError -- is counted and swallowed: the next ping comes anyway.
    Stats: sent, ok, fails, max_gap_s (the longest time between two ping starts)."""

    def __init__(self, ping, period: float = PERIOD_S, clock=time.monotonic):
        self.ping, self.period, self.clock = ping, float(period), clock
        self.sent = self.ok = self.fails = 0
        self.max_gap_s = 0.0
        self.last_error = ""
        self._stop = threading.Event()
        self._th = None

    def _loop(self):
        last = None
        while not self._stop.is_set():
            t0 = self.clock()
            if last is not None:
                self.max_gap_s = max(self.max_gap_s, t0 - last)
            last = t0
            self.sent += 1
            try:
                self.ping()
                self.ok += 1
            except BaseException as e:            # SystemExit from bw.req, HttpError, URLError, a timeout ...
                if isinstance(e, KeyboardInterrupt):
                    raise
                self.fails += 1
                self.last_error = "%s: %s" % (type(e).__name__, e)
            self._stop.wait(max(0.0, self.period - (self.clock() - t0)))

    def start(self) -> "KeepAlive":
        if self._th is None or not self._th.is_alive():
            self._stop.clear()
            self._th = threading.Thread(target=self._loop, name="bw-keepalive", daemon=True)
            self._th.start()
        return self

    def stop(self, join_s: float = 1.0) -> None:
        self._stop.set()
        if self._th is not None:
            self._th.join(join_s)

    def alive(self) -> bool:
        return self._th is not None and self._th.is_alive()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False


def interruptible(fn, poll_s: float = 0.1):
    """fn() in a worker thread while this (main) thread waits in time.sleep: Ctrl-C lands at once.  On Windows a Ctrl-C
    does not wake a socket read -- it waits for the request's own timeout (bw test's POST: the whole test) -- and with
    the keepalive running the car would drive on meanwhile; before the keepalive the silence stopped it after 3 s.
    fn's exception (bw.req's SystemExit included) is raised here; on Ctrl-C the worker is abandoned (a daemon)."""
    box = {}

    def run():
        try:
            box["r"] = fn()
        except BaseException as e:                # re-raised in the caller's thread
            box["e"] = e
    th = threading.Thread(target=run, name="bw-request", daemon=True)
    th.start()
    while th.is_alive():
        time.sleep(poll_s)
    if "e" in box:
        raise box["e"]
    return box.get("r")


def status_ping(req, timeout: float = TIMEOUT_S):
    """The ping of bw / bw mat: one GET /api/status through the caller's own req (its address, token, client name)."""
    return lambda: req("GET", "/api/status", timeout=timeout)
