"""Stall watch: which code holds the agent's GIL when every thread freezes.

The mat 2026-09-30 showed a 0.2-0.5 s freeze of the WHOLE agent about every 5.4 s, idle or driving (tel, the IMU
reader and the program all stopped together), so something holds the GIL.  A Python sampler cannot see it -- it needs
the GIL itself -- but faulthandler's watchdog is a C thread: armed for `thresh_s` and re-armed every 50 ms by a Python
thread, it fires only when that thread could not run for `thresh_s`, and then prints EVERY thread's stack, the GIL
holder's included, to runs/stalls.txt.  Stops by itself once the file passes `max_bytes`.
"""
from __future__ import annotations

import faulthandler
import os
import threading
import time


def start(path: str, thresh_s: float = 0.25, max_bytes: int = 2_000_000) -> bool:
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        f = open(path, "a", buffering=1)
    except OSError:
        return False
    f.write("\n==== stallwatch start %s (threshold %.2f s)\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), thresh_s))

    def loop():
        while True:
            try:
                if os.path.getsize(path) > max_bytes:
                    f.write("==== stallwatch stopped: %s is over %d bytes\n" % (path, max_bytes))
                    return
            except OSError:
                return
            faulthandler.dump_traceback_later(thresh_s, repeat=False, file=f, exit=False)
            time.sleep(0.05)
            faulthandler.cancel_dump_traceback_later()

    threading.Thread(target=loop, name="stallwatch", daemon=True).start()
    return True
