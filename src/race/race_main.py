"""Race entry point -- what bluewave-race.service runs.  By the time it starts, bluewave-mode.sh has blocked every
radio (WRO FE 11.10); nothing here opens a socket.

    1. open the hardware, compute what the program would compute before the car moves (wro_next's exit plans:
       12-17 s on a cold cache on the Pi, which used to sit between KEY1 and the first metre), two beeps = READY
    2. wait for KEY1 -- the ONE start button (WRO FE 9.11); the car does not move before it
       (KEY2 held long = back to DEV mode: radios on, the agent started -- a way back in without a keyboard)
    3. run the challenge's program (params["race"]["challenge"] -> ["programs"], or ["program"] to force one) with
       params["race"]["params"]; everything it logs, plus 20 Hz telemetry, goes to
       runs/<time>-race-<program>.jsonl, read back later in dev mode with `bw runs` / `bw summary`
    4. stop, one long beep
"""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import threading
import time

from bluewave import params as P
from bluewave import rrc
from bluewave.hw import Robot

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def to_dev_mode(robot: Robot):
    robot.beep(1200, 0.4)
    subprocess.call(["sudo", "tee", "/boot/firmware/bluewave.mode"], input=b"dev\n", stdout=subprocess.DEVNULL)
    subprocess.call(["sudo", "rfkill", "unblock", "wifi"])
    subprocess.call(["sudo", "systemctl", "start", "bluewave-agent"])


def main() -> int:
    p = P.load()
    race = p.get("race", {})
    name = race.get("program") or race.get("programs", {}).get(race.get("challenge", "obstacle"), "wro_next")
    path = os.path.join(ROOT, "programs", name + ".py")
    spec = importlib.util.spec_from_file_location("race_prog", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)                         # a broken program fails HERE, before READY -- no beeps

    robot = Robot(p, mock=False)
    warm = getattr(mod, "warm_exit_plans", None)
    if warm is not None:
        try:
            warm(p, race.get("params", {}))
        except Exception as e:                           # no plan: the program itself says so and laps without a park
            print("race: exit plans not warmed:", type(e).__name__, e, flush=True)
    time.sleep(0.5)
    robot.beep(1800)
    time.sleep(0.15)
    robot.beep(2400)                                     # READY
    while True:
        ev = robot.button()
        if ev:
            kid, e = ev
            if kid == 2 and e in (rrc.KEY_LONGPRESS, rrc.KEY_LONGPRESS_REPEAT):
                robot.close()
                to_dev_mode(robot)
                return 0
            if kid == 1 and e in (rrc.KEY_CLICK, rrc.KEY_RELEASE_FROM_SP, rrc.KEY_PRESSED):
                break
        time.sleep(0.01)

    os.makedirs(os.path.join(ROOT, "runs"), exist_ok=True)
    rid = time.strftime("%Y%m%d-%H%M%S-race-") + name
    f = open(os.path.join(ROOT, "runs", rid + ".jsonl"), "w", encoding="utf-8")
    lock = threading.Lock()

    def write(rec):
        with lock:
            f.write(json.dumps(rec, default=str) + "\n")

    write(dict(t=time.time(), meta=dict(program=name, params=race.get("params", {}), robot_params=p, race=True)))
    robot.run_id = rid                                   # wro_next's Recorder header names the run (BRAIN4_SPEC W19)

    def clock():
        # BRAIN4_SPEC 2.3: this file's Unix time and the monotonic time of the recording, read together (G6)
        t, mono = time.time(), time.monotonic()
        write(dict(t=t, clock=dict(mono=round(mono, 4), run_id=rid, agent_started=None)))

    clock()
    stop = threading.Event()

    def telemetry():
        # tel 20 Hz; every 0.2 s the board's IMU samples since the last `imu` record (bluewave/blackbox.py: the
        # analyzer sees a contact the 20 Hz tel misses); every 10 s a `clock` record.  Nothing here may stop the race
        try:
            from bluewave import blackbox
        except Exception:
            blackbox = None
        k, last = 0, time.monotonic()
        while not stop.is_set():
            write(dict(t=time.time(), tel=robot.snapshot()))
            k += 1
            if k % 4 == 0 and blackbox is not None:
                try:
                    rec, last = blackbox.take(robot, last)
                    if rec:
                        write(dict(t=time.time(), imu=rec))
                except Exception:
                    pass
            if k % 200 == 0:
                clock()
            time.sleep(0.05)

    threading.Thread(target=telemetry, daemon=True).start()
    robot.reset_yaw()
    try:
        mod.run(robot, race.get("params", {}), lambda x: write(dict(t=time.time(), log=x)), stop)
    except Exception as e:                               # logged, and the car still stops
        write(dict(t=time.time(), log={"error": repr(e)}))
    finally:
        stop.set()
        robot.stop()
        robot.beep(1500, 0.5)
        time.sleep(0.3)
        with lock:
            f.close()
        robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
