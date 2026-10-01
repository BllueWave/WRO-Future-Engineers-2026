"""The robot's own health for the console's SYSTEM page and `bw sys`, plus the confirmed system actions.

No psutil (no new pip packages on the robot): /proc and /sys are read directly, vcgencmd / nmcli / ip / systemctl /
docker / timedatectl run as subprocesses with a timeout and stdin closed.  A reader that fails gives null, never an
exception; on the Windows laptop (mock mode) almost everything is null.

Actions change the robot, so every one needs body.confirm == its name (the console asks first), and all are refused
while a program runs (sync_clock too: a clock step mid-run breaks that run's time stamps).  They use `sudo -n` (never a
password prompt).  The stock Hiwonder card's user has passwordless sudo; the BlueWave OS card needs
os/console_sudoers.sh once for reboot, shutdown, sync_clock and the camera restart.

Cost on the robot: the fast sample (1 Hz) reads files only.  The slow one forks 7 subprocesses, so it runs on its own
thread (a hung docker / nmcli never freezes the 1 Hz readings), only while a console subscribes `sys` or `bw sys`
asked in the last 10 s, and never while a DRIVES program runs.  nmcli is asked with `--rescan no`: its default rescans
whenever the AP list is older than 30 s, and a scan takes the Pi's one radio off the hotspot's channel.
"""
from __future__ import annotations

import json
import os
import platform
import shutil
import subprocess
import sys
import threading
import time
import urllib.request

LINUX = sys.platform.startswith("linux")
SERVICES = ("bluewave-agent", "bluewave-camera", "start_node", "bluewave-mode", "bluewave-perf")
CONTAINER = "MentorPi"
STREAM_URL = "http://127.0.0.1:8080/"
NOT_PERMITTED = "not permitted: run os/console_sudoers.sh once"
NMCLI_WIFI = ["nmcli", "-t", "-f", "ACTIVE,SSID,CHAN,FREQ", "dev", "wifi", "list", "--rescan", "no"]   # passive
THROTTLE_BITS = {"undervolt_now": 0, "freq_capped_now": 1, "throttled_now": 2, "soft_temp_now": 3,
                 "undervolt_seen": 16, "freq_capped_seen": 17, "throttled_seen": 18, "soft_temp_seen": 19}


def _read(path: str):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return None


def run(cmd, timeout: float = 2.0, inp: str | None = None):
    """(rc, stdout+stderr stripped); rc None when the command could not run at all."""
    try:
        p = subprocess.run(cmd, input=inp, capture_output=True, text=True, timeout=timeout,
                           stdin=None if inp is not None else subprocess.DEVNULL)
        return p.returncode, ((p.stdout or "") + (p.stderr or "")).strip()
    except FileNotFoundError:
        return None, "not installed: %s" % cmd[0]
    except subprocess.TimeoutExpired:
        return None, "timed out after %.0f s" % timeout
    except Exception as e:                                 # never raise into the sampler or a route
        return None, str(e)


def _cpu_times():
    s = _read("/proc/stat")
    if not s:
        return None
    out = {}
    for line in s.splitlines():
        if line.startswith("cpu"):
            f = line.split()
            v = [int(x) for x in f[1:]]
            idle = v[3] + (v[4] if len(v) > 4 else 0)
            out[f[0]] = (sum(v[:8]), idle)
    return out


def _net_dev():
    s = _read("/proc/net/dev")
    if not s:
        return None
    out = {}
    for line in s.splitlines()[2:]:
        if ":" in line:
            name, rest = line.split(":", 1)
            f = rest.split()
            out[name.strip()] = (int(f[0]), int(f[8]))
    return out


def _unescape_nmcli(line: str):
    """nmcli -t fields: ':' separates, '\\:' is a colon inside a value."""
    out, cur, esc = [], "", False
    for ch in line:
        if esc:
            cur += ch
            esc = False
        elif ch == "\\":
            esc = True
        elif ch == ":":
            out.append(cur)
            cur = ""
        else:
            cur += ch
    out.append(cur)
    return out


def throttle_flags(value: int) -> dict:
    return {k: bool(value >> b & 1) for k, b in THROTTLE_BITS.items()}


class Sampler:
    """fast (1 Hz, file reads only): /proc/stat per core, /proc/loadavg, cpufreq, thermal_zone0, /proc/meminfo,
    /proc/uptime, /proc/net/wireless, /proc/net/dev, the agent's own CPU time and RSS; slow (every 5 s while wanted,
    its own thread, subprocesses with timeout=2, stdin=DEVNULL): vcgencmd get_throttled / pmic_read_adc EXT5V_V,
    nmcli, ip -4 -j addr, systemctl show, docker inspect, timedatectl, the web_video_server probe; the size of the run
    files every 30 s while no program runs.  A reader that fails gives null, never an exception; on Windows (mock)
    almost everything is null.  `program` is set by the agent: () -> (busy, drives)."""

    WANT_S = 10.0

    def __init__(self, runs_dir: str, mock: bool, battery=None, start: bool = True):
        self.runs_dir, self.mock, self.battery = runs_dir, bool(mock), battery
        self.t_agent = time.time()
        self._lock = threading.Lock()
        self._slow_lock = threading.Lock()
        self._fast, self._slow = {}, {}
        self._t_fast = self._t_slow = 0.0
        self._prev_cpu = self._prev_net = None
        self._prev_proc = (time.monotonic(), time.process_time())
        self._runs_mb, self._rec_mb, self._t_du = None, None, -1e9
        self.wanted_t = -1e9
        self.program = lambda: (False, False)
        self.card, self.mode = self._card()
        self._run = True
        if start:
            threading.Thread(target=self._loop, name="sysinfo", daemon=True).start()
            threading.Thread(target=self._slow_loop, name="sysinfo-slow", daemon=True).start()

    def want(self):
        """A console shows `sys` (the hub calls this every tick) or `bw sys` asked: keep the slow sample fresh."""
        self.wanted_t = time.monotonic()

    def _prog(self):
        try:
            busy, drives = self.program()
            return bool(busy), bool(busy and drives)
        except Exception:
            return False, False

    def _driving(self) -> bool:
        return self._prog()[1]

    def refresh_slow(self, max_age_s: float):
        """Sample the slow part now when it is older than max_age_s (the caller runs off the event loop)."""
        if time.monotonic() - self._t_slow >= max_age_s:
            self._sample_slow_once()

    def _sample_slow_once(self):
        if not self._slow_lock.acquire(blocking=False):
            return                                         # the slow thread is sampling right now
        try:
            self.sample_slow()
        except Exception:
            pass
        finally:
            self._slow_lock.release()

    # ------------------------------------------------------------------ identity
    def _card(self):
        """The BlueWave OS card is the one with bluewave-mode.service (os/setup_bluewave_os.sh), not merely a mode file:
        `bw mode race` on the stock card writes /boot/firmware/bluewave.mode too, and nothing there reads it."""
        if self.mock:
            return "laptop", "mock"
        if os.path.exists("/etc/systemd/system/bluewave-mode.service"):
            m = _read("/boot/firmware/bluewave.mode")
            return "bluewave_os", ((m or "").strip() or "dev")
        if os.path.exists("/etc/systemd/system/bluewave-camera.service"):
            return "stock", "dev"
        return None, "dev"

    # ------------------------------------------------------------------ loop
    def _loop(self):
        while self._run:
            t = time.monotonic()
            try:
                self.sample_fast()
            except Exception:
                pass
            time.sleep(max(0.05, 1.0 - (time.monotonic() - t)))

    def _slow_loop(self):
        while self._run:
            now = time.monotonic()
            if now - self._t_slow >= 5.0 and now - self.wanted_t < self.WANT_S and not self._driving():
                self._sample_slow_once()
            time.sleep(0.5)

    def close(self):
        self._run = False

    def sample_fast(self):
        now = time.monotonic()
        f = {}
        # CPU %
        ct = _cpu_times() if LINUX else None
        cores, pct = None, None
        if ct and self._prev_cpu:
            def load(k):
                a, b = ct.get(k), self._prev_cpu.get(k)
                if not a or not b or a[0] <= b[0]:
                    return None
                dt, di = a[0] - b[0], a[1] - b[1]
                return round(100.0 * (dt - di) / dt, 1)
            pct = load("cpu")
            cores = [load(k) for k in sorted((k for k in ct if k != "cpu"), key=lambda k: int(k[3:]))]
        self._prev_cpu = ct
        mhz = []
        k = 0
        while LINUX:
            s = _read("/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq" % k)
            if s is None:
                break
            mhz.append(int(int(s) / 1000))
            k += 1
        gov = _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor") if LINUX else None
        temp = _read("/sys/class/thermal/thermal_zone0/temp") if LINUX else None
        try:
            la = [round(x, 2) for x in os.getloadavg()]
        except (AttributeError, OSError):
            la = None
        f["cpu"] = dict(pct=pct, cores=cores, load=la, mhz=mhz or None, governor=gov.strip() if gov else None,
                        temp_c=round(int(temp) / 1000.0, 1) if temp and temp.strip().lstrip("-").isdigit() else None)
        # memory
        mem = None
        s = _read("/proc/meminfo") if LINUX else None
        if s:
            kv = {}
            for line in s.splitlines():
                a, _, b = line.partition(":")
                try:
                    kv[a] = int(b.split()[0])
                except (ValueError, IndexError):
                    pass
            tot, av = kv.get("MemTotal"), kv.get("MemAvailable")
            if tot and av is not None:
                mem = dict(total_mb=round(tot / 1024), used_mb=round((tot - av) / 1024),
                           pct=round(100.0 * (tot - av) / tot, 1),
                           swap_used_mb=round((kv.get("SwapTotal", 0) - kv.get("SwapFree", 0)) / 1024))
        f["mem"] = mem
        # disk
        try:
            root = os.path.abspath(os.sep) if not LINUX else "/"
            du = shutil.disk_usage(root)
            disk = dict(total_gb=round(du.total / 2 ** 30, 1), free_gb=round(du.free / 2 ** 30, 1),
                        pct=round(100.0 * du.used / du.total, 1))
        except OSError:
            disk = dict(total_gb=None, free_gb=None, pct=None)
        if now - self._t_du >= 30.0 and not self._prog()[0]:
            self._t_du = now
            self._runs_mb = self._du(self.runs_dir)
            self._rec_mb = self._du_rec(os.path.join(self.runs_dir, "rec")) if LINUX else None
        disk["runs_mb"] = self._runs_mb
        disk["rec_mb"] = self._rec_mb
        f["disk"] = disk
        # network counters and link quality (the interface the slow sample found, wlan0 by default)
        iface = (self._slow.get("net") or {}).get("iface") or "wlan0"
        rssi = quality = None
        s = _read("/proc/net/wireless") if LINUX else None
        if s:
            for line in s.splitlines()[2:]:
                if line.strip().startswith(iface + ":"):
                    p = line.split()
                    try:
                        quality, rssi = int(float(p[2].rstrip("."))), int(float(p[3].rstrip(".")))
                    except (ValueError, IndexError):
                        pass
        nd = _net_dev() if LINUX else None
        rx = tx = None
        if nd and self._prev_net and iface in nd and iface in self._prev_net[1]:
            dt = now - self._prev_net[0]
            if dt > 0:
                rx = round((nd[iface][0] - self._prev_net[1][iface][0]) / 1024.0 / dt, 1)
                tx = round((nd[iface][1] - self._prev_net[1][iface][1]) / 1024.0 / dt, 1)
        self._prev_net = (now, nd) if nd else None
        f["netq"] = dict(rssi_dbm=rssi, quality=quality, rx_kBps=rx, tx_kBps=tx)
        # this process
        pt = time.process_time()
        t0, p0 = self._prev_proc
        cpu_pct = round(100.0 * (pt - p0) / (now - t0), 1) if now > t0 else None
        self._prev_proc = (now, pt)
        rss = None
        s = _read("/proc/self/status") if LINUX else None
        if s:
            for line in s.splitlines():
                if line.startswith("VmRSS:"):
                    rss = round(int(line.split()[1]) / 1024.0, 1)
        up = _read("/proc/uptime") if LINUX else None
        f["proc"] = dict(agent_rss_mb=rss, agent_cpu_pct=cpu_pct,
                         uptime_s=int(float(up.split()[0])) if up else None)
        with self._lock:
            self._fast, self._t_fast = f, now

    @staticmethod
    def _du(path: str):
        """MB of the run files (the top-level .jsonl only).  os.walk over runs/ with rec/ inside took 106-112 ms per
        call on the laptop (4,866 files) and grows all day: wro_next writes ~900 frames a minute to runs/rec."""
        tot = 0
        try:
            with os.scandir(path) as it:
                for e in it:
                    if e.name.endswith(".jsonl") and e.is_file():
                        try:
                            tot += e.stat().st_size
                        except OSError:
                            pass
        except OSError:
            return None
        return round(tot / 2 ** 20, 1)

    @staticmethod
    def _du_rec(path: str):
        """MB of runs/rec (the camera frame folders) by `du -sk` in a subprocess: no walk in this process."""
        if not os.path.isdir(path):
            return 0.0
        rc, out = run(["du", "-sk", path], timeout=5)
        try:
            return round(int(out.split()[0]) / 1024.0, 1) if rc == 0 else None
        except (ValueError, IndexError):
            return None

    def sample_slow(self):
        now = time.monotonic()
        s = {}
        on_robot = LINUX and not self.mock
        # power
        thr, flags, ext5v = None, None, None
        if on_robot:
            rc, out = run(["vcgencmd", "get_throttled"])
            if rc == 0 and "=" in out:
                thr = out.split("=", 1)[1].strip()
                try:
                    flags = throttle_flags(int(thr, 16))
                except ValueError:
                    flags = None
            rc, out = run(["vcgencmd", "pmic_read_adc", "EXT5V_V"])
            if rc == 0 and "=" in out:
                try:
                    ext5v = round(float(out.split("=", 1)[1].strip().rstrip("V")), 2)
                except ValueError:
                    pass
        s["power"] = dict(throttled=thr, flags=flags, ext5v_v=ext5v)
        # network identity
        net = dict(iface=None, ssid=None, ip=None, freq_mhz=None)
        if on_robot:
            rc, out = run(NMCLI_WIFI)
            if rc == 0:
                for line in out.splitlines():
                    if line.startswith("yes:"):
                        f = _unescape_nmcli(line)
                        net["ssid"] = f[1] if len(f) > 1 else None
                        try:
                            net["freq_mhz"] = int(f[3].split()[0]) if len(f) > 3 else None
                        except (ValueError, IndexError):
                            pass
                        break
            rc, out = run(["ip", "-4", "-j", "addr", "show"])
            if rc == 0:
                try:
                    ifs = [i for i in json.loads(out) if i.get("ifname") != "lo" and i.get("addr_info")]
                    ifs.sort(key=lambda i: i.get("ifname") != "wlan0")
                    if ifs:
                        net["iface"] = ifs[0]["ifname"]
                        net["ip"] = ifs[0]["addr_info"][0].get("local")
                except (ValueError, KeyError, IndexError, TypeError):
                    pass
        s["net"] = net
        # services
        services = {}
        if on_robot:
            rc, out = run(["systemctl", "show", "-p", "Id,LoadState,ActiveState,SubState"] + list(SERVICES))
            if rc == 0:
                for block in out.split("\n\n"):
                    kv = dict(l.split("=", 1) for l in block.splitlines() if "=" in l)
                    if kv.get("LoadState") == "loaded":
                        services[kv.get("Id", "?").replace(".service", "")] = kv.get("ActiveState")
        s["services"] = services or None
        # the camera container and its MJPEG stream
        cam = dict(container=None, state=None, started=None, restarts=None, stream_ok=None)
        if on_robot:
            rc, out = run(["docker", "inspect", "-f", "{{.State.Status}}|{{.State.StartedAt}}|{{.RestartCount}}",
                           CONTAINER])
            if rc == 0 and out.count("|") == 2:
                st, started, rs = out.split("|")
                cam.update(container=CONTAINER, state=st, started=started[:19] + "Z" if started else None,
                           restarts=int(rs) if rs.isdigit() else None)
            try:
                with urllib.request.urlopen(STREAM_URL, timeout=0.5) as r:
                    cam["stream_ok"] = r.status < 500
            except Exception:
                cam["stream_ok"] = False
        s["camera"] = cam
        # clock
        ntp, tz = None, None
        if on_robot:
            rc, out = run(["timedatectl", "show", "-p", "NTPSynchronized", "-p", "Timezone"])
            if rc == 0:
                kv = dict(l.split("=", 1) for l in out.splitlines() if "=" in l)
                ntp = {"yes": True, "no": False}.get(kv.get("NTPSynchronized"))
                tz = kv.get("Timezone")
        s["clock"] = dict(ntp_synced=ntp, tz=tz)
        with self._lock:
            self._slow, self._t_slow = s, now

    # ------------------------------------------------------------------ the payload
    def host(self) -> dict:
        """The static identity (read once) + card and mode."""
        if getattr(self, "_host", None) is None:
            self._host = self._host_static()
        return dict(self._host, card=self.card, mode=self.mode, mock=self.mock)

    @staticmethod
    def _host_static() -> dict:
        model = _read("/proc/device-tree/model") if LINUX else None
        osr = _read("/etc/os-release") if LINUX else None
        pretty = None
        if osr:
            for line in osr.splitlines():
                if line.startswith("PRETTY_NAME="):
                    pretty = line.split("=", 1)[1].strip().strip('"')
        elif not LINUX:
            pretty = "%s %s" % (platform.system(), platform.release())
        return dict(hostname=platform.node(), model=model.strip("\x00\n ") if model else None, os=pretty,
                    kernel=os.uname().release if hasattr(os, "uname") else None, python=platform.python_version(),
                    agent_pid=os.getpid())

    def snapshot(self) -> dict:
        """The `sys` payload of CONSOLE_SPEC 5.4 (without type / t).  Never blocks on a subprocess."""
        now = time.monotonic()
        with self._lock:
            f, s, tf, ts = dict(self._fast), dict(self._slow), self._t_fast, self._t_slow
        proc = f.get("proc") or {}
        host = self.host()
        host.update(uptime_s=proc.get("uptime_s"), agent_uptime_s=int(time.time() - self.t_agent),
                    agent_rss_mb=proc.get("agent_rss_mb"), agent_cpu_pct=proc.get("agent_cpu_pct"))
        bat = None
        try:
            bat = self.battery() if self.battery else None
            bat = round(bat, 2) if bat is not None else None
        except Exception:
            pass
        power = dict(s.get("power") or dict(throttled=None, flags=None, ext5v_v=None))
        power["battery_v"] = bat
        net = dict(s.get("net") or dict(iface=None, ssid=None, ip=None, freq_mhz=None))
        netq = f.get("netq") or {}
        net.update(rssi_dbm=netq.get("rssi_dbm"), quality=netq.get("quality"), rx_kBps=netq.get("rx_kBps"),
                   tx_kBps=netq.get("tx_kBps"))
        clock = dict(unix=round(time.time(), 3), **(s.get("clock") or dict(ntp_synced=None, tz=None)))
        return dict(host=host,
                    cpu=f.get("cpu") or dict(pct=None, cores=None, load=None, mhz=None, governor=None, temp_c=None),
                    power=power, mem=f.get("mem"), disk=f.get("disk"), net=net, services=s.get("services"),
                    camera=s.get("camera") or dict(container=None, state=None, started=None, restarts=None,
                                                   stream_ok=None),
                    clock=clock,
                    age_ms=dict(fast=int((now - tf) * 1000) if tf else None,
                                slow=int((now - ts) * 1000) if ts else None))


# ---------------------------------------------------------------------------------------------------------- actions
def _sudo_ok(cmd) -> bool:
    """`sudo -n -l CMD` exits 0 only when CMD is allowed without a password."""
    rc, _ = run(["sudo", "-n", "-l"] + list(cmd), timeout=5)
    return rc == 0


def _later(cmd):
    threading.Timer(0.5, lambda: run(cmd, timeout=30)).start()


def _step(cmd, timeout, inp=None):
    rc, out = run(cmd, timeout=timeout, inp=inp)
    return dict(cmd=" ".join(cmd), rc=rc, out=out[-400:])


def _restart_camera(body):
    steps = []
    rc, out = run(["docker", "inspect", "-f", "{{.State.Status}}", CONTAINER], timeout=5)
    if rc == 0 and out.strip() != "running":
        steps.append(_step(["docker", "start", CONTAINER], 20))
    steps.append(_step(["sudo", "-n", "systemctl", "restart", "bluewave-camera"], 30))
    last = steps[-1]
    if last["rc"] == 0:
        return True, "camera service restarted: frames return in about 5-10 s", steps
    if "password" in last["out"].lower():
        return False, NOT_PERMITTED, steps
    return False, "camera restart failed (rc %s): %s" % (last["rc"], last["out"][-160:]), steps


def _deferred(cmd, what):
    def go(body):
        if not _sudo_ok(cmd):
            return False, NOT_PERMITTED, [dict(cmd="sudo -n -l " + " ".join(cmd), rc=1, out="not allowed")]
        _later(["sudo", "-n"] + list(cmd))
        return True, what, [dict(cmd="sudo -n " + " ".join(cmd), rc=None, out="scheduled in 0.5 s")]
    return go


def _sync_clock(body):
    try:
        epoch = int(float(body.get("epoch_s")))
    except (TypeError, ValueError):
        return False, "epoch_s missing: the console sends this device's clock", []
    if epoch < 1700000000:
        return False, "epoch_s %d is not a plausible time" % epoch, []
    before = time.time()
    st = _step(["sudo", "-n", "/usr/bin/date", "-u", "-s", "@%d" % epoch], 10)
    if st["rc"] == 0:
        return True, "clock set (was %+.1f s off)" % (before - epoch), [st]
    if "password" in st["out"].lower():
        return False, NOT_PERMITTED, [st]
    return False, "date failed (rc %s): %s" % (st["rc"], st["out"][-160:]), [st]


def _race_mode(body):
    card = body.get("_card")
    if card != "bluewave_os":
        return False, "race mode is not installed on this card (stock image)", []
    st = _step(["sudo", "-n", "/usr/bin/tee", "/boot/firmware/bluewave.mode"], 10, inp="race\n")
    if st["rc"] == 0:
        return True, ("next boot is RACE: radios off, the console is unreachable until KEY2 is held long.  "
                      "Reboot to enter it (the separate reboot action)."), [st]
    return False, "could not write the mode file (rc %s): %s" % (st["rc"], st["out"][-160:]), [st]


ACTIONS = {
    "restart_camera": ("Restart camera", True, _restart_camera),
    "restart_agent": ("Restart agent", True, _deferred(["systemctl", "restart", "bluewave-agent"],
                                                      "agent restarting: the console reconnects in about 5 s")),
    "reboot": ("Reboot", True, _deferred(["/usr/sbin/reboot"], "rebooting: the console disconnects for about 60 s")),
    "shutdown": ("Shut down", True, _deferred(["/usr/sbin/poweroff"],
                                              "powering off: switch the robot off at the battery after 20 s")),
    "sync_clock": ("Sync clock", True, _sync_clock),
    "race_mode": ("Switch to race mode", True, _race_mode),
}


def action(name: str, body: dict, mock: bool, busy: bool, card: str | None = None) -> dict:
    """{ok, action, detail, steps}.  The route has already checked confirm == name and busy (409)."""
    if name not in ACTIONS:
        return dict(ok=False, action=name, detail="unknown action", steps=[])
    if busy and ACTIONS[name][1]:
        return dict(ok=False, action=name, detail="a program is running: stop it first", steps=[])
    if mock:
        return dict(ok=False, action=name, detail="simulator: not executed", steps=[])
    try:
        ok, detail, steps = ACTIONS[name][2](dict(body or {}, _card=card))
    except Exception as e:
        ok, detail, steps = False, "failed: %s" % e, []
    return dict(ok=bool(ok), action=name, detail=detail, steps=steps)
