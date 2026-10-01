"""The console's data as ROS 2 topics, for Foxglove Studio or RViz -- read-only.  Runs INSIDE the vendor container
`MentorPi` (ROS 2 Humble, net=host), like depth_bridge.py, never in the agent and never in a race:

    python3 /tmp/bw_ros_bridge.py [--agent http://127.0.0.1:8000] [--hz 10] [--no-map]

os/ros_bridge.sh start copies this file in and starts it with rosbridge_server (Foxglove's "Rosbridge" connection to
ws://<robot>:9090); RViz runs inside the container on the stock card's VNC desktop.

Source: the agent's passive NDJSON stream GET /api/stream?kinds=scan,live,map_ros (header X-BW-Passive: 1).  Passive
means the link watchdog never counts it (S13): with every console closed a DRIVES program still stops at
safety.link_timeout_ms.  Read-only: there is NO /cmd_vel -- driving through ROS would bypass the console's E-STOP latch,
drive lock and dead-man, so nothing here can move the car.

    /scan               sensor_msgs/LaserScan      frame laser; angle_min = rad(-180 + res/2), increment rad(res),
                                                   ranges m (inf = no return), intensities = LD19 confidence
    /tf                 map -> odom (identity), odom -> base_link (the live pose: the WRO field, or SLAM's map frame)
    /tf_static          base_link -> laser (lidar.pos_mm, lidar.h_mm), base_link -> camera_link (x, y, h, pitch)
    /odom               nav_msgs/Odometry          the live pose, covariance from its sxy / sth
    /map                nav_msgs/OccupancyGrid     the served map (SLAM's or the known-pose field map), cropped,
                                                   <= 0.5 Hz (rosbridge sends JSON: a full 512^2 grid is ~1 MB of text)
    /bluewave/pillars   visualization_msgs/MarkerArray   cylinders 0.05 x 0.10 m in the signs' colours
    /plan               nav_msgs/Path              the program's planned path

The camera image is not republished: the vendor driver already publishes /ascamera/camera_publisher/rgb0/image while
bluewave-camera.service runs.  Frames: ROS REP-103 (x forward, y left, z up, metres, radians) -- the car frame of
BRAIN_SPEC 2 in metres; the camera's pitch sign: vision.py's + = looking down = +pitch about y in ROS ([DAY1] check in
RViz that the camera_link x axis points at the floor ahead).

The conversions are plain functions of the stream's JSON (no rclpy), so tests_console/test_ros_bridge.py runs them on
the laptop with stub message classes.
"""
from __future__ import annotations

import argparse
import array
import base64
import json
import math
import sys
import threading
import time
import urllib.request

import numpy as np

KINDS = "scan,live,map_ros"
RANGE_MIN_M, RANGE_MAX_M = 0.06, 12.0
PILLAR_RGBA = {"red": (0.85, 0.12, 0.12, 1.0), "green": (0.10, 0.65, 0.20, 1.0), None: (0.6, 0.6, 0.6, 0.8)}


# ------------------------------------------------------------------------------------------------ pure conversions
def scan_fields(m: dict) -> dict:
    """The `scan` message -> LaserScan fields (m, rad, s).  Bin k of the scan is at -180 + (k + 0.5) res deg, CCW +,
    0 = the car's nose -- exactly LaserScan's angle_min + k increment with angle_min = rad(-180 + res / 2)."""
    d = np.frombuffer(base64.b64decode(m["d"]), "<u2").astype(np.float64)
    q = np.frombuffer(base64.b64decode(m["q"]), np.uint8).astype(np.float64)
    res = float(m["res"])
    n = len(d)
    rpm = float(m.get("rpm") or 600.0)
    scan_time = 60.0 / rpm if rpm > 1.0 else 0.1
    r = np.where(d > 0, d / 1000.0, np.inf)
    r = np.where((r < RANGE_MIN_M) | (r > RANGE_MAX_M), np.inf, r)
    return dict(angle_min=math.radians(-180.0 + res / 2.0), angle_max=math.radians(-180.0 + res / 2.0 + res * (n - 1)),
                angle_increment=math.radians(res), scan_time=scan_time, time_increment=scan_time / max(1, n),
                range_min=RANGE_MIN_M, range_max=RANGE_MAX_M, ranges=r.tolist(), intensities=q.tolist(),
                age_s=float(m.get("age_ms") or 0) / 1000.0)


def grid_fields(m: dict) -> dict:
    """The `map_ros` message -> OccupancyGrid fields: resolution m, width, height, origin (x, y) m, data int8 list
    (-1 unknown, 0 free, 100 occupied, row-major, row 0 = the lowest y -- ROS's own order)."""
    meta = m["meta"]
    data = np.frombuffer(base64.b64decode(m["data"]), np.int8)
    w, h = int(meta["width"]), int(meta["height"])
    if data.size != w * h:
        raise ValueError("map_ros: %d cells for %d x %d" % (data.size, w, h))
    return dict(resolution=float(meta["res_mm"]) / 1000.0, width=w, height=h,
                origin=(float(meta["origin_mm"][0]) / 1000.0, float(meta["origin_mm"][1]) / 1000.0),
                data=data.tolist(), frame=meta.get("frame", "map"), v=meta.get("v"))


def pose_fields(live: dict):
    """The live pose -> (x m, y m, yaw rad, var_xy m^2, var_yaw rad^2, age s), or None without a pose."""
    p = live.get("pose")
    if not p:
        return None
    sxy = (p.get("sxy") or 50.0) / 1000.0
    sth = math.radians(p.get("sth") or 5.0)
    return (p["x"] / 1000.0, p["y"] / 1000.0, math.radians(p["th"]), sxy * sxy, sth * sth,
            float(live.get("age_ms") or 0) / 1000.0)


def pillar_rows(live: dict) -> list:
    """[(x m, y m, colour | None, text)] from the lidar seats (lpil) when present, else the camera signs (pillars)."""
    out = []
    for q in live.get("lpil") or []:
        out.append((q[0] / 1000.0, q[1] / 1000.0, q[2], "s%s %s hits" % (q[5] if len(q) > 5 else "?", q[3])))
    if not out:
        for q in live.get("pillars") or []:
            out.append((q[0] / 1000.0, q[1] / 1000.0, q[2], "%s x%s" % (q[2], q[3] if len(q) > 3 else 1)))
    return out


def path_points(live: dict) -> list:
    xy = live.get("path") or []
    return [(xy[i] / 1000.0, xy[i + 1] / 1000.0) for i in range(0, len(xy) - 1, 2)]


def quat(roll: float, pitch: float, yaw: float):
    """(x, y, z, w) of the ZYX Euler angles (radians)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def static_tfs(cfg: dict) -> list:
    """[(parent, child, (x, y, z) m, (roll, pitch, yaw) rad)] from cfg: its `sensors` list when the agent sends one
    (BRAIN_SPEC 8.4), else lidar.pos_mm / h_mm and the camera block."""
    out = []
    sens = cfg.get("sensors")
    if sens:
        for s in sens:
            if s.get("name") in ("lidar", "camera") and s.get("xyz") is not None:
                x, y, z = (float(v) / 1000.0 for v in s["xyz"])
                r, p, yw = (math.radians(float(v)) for v in (s.get("rpy") or (0.0, 0.0, 0.0)))
                out.append(("base_link", "laser" if s["name"] == "lidar" else "camera_link", (x, y, z), (r, p, yw)))
        if out:
            return out
    li, cam = cfg.get("lidar") or {}, cfg.get("camera") or {}
    px, py = (li.get("pos_mm") or (0.0, 0.0))[:2]
    out.append(("base_link", "laser", (px / 1000.0, py / 1000.0, float(li.get("h_mm") or 145.7) / 1000.0),
                (0.0, 0.0, 0.0)))
    out.append(("base_link", "camera_link", (float(cam.get("x_mm") or 150.0) / 1000.0,
                                             float(cam.get("y_mm") or 0.0) / 1000.0,
                                             float(cam.get("h_mm") or 125.0) / 1000.0),
                (0.0, math.radians(float(cam.get("pitch_deg") or 0.0)), 0.0)))
    return out


def lines(resp, stop):
    """NDJSON messages from a streaming HTTP response until stop is set or the stream ends."""
    buf = b""
    while not stop.is_set():
        chunk = resp.read1(65536) if hasattr(resp, "read1") else resp.read(4096)
        if not chunk:
            return
        buf += chunk
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            if ln.strip():
                try:
                    yield json.loads(ln)
                except ValueError:
                    continue


def open_stream(agent: str, hz: float, kinds: str = KINDS, token: str = "", timeout: float = 10.0):
    url = "%s/api/stream?kinds=%s&hz=%g" % (agent.rstrip("/"), kinds, hz)
    req = urllib.request.Request(url, headers={"X-BW-Passive": "1", "X-BW-Client": "ros_bridge",
                                               **({"X-BW-Token": token} if token else {})})
    return urllib.request.urlopen(req, timeout=timeout)


# ------------------------------------------------------------------------------------------------ ROS messages
def stamp_of(msg_time_cls, t_ns: int):
    st = msg_time_cls()
    st.sec, st.nanosec = int(t_ns // 1_000_000_000), int(t_ns % 1_000_000_000)
    return st


def make_scan(M, m: dict, now_ns: int):
    f = scan_fields(m)
    s = M.LaserScan()
    s.header.stamp = stamp_of(M.Time, now_ns - int(f["age_s"] * 1e9))
    s.header.frame_id = "laser"
    for k in ("angle_min", "angle_max", "angle_increment", "time_increment", "scan_time", "range_min", "range_max"):
        setattr(s, k, float(f[k]))
    s.ranges = array.array("f", f["ranges"])              # rclpy takes array.array for float32[] without a
    s.intensities = array.array("f", f["intensities"])    # per-element check (a list of 360 is checked one by one)
    return s


def make_grid(M, m: dict, now_ns: int):
    f = grid_fields(m)
    g = M.OccupancyGrid()
    g.header.stamp = stamp_of(M.Time, now_ns)
    g.header.frame_id = "map"
    g.info.resolution = float(f["resolution"])
    g.info.width, g.info.height = int(f["width"]), int(f["height"])
    g.info.origin.position.x, g.info.origin.position.y = f["origin"]
    g.info.origin.orientation.w = 1.0
    g.data = array.array("b", f["data"])                  # int8[]: ~60 000 cells, array.array is not re-validated
    return g


def make_tf(M, parent, child, xyz, rpy, t_ns):
    t = M.TransformStamped()
    t.header.stamp = stamp_of(M.Time, t_ns)
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in xyz)
    qx, qy, qz, qw = quat(*rpy)
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
    return t


def make_odom(M, live: dict, now_ns: int):
    pf = pose_fields(live)
    if pf is None:
        return None, None
    x, y, yaw, vxy, vth, age = pf
    t_ns = now_ns - int(age * 1e9)
    o = M.Odometry()
    o.header.stamp = stamp_of(M.Time, t_ns)
    o.header.frame_id, o.child_frame_id = "odom", "base_link"
    o.pose.pose.position.x, o.pose.pose.position.y = x, y
    qx, qy, qz, qw = quat(0.0, 0.0, yaw)
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
    cov = [0.0] * 36
    cov[0] = cov[7] = vxy / 2.0
    cov[35] = vth
    cov[14] = cov[21] = cov[28] = 1e6                      # z, roll, pitch: not estimated
    o.pose.covariance = cov
    return o, make_tf(M, "odom", "base_link", (x, y, 0.0), (0.0, 0.0, yaw), t_ns)


def make_pillars(M, live: dict, now_ns: int):
    arr = M.MarkerArray()
    clear = M.Marker()
    clear.action = 3                                       # DELETEALL: the set is replaced each time
    arr.markers.append(clear)
    for i, (x, y, colour, _txt) in enumerate(pillar_rows(live)):
        mk = M.Marker()
        mk.header.stamp = stamp_of(M.Time, now_ns)
        mk.header.frame_id = "map"
        mk.ns, mk.id, mk.type, mk.action = "bluewave_pillars", i, 3, 0      # CYLINDER, ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = x, y, 0.05
        mk.pose.orientation.w = 1.0
        mk.scale.x, mk.scale.y, mk.scale.z = 0.05, 0.05, 0.10
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = PILLAR_RGBA.get(colour, PILLAR_RGBA[None])
        arr.markers.append(mk)
    return arr


def make_path(M, live: dict, now_ns: int):
    pts = path_points(live)
    if not pts:
        return None
    p = M.Path()
    p.header.stamp = stamp_of(M.Time, now_ns)
    p.header.frame_id = "map"
    for x, y in pts:
        ps = M.PoseStamped()
        ps.header = p.header
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


# ------------------------------------------------------------------------------------------------ the node
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="BlueWave agent stream -> ROS 2 topics (read-only)")
    ap.add_argument("--agent", default="http://127.0.0.1:8000")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--token", default="")
    ap.add_argument("--no-map", action="store_true", help="do not publish /map (saves rosbridge bandwidth)")
    a = ap.parse_args(argv)
    import types

    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker, MarkerArray

    M = types.SimpleNamespace(Time=Time, PoseStamped=PoseStamped, TransformStamped=TransformStamped,
                              OccupancyGrid=OccupancyGrid, Odometry=Odometry, Path=Path, LaserScan=LaserScan,
                              Marker=Marker, MarkerArray=MarkerArray)
    rclpy.init()
    node = Node("bw_ros_bridge")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # /scan reliable: a reliable publisher serves best-effort AND reliable subscribers (RViz, rosbridge), not the
    # other way round
    pub = dict(scan=node.create_publisher(LaserScan, "/scan", 5),
               tf=node.create_publisher(TFMessage, "/tf", 10),
               tf_static=node.create_publisher(TFMessage, "/tf_static", latched),
               odom=node.create_publisher(Odometry, "/odom", 10),
               map=node.create_publisher(OccupancyGrid, "/map", latched),
               pillars=node.create_publisher(MarkerArray, "/bluewave/pillars", 10),
               plan=node.create_publisher(Path, "/plan", latched))
    stop = threading.Event()
    log = node.get_logger()
    kinds = "scan,live" + ("" if a.no_map else ",map_ros")

    def now_ns():
        return node.get_clock().now().nanoseconds

    def pump():
        backoff = 0.5
        while not stop.is_set():
            try:
                with open_stream(a.agent, a.hz, kinds, a.token) as resp:
                    log.info("bw_ros_bridge: streaming %s from %s (passive, read-only)" % (kinds, a.agent))
                    backoff = 0.5
                    last_pill = 0.0
                    for m in lines(resp, stop):
                        t = m.get("type")
                        n = now_ns()
                        if t == "cfg":
                            pub["tf_static"].publish(TFMessage(transforms=[make_tf(M, *row, n)
                                                                           for row in static_tfs(m)]))
                        elif t == "scan":
                            pub["scan"].publish(make_scan(M, m, n))
                        elif t == "map_ros":
                            pub["map"].publish(make_grid(M, m, n))
                        elif t == "live":
                            odom, tf = make_odom(M, m, n)
                            ident = make_tf(M, "map", "odom", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n)
                            pub["tf"].publish(TFMessage(transforms=[ident] + ([tf] if tf is not None else [])))
                            if odom is not None:
                                pub["odom"].publish(odom)
                            if time.monotonic() - last_pill > 0.5:
                                last_pill = time.monotonic()
                                pub["pillars"].publish(make_pillars(M, m, n))
                            path = make_path(M, m, n)
                            if path is not None:
                                pub["plan"].publish(path)
            except Exception as e:                           # the agent restarted, the network blinked: reconnect
                log.warn("bw_ros_bridge: %s: %s -- reconnecting in %.1f s" % (type(e).__name__, e, backoff))
                stop.wait(backoff)
                backoff = min(5.0, backoff * 2)

    th = threading.Thread(target=pump, name="bw-stream", daemon=True)
    th.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
