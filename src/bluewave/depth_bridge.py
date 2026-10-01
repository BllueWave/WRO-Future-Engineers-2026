"""The HP60C depth stream out of Hiwonder's container, raw.  Runs INSIDE the Docker container `MentorPi` (net=host),
next to the vendor's ascamera driver -- NOT in the agent (it needs rclpy, which only the container has):

    python3 /tmp/bw_depth_bridge.py [--port 8091] [--topic /ascamera/camera_publisher/depth0/image_raw] [--sub 1|2]

os/install_on_stock.sh copies this file into the container and starts it with the camera service.  ascamera publishes
depth as sensor_msgs/Image 16UC1 (mm); web_video_server would make it 8-bit MJPEG, which destroys it
(research/electronics_wl.md 6.3).  So this node serves every frame on TCP 127.0.0.1:8091 to one client -- the agent's
camera.DepthCamera:

    header   HDR "<4sIHHHHqqqI": magic b"BWD1", seq, width, height, encoding (1 = uint16 mm), flags (bit 0 = big-endian
             source, swapped here), stamp_ns (the message's header stamp, ROS clock), recv_real_ns and recv_mono_ns
             (this node's clocks when the message arrived), payload bytes
    payload  width x height uint16 little-endian, row-major; 0 = no return

The client turns stamp and recv_real into the frame's age (both wall-clock), so the host needs no shared monotonic clock.
Zero cost when nobody listens: the subscription exists only while a client is connected.  The newest frame wins: a slow
client gets the latest frame, never a queue.  `--sub 2` sends every 2nd pixel of every 2nd row (320 x 240, 154 KB).
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import threading
import time

HDR = struct.Struct("<4sIHHHHqqqI")                # bluewave/camera.py DEPTH_HDR reads it
MAGIC = b"BWD1"
ENC_U16_MM = 1
ENCODINGS = ("16UC1", "mono16")
TOPIC = "/ascamera/camera_publisher/depth0/image_raw"
PORT = 8091


def pack(seq: int, width: int, height: int, encoding: str, big_endian: bool, data, stamp_ns: int,
         recv_real_ns: int, recv_mono_ns: int, sub: int = 1):
    """(header bytes, payload bytes) for one depth image; ValueError when it is not uint16 depth."""
    if encoding not in ENCODINGS:
        raise ValueError("encoding %r is not 16-bit depth" % encoding)
    raw = bytes(data)
    if len(raw) < width * height * 2:
        raise ValueError("%d bytes for %d x %d uint16" % (len(raw), width, height))
    flags = 1 if big_endian else 0
    if big_endian or sub > 1 or len(raw) != width * height * 2:
        import numpy as np
        stride = len(raw) // height                         # a row may be padded (msg.step)
        a = np.frombuffer(raw, dtype=">u2" if big_endian else "<u2", count=height * stride // 2)
        a = a.reshape(height, stride // 2)[:, :width]
        if sub > 1:
            a = a[::sub, ::sub]
        height, width = a.shape
        raw = np.ascontiguousarray(a, dtype="<u2").tobytes()
    hdr = HDR.pack(MAGIC, seq & 0xFFFFFFFF, width, height, ENC_U16_MM, flags, int(stamp_ns), int(recv_real_ns),
                   int(recv_mono_ns), len(raw))
    return hdr, raw


class Server:
    """TCP on host:port for one client at a time (a new client replaces the old); send() never blocks the caller."""

    def __init__(self, port: int = PORT, host: str = "127.0.0.1"):
        self.ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.ls.bind((host, port))
        self.ls.listen(1)
        self.port = self.ls.getsockname()[1]
        self.cond = threading.Condition()
        self.client = None
        self.latest = None
        self.sent = self.dropped = self.clients = 0
        self._run = True
        threading.Thread(target=self._accept, name="bridge-accept", daemon=True).start()
        threading.Thread(target=self._send, name="bridge-send", daemon=True).start()

    def wanted(self) -> bool:
        return self.client is not None

    def offer(self, hdr: bytes, payload: bytes):
        with self.cond:
            if self.client is None:
                return
            if self.latest is not None:
                self.dropped += 1                            # the client was slower than the camera: newest wins
            self.latest = (hdr, payload)
            self.cond.notify()

    def _drop(self, c):
        with self.cond:
            if self.client is c:
                self.client, self.latest = None, None
        try:
            c.close()
        except OSError:
            pass

    def _accept(self):
        while self._run:
            try:
                c, _addr = self.ls.accept()
            except OSError:
                return
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.cond:
                old, self.client, self.latest = self.client, c, None
                self.clients += 1
                self.cond.notify()
            if old is not None:
                try:
                    old.close()
                except OSError:
                    pass
            threading.Thread(target=self._watch, args=(c,), daemon=True).start()

    def _watch(self, c):
        """A client never sends: a read that returns means it hung up (the subscription can go)."""
        try:
            while self._run and c.recv(64):
                pass
        except OSError:
            pass
        self._drop(c)

    def _send(self):
        while self._run:
            with self.cond:
                while self._run and (self.latest is None or self.client is None):
                    self.cond.wait(0.5)
                if not self._run:
                    return
                (hdr, payload), c, self.latest = self.latest, self.client, None
            try:
                c.sendall(hdr)
                c.sendall(payload)
                self.sent += 1
            except OSError:
                self._drop(c)

    def close(self):
        self._run = False
        with self.cond:
            c, self.client = self.client, None
            self.cond.notify_all()
        for s in (c, self.ls):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="HP60C depth -> TCP for the BlueWave agent")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--sub", type=int, default=1, help="send every Nth pixel of every Nth row")
    a = ap.parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("bw_depth_bridge")
    srv = Server(a.port, a.host)
    st = dict(sub=None, seq=0, bad=0)

    def on_image(msg):
        if not srv.wanted():
            return
        recv_real, recv_mono = time.time_ns(), time.monotonic_ns()
        stamp = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        st["seq"] += 1
        try:
            hdr, payload = pack(st["seq"], msg.width, msg.height, msg.encoding, bool(msg.is_bigendian), msg.data,
                                stamp, recv_real, recv_mono, a.sub)
        except ValueError as e:
            st["bad"] += 1
            if st["bad"] in (1, 100):
                node.get_logger().warn("bw_depth_bridge: %s" % e)
            return
        srv.offer(hdr, payload)

    def tick():
        # the subscription only while someone listens: with nobody connected ascamera's depth is not even delivered here
        if srv.wanted() and st["sub"] is None:
            st["sub"] = node.create_subscription(Image, a.topic, on_image, qos_profile_sensor_data)
            node.get_logger().info("bw_depth_bridge: client connected, subscribed to %s" % a.topic)
        elif not srv.wanted() and st["sub"] is not None:
            node.destroy_subscription(st["sub"])
            st["sub"] = None
            node.get_logger().info("bw_depth_bridge: client gone (sent %d, dropped %d)" % (srv.sent, srv.dropped))

    node.create_timer(0.2, tick)
    node.get_logger().info("bw_depth_bridge: listening on %s:%d" % (a.host, srv.port))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
