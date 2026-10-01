"""The map and cloud routes of the agent (docs/BRAIN_SPEC.md 8.5), over bluewave/mapper.py.  agent.py calls install()
once; every handler is a plain `def`, so FastAPI runs it on its thread pool -- a PNG or a PLY is never encoded on the
event loop the hub's 20 Hz producer runs on.

    GET  /api/map                         the served map's meta (no PNG), or {"map": null}
    GET  /api/map.png?crop=1&scale=1&overlay=0|1   PNG; overlay=1 = colour, with the pose, trail, scan, lidar pillars
    GET  /api/map.ros                     {meta, data}: OccupancyGrid int8 (-1 / 0 / 100), base64, row 0 = lowest y
    POST /api/map/clear                   {"confirm": "clear_map"}; refused while slam runs
    POST /api/map/save                    {"name"} -> {"name", "paths"}: runs/maps/<name>.png / .json / .npy (never
                                          replaces a saved map: name-2, name-3 ...)
    GET  /api/maps                        the saved maps, newest first
    GET  /api/maps/{name}.png             one saved map's picture
    GET  /api/cloud.ply                   the accumulated HP60C cloud, binary PLY (409 when there is none)
    GET  /api/cloud.png?view=top|side     an orthographic picture of it (for bw and the team)
    POST /api/cloud/clear                 {"confirm": "clear_cloud"}
    GET  /api/stream?kinds=scan,live,map_ros&hz=10   NDJSON of the WS messages for the ROS bridge; always passive (S13)

Every map request marks the known-pose map wanted for 60 s (mapper.rest_touch), so `bw map` works without a console.
"""
from __future__ import annotations

import base64
import json
import os
import time

from fastapi import Body, HTTPException, Request
from fastapi.responses import FileResponse, Response

from . import gridmap as GM


def install(app, robot, runner, hub, mapper, by=lambda request: "?"):
    """Register the routes on `app`.  runner: the agent's Runner (clear is refused while slam runs); hub: notes;
    by(request): who asked, for the notes."""

    def _need_map():
        mapper.rest_touch()
        g, src = mapper.served()
        if g is None:
            raise HTTPException(409, "no map: start SLAM (bw run slam, or the Map page) or run a program that has a "
                                     "pose -- the known-pose map builds while it runs")
        return g, src

    @app.get("/api/map")
    def map_meta():
        mapper.rest_touch()
        m = mapper.map_meta()
        return {"map": m} if m is not None else {"map": None}

    @app.get("/api/map.png")
    def map_png(crop: int = 1, scale: int = 1, overlay: int = 0):
        g, src = _need_map()
        if overlay:
            png = mapper.overlay_png()
            meta = mapper.map_meta() or {}
        else:
            png, meta = g.png(bool(crop), 2 if int(scale) == 2 else 1)
            meta = dict(meta, src=src)
        if not png:
            raise HTTPException(500, "PNG encode failed")
        return Response(png, media_type="image/png", headers={
            "Cache-Control": "no-store", "X-BW-Map": json.dumps(dict(src=src, frame=g.frame, v=meta.get("v"),
                                                                     crop=meta.get("crop"), res_mm=g.res,
                                                                     origin_mm=list(g.origin)))})

    @app.get("/api/map.ros")
    def map_ros():
        g, src = _need_map()
        meta, data = g.ros(True)
        meta.update(src=src, t=round(time.time(), 3))
        return dict(meta=meta, data=base64.b64encode(data).decode("ascii"))

    @app.post("/api/map/clear")
    def map_clear(request: Request, body: dict = Body(default={})):
        if (body or {}).get("confirm") != "clear_map":
            raise HTTPException(400, 'confirm must be "clear_map"')
        if runner.busy() and runner.name == "slam":
            raise HTTPException(409, "slam is running: stop it first (its map is saved when it ends)")
        mapper.clear_map()
        hub.note("info", "map", "map cleared by %s" % by(request))
        return {"cleared": True}

    @app.post("/api/map/save")
    def map_save(request: Request, body: dict = Body(default={})):
        name = GM.safe_name((body or {}).get("name") or "") or time.strftime("%Y%m%d-%H%M%S")
        _need_map()
        paths = mapper.save_map(name)
        name = os.path.splitext(os.path.basename(paths[0]))[0] if paths else name     # name-2 when taken
        hub.note("info", "map", "map saved as %s by %s" % (name, by(request)))
        return {"name": name, "paths": paths}

    @app.get("/api/maps")
    def maps_list():
        d = GM.maps_dir()
        out = []
        try:
            names = [fn[:-5] for fn in os.listdir(d) if fn.endswith(".json")]
        except OSError:
            names = []
        for n in names:
            try:
                with open(os.path.join(d, n + ".json"), encoding="utf-8") as f:
                    m = json.load(f)
                size = sum(os.path.getsize(os.path.join(d, n + ext)) for ext in (".png", ".json", ".npy")
                           if os.path.exists(os.path.join(d, n + ext)))
                out.append(dict(name=n, saved=m.get("saved"), frame=m.get("frame"), res_mm=m.get("res_mm"),
                                crop=m.get("crop"), v=m.get("v"), scans=m.get("scans"), known_m2=m.get("known_m2"),
                                bytes=size))
            except (OSError, ValueError):
                continue
        out.sort(key=lambda q: q.get("saved") or 0, reverse=True)
        return out

    @app.get("/api/maps/{name}.png")
    def maps_png(name: str):
        n = GM.safe_name(os.path.basename(name))
        path = os.path.join(GM.maps_dir(), n + ".png")
        if not n or not os.path.isfile(path):
            raise HTTPException(404, "no saved map %s" % name)
        return FileResponse(path, media_type="image/png")

    # -------------------------------------------------------------- cloud
    def _need_cloud():
        cl = mapper.cloud
        if cl is None or not cl.count:
            raise HTTPException(409, "no cloud: depth.on = 1 (bw sys restart_agent), then open the Map page's 3D view "
                                     "-- the cloud builds only while a console subscribes")
        return cl

    @app.get("/api/cloud.ply")
    def cloud_ply():
        cl = _need_cloud()
        return Response(cl.ply(), media_type="application/octet-stream",
                        headers={"Content-Disposition": 'attachment; filename="bluewave-cloud.ply"',
                                 "Cache-Control": "no-store"})

    @app.get("/api/cloud.png")
    def cloud_png(view: str = "top", size: int = 800):
        cl = _need_cloud()
        return Response(cl.render_png("side" if view == "side" else "top", size), media_type="image/png",
                        headers={"Cache-Control": "no-store"})

    @app.post("/api/cloud/clear")
    def cloud_clear(request: Request, body: dict = Body(default={})):
        if (body or {}).get("confirm") != "clear_cloud":
            raise HTTPException(400, 'confirm must be "clear_cloud"')
        mapper.clear_cloud()
        hub.note("info", "cloud", "cloud cleared by %s" % by(request))
        return {"cleared": True}

    # -------------------------------------------------------------- the passive stream (the ROS bridge)
    @app.get("/api/stream")
    async def stream(kinds: str = "scan,live,map_ros", hz: float = 10.0):
        """NDJSON, one message per line, the WS texts: first `cfg` (again whenever the params change), then per tick
        the kinds asked for -- tel (every tick), scan (a new revolution), live (the bus, path only when it changed),
        perc (a new scan), map_ros (the /api/map.ros body as {"type": "map_ros", "meta", "data"}, <= 0.5 Hz, when the
        served map changed).  ALWAYS passive (S13): the middleware never renews the link watchdog for this path, so a
        bridge left open cannot keep a DRIVES program alive with every console closed.  Read-only: nothing here
        drives, arms or changes a param."""
        import asyncio
        from fastapi.responses import StreamingResponse
        from .hub import dumps, params_digest, scan_body
        want = {k.strip() for k in (kinds or "").split(",") if k.strip()} & STREAM_KINDS
        period = 1.0 / max(1.0, min(float(hz), 20.0))

        async def gen():
            last_scan, last_perc, path_v, map_key, t_map, digest = None, None, None, None, -1e9, None
            nxt = time.monotonic()
            while True:
                now = time.monotonic()
                out = []
                dg = params_digest(robot.p)
                if dg != digest:
                    digest = dg
                    out.append(dumps(hub.cfg("stream (passive)")))
                if "tel" in want:
                    out.append(dumps(hub.tel()))
                sc = robot.scan
                if "scan" in want and sc is not None and sc.t != last_scan:
                    last_scan = sc.t
                    out.append(dumps(scan_body(sc, 1.0)))
                if "perc" in want and sc is not None and sc.t != last_perc:
                    last_perc = sc.t
                    d = mapper.perc_body()
                    if d is not None:
                        out.append(dumps(d))
                if "live" in want:
                    d = robot.live.read(pts=False, trail=False)
                    if d.get("path_v") == path_v:
                        d.pop("path", None)
                    else:
                        path_v = d.get("path_v")
                    out.append(dumps(dict(type="live", t=round(time.time(), 3), **d)))
                if "map_ros" in want and now - t_map >= 2.0:
                    t_map = now
                    mapper.rest_touch()
                    g, src = mapper.served()
                    if g is not None and (id(g), g.version) != map_key:
                        map_key = (id(g), g.version)
                        meta, data = await asyncio.to_thread(g.ros, True)
                        meta.update(src=src, t=round(time.time(), 3))
                        out.append(dumps(dict(type="map_ros", t=meta["t"], meta=meta,
                                              data=base64.b64encode(data).decode("ascii"))))
                if out:
                    yield ("\n".join(out) + "\n").encode("utf-8")
                nxt += period
                if time.monotonic() - nxt > 0.5:
                    nxt = time.monotonic()                               # fell behind: re-anchor, never burst
                await asyncio.sleep(max(0.0, nxt - time.monotonic()))

        return StreamingResponse(gen(), media_type="application/x-ndjson",
                                 headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


STREAM_KINDS = {"tel", "scan", "live", "perc", "map_ros"}
