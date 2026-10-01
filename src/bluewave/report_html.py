"""The crash report as ONE offline HTML file (BRAIN4_SPEC 6.8): runs/<id>.report.html.

    html = render(report, thumbs)          # thumbs: {frame number: JPEG bytes} (analyze puts them in report["_thumbs"])
    write_html(report, thumbs, path)

Inline CSS and JS, no CDN, no font, no request outside the file: the report JSON sits in
<script type="application/json" id="bw-report">, the thumbnails in id="bw-thumbs" as data:image/jpeg URIs, and the
page builds itself from them.  CONSOLE_SPEC 9.1: its tokens (dark base, light by prefers-color-scheme or ?theme=),
the FIELD mat palette (fixed in both themes), units on every number, no gradients / shadows (the thumbnail dialog
only) / emoji / pills; no horizontal scroll at 390 px; nothing depends on hover (a tap scrubs, a tap opens).

Sections: header (id, program, profile / body, the verdict, duration, end, laps, counts) -> field (walls, island,
seats, the run's signs, the lot, the path coloured by state and dashed where sxy > 60 mm, incident markers, the car
at the scrub time with its uncertainty, the sim truth dashed) -> timeline (play / pause, 1x / 4x, state bands and
incident ticks; lanes v cmd vs measured, steer, clearance, sxy, acc deviation with both thresholds, loop, battery)
-> incidents (a tap scrubs to t; the thumbnails at t, t - 0.5, t - 1.0 s open at 320 x 240) -> footer
(thresholds, sources, fix hints, analyzer ms).  Size <= 2 MB: thumbnails beyond the budget are left out, contacts
keep theirs first.
"""
from __future__ import annotations

import base64
import json

MAX_BYTES = 2_000_000


def _thumbs_json(report: dict, thumbs: dict, budget: int) -> str:
    """{frame: data URI} within `budget` bytes: the frames of contacts first, then near misses, then the rest."""
    order = {"contact": 0, "near": 1, "bump": 2, "loc_loss": 3, "pose_out": 3}
    want = []
    for inc in sorted(report.get("incidents") or [], key=lambda i: (order.get(i.get("kind"), 5), i.get("t") or 0)):
        for n in inc.get("frames") or []:
            if n not in want:
                want.append(n)
    out, used = {}, 0
    for n in want:
        b = thumbs.get(n) if n in thumbs else thumbs.get(str(n))
        if not b:
            continue
        uri = "data:image/jpeg;base64," + base64.b64encode(b).decode("ascii")
        if used + len(uri) + 16 > budget:
            break
        out[str(n)] = uri
        used += len(uri) + 16
    return json.dumps(out, separators=(",", ":"))


def _safe(s: str) -> str:
    """JSON inside <script>: no '</' can close the tag, no '<!--' can open a comment."""
    return s.replace("</", "<\\/").replace("<!--", "<\\!--")


def render(report: dict, thumbs: dict | None = None, theme: str = "auto") -> str:
    rep = {k: v for k, v in report.items() if not str(k).startswith("_")}
    rj = _safe(json.dumps(rep, separators=(",", ":"), default=str))
    budget = max(0, MAX_BYTES - len(rj) - len(_PAGE) - 20_000)
    tj = _safe(_thumbs_json(rep, thumbs or {}, budget))
    title = "BlueWave report %s" % str(rep.get("id") or "")
    theme_attr = ' data-theme="%s"' % theme if theme in ("dark", "light") else ""
    return (_PAGE.replace("{{THEME}}", theme_attr)
            .replace("{{TITLE}}", title.replace("&", "&amp;").replace("<", "&lt;"))
            .replace("{{REPORT}}", rj).replace("{{THUMBS}}", tj))


def write_html(report: dict, thumbs: dict | None, path: str) -> str:
    import os
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        f.write(render(report, thumbs))
    os.replace(tmp, path)
    return path


_PAGE = r"""<!doctype html>
<html lang="en"{{THEME}}>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">
<title>{{TITLE}}</title>
<style>
:root{--ground:#0a0e13;--panel:#10161e;--panel2:#141c26;--line:#1e2835;--line2:#2a3646;--text:#d9e2ec;--muted:#8190a4;
--faint:#56647a;--accent:#3b9eff;--accent-ink:#06121f;--ok:#35c98a;--warn:#f2b441;--bad:#ff5252;--cam:#e8a33d;
--sans:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;--mono:ui-monospace,"SF Mono","Cascadia Mono",Consolas,monospace;color-scheme:dark}
@media (prefers-color-scheme: light){:root:not([data-theme="dark"]){--ground:#f5f7fa;--panel:#ffffff;--panel2:#f0f3f7;
--line:#dfe5ec;--line2:#c9d2dc;--text:#111822;--muted:#56657a;--faint:#8594a6;--accent:#0a66c2;--accent-ink:#ffffff;
--ok:#17804f;--warn:#9a6700;--bad:#c62828;--cam:#b26b00;color-scheme:light}}
:root[data-theme="light"]{--ground:#f5f7fa;--panel:#ffffff;--panel2:#f0f3f7;--line:#dfe5ec;--line2:#c9d2dc;--text:#111822;
--muted:#56657a;--faint:#8594a6;--accent:#0a66c2;--accent-ink:#ffffff;--ok:#17804f;--warn:#9a6700;--bad:#c62828;--cam:#b26b00;color-scheme:light}
*{box-sizing:border-box}
html,body{margin:0;background:var(--ground);color:var(--text);font:14px/1.45 var(--sans)}
body{padding:12px;max-width:1600px;margin:0 auto;overflow-x:hidden}
.mono,.num{font-family:var(--mono);font-variant-numeric:tabular-nums}
.u{color:var(--muted)}
.ml{font-size:11px;text-transform:uppercase;letter-spacing:.08em;font-weight:600;color:var(--muted)}
.card{background:var(--panel);border:1px solid var(--line);border-radius:6px;min-width:0}
.ph{display:flex;align-items:center;gap:8px;padding:10px 12px;border-bottom:1px solid var(--line);flex-wrap:wrap}
.ph h2{margin:0;font-size:13px;font-weight:600;letter-spacing:.02em}
.pad{padding:12px}
header.card{padding:12px;display:flex;flex-direction:column;gap:8px;margin-bottom:12px}
.hrow{display:flex;flex-wrap:wrap;gap:8px 16px;align-items:baseline}
.rid{font-family:var(--mono);font-size:15px;word-break:break-all}
.chip{display:inline-block;padding:1px 7px;border-radius:4px;border:1px solid var(--line2);font:600 11px/18px var(--mono);
letter-spacing:.04em;white-space:nowrap}
.chip.ok{color:var(--ok);border-color:var(--ok)}.chip.warn{color:var(--warn);border-color:var(--warn)}
.chip.bad{color:var(--bad);border-color:var(--bad)}.chip.acc{color:var(--accent);border-color:var(--accent)}
.chip.mut{color:var(--muted)}
.bad-t{color:var(--bad)}
.verdict{font-size:13px;padding:2px 10px}
.counts{display:flex;flex-wrap:wrap;gap:4px 12px;font-size:13px}
.counts b{font-family:var(--mono);font-weight:600}
.grid{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:12px;align-items:start}
@media (max-width:860px){.grid{grid-template-columns:minmax(0,1fr)}}
.fieldbox{position:relative;padding:12px}
canvas{display:block;width:100%;touch-action:none}
#field{aspect-ratio:1/1;max-width:min(100%,74vh);margin:0 auto;border-radius:4px}
.nopose #field{display:none}
.cap{font-size:12px;color:var(--muted);margin-top:6px}
.scrub{display:flex;align-items:center;gap:8px;padding:10px 12px;border-top:1px solid var(--line);flex-wrap:wrap}
button{font:600 12px var(--sans);color:var(--text);background:var(--panel2);border:1px solid var(--line2);border-radius:4px;
min-height:32px;padding:0 12px;cursor:pointer}
button[aria-pressed="true"]{background:var(--accent);color:var(--accent-ink);border-color:var(--accent)}
button:focus-visible,input:focus-visible,.inc:focus-visible,img:focus-visible{outline:2px solid var(--accent);outline-offset:1px}
@media (pointer:coarse){button{min-height:44px}}
.seg{display:inline-flex;gap:4px}
input[type=range]{flex:1;min-width:120px;accent-color:var(--accent)}
.time{font:600 14px var(--mono);min-width:64px;text-align:right}
#tl{height:420px}
.incs{display:flex;flex-direction:column}
.inc{display:grid;grid-template-columns:34px minmax(0,1fr);gap:4px 10px;padding:10px 12px;border-bottom:1px solid var(--line);cursor:pointer}
.inc:last-child{border-bottom:0}
.inc.sel{background:var(--panel2);box-shadow:inset 3px 0 0 var(--accent)}
.inc .n{font:600 13px var(--mono);color:var(--muted)}
.inc .l1{display:flex;flex-wrap:wrap;gap:6px 10px;align-items:baseline}
.inc .where{font-size:13px}
.inc .dec,.inc .why,.inc .fix,.inc .sub{font-size:13px}
.inc .dec{color:var(--muted)}
.inc .why b{font-family:var(--mono);font-size:12px}
.inc .fix{color:var(--text)}
.inc .sub{color:var(--muted);padding-left:10px;border-left:1px solid var(--line2)}
.evs{display:flex;flex-wrap:wrap;gap:4px}
.ev{font:12px/18px var(--mono);padding:0 6px;border:1px solid var(--line2);border-radius:4px;color:var(--text);
overflow-wrap:anywhere;max-width:100%}
.ev.s{border-color:var(--bad)}
.thumbs{display:flex;gap:6px;flex-wrap:wrap}
.thumbs img{width:120px;height:90px;border-radius:4px;border:1px solid var(--line2);cursor:zoom-in;background:var(--panel2)}
.tcap{font:11px var(--mono);color:var(--muted)}
footer.card{margin-top:12px}
dl.kv{display:grid;grid-template-columns:max-content minmax(0,1fr);gap:4px 12px;margin:0;font-size:13px}
dl.kv dt{color:var(--muted)}dl.kv dd{margin:0;font-family:var(--mono);font-variant-numeric:tabular-nums;word-break:break-word}
.cols3{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}
@media (max-width:900px){.cols3{grid-template-columns:minmax(0,1fr)}}
ul.hints{margin:0;padding-left:18px}ul.hints li{margin:2px 0}
.empty{padding:14px 12px;color:var(--muted)}
.errbox{border:1px solid var(--bad);color:var(--bad);border-radius:4px;padding:8px 10px;font:12px var(--mono);white-space:pre-wrap}
#dlg{position:fixed;inset:0;display:none;align-items:center;justify-content:center;background:rgba(0,0,0,.55);z-index:10;padding:12px}
#dlg.on{display:flex}
#dlg .box{background:var(--panel);border:1px solid var(--line2);border-radius:6px;padding:10px;box-shadow:0 8px 24px rgba(0,0,0,.35);max-width:100%}
#dlg img{width:320px;height:240px;max-width:calc(100vw - 48px);height:auto;display:block;border-radius:4px}
#dlg .row{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-top:8px}
@media (prefers-reduced-motion:no-preference){.inc{transition:background-color .1s}}
</style>
</head>
<body>
<header class="card" id="hdr"></header>
<div class="grid">
  <div class="stack" style="display:flex;flex-direction:column;gap:12px;min-width:0">
    <section class="card">
      <div class="ph"><h2>Field</h2><span class="ml" id="fcap"></span></div>
      <div class="fieldbox"><canvas id="field" role="img" aria-label="The field with the recorded path and the incidents"></canvas>
      <div class="cap" id="fnote"></div></div>
    </section>
    <section class="card">
      <div class="ph"><h2>Timeline</h2><span class="ml">tap or drag to scrub</span></div>
      <div class="pad" style="padding-bottom:0"><canvas id="tl" role="img" aria-label="Telemetry lanes over time"></canvas></div>
      <div class="scrub">
        <button id="play" type="button" aria-pressed="false">Play</button>
        <span class="seg"><button type="button" data-s="1" aria-pressed="true">1×</button><button type="button" data-s="4" aria-pressed="false">4×</button></span>
        <input id="rng" type="range" min="0" max="1" step="0.1" value="0" aria-label="Time">
        <span class="time" id="tm">00:00.0</span>
      </div>
    </section>
  </div>
  <section class="card" style="min-width:0">
    <div class="ph"><h2>Incidents</h2><span class="ml" id="icap"></span></div>
    <div class="incs" id="incs"></div>
  </section>
</div>
<footer class="card"><div class="ph"><h2>Sources, thresholds, fixes</h2></div><div class="pad cols3" id="ftr"></div></footer>
<div id="dlg" role="dialog" aria-modal="true" aria-label="Camera frame"><div class="box"><img id="dlgimg" alt="Camera frame"><div class="row"><span class="tcap" id="dlgcap"></span><button type="button" id="dlgx">Close</button></div></div></div>
<script type="application/json" id="bw-report">{{REPORT}}</script>
<script type="application/json" id="bw-thumbs">{{THUMBS}}</script>
<script>
"use strict";
(function () {
  try { const q = new URLSearchParams(location.search).get("theme"); if (q === "dark" || q === "light") document.documentElement.setAttribute("data-theme", q); } catch (e) {}
  const R = JSON.parse(document.getElementById("bw-report").textContent || "{}");
  let TH = {}; try { TH = JSON.parse(document.getElementById("bw-thumbs").textContent || "{}"); } catch (e) {}
  const $ = s => document.querySelector(s);
  const esc = s => String(s === null || s === undefined ? "" : s).replace(/[&<>"']/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"}[c]));
  const isNum = v => typeof v === "number" && isFinite(v);
  const U = (v, u, d = 0) => isNum(v) ? `<span class="num">${v.toFixed(d)}</span><span class="u">&thinsp;${u}</span>` : `<span class="u">—</span>`;
  const mmss = t => { t = Math.max(0, t || 0); const m = Math.floor(t / 60), s = t - m * 60; return String(m).padStart(2, "0") + ":" + s.toFixed(1).padStart(4, "0"); };
  const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  const FIELD = {mat: "#f3f4f1", wall: "#15181c", island: "#e2e4e0", sec: "#b9bec4", seat: "#aab0b6", path: "#1668c9", robot: "#1668c9",
    trail: "#6b7785", red: "#d93636", green: "#1f9d55", lot: "#c2378f", truth: "#444444", bad: "#c62828", warn: "#9a6700", cam: "#d97706"};
  const TL = R.timeline || {t: []}, N = (TL.t || []).length, DUR = R.duration_s || (N ? TL.t[N - 1] : 0) || 0;
  const INC = R.incidents || [];
  const VCLS = {CLEAN: "ok", NEAR: "warn", LOST: "warn", CONTACT: "bad", INCOMPLETE: "warn", REFUSED: "warn", NOT_RUN: "warn", NOT_STANDARD: "bad"};
  const KCLS = {contact: "bad", near: "warn", bump: "mut", loc_loss: "acc", pose_out: "acc", start_mismatch: "warn",
    intervention: "mut", overrun: "mut", sensor: "bad", error: "bad", battery: "warn"};
  let T = 0, playing = false, speed = 1, last = 0, sel = -1;

  /* ---------------------------------------------------------------- header */
  const c = R.counts || {}, e = R.end || {}, b = R.body || {};
  const cnt = [["contact", "contacts"], ["near", "near misses"], ["bump", "bumps?"], ["loc_loss", "localisation losses"],
    ["pose_out", "pose out"], ["intervention", "interventions"], ["overrun", "overruns"], ["sensor", "sensor / error"],
    ["battery", "battery"], ["start_mismatch", "start mismatch"]];
  $("#hdr").innerHTML = `<div class="hrow"><span class="rid">${esc(R.id)}</span><span class="chip verdict ${VCLS[R.verdict] || "mut"}">${esc(R.verdict || "?")}</span></div>
    <div class="hrow" style="font-size:13px"><span><span class="ml">Program</span> <span class="mono">${esc(R.program || "—")}</span></span>
    <span><span class="ml">Profile</span> <span class="mono">${esc(R.profile || "none")}</span> <span class="u">· ${esc(b.car || "")}${b.shell ? " · " + esc(b.shell) : ""}</span></span>
    <span><span class="ml">Duration</span> ${U(R.duration_s, "s", 1)}</span>
    <span><span class="ml">End</span> <span class="mono">${esc(e.reason || "none")}</span></span>
    <span><span class="ml">Laps</span> ${isNum(e.laps) ? `<span class="num">${e.laps}</span>` : "—"}</span></div>
    <div class="counts">${cnt.map(([k, n]) => `<span>${esc(n)} <b class="${c[k] ? (k === "contact" || k === "sensor" ? "bad-t" : "") : "u"}">${c[k] || 0}</b></span>`).join("")}</div>
    ${R.error ? `<div class="errbox">${esc(R.error)}</div>` : ""}`;

  /* ---------------------------------------------------------------- field */
  const fcv = $("#field"), tcv = $("#tl");
  const M = R.map || {}, CAR = M.car || [34, 179, 80.5];
  const hasPose = (TL.x || []).some(isNum);
  $("#fnote").textContent = hasPose ? `Path: ${R.sources && R.sources.pose ? R.sources.pose : ""} pose, coloured by state, dashed where the pose is uncertain (sxy above 60 mm). The car is drawn at the scrub time.`
    : "No pose in this run (this program publishes none): the incidents are placed by time only, on the timeline.";
  if (!hasPose) document.body.classList.add("nopose");
  $("#fcap").textContent = M.layout ? `${M.layout} field · north up` : "north up";
  function fit(cv) {
    const dpr = Math.min(window.devicePixelRatio || 1, 3), w = cv.clientWidth || 300, h = cv.clientHeight || w;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    const x = cv.getContext("2d"); x.setTransform(dpr, 0, 0, dpr, 0, 0); return {x, w, h};
  }
  function rot(k, x, y) { const cs = [[1, 0], [0, 1], [-1, 0], [0, -1]][((k % 4) + 4) % 4]; return [cs[0] * x - cs[1] * y, cs[1] * x + cs[0] * y]; }
  function stateColor(s) {
    if (s === null || s === undefined) return FIELD.trail;
    if (s === "LOST") return FIELD.warn;
    if (s === "APPROACH" || s === "PARK" || s === "EXIT") return FIELD.lot;
    if (s === "SETTLE" || s === "LOCATE" || s === "LOOK" || s === "WAIT") return FIELD.trail;
    return FIELD.path;
  }
  function idxAt(t) { let lo = 0, hi = N - 1; if (hi < 0) return -1; if (t <= TL.t[0]) return 0; while (lo < hi) { const m = (lo + hi + 1) >> 1; if (TL.t[m] <= t) lo = m; else hi = m - 1; } return lo; }
  function drawField() {
    const {x, w} = fit(fcv), P = 90, S = w / (3000 + 2 * P), X = v => (v + 1500 + P) * S, Y = v => (1500 - v + P) * S;
    x.clearRect(0, 0, w, w); x.fillStyle = css("--panel2"); x.fillRect(0, 0, w, w);
    x.fillStyle = FIELD.mat; x.fillRect(X(-1500), Y(1500), 3000 * S, 3000 * S);
    const isl = M.island || [-500, 500, -500, 500];
    x.fillStyle = FIELD.island; x.fillRect(X(isl[0]), Y(isl[3]), (isl[1] - isl[0]) * S, (isl[3] - isl[2]) * S);
    x.strokeStyle = FIELD.sec; x.lineWidth = 1;
    [[isl[1], isl[2], 1500, -1500], [isl[1], isl[3], 1500, 1500], [isl[0], isl[3], -1500, 1500], [isl[0], isl[2], -1500, -1500]].forEach(([a, bb, c2, d]) => { x.beginPath(); x.moveTo(X(a), Y(bb)); x.lineTo(X(c2), Y(d)); x.stroke(); });
    x.strokeStyle = FIELD.seat;
    for (let k = 0; k < 4; k++) for (const al of [-500, 0, 500]) for (const ac of [400, 600]) { const [sx, sy] = rot(k, al, -1500 + ac); x.beginPath(); x.moveTo(X(sx) - 3, Y(sy)); x.lineTo(X(sx) + 3, Y(sy)); x.moveTo(X(sx), Y(sy) - 3); x.lineTo(X(sx), Y(sy) + 3); x.stroke(); }
    x.strokeStyle = FIELD.wall; x.lineWidth = Math.max(2, 20 * S);
    x.strokeRect(X(-1500) - 10 * S, Y(1500) - 10 * S, 3020 * S, 3020 * S);
    x.strokeRect(X(isl[0]), Y(isl[3]), (isl[1] - isl[0]) * S, (isl[3] - isl[2]) * S);
    if (M.lot) { const L = M.lot; x.fillStyle = "rgba(194,55,143,.14)"; x.fillRect(X(L[0]), Y(L[3]), (L[2] - L[0]) * S, (L[3] - L[1]) * S);
      x.fillStyle = FIELD.lot; x.fillRect(X(L[0] - 20), Y(L[3]), 20 * S, 200 * S); x.fillRect(X(L[2]), Y(L[3]), 20 * S, 200 * S); }
    x.setLineDash([4, 3]); x.lineWidth = 1;
    for (const s of M.truth_signs || []) { x.strokeStyle = s[2] === "green" ? FIELD.green : s[2] === "red" ? FIELD.red : FIELD.truth; x.strokeRect(X(s[0] - 25), Y(s[1] + 25), 50 * S, 50 * S); }
    x.setLineDash([]);
    for (const s of M.signs || []) { if (isNum(s[3]) && s[3] > T) continue; x.fillStyle = s[2] === "green" ? FIELD.green : s[2] === "red" ? FIELD.red : FIELD.trail; x.fillRect(X(s[0] - 25), Y(s[1] + 25), 50 * S, 50 * S); }
    for (const o of M.obstacles || []) { x.fillStyle = FIELD.wall; x.fillRect(X(o[0] - 25), Y(o[1] + 25), 50 * S, 50 * S); }
    // the truth, dashed
    if (TL.truth_x) { x.strokeStyle = FIELD.truth; x.setLineDash([5, 4]); x.lineWidth = 1.2; x.beginPath(); let on = false;
      for (let i = 0; i < N; i++) { const a = TL.truth_x[i], bb = TL.truth_y[i]; if (!isNum(a) || !isNum(bb)) { on = false; continue; } on ? x.lineTo(X(a), Y(bb)) : x.moveTo(X(a), Y(bb)); on = true; }
      x.stroke(); x.setLineDash([]); }
    // the path by state
    x.save(); x.beginPath(); x.rect(X(-1560), Y(1560), 3120 * S, 3120 * S); x.clip();
    if (hasPose) {
      x.lineWidth = 2.2; x.lineCap = "round";
      for (let i = 1; i < N; i++) {
        const a = TL.x[i - 1], bb = TL.y[i - 1], c2 = TL.x[i], d = TL.y[i];
        if (!isNum(a) || !isNum(bb) || !isNum(c2) || !isNum(d) || Math.hypot(c2 - a, d - bb) > 400) continue;
        if (Math.abs(a) > 1700 || Math.abs(bb) > 1700) { x.globalAlpha = .35; } else x.globalAlpha = 1;
        const st = (TL.states || [])[TL.state[i]];
        x.strokeStyle = stateColor(st); x.setLineDash(isNum(TL.sxy[i]) && TL.sxy[i] > 60 ? [4, 4] : []);
        x.beginPath(); x.moveTo(X(a), Y(bb)); x.lineTo(X(c2), Y(d)); x.stroke();
      }
      x.globalAlpha = 1; x.setLineDash([]);
    }
    x.restore();
    // incidents
    x.font = "600 11px " + css("--sans").split(",")[0]; x.textBaseline = "middle";
    INC.forEach((inc, k) => {
      const w2 = inc.where || {}; if (!isNum(w2.x) || !isNum(w2.y)) return;
      const px = X(Math.max(-1600, Math.min(1600, w2.x))), py = Y(Math.max(-1600, Math.min(1600, w2.y))), big = k === sel ? 1.4 : 1;
      x.lineWidth = 2;
      if (inc.kind === "contact") { x.strokeStyle = FIELD.bad; const r = 7 * big; x.beginPath(); x.moveTo(px - r, py - r); x.lineTo(px + r, py + r); x.moveTo(px + r, py - r); x.lineTo(px - r, py + r); x.stroke();
        x.fillStyle = FIELD.bad; x.fillText(String(inc.id), px + 9 * big, py - 8 * big); }
      else if (inc.kind === "near") { x.strokeStyle = FIELD.warn; x.beginPath(); x.arc(px, py, 7 * big, 0, 7); x.stroke(); x.fillStyle = FIELD.warn; x.fillText(String(inc.id), px + 9, py - 8); }
      else if (inc.kind === "bump") { x.strokeStyle = FIELD.trail; x.lineWidth = 1.2; x.beginPath(); x.arc(px, py, 5 * big, 0, 7); x.stroke(); }
      else if (inc.kind === "loc_loss" || inc.kind === "pose_out" || inc.kind === "start_mismatch") { x.fillStyle = FIELD.path; const r = 6 * big; x.beginPath(); x.moveTo(px, py - r); x.lineTo(px + r, py); x.lineTo(px, py + r); x.lineTo(px - r, py); x.closePath(); x.fill(); }
      else if (inc.kind === "intervention") { x.fillStyle = FIELD.trail; x.beginPath(); x.arc(px, py, 3 * big, 0, 7); x.fill(); }
    });
    // the car at the scrub time
    const i = idxAt(T);
    if (hasPose && i >= 0 && isNum(TL.x[i]) && isNum(TL.y[i]) && isNum(TL.th[i])) {
      const px = X(TL.x[i]), py = Y(TL.y[i]), th = TL.th[i] * Math.PI / 180;
      if (isNum(TL.sxy[i])) { x.strokeStyle = "rgba(22,104,201,.45)"; x.lineWidth = 1; x.beginPath(); x.arc(px, py, Math.max(2, TL.sxy[i] * S), 0, 7); x.stroke(); }
      x.save(); x.translate(px, py); x.rotate(-th); x.strokeStyle = FIELD.robot; x.fillStyle = "rgba(22,104,201,.18)"; x.lineWidth = 2;
      x.beginPath(); x.rect(-CAR[0] * S, -CAR[2] * S, (CAR[0] + CAR[1]) * S, 2 * CAR[2] * S); x.fill(); x.stroke();
      x.beginPath(); x.moveTo(CAR[1] * S, 0); x.lineTo((CAR[1] - 40) * S, 0); x.stroke(); x.restore();
    }
  }

  /* ---------------------------------------------------------------- timeline */
  const lanes = [
    {k: ["v", "vm"], name: "Speed cmd / measured", u: "m/s", d: 2},
    {k: ["steer"], name: "Steer", u: "°", d: 0},
    {k: ["clear"], name: "Clearance", u: "mm", d: 0, ref: [0, (R.thresholds || {}).near_mm]},
    {k: ["sxy"], name: "Pose sxy", u: "mm", d: 0, ref: [(R.thresholds || {}).sxy_bad_mm]},
    {k: ["acc"], name: "Acc deviation", u: "g", d: 2, ref: [(R.thresholds || {}).acc_bump_g, (R.thresholds || {}).acc_contact_g], lo0: true},
    {k: ["img"], name: "Image motion", u: "grey", d: 1, lo0: true},
    {k: ["loop"], name: "Loop", u: "ms", d: 0, ref: [(R.thresholds || {}).loop_gap_ms], lo0: true},
    {k: ["bat"], name: "Battery", u: "V", d: 2, ref: [7.0, 7.5]},
  ].filter(l => l.k.some(k => (TL[k] || []).some(isNum)));
  const STC = s => s === "LOST" ? css("--warn") : (s === "APPROACH" || s === "PARK" || s === "EXIT") ? "#c2378f" : (s === "SETTLE" || s === "LOCATE" || s === "LOOK" || s === "WAIT") ? css("--faint") : css("--accent");
  function drawTL() {
    const {x, w, h} = fit(tcv);
    x.clearRect(0, 0, w, h); x.fillStyle = css("--panel"); x.fillRect(0, 0, w, h);
    const Lp = Math.min(118, w * 0.3), Rp = 8, band = 12, top = band + 8, lh = lanes.length ? (h - top - 6) / lanes.length : 1;
    const X = t => Lp + (DUR ? t / DUR : 0) * (w - Lp - Rp);
    // state bands
    if (TL.state) for (let i = 0; i < N; i++) { const s = (TL.states || [])[TL.state[i]]; if (s === undefined) continue; x.fillStyle = STC(s); x.globalAlpha = .55; x.fillRect(X(TL.t[i]), 2, Math.max(1, X(TL.t[i] + 0.1) - X(TL.t[i])), band); }
    x.globalAlpha = 1;
    // incident ticks
    for (const inc of INC) { const col = inc.kind === "contact" ? css("--bad") : inc.kind === "near" || inc.kind === "battery" ? css("--warn") : inc.kind === "loc_loss" || inc.kind === "pose_out" ? css("--accent") : css("--faint");
      x.fillStyle = col; x.fillRect(X(inc.t) - 1, 0, 2, h); if (inc.kind === "contact" && isNum(inc.t_end) && inc.t_end > inc.t + 0.2) { x.globalAlpha = .07; x.fillRect(X(inc.t), top, X(inc.t_end) - X(inc.t), h - top); x.globalAlpha = 1; } }
    const font = css("--sans").split(",")[0];
    lanes.forEach((ln, li) => {
      const y0 = top + li * lh;
      x.strokeStyle = css("--line"); x.lineWidth = 1; x.beginPath(); x.moveTo(0, y0 + lh - .5); x.lineTo(w, y0 + lh - .5); x.stroke();
      let lo = Infinity, hi = -Infinity;
      for (const k of ln.k) for (const v of TL[k] || []) if (isNum(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
      for (const r of ln.ref || []) if (isNum(r) && ln.k[0] !== "clear") { lo = Math.min(lo, r); hi = Math.max(hi, r); }
      if (ln.lo0) lo = Math.min(lo, 0);
      if (ln.k[0] === "clear") { hi = Math.min(hi, 400); lo = Math.max(Math.min(lo, 0), -100); }
      if (!(hi > lo)) { hi = lo + 1; }
      x.fillStyle = css("--muted"); x.font = "600 10px " + font; x.textBaseline = "top"; x.fillText(ln.name.toUpperCase(), 6, y0 + 3);
      x.font = "10px " + css("--mono").split(",")[0]; x.fillText(`${lo.toFixed(ln.d)}…${hi.toFixed(ln.d)} ${ln.u}`, 6, y0 + 16);
      const Y = v => y0 + 4 + (hi - Math.max(lo, Math.min(hi, v))) / (hi - lo) * (lh - 10);
      x.setLineDash([3, 3]); x.strokeStyle = css("--faint");
      for (const r of ln.ref || []) if (isNum(r) && r >= lo && r <= hi) { x.beginPath(); x.moveTo(Lp, Y(r)); x.lineTo(w - Rp, Y(r)); x.stroke(); }
      x.setLineDash([]);
      ln.k.forEach((k, ki) => {
        const arr = TL[k] || []; x.strokeStyle = ki ? css("--accent") : css("--text"); x.lineWidth = 1.2; x.beginPath(); let on = false;
        for (let i = 0; i < N; i++) { const v = arr[i]; if (!isNum(v)) { on = false; continue; } const px = X(TL.t[i]), py = Y(v); on ? x.lineTo(px, py) : x.moveTo(px, py); on = true; }
        x.stroke();
      });
    });
    x.strokeStyle = css("--accent"); x.lineWidth = 1.5; x.beginPath(); x.moveTo(X(T), 0); x.lineTo(X(T), h); x.stroke();
    tcv._X = X; tcv._Lp = Lp; tcv._Rp = Rp;
  }
  function setT(t, from) {
    T = Math.max(0, Math.min(DUR, t)); $("#tm").textContent = mmss(T);
    if (from !== "rng") $("#rng").value = T;
    drawField(); drawTL();
  }
  const rng = $("#rng"); rng.max = DUR; rng.step = 0.1;
  rng.addEventListener("input", () => setT(+rng.value, "rng"));
  function tFromEv(ev) { const r = tcv.getBoundingClientRect(), px = ev.clientX - r.left, Lp = tcv._Lp || 0, Rp = tcv._Rp || 0; return (px - Lp) / Math.max(1, r.width - Lp - Rp) * DUR; }
  let drag = false;
  tcv.addEventListener("pointerdown", ev => { drag = true; tcv.setPointerCapture(ev.pointerId); setT(tFromEv(ev)); });
  tcv.addEventListener("pointermove", ev => { if (drag) setT(tFromEv(ev)); });
  tcv.addEventListener("pointerup", () => { drag = false; });
  const play = $("#play");
  function step(ts) { if (!playing) return; const dt = last ? (ts - last) / 1000 : 0; last = ts; setT(T + dt * speed); if (T >= DUR) { playing = false; play.textContent = "Play"; play.setAttribute("aria-pressed", "false"); return; } requestAnimationFrame(step); }
  play.addEventListener("click", () => { playing = !playing; play.textContent = playing ? "Pause" : "Play"; play.setAttribute("aria-pressed", String(playing)); if (playing) { if (T >= DUR) T = 0; last = 0; requestAnimationFrame(step); } });
  document.querySelectorAll("[data-s]").forEach(bt => bt.addEventListener("click", () => { speed = +bt.dataset.s; document.querySelectorAll("[data-s]").forEach(o => o.setAttribute("aria-pressed", String(o === bt))); }));

  /* ---------------------------------------------------------------- incidents */
  function evText(ev) {
    const s = ev.sig;
    if (s === "acc") return `acc ${isNum(ev.val) ? ev.val.toFixed(2) : "?"} g${ev.corroborated ? " + " + ev.corroborated.join(", ") : " alone"}`;
    if (s === "push") return `push ${isNum(ev.dur_s) ? ev.dur_s.toFixed(1) : "?"} s (${ev.how || ""})`;
    if (s === "lens_blocked") return `lens blocked ${isNum(ev.dur_s) ? ev.dur_s.toFixed(1) : "?"} s`;
    if (s === "near_wall") return `near wall f${(ev.frames || []).join("-")}`;
    if (s === "map") return `map ${isNum(ev.clr) ? ev.clr : "?"} mm ${ev.object || ""}`;
    if (s === "lidar") return `lidar ${isNum(ev.clr) ? ev.clr : "?"} mm at ${isNum(ev.bearing) ? ev.bearing : "?"}°`;
    if (s === "yaw_jerk") return `yaw jerk ${ev.val} °/s`;
    if (s === "lost") return `lost fit ${ev.fit ?? "?"}`;
    if (s === "relocalised" || s === "reloc_rejected" || s === "reloc_heading") return `${s} ${isNum(ev.cost) ? "cost " + ev.cost : ""}${isNum(ev.jump_mm) ? " jump " + ev.jump_mm + " mm" : ""}`;
    if (s === "guard") return `guard ×${ev.n} clr ${ev.clr_min ?? "?"} mm`;
    if (s === "shield") return `shield ${(ev.acts || []).join("/")} ×${ev.n}`;
    if (s === "loop_gap") return `loop ${ev.ms} ms`;
    if (s === "battery") return `battery ${ev.rest_v} → ${ev.min_v} V`;
    return Object.entries(ev).filter(([k]) => !["det", "t", "strong"].includes(k)).map(([k, v]) => `${k} ${typeof v === "object" ? JSON.stringify(v) : v}`).join(" ").slice(0, 60);
  }
  const box = $("#incs");
  $("#icap").textContent = `${INC.length} · tap one to scrub`;
  if (!INC.length) box.innerHTML = `<div class="empty">No incidents: nothing touched, nothing lost.</div>`;
  box.innerHTML += INC.map((inc, k) => {
    const w2 = inc.where || {}, ca = inc.cause || {}, dc = inc.decision || {};
    const place = [w2.object, w2.part, w2.section].filter(Boolean).map(esc).join(" · ");
    const pos = isNum(w2.x) ? ` <span class="u num">(${w2.x}, ${w2.y})&thinsp;mm${isNum(w2.th) ? ", " + w2.th.toFixed(1) + "°" : ""}${isNum(w2.sxy) ? " ±" + w2.sxy + "&thinsp;mm" : ""}</span>` : "";
    const th = (inc.frames || []).map((n, j) => TH[String(n)] ? `<figure style="margin:0"><img tabindex="0" data-f="${n}" data-t="${esc(inc.t)}" alt="frame ${n}" data-src-key="${n}"><figcaption class="tcap">f${n} · t−${(0.5 * j).toFixed(1)}&thinsp;s</figcaption></figure>` : "").join("");
    const evs = (inc.evidence || []).slice(0, 10).map(ev => `<span class="ev ${ev.strong ? "s" : ""}">${esc(evText(ev))}</span>`).join("");
    return `<div class="inc" tabindex="0" data-k="${k}" data-t="${inc.t}">
      <div class="n">#${inc.id}</div>
      <div style="display:flex;flex-direction:column;gap:5px;min-width:0">
        <div class="l1"><span class="chip ${KCLS[inc.kind] || "mut"}">${esc(inc.kind)}</span><span class="num">${isNum(inc.t) ? inc.t.toFixed(2) : "?"}<span class="u">&thinsp;s</span></span>${isNum(inc.dur_s) ? `<span class="u num">for ${inc.dur_s.toFixed(1)}&thinsp;s</span>` : ""}<span class="u">sev ${inc.sev}</span><span class="mono u">${esc(inc.state || "")}</span></div>
        <div class="where">${place || "<span class='u'>where: unknown</span>"}${pos}</div>
        ${dc.text ? `<div class="dec">${esc(dc.text)}</div>` : ""}
        <div class="why"><b>${esc(ca.code || "")}</b> ${esc(ca.text || "")}${(ca.also || []).length ? ` <span class="u">also ${(ca.also || []).map(esc).join(", ")}</span>` : ""}</div>
        ${(ca.sub || []).length ? `<div class="sub">${ca.sub.map(esc).join("<br>")}</div>` : ""}
        ${inc.fix_hint ? `<div class="fix"><span class="ml">Fix</span> ${esc(inc.fix_hint)}</div>` : ""}
        ${evs ? `<div class="evs">${evs}</div>` : ""}
        ${inc.truth ? `<div class="u" style="font-size:12px">sim truth: ${inc.truth.matched ? "matched at " + inc.truth.t + " s" : "no true contact near"}</div>` : ""}
        ${th ? `<div class="thumbs">${th}</div>` : ""}
      </div></div>`;
  }).join("");
  box.querySelectorAll("img[data-src-key]").forEach(im => { const u = TH[im.getAttribute("data-src-key")]; if (u && u.slice(0, 11) === "data:image/") im.setAttribute("src", u); });
  function selInc(k) { sel = k; box.querySelectorAll(".inc").forEach(d => d.classList.toggle("sel", +d.dataset.k === k)); const inc = INC[k]; if (inc && isNum(inc.t)) setT(inc.t); }
  box.addEventListener("click", ev => {
    const im = ev.target.closest("img[data-f]");
    if (im) { openDlg(im); return; }
    const d = ev.target.closest(".inc"); if (d) selInc(+d.dataset.k);
  });
  box.addEventListener("keydown", ev => { if (ev.key !== "Enter" && ev.key !== " ") return; const im = ev.target.closest("img[data-f]"); if (im) { ev.preventDefault(); openDlg(im); return; } const d = ev.target.closest(".inc"); if (d) { ev.preventDefault(); selInc(+d.dataset.k); } });
  const dlg = $("#dlg");
  function openDlg(im) { $("#dlgimg").setAttribute("src", im.getAttribute("src") || ""); $("#dlgcap").textContent = `frame ${im.dataset.f} · incident at ${im.dataset.t} s`; dlg.classList.add("on"); $("#dlgx").focus(); }
  function closeDlg() { dlg.classList.remove("on"); }
  $("#dlgx").addEventListener("click", closeDlg);
  dlg.addEventListener("click", ev => { if (ev.target === dlg) closeDlg(); });
  document.addEventListener("keydown", ev => { if (ev.key === "Escape") closeDlg(); });

  /* ---------------------------------------------------------------- footer */
  const src = R.sources || {}, thr = R.thresholds || {}, st = R.stats || {}, bat = st.battery || {}, lp = st.loop_ms || {}, ad = st.acc_dev_g || {};
  const dl = rows => `<dl class="kv">${rows.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${v}</dd>`).join("")}</dl>`;
  const det = R.detector;
  $("#ftr").innerHTML = `<div><div class="ml" style="margin-bottom:6px">Sources</div>${dl([["IMU", esc(src.imu)], ["Clock", esc(src.clock)], ["Recording", esc(src.recording || "none")],
      ["Frames", `${src.frames ?? 0} (${src.frames_analysed ?? 0} analysed)`], ["Scans", esc(src.scans ?? 0)], ["Pose", esc(src.pose || "none")],
      ["Camera", src.camera_verified ? "verified this session" : "<span style='color:var(--warn)'>NOT verified this session</span>"], ["Sim truth", src.truth ? "yes" : "no"]])}
      ${det ? `<div class="ml" style="margin:10px 0 6px">Detector vs truth</div>${dl([["True contacts", esc((det.truth_contacts || []).length)], ["Matched", esc(det.matched)], ["Recall", esc(det.recall ?? "—")], ["Precision", esc(det.precision ?? "—")], ["Missed", esc((det.missed || []).join(", ") || "none")]])}` : ""}</div>
    <div><div class="ml" style="margin-bottom:6px">Run numbers</div>${dl([["Battery rest / min / end", `${U(bat.rest_v, "V", 2)} / ${U(bat.min_v, "V", 2)} / ${U(bat.end_v, "V", 2)}`], ["Battery sag", U(bat.sag_v, "V", 2)],
      ["Loop p50 / p99", `${U(lp.p50, "ms")} / ${U(lp.p99, "ms")}`], ["Acc dev p99 / max", `${U(ad.p99, "g", 2)} / ${U(ad.max, "g", 2)}`],
      ["Min clearance", U((R.score || {}).min_clearance_mm, "mm")], ["Push total", U((R.score || {}).push_s, "s", 1)], ["Guard / shield brakes", `${esc((R.score || {}).guard_n ?? 0)} / ${esc((R.score || {}).shield_brakes ?? 0)}`],
      ["Analyzer", `${U((R.analyzer || {}).ms, "ms")} · v${esc((R.analyzer || {}).version)}`]])}
      <div class="ml" style="margin:10px 0 6px">Thresholds</div>${dl([["Contact / bump", `${U(thr.acc_contact_g, "g", 2)} / ${U(thr.acc_bump_g, "g", 2)}`], ["Near / touch", `${U(thr.near_mm, "mm")} / ${U(thr.touch_mm, "mm")}`],
      ["Push", `${U(thr.push_s, "s", 1)} at < ${esc(thr.push_ratio)} × cmd`], ["Merge", U(thr.merge_s, "s", 1)], ["Loop gap", U(thr.loop_gap_ms, "ms")], ["Pose jump / sxy bad", `${U(thr.pose_jump_mm, "mm")} / ${U(thr.sxy_bad_mm, "mm")}`]])}</div>
    <div><div class="ml" style="margin-bottom:6px">Fix hints</div>${(R.fix_hints || []).length ? `<ul class="hints">${R.fix_hints.map(h => `<li>${esc(h)}</li>`).join("")}</ul>` : `<div class="u">none</div>`}
      ${(R.anomalies || []).length ? `<div class="ml" style="margin:10px 0 6px">Anomalies</div><ul class="hints">${R.anomalies.map(a => `<li><span class="num">${isNum(a.t) ? a.t.toFixed(1) : "?"}–${isNum(a.t_end) ? a.t_end.toFixed(1) : "?"}&thinsp;s</span> ${esc(a.text)}</li>`).join("")}</ul>` : ""}
      ${(src.notes || []).length ? `<div class="ml" style="margin:10px 0 6px">Notes</div><div class="u">${src.notes.map(esc).join("<br>")}</div>` : ""}</div>`;

  window.addEventListener("resize", () => { drawField(); drawTL(); });
  const first = INC.findIndex(i => i.kind === "contact");
  if (first >= 0) selInc(first); else setT(0);
})();
</script>
</body>
</html>
"""
