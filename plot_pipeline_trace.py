#!/usr/bin/env python3
"""Render an rdk arm-streaming pipeline trace as a standalone HTML line graph.

The motion service's arm-streaming executor (services/motion/builtin) records a
pipelineTrace for each stream_start session and returns it (as JSON-shaped data) in the
"trace" field of stream_status/stream_flush/stream_abort DoCommand responses: an object
{samples, events, timings, velocities}. Save that field to a file, then run this script to
turn it into a self-contained HTML file (inlined data + a vanilla-canvas chart, no external
deps, opens offline in any browser) showing jointPositionsCh/armQ occupancy, call timings, and
arm velocity over the run.

Usage:
    python3 plot_pipeline_trace.py <trace.json> [out.html]

If out.html is omitted, writes pipeline_trace.html next to the input.
"""

import json
import os
import sys
import webbrowser


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    in_path = argv[1]
    out_path = argv[2] if len(argv) > 2 else os.path.join(
        os.path.dirname(os.path.abspath(in_path)), "pipeline_trace.html"
    )

    with open(in_path) as f:
        data = json.load(f)
    if not isinstance(data, dict):
        # The demo writes trace.json as `null` when the server returned no trace (e.g. a
        # viam-server built without the pipeline-tracing branch).
        print(f"{in_path}: no trace in file (server returned none? needs a tracing-enabled viam-server build)")
        return 1
    samples = data.get("samples", [])
    events = data.get("events", [])
    timings = data.get("timings", [])
    velocities = data.get("velocities", [])
    if not samples and not events and not timings and not velocities:
        print(f"{in_path}: no samples")
        return 1

    html = (HTML_TEMPLATE
            .replace("/*DATA*/", json.dumps(samples))
            .replace("/*EVENTS*/", json.dumps(events))
            .replace("/*TIMINGS*/", json.dumps(timings))
            .replace("/*VELOCITIES*/", json.dumps(velocities)))
    with open(out_path, "w") as f:
        f.write(html)
    abs_out = os.path.abspath(out_path)
    print(f"wrote {abs_out} ({len(samples)} samples, {len(events)} events, {len(timings)} timings, {len(velocities)} velocities) — opening in browser")
    webbrowser.open(f"file://{abs_out}")
    return 0


HTML_TEMPLATE = r"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>rdk arm-streaming pipeline trace</title>
<style>
  body { font: 13px -apple-system, system-ui, sans-serif; margin: 16px; background:#fafafa; }
  .wrap { position: relative; margin-bottom: 8px; }
  canvas { background:#fff; border:1px solid #ddd; }
  .tip { position:absolute; pointer-events:none; background:#000; color:#fff; padding:4px 6px;
         border-radius:4px; font-size:12px; display:none; white-space:pre; }
  .legend span { display:inline-block; margin-right:16px; }
  .sw { display:inline-block; width:12px; height:12px; vertical-align:middle; margin-right:4px; }
  h3 { margin: 10px 0 4px; }
</style>
</head>
<body>
<h3>joint positions received <small style="color:#888">(cumulative count of stream_push targets dequeued by the trajex session)</small></h3>
<div class="wrap"><canvas id="c9" width="1400" height="200"></canvas><div class="tip" id="tip9"></div></div>

<h3>jointPositionsCh depth <small style="color:#888">(stream_push &rarr; trajex producer)</small></h3>
<div class="wrap"><canvas id="c1" width="1400" height="200"></canvas><div class="tip" id="tip1"></div></div>

<h3>trajex-extend duration
  <span class="legend"><span><i class="sw" style="background:#ff7f0e"></i>trajex-extend</span></span>
  <small style="color:#888">(one trajexSession.addJointPositions call per pushed waypoint)</small></h3>
<div class="wrap"><canvas id="c2" width="1400" height="220"></canvas><div class="tip" id="tip2"></div></div>

<h3>trajex generation <small style="color:#888">(trajectory_generation_count after each Extend: increments on every pivot or rebase; flat = same trajectory)</small></h3>
<div class="wrap"><canvas id="c14" width="1400" height="180"></canvas><div class="tip" id="tip14"></div></div>

<h3>trajectory sent to arm per tick <small style="color:#888">(deficit filled each send interval, ms; only recorded at actual send events, not skipped ticks)</small></h3>
<div class="wrap"><canvas id="c10" width="1400" height="200"></canvas><div class="tip" id="tip10"></div></div>

<h3>send-to-arm latency <small style="color:#888">(one armStream.send call, i.e. one batch delivered to MoveThroughJointPositionsStreamed)</small></h3>
<div class="wrap"><canvas id="c4" width="1400" height="200"></canvas><div class="tip" id="tip4"></div></div>

<h3>arm buffer (estimated) <small style="color:#888">(currentEstimatedRunwayInArm in ms vs the target_runway_in_arm_ms runway; open-loop estimate)</small></h3>
<div class="wrap"><canvas id="c5" width="1400" height="200"></canvas><div class="tip" id="tip5"></div></div>

<h3>trajex runway <small style="color:#888">(active_duration − current_time in ms; unsampled trajectory inside trajex — fills on Extend, drains on sampleAtLeast; ~0 = staging window)</small></h3>
<div class="wrap"><canvas id="c15" width="1400" height="200"></canvas><div class="tip" id="tip15"></div></div>

<h3>arm velocity <small style="color:#888">(max |joint velocity| per PVAT, deg/s, from the trajex output)</small></h3>
<div class="wrap"><canvas id="c6" width="1400" height="200"></canvas><div class="tip" id="tip6"></div></div>

<h3>per-joint velocity <small style="color:#888">(deg/s per joint, signed -- which joint was moving fastest right before a fault)</small>
  <span class="legend" id="legend7"></span></h3>
<div class="wrap"><canvas id="c7" width="1400" height="200"></canvas><div class="tip" id="tip7"></div></div>

<h3>per-joint position <small style="color:#888">(deg per joint -- where each joint was at the same moment)</small>
  <span class="legend" id="legend8"></span></h3>
<div class="wrap"><canvas id="c8" width="1400" height="200"></canvas><div class="tip" id="tip8"></div></div>

<h3>events <small style="color:#888">(producer trajex-session vs consumer arm-stream lifecycle; gap = buffer latency)</small></h3>
<div class="wrap"><canvas id="c0" width="1400" height="180"></canvas><div class="tip" id="tip0"></div></div>

<h2 style="margin-top:24px;border-top:1px solid #ddd;padding-top:16px">sanding planning (client-side)</h2>

<h3>planQ depth <small style="color:#888">(IK producer &rarr; stream_push sender; fills if IK is ahead of the push consumer)</small></h3>
<div class="wrap"><canvas id="c11" width="1400" height="200"></canvas><div class="tip" id="tip11"></div></div>

<h3>ik-solve duration <small style="color:#888">(one planToInterpolatedPose call per coverage point: IK + trajectory planning)</small></h3>
<div class="wrap"><canvas id="c12" width="1400" height="200"></canvas><div class="tip" id="tip12"></div></div>

<h3>adjust-offset duration <small style="color:#888">(load-cell read + Z-offset computation before each IK solve)</small></h3>
<div class="wrap"><canvas id="c13" width="1400" height="200"></canvas><div class="tip" id="tip13"></div></div>
<script>
const SAMPLES = /*DATA*/;
const EVENTS = /*EVENTS*/;
const TIMINGS = /*TIMINGS*/;
const VELOCITIES = /*VELOCITIES*/;
const BLUE = "#1f77b4", ORANGE = "#ff7f0e", PURPLE = "#9467bd", YELLOW = "#eab308", CYAN = "#17becf", GREEN = "#2ca02c", RED = "#d62728";

// Shared time axis across all charts.
// Use reduce instead of spread so large traces don't hit the browser's argument-count limit (~65k).
const arrMax = (arr, fn, init=0) => arr.reduce((m, x) => Math.max(m, fn(x)), init);
const tMax = Math.max(1, arrMax(SAMPLES, s => s.t_ms), arrMax(EVENTS, e => e.t_ms), arrMax(TIMINGS, x => x.t_ms), arrMax(VELOCITIES, v => v.t_ms));

// Shared hover cursor: the time under the mouse on any chart. Every chart/timeline registers its
// redraw fn in `redrawers`, so moving over one plot draws the vertical line on all of them at the same t.
let cursorTime = null;
const redrawers = [];

// Queue occupancy: per-channel [t_ms, len]; cap is constant per channel.
const qseries = { jointPositionsCh: [], armQ: [] }, caps = { jointPositionsCh: 0, armQ: 0 };
for (const s of SAMPLES) { if (!qseries[s.ch]) continue; qseries[s.ch].push([s.t_ms, s.len]); caps[s.ch] = s.cap; }

// planQ: sanding IK producer → stream_push sender (client-side buffer).
const planQpts = [], planQcaps = [];
for (const s of SAMPLES) { if (s.ch !== "planQ") continue; planQpts.push([s.t_ms, s.len]); planQcaps.push(s.cap); }
const planQcap = planQcaps.length ? planQcaps[planQcaps.length - 1] : 1;

// Joint positions received: one point per stream_push target dequeued by the trajex session --
// each jointPositionsCh "deq" sample is exactly one arrival, so a running count over them shows
// the rate the client is pushing at, and flattens the moment it stops (even though the arm may
// keep executing the already-received trajectory for much longer).
const receivedPts = SAMPLES
  .filter(s => s.ch === "jointPositionsCh" && s.op === "deq")
  .map((s, i) => [s.t_ms, i + 1]);
const receivedMax = Math.max(1, arrMax(receivedPts, p => p[1]));

// Trajectory duration sent to arm per tick: deficit handed to sampleAtLeast at each send event.
// Only recorded at actual sends (skipped ticks where buffer was full are absent). Empty on older traces.
const trajSentPts = TIMINGS.filter(x => x.kind === "traj-sent").map(x => [x.t_ms, x.ms]);
const trajSentMax = Math.max(1, caps.armQ || 1, arrMax(trajSentPts, p => p[1]));

// Trajex generation: generation_count after each Extend. Flat = same active trajectory;
// a step up = pivot (or rebase). Empty on older traces.
const trajGenPts = SAMPLES.filter(s => s.ch === "trajex-gen").map(s => [s.t_ms, s.len]);
const trajGenMax = Math.max(1, arrMax(trajGenPts, p => p[1]));

// Trajex runway: ActiveDuration − CurrentTime in ms — unsampled trajectory still inside trajex.
// Recorded after each Extend (enq, runway fills) and after each sampleAtLeast (deq, runway drains).
// Drops to ~0 during the staging window (arm has caught up to the trajectory horizon).
const trajexRunwayPts = SAMPLES.filter(s => s.ch === "trajex-runway").map(s => [s.t_ms, s.len]);
const trajexRunwayMax = Math.max(1, arrMax(trajexRunwayPts, p => p[1]));

// Step durations: per-kind [t_ms, ms].
const tseries = { "trajex-extend": [], "send-point": [] };
const tsandingSeries = { "ik-solve": [], "adjust-offset": [] };
for (const x of TIMINGS) {
  if (tseries[x.kind]) tseries[x.kind].push([x.t_ms, x.ms]);
  if (tsandingSeries[x.kind]) tsandingSeries[x.kind].push([x.t_ms, x.ms]);
}
// Arm velocity: [t_ms, deg/s] from the trajex output.
const vseries = VELOCITIES.map(v => [v.t_ms, v.deg_per_sec]);
const velMax = Math.max(1, arrMax(vseries, p => p[1]));
const exMax = Math.max(0.1, arrMax(tseries["trajex-extend"], p => p[1]));
const sendMax = Math.max(0.1, arrMax(tseries["send-point"], p => p[1]));
const ikMax = Math.max(0.1, arrMax(tsandingSeries["ik-solve"], p => p[1]));
const aoMax = Math.max(0.1, arrMax(tsandingSeries["adjust-offset"], p => p[1]));

// Per-joint velocity/position: one signed series per joint index, so a fault right before a
// trajectory rejection can be pinned on a specific joint instead of just "some joint, somewhere".
// Older traces predating per-joint tracking simply have no joint_deg_per_sec/joint_positions_deg
// fields, so these charts render empty rather than erroring.
const JOINT_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#7f7f7f"];
const numJoints = arrMax(VELOCITIES, v => (v.joint_deg_per_sec || []).length);
const jointColor = j => JOINT_COLORS[j % JOINT_COLORS.length];
const jointVelSeries = [], jointPosSeries = [];
for (let j = 0; j < numJoints; j++) {
  jointVelSeries.push({
    name: "j" + j, color: jointColor(j),
    pts: VELOCITIES.filter(v => v.joint_deg_per_sec).map(v => [v.t_ms, v.joint_deg_per_sec[j]]),
    fmt: v => v == null ? "-" : v.toFixed(1) + "°/s",
  });
  jointPosSeries.push({
    name: "j" + j, color: jointColor(j),
    pts: VELOCITIES.filter(v => v.joint_positions_deg).map(v => [v.t_ms, v.joint_positions_deg[j]]),
    fmt: v => v == null ? "-" : v.toFixed(1) + "°",
  });
}
const jointVelAbsMax = Math.max(1, ...jointVelSeries.flatMap(s => s.pts.map(p => Math.abs(p[1]))));
const jointPosVals = jointPosSeries.flatMap(s => s.pts.map(p => p[1]));
const jointPosMin = jointPosVals.length ? Math.min(0, ...jointPosVals) : 0;
const jointPosMax = jointPosVals.length ? Math.max(1, ...jointPosVals) : 1;

// Event timeline lanes (one row per kind).
const EVENT_LANES = [
  { kind: "plan-line", label: "plan-line", color: "#8c564b", text: true },
  { kind: "trajex-session-open", label: "session open", color: "#aef5ae", text: false },
  { kind: "trajex-session-close", label: "session close", color: "#0a4d0a", text: false },
  { kind: "stream-open", label: "stream open", color: "#b8d4fb", text: false },
  { kind: "stream-close", label: "stream close", color: "#0a2472", text: false },
  { kind: "stream-died", label: "stream died", color: "#9467bd", text: false },
];

function niceStep(span, target) {
  const raw = (span / target) || 1, mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const n = raw / mag;
  return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
}
function valAt(pts, t) { let v = null; for (const p of pts) { if (p[0] <= t) v = p[1]; else break; } return v; }

// shared x grid + time labels, used by every chart so they line up vertically.
function drawTimeAxis(ctx, px, M, W, H) {
  ctx.fillStyle = "#666"; ctx.textAlign = "center"; ctx.textBaseline = "top";
  for (let t = 0; t <= tMax; t += niceStep(tMax, 10)) {
    const x = px(t);
    ctx.strokeStyle = "#f3f3f3"; ctx.beginPath(); ctx.moveTo(x, M.t); ctx.lineTo(x, H - M.b); ctx.stroke();
    ctx.fillText((t / 1000).toFixed(1) + "s", x, H - M.b + 5);
  }
}

// makeChart draws one time-series scatter chart (shared x = time) and wires a hover crosshair/tooltip.
// cfg.yMin defaults to 0 (the original charts are all non-negative); pass it explicitly for
// signed data like per-joint velocity/position, which can go below zero.
function makeChart(canvasId, tipId, cfg) {
  const cv = document.getElementById(canvasId), ctx = cv.getContext("2d"), tip = document.getElementById(tipId);
  const M = { l: 100, r: 20, t: 16, b: 34 }, W = cv.width, H = cv.height;
  const yMin = cfg.yMin || 0;
  const px = t => M.l + (t / tMax) * (W - M.l - M.r);
  const py = v => H - M.b - ((v - yMin) / (cfg.yMax - yMin)) * (H - M.t - M.b);
  function draw() {
    ctx.clearRect(0, 0, W, H);
    // y grid + labels
    ctx.fillStyle = "#666"; ctx.textAlign = "right"; ctx.textBaseline = "middle";
    const yStep = niceStep(cfg.yMax - yMin, 5);
    for (let v = Math.ceil(yMin / yStep) * yStep; v <= cfg.yMax + 1e-9; v += yStep) {
      const y = py(v);
      ctx.strokeStyle = "#eee"; ctx.beginPath(); ctx.moveTo(M.l, y); ctx.lineTo(W - M.r, y); ctx.stroke();
      ctx.fillText(cfg.yFmt(v), M.l - 6, y);
    }
    if (yMin < 0 && cfg.yMax > 0) {
      // zero line, so signed charts read at a glance without counting grid lines.
      ctx.strokeStyle = "#ccc"; ctx.beginPath(); ctx.moveTo(M.l, py(0)); ctx.lineTo(W - M.r, py(0)); ctx.stroke();
    }
    drawTimeAxis(ctx, px, M, W, H);
    for (const c of cfg.caps || []) {
      if (!c.v) continue;
      ctx.strokeStyle = c.color; ctx.setLineDash([5, 4]); ctx.globalAlpha = 0.5;
      ctx.beginPath(); ctx.moveTo(M.l, py(c.v)); ctx.lineTo(W - M.r, py(c.v)); ctx.stroke();
      ctx.setLineDash([]); ctx.globalAlpha = 1;
    }
    // scatter: one dot per sample, one color per series
    for (const s of cfg.series) {
      ctx.fillStyle = s.color;
      for (const p of s.pts) { ctx.beginPath(); ctx.arc(px(p[0]), py(p[1]), 1.5, 0, 2 * Math.PI); ctx.fill(); }
    }
    if (cursorTime != null) {
      const cx = px(cursorTime);
      ctx.strokeStyle = "#999"; ctx.beginPath(); ctx.moveTo(cx, M.t); ctx.lineTo(cx, H - M.b); ctx.stroke();
    }
  }
  redrawers.push(draw);
  cv.addEventListener("mousemove", e => {
    const r = cv.getBoundingClientRect(), x = (e.clientX - r.left) * (W / r.width);
    if (x < M.l || x > W - M.r) { tip.style.display = "none"; cursorTime = null; redrawers.forEach(f => f()); return; }
    const t = ((x - M.l) / (W - M.l - M.r)) * tMax;
    cursorTime = t; redrawers.forEach(f => f());
    tip.style.display = "block";
    tip.style.left = (e.clientX - r.left + 12) + "px";
    tip.style.top = (e.clientY - r.top + 12) + "px";
    tip.textContent = "t=" + (t / 1000).toFixed(2) + "s\n" +
      cfg.series.map(s => s.name + "=" + s.fmt(valAt(s.pts, t))).join("\n");
  });
  cv.addEventListener("mouseleave", () => { tip.style.display = "none"; cursorTime = null; redrawers.forEach(f => f()); });
  draw();
}

// makeTimeline draws the event swimlanes: one row per kind, a dot per event. Shares the x time
// axis with the charts below.
function makeTimeline(canvasId, tipId, lanes) {
  const cv = document.getElementById(canvasId), ctx = cv.getContext("2d"), tip = document.getElementById(tipId);
  const M = { l: 100, r: 20, t: 16, b: 34 }, W = cv.width, H = cv.height;
  const px = t => M.l + (t / tMax) * (W - M.l - M.r);
  const rowY = i => M.t + (i + 0.5) * (H - M.t - M.b) / lanes.length;
  function draw() {
    ctx.clearRect(0, 0, W, H);
    drawTimeAxis(ctx, px, M, W, H);
    ctx.font = "10px monospace";
    lanes.forEach((lane, i) => {
      const y = rowY(i);
      ctx.strokeStyle = "#eee"; ctx.beginPath(); ctx.moveTo(M.l, y); ctx.lineTo(W - M.r, y); ctx.stroke();
      ctx.fillStyle = lane.color; ctx.textAlign = "right"; ctx.textBaseline = "middle";
      ctx.fillText(lane.label, M.l - 6, y);
      ctx.textAlign = "left";
      for (const e of EVENTS) {
        if (e.kind !== lane.kind) continue;
        const x = px(e.t_ms);
        ctx.beginPath(); ctx.arc(x, y, 2.5, 0, 2 * Math.PI); ctx.fill();
        if (lane.text && e.label) ctx.fillText(e.label.replace("line ", ""), x + 4, y);
      }
    });
    if (cursorTime != null) {
      const cx = px(cursorTime);
      ctx.strokeStyle = "#999"; ctx.beginPath(); ctx.moveTo(cx, M.t); ctx.lineTo(cx, H - M.b); ctx.stroke();
    }
  }
  redrawers.push(draw);
  cv.addEventListener("mousemove", e => {
    const r = cv.getBoundingClientRect(), x = (e.clientX - r.left) * (W / r.width);
    if (x < M.l || x > W - M.r) { tip.style.display = "none"; cursorTime = null; redrawers.forEach(f => f()); return; }
    const t = ((x - M.l) / (W - M.l - M.r)) * tMax;
    cursorTime = t; redrawers.forEach(f => f());
    const tol = tMax * 0.01;
    const near = EVENTS.filter(e => Math.abs(e.t_ms - t) <= tol).sort((a, b) => a.t_ms - b.t_ms);
    tip.style.display = "block";
    tip.style.left = (e.clientX - r.left + 12) + "px";
    tip.style.top = (e.clientY - r.top + 12) + "px";
    tip.textContent = "t=" + (t / 1000).toFixed(2) + "s" +
      (near.length ? "\n" + near.map(e => e.kind + (e.label ? " " + e.label : "")).join("\n") : "");
  });
  cv.addEventListener("mouseleave", () => { tip.style.display = "none"; cursorTime = null; redrawers.forEach(f => f()); });
  draw();
}

// Headroom above the data max so the top of the plot has a little breathing room.
const HEAD = 1.1;
const msFmt = v => v == null ? "-" : v.toFixed(1) + "ms";
makeTimeline("c0", "tip0", EVENT_LANES);
makeChart("c9", "tip9", {
  yMax: receivedMax * HEAD, yFmt: v => String(Math.round(v)),
  series: [{ name: "received", color: BLUE, pts: receivedPts, fmt: v => v == null ? "-" : String(v) }],
});
makeChart("c14", "tip14", {
  yMax: (trajGenMax + 1) * HEAD, yFmt: v => String(Math.round(v)),
  series: [{ name: "generation", color: CYAN, pts: trajGenPts, fmt: v => v == null ? "-" : String(Math.round(v)) }],
});
makeChart("c10", "tip10", {
  yMax: trajSentMax * HEAD, yFmt: v => Math.round(v) + "ms",
  caps: [{ v: caps.armQ, color: GREEN }],
  series: [{ name: "traj sent", color: GREEN, pts: trajSentPts, fmt: v => v == null ? "-" : Math.round(v) + "ms" }],
});
makeChart("c1", "tip1", {
  yMax: Math.max(1, caps.jointPositionsCh) * HEAD, yFmt: v => String(Math.round(v)),
  caps: [{ v: caps.jointPositionsCh, color: PURPLE }],
  series: [{ name: "jointPositionsCh", color: PURPLE, pts: qseries.jointPositionsCh, fmt: v => v == null ? "-" : v + "/" + caps.jointPositionsCh }],
});
makeChart("c2", "tip2", {
  yMax: exMax * HEAD, yFmt: v => v.toFixed(0) + "ms",
  series: [
    { name: "trajex-extend", color: ORANGE, pts: tseries["trajex-extend"], fmt: msFmt },
  ],
});
makeChart("c5", "tip5", {
  yMax: Math.max(1, caps.armQ, ...qseries.armQ.map(p => p[1])) * HEAD, yFmt: v => Math.round(v) + "ms",
  caps: [{ v: caps.armQ, color: GREEN }],
  series: [{ name: "arm buffer (est)", color: GREEN, pts: qseries.armQ, fmt: v => v == null ? "-" : v + "/" + caps.armQ + "ms" }],
});
makeChart("c15", "tip15", {
  yMax: trajexRunwayMax * HEAD, yFmt: v => Math.round(v) + "ms",
  series: [{ name: "trajex runway", color: RED, pts: trajexRunwayPts, fmt: v => v == null ? "-" : Math.round(v) + "ms" }],
});
makeChart("c6", "tip6", {
  yMax: velMax * HEAD, yFmt: v => v.toFixed(1) + "°/s",
  series: [{ name: "max joint vel", color: PURPLE, pts: vseries, fmt: v => v == null ? "-" : v.toFixed(1) + "°/s" }],
});
for (let j = 0; j < numJoints; j++) {
  const sw = document.createElement("span");
  sw.innerHTML = `<i class="sw" style="background:${jointColor(j)}"></i>j${j}`;
  document.getElementById("legend7").appendChild(sw.cloneNode(true));
  document.getElementById("legend8").appendChild(sw.cloneNode(true));
}
makeChart("c7", "tip7", {
  yMin: -jointVelAbsMax * HEAD, yMax: jointVelAbsMax * HEAD, yFmt: v => v.toFixed(1) + "°/s",
  series: jointVelSeries,
});
makeChart("c8", "tip8", {
  yMin: jointPosMin - 0.1 * (jointPosMax - jointPosMin), yMax: jointPosMax + 0.1 * (jointPosMax - jointPosMin),
  yFmt: v => v.toFixed(0) + "°",
  series: jointPosSeries,
});
makeChart("c4", "tip4", {
  yMax: sendMax * HEAD, yFmt: v => v.toFixed(1) + "ms",
  series: [
    { name: "send-point", color: CYAN, pts: tseries["send-point"], fmt: msFmt },
  ],
});
makeChart("c11", "tip11", {
  yMax: Math.max(1, planQcap) * HEAD, yFmt: v => String(Math.round(v)),
  caps: [{ v: planQcap, color: ORANGE }],
  series: [{ name: "planQ", color: ORANGE, pts: planQpts, fmt: v => v == null ? "-" : v + "/" + planQcap }],
});
makeChart("c12", "tip12", {
  yMax: ikMax * HEAD, yFmt: v => v.toFixed(0) + "ms",
  series: [{ name: "ik-solve", color: BLUE, pts: tsandingSeries["ik-solve"], fmt: msFmt }],
});
makeChart("c13", "tip13", {
  yMax: aoMax * HEAD, yFmt: v => v.toFixed(1) + "ms",
  series: [{ name: "adjust-offset", color: YELLOW, pts: tsandingSeries["adjust-offset"], fmt: msFmt }],
});
</script>
</body>
</html>
"""


if __name__ == "__main__":
    sys.exit(main(sys.argv))
