// Speed profile design.
//
// Mode A (fixed gates, re-plan speed): geometry is locked. Curvature, climb
// rate and per-gate speed limits define the largest feasible speed at every
// station; accel/decel limits are then propagated both ways so the aircraft
// can actually reach each target. Changing any speed target therefore moves
// the acceleration (and thus the guidance points) before AND after it.
//
// Mode B (locked speed, reshape curve) lives in reshape.js and consumes the
// per-station target speeds produced here as hard requirements.

import { clamp } from './math.js';

function gateLimitAt(s, gates, field) {
  // Piecewise-linear envelope between gate crossing stations.
  const pts = gates
    .filter((g) => g._crossS != null && g[field] != null)
    .map((g) => ({ s: g._crossS, v: g[field] }))
    .sort((a, b) => a.s - b.s);
  if (!pts.length) return null;
  if (s <= pts[0].s) return pts[0].v;
  if (s >= pts[pts.length - 1].s) return pts[pts.length - 1].v;
  for (let i = 0; i < pts.length - 1; i++) {
    if (s >= pts[i].s && s <= pts[i + 1].s) {
      const t = (s - pts[i].s) / (pts[i + 1].s - pts[i].s);
      return pts[i].v + (pts[i + 1].v - pts[i].v) * t;
    }
  }
  return null;
}

// Largest dynamically feasible speed at one station.
export function speedCeilingAt(sample, aircraft, gates) {
  const g = aircraft.g ?? 9.81;
  let v = aircraft.vMax;
  // Turn: v <= sqrt(g tan phiMax / kappa).
  if (sample.curvature > 1e-6) {
    v = Math.min(v, Math.sqrt((g * Math.tan(aircraft.phiMax)) / sample.curvature));
  }
  // Climb angle (energy/geometry limit already implicit); climb rate:
  // |v sin gamma| <= vzMax  =>  v <= vzMax / |sin gamma|.
  const sg = Math.abs(Math.sin(sample.climbAngle));
  if (sg > 1e-4) v = Math.min(v, aircraft.vzMax / sg);
  // Gate speed windows.
  const vMaxG = gateLimitAt(sample.s, gates, 'vMax');
  const vMinG = gateLimitAt(sample.s, gates, 'vMin');
  if (vMaxG != null) v = Math.min(v, vMaxG);
  v = Math.max(v, aircraft.vMin);
  return { ceiling: v, floor: Math.max(aircraft.vMin, vMinG ?? aircraft.vMin) };
}

// Forward/backward accel-limited propagation on an arc-length grid.
function propagate(speeds, samples, aircraft) {
  const aMax = aircraft.aMax;          // longitudinal accel magnitude
  const aMin = aircraft.aDecel;        // braking magnitude (positive number)
  // Forward pass: limit acceleration leaving each station.
  for (let i = 1; i < speeds.length; i++) {
    const ds = samples[i].s - samples[i - 1].s;
    const vPrev = speeds[i - 1];
    const reach = Math.sqrt(vPrev * vPrev + 2 * aMax * ds);
    speeds[i] = Math.min(speeds[i], reach);
  }
  // Backward pass: ensure we can slow down in time.
  for (let i = speeds.length - 2; i >= 0; i--) {
    const ds = samples[i + 1].s - samples[i].s;
    const vNext = speeds[i + 1];
    const need = Math.sqrt(vNext * vNext + 2 * aMin * ds);
    speeds[i] = Math.min(speeds[i], need);
  }
  return speeds;
}

// Mode A: build a full feasible speed profile for fixed geometry.
export function planSpeedProfile(samples, aircraft, gates, cruiseFactor = 0.95) {
  const target = samples.map((smp) => {
    const { ceiling, floor } = speedCeilingAt(smp, aircraft, gates);
    return { ceiling: Math.max(ceiling, floor), floor };
  });
  // Desired speed honours every gate vMin/vMax window and the cruise margin.
  const requested = target.map((t) =>
    clamp(t.ceiling * cruiseFactor, t.floor, t.ceiling));
  // Pin the exact crossing station to each gate's hard speed window so a tight
  // gate cannot be diluted by interpolation across a long leg.
  for (const g of gates) {
    if (g._crossS == null) continue;
    let gi = 0, gd = Infinity;
    samples.forEach((smp, i) => {
      const d = Math.abs(smp.s - g._crossS);
      if (d < gd) { gd = d; gi = i; }
    });
    if (g.vMax != null) requested[gi] = Math.min(requested[gi], g.vMax);
    if (g.vMin != null) requested[gi] = Math.max(requested[gi], g.vMin);
    requested[gi] = clamp(requested[gi], aircraft.vMin, aircraft.vMax);
  }
  // One physically reachable pass under accel/decel limits. Floors that the
  // propagation pulls down are genuinely unreachable, not quietly re-raised.
  const speeds = requested.slice();
  propagate(speeds, samples, aircraft);
  const conflicts = [];
  for (let i = 0; i < samples.length; i++) {
    if (requested[i] - speeds[i] > 0.15) {
      conflicts.push({
        type: 'speed-window-conflict', severity: 'error', s: samples[i].s,
        message: `s=${samples[i].s.toFixed(0)}m 处门速下限 ${requested[i].toFixed(1)}m/s 在加减速限制下不可达（最多 ${speeds[i].toFixed(1)}m/s）`,
      });
    }
  }
  // Guidance points: look-ahead distance tied to current speed (time headway).
  const headway = aircraft.guidanceHeadway ?? 2.5;
  const guidance = speeds.map((v) => 0).map((_, i) => ({
    s: samples[i].s + speeds[i] * headway,
    speed: speeds[i],
  }));
  return {
    speeds,
    targetCeilings: target.map((t) => t.ceiling),
    guidance,
    conflicts,
    headway,
  };
}

// Re-plan after a speed edit at one station: recompute ceilings then propagate
// both directions, returning the guidance points that moved. This is the
// explicit "回算前后引导点" behaviour used by the editor's speed handle.
export function replanAtStation(samples, aircraft, gates, editIndex, editSpeed) {
  const plan = planSpeedProfile(samples, aircraft, gates);
  plan.speeds[editIndex] = clamp(editSpeed, aircraft.vMin, aircraft.vMax);
  propagate(plan.speeds, samples, aircraft);
  const headway = aircraft.guidanceHeadway ?? 2.5;
  plan.guidance = plan.speeds.map((v) => ({ s: 0, speed: v }));
  for (let i = 0; i < samples.length; i++) {
    plan.guidance[i] = { s: samples[i].s + plan.speeds[i] * headway, speed: plan.speeds[i] };
  }
  plan.editedStation = editIndex;
  return plan;
}
