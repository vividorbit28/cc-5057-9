// Reachable gate time windows and gate-opening scheduling.
//
// The curve solver returns, for every gate, the interval [tMin, tMax] in which
// the aircraft can physically arrive given its turn/climb envelope and the
// accel/decel budget. A level's gate-opening scheduler then allocates a passage
// window inside each gate's reachable interval and its own open/close time.
//
// Both directions are derived from the geometry only: no hand-tuned arrival
// times. tMin comes from the fastest feasible speed profile (ceilings are
// curvature/climb/gate speed limits), tMax from the slowest feasible profile
// (floors are vMin and gate vMin walls). Accel/decel propagation makes the two
// profiles physically reachable rather than a per-station wish list.

import { clamp } from './math.js';
import { speedCeilingAt } from './speed.js';

const EPS = 1e-9;

// Time at each station for a given speed table, using trapezoidal ds/v.
function arrivalTimes(samples, speeds) {
  const t = new Array(samples.length).fill(0);
  for (let i = 1; i < samples.length; i++) {
    const ds = samples[i].s - samples[i - 1].s;
    const vAvg = Math.max((speeds[i - 1] + speeds[i]) * 0.5, 1e-6);
    t[i] = t[i - 1] + ds / vAvg;
  }
  return t;
}

// Fastest profile: spend every metre at the highest speed the local envelope
// allows, limited by how quickly the aircraft can accelerate (forward) and
// still brake for a lower ceiling downstream (backward).
function fastestSpeeds(samples, aircraft, ceilings, floors) {
  const aMax = aircraft.aMax;
  const aDecel = aircraft.aDecel;
  const v = ceilings.slice();
  for (let i = 1; i < v.length; i++) {
    const ds = samples[i].s - samples[i - 1].s;
    v[i] = Math.min(v[i], Math.sqrt(v[i - 1] * v[i - 1] + 2 * aMax * ds));
  }
  for (let i = v.length - 2; i >= 0; i--) {
    const ds = samples[i + 1].s - samples[i].s;
    v[i] = Math.min(v[i], Math.sqrt(v[i + 1] * v[i + 1] + 2 * aDecel * ds));
  }
  let feasible = true;
  for (let i = 0; i < v.length; i++) {
    if (v[i] < floors[i] - 1e-3) feasible = false;
    v[i] = Math.max(v[i], Math.min(floors[i], ceilings[i]));
  }
  return { speeds: v, feasible };
}

// Slowest profile: stay on (or near) the floor, but stay fast enough to
// accelerate to any higher floor downstream (backward) and respect how
// quickly the aircraft can bleed speed from the previous station (forward).
function slowestSpeeds(samples, aircraft, ceilings, floors) {
  const aMax = aircraft.aMax;
  const aDecel = aircraft.aDecel;
  const v = floors.slice();
  for (let i = v.length - 2; i >= 0; i--) {
    const ds = samples[i + 1].s - samples[i].s;
    const need = Math.max(v[i + 1] * v[i + 1] - 2 * aMax * ds, 0);
    v[i] = Math.max(v[i], Math.sqrt(need));
  }
  for (let i = 1; i < v.length; i++) {
    const ds = samples[i].s - samples[i - 1].s;
    const brakeFloor = Math.max(v[i - 1] * v[i - 1] - 2 * aDecel * ds, 0);
    v[i] = Math.max(v[i], Math.sqrt(brakeFloor));
  }
  let feasible = true;
  for (let i = 0; i < v.length; i++) {
    if (v[i] > ceilings[i] + 1e-3) feasible = false;
    v[i] = clamp(v[i], Math.min(floors[i], ceilings[i]), Math.max(ceilings[i], floors[i]));
  }
  return { speeds: v, feasible };
}

function nearestStationIndex(samples, s) {
  let bi = 0, bd = Infinity;
  for (let i = 0; i < samples.length; i++) {
    const d = Math.abs(samples[i].s - s);
    if (d < bd) { bd = d; bi = i; }
  }
  return bi;
}

// Reachable arrival-time interval per gate plus the fastest/slowest gap bounds
// between consecutive gates. Returns { feasible, reason, profiles, windows }.
export function reachableGateWindows({ samples, gates, aircraft }) {
  if (!samples?.length || !gates?.length) {
    return { feasible: true, reason: null, windows: [], profiles: null };
  }
  const ceilings = [];
  const floors = [];
  for (const smp of samples) {
    const { ceiling, floor } = speedCeilingAt(smp, aircraft, gates);
    ceilings.push(ceiling);
    floors.push(floor);
  }
  const fast = fastestSpeeds(samples, aircraft, ceilings, floors);
  const slow = slowestSpeeds(samples, aircraft, ceilings, floors);
  const tFast = arrivalTimes(samples, fast.speeds);
  const tSlow = arrivalTimes(samples, slow.speeds);

  const windows = gates.map((g) => {
    const s = g._crossS != null ? g._crossS : samples[0].s;
    const idx = nearestStationIndex(samples, s);
    return {
      gateId: g.id,
      label: g.label ?? g.id,
      s: samples[idx].s,
      stationIndex: idx,
      tMin: tFast[idx],
      tMax: tSlow[idx],
      reachable: fast.feasible && slow.feasible,
    };
  });
  for (let i = 0; i < windows.length - 1; i++) {
    windows[i].gapMinToNext = Math.max(windows[i + 1].tMin - windows[i].tMin, 0);
    windows[i].gapMaxToNext = Math.max(windows[i + 1].tMax - windows[i].tMax, 0);
  }
  const reason = !fast.feasible
    ? '最快航速剖面无法同时满足门速下限（几何或加减速能力不足）'
    : !slow.feasible
      ? '最慢航速剖面无法同时满足曲率/爬升上限（不可飞几何）'
      : null;
  return {
    feasible: fast.feasible && slow.feasible,
    reason,
    windows,
    profiles: { fast: fast.speeds, slow: slow.speeds, tFast, tSlow },
  };
}

// Allocate a passage window per gate. Earliest-feasible forward pass against
// the reachable interval, the gate's own open/close time and the fastest gap;
// then a backward pass to shrink each window to what is still feasible.
export function scheduleGateWindows(windows, gateOpen = {}) {
  const n = windows.length;
  if (!n) return { feasible: true, allocations: [], rejectedLeg: null };
  const openOf = (w) => gateOpen[w.gateId] ?? {};
  const lo = new Array(n);
  const hi = new Array(n);
  let rejectedLeg = null;

  for (let i = 0; i < n; i++) {
    const w = windows[i];
    const op = openOf(w);
    const open = op.open ?? -Infinity;
    let t = Math.max(w.tMin, open);
    if (i > 0) t = Math.max(t, lo[i - 1] + (windows[i - 1].gapMinToNext ?? 0));
    lo[i] = t;
    const upper = Math.min(w.tMax, op.close ?? Infinity);
    if (t > upper + 1e-9) {
      rejectedLeg = {
        index: i,
        from: i > 0 ? windows[i - 1].gateId : null,
        to: w.gateId,
        requiredBy: t,
        latestReachable: upper,
        reason: `门 ${w.label} 的最早可达 ${t.toFixed(2)}s 晚于开放窗截止 ` +
          `${upper.toFixed(2)}s（航段 ${i > 0 ? windows[i - 1].label + '→' : ''}${w.label} 需重拟合）`,
        kind: t > w.tMax + 1e-9 ? 'too-late' : 'window',
      };
      return { feasible: false, allocations: partial(lo, windows, gateOpen, i), rejectedLeg };
    }
    if (i > 0) {
      const gapMax = windows[i - 1].gapMaxToNext ?? Infinity;
      if (t > lo[i - 1] + gapMax + 1e-9) {
        rejectedLeg = {
          index: i,
          from: windows[i - 1].gateId,
          to: w.gateId,
          requiredBy: t,
          latestReachable: lo[i - 1] + gapMax,
          reason: `门 ${w.label} 最早到达 ${t.toFixed(2)}s 与上一门间隔超过最慢航段 ` +
            `${gapMax.toFixed(2)}s（航段 ${windows[i - 1].label}→${w.label} 需重拟合）`,
          kind: 'too-slow',
        };
        return { feasible: false, allocations: partial(lo, windows, gateOpen, i), rejectedLeg };
      }
    }
  }

  for (let i = n - 1; i >= 0; i--) {
    const w = windows[i];
    const op = openOf(w);
    let upper = Math.min(w.tMax, op.close ?? Infinity);
    if (i < n - 1) upper = Math.min(upper, hi[i + 1] - (w.gapMinToNext ?? 0));
    hi[i] = upper;
    if (lo[i] > hi[i] + 1e-9) {
      rejectedLeg = {
        index: i,
        from: i > 0 ? windows[i - 1].gateId : null,
        to: w.gateId,
        requiredBy: lo[i],
        latestReachable: hi[i],
        reason: `门 ${w.label} 可达区间 [${w.tMin.toFixed(2)}, ${w.tMax.toFixed(2)}]s 与开放窗无交集`,
        kind: 'window',
      };
      return { feasible: false, allocations: partial(lo, windows, gateOpen, n), rejectedLeg };
    }
  }

  const allocations = windows.map((w, i) => {
    const op = openOf(w);
    return {
      gateId: w.gateId,
      label: w.label,
      assigned: lo[i],
      windowStart: lo[i],
      windowEnd: hi[i],
      tMin: w.tMin,
      tMax: w.tMax,
      open: op.open ?? null,
      close: op.close ?? null,
      slack: hi[i] - lo[i],
    };
  });
  return { feasible: true, allocations, rejectedLeg: null };
}

function partial(lo, windows, gateOpen, upTo) {
  const out = [];
  for (let i = 0; i < Math.min(upTo, windows.length); i++) {
    const w = windows[i];
    const op = gateOpen[w.gateId] ?? {};
    out.push({
      gateId: w.gateId, label: w.label, assigned: lo[i],
      windowStart: lo[i], windowEnd: null, tMin: w.tMin, tMax: w.tMax,
      open: op.open ?? null, close: op.close ?? null, slack: null,
    });
  }
  return out;
}

// Arc-length span of a rejected leg, so the editor can highlight exactly the
// stretch that must be re-fitted rather than the whole course.
export function rejectedLegSpan(schedule, windows) {
  if (!schedule?.rejectedLeg || !windows?.length) return null;
  const i = schedule.rejectedLeg.index;
  const from = i > 0 ? windows[i - 1].s : windows[i].s;
  const to = windows[i].s;
  return { s0: Math.min(from, to), s1: Math.max(from, to) };
}

// Attach a schedule violation to an evaluation object so the same violation
// list / ribbon segmentation the editor already renders reports it. The span
// is the rejected leg only (or the whole course for a first-gate failure).
export function mergeScheduleViolation(evaluation, schedule, aggregateSegments, windows) {
  if (!schedule || schedule.feasible) return evaluation;
  const leg = schedule.rejectedLeg;
  const span = rejectedLegSpan(schedule, windows);
  const last = evaluation.samplesLastS ?? 0;
  const s0 = span ? span.s0 : 0;
  const s1 = span ? span.s1 : Math.max(last, 1);
  const violation = {
    type: 'gate-schedule', severity: 'error',
    gateId: leg?.to, s0, s1,
    message: leg?.reason ?? '门开放窗调度不可行',
  };
  evaluation.violations.push(violation);
  if (aggregateSegments) {
    evaluation.segments = aggregateSegments(evaluation.violations);
  }
  evaluation.feasible = false;
  evaluation.stats.errorCount += 1;
  return evaluation;
}
