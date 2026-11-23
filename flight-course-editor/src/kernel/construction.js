// Course construction strategies and the curve-solver comparison.
//
// Two construction families share the SAME inputs (aircraft turn/climb limits,
// gate frames, forbidden volumes):
//
//   control-point : start from the authored control polygon and relax tangent
//                   handles where the sampled envelope is violated.
//   segmented     : between consecutive gates, search a discrete set of
//                   candidate waypoints and keep the lowest-penalty one.
//
// Each family can be driven in two directions:
//
//   curve-first   : build the curve from the gate positions alone, then check
//                   climb (and clearance) afterwards.
//   gate-reverse  : force the tangent at every gate to the gate normal and
//                   take the local curvature budget from that gate's speed
//                   window, so the gate constrains the curve rather than the
//                   other way round.
//
// Repairs are chosen from measured evidence, not aesthetics: how many
// contiguous stretches are affected and how large the worst normalised
// envelope overshoot is decide local repair vs. a full-segment refit. Gate
// order is never reshuffled to make a construction succeed; a geometry that
// would require crossing gates out of order is returned as a hard failure.

import { Spline } from './spline.js';
import { V, clamp, distanceToVolume, segmentClearance } from './math.js';
import { curvatureCeiling } from './reshape.js';

const EPS = 1e-9;

export function cruiseSpeedOf(aircraft) {
  return aircraft.cruiseSpeed ?? aircraft.vMax * 0.6;
}

function nearestS(samples, pos) {
  let best = Infinity, bs = 0;
  for (const s of samples) {
    const d = V.dist2(s.pos, pos);
    if (d < best) { best = d; bs = s.s; }
  }
  return bs;
}

// All plane crossings of one gate, each tagged with whether the curve is
// travelling through the gate in the forward (required) direction.
function gateCrossings(gate, spline) {
  const out = [];
  const n = V.norm(gate.normal);
  const perSeg = 24;
  for (let seg = 0; seg < spline.segmentCount; seg++) {
    let prevP = spline.positionAt(seg);
    let prevD = V.dot(V.sub(prevP, gate.pos), n);
    for (let k = 1; k <= perSeg; k++) {
      const u = seg + k / perSeg;
      const p = spline.positionAt(u);
      const d = V.dot(V.sub(p, gate.pos), n);
      if (prevD === 0 || prevD * d <= 0) {
        const t = prevD === d ? 0 : prevD / (prevD - d);
        const tan = V.norm(V.sub(
          spline.positionAt(Math.min(spline.segmentCount, u + 0.002)),
          spline.positionAt(Math.max(0, u - 0.002))
        ));
        out.push({ u, pos: V.lerp(prevP, p, t), tan, forward: V.dot(tan, n) > 0 });
      }
      prevP = p;
      prevD = d;
    }
  }
  return out;
}

// Ordered gate crossings: each gate's first forward crossing after the
// previous gate. Returns orderOk=false if a gate is missed or reached out of
// the declared sequence.
function orderedGateCrossings(spline, samples, gates) {
  let lastS = -Infinity;
  const crossings = [];
  let orderOk = true;
  for (const gate of gates) {
    const all = gateCrossings(gate, spline)
      .map((c) => ({ ...c, s: nearestS(samples, c.pos) }));
    const ordered = all.filter((c) => c.s >= lastS - 1e-6).sort((a, b) => a.s - b.s);
    const pick = ordered.find((c) => c.forward) ?? null;
    if (!pick) {
      crossings.push(null);
      orderOk = false;
      continue;
    }
    lastS = pick.s;
    crossings.push({ ...pick, gate });
  }
  return { crossings, orderOk };
}

function gateFrames(gate) {
  const n = V.norm(gate.normal);
  let axisH = V.cross(n, [0, 1, 0]);
  if (V.len2(axisH) < 1e-8) axisH = [1, 0, 0];
  axisH = V.norm(axisH);
  return { n, axisH, axisV: V.norm(V.cross(axisH, n)) };
}

// Per-sample curvature budget. Gate-reverse mode lowers the budget near a
// gate that carries a tight speed window (lower v -> smaller allowed radius).
function budgetAt(sample, gates, aircraft, direction, cruise) {
  let v = cruise;
  if (direction === 'gate-reverse') {
    let best = Infinity;
    for (const g of gates) {
      const d = V.dist2(sample.pos, g.pos);
      if (d < best) { best = d; v = g.vMax ?? g.vMin ?? cruise; }
    }
    v = clamp(v, aircraft.vMin, aircraft.vMax);
  }
  return curvatureCeiling(v, aircraft);
}

// Evaluate a candidate geometry with the same envelope the editor enforces.
export function evaluateGeometry({
  points, tangentOverrides = {}, gates, volumes = [], aircraft,
  sampleDs = 2, direction = 'curve-first', cruise = null,
}) {
  const spline = new Spline(points, tangentOverrides);
  const samples = spline.sample(sampleDs);
  const vCruise = cruise ?? cruiseSpeedOf(aircraft);
  const craftRadius = aircraft.craftRadius ?? 2;
  const violations = [];

  for (const smp of samples) {
    const kMax = budgetAt(smp, gates, aircraft, direction, vCruise);
    if (smp.curvature > kMax * (1 + 1e-4)) {
      violations.push({
        type: 'curvature', severity: 'error', s: smp.s,
        overshoot: (smp.curvature - kMax) / kMax,
        radius: 1 / Math.max(smp.curvature, EPS), allowed: 1 / Math.max(kMax, EPS),
      });
    }
    if (Math.abs(smp.climbAngle) > aircraft.gammaMax + 1e-4) {
      violations.push({
        type: 'climb-angle', severity: 'error', s: smp.s,
        overshoot: (Math.abs(smp.climbAngle) - aircraft.gammaMax) / aircraft.gammaMax,
      });
    }
    const vz = Math.abs(vCruise * Math.sin(smp.climbAngle));
    if (vz > aircraft.vzMax + 1e-3) {
      violations.push({
        type: 'climb-rate', severity: 'error', s: smp.s,
        overshoot: (vz - aircraft.vzMax) / aircraft.vzMax,
      });
    }
  }

  // Volume clearance on the sampled polyline.
  let minClearance = Infinity;
  for (const vol of volumes) {
    for (let i = 0; i < samples.length - 1; i++) {
      const clear = segmentClearance(samples[i].pos, samples[i + 1].pos, vol);
      minClearance = Math.min(minClearance, clear);
      if (clear < craftRadius) {
        violations.push({
          type: 'clearance', severity: 'error', volumeId: vol.id,
          s: samples[i].s, overshoot: (craftRadius - clear) / Math.max(craftRadius, EPS),
        });
      }
    }
  }
  if (!Number.isFinite(minClearance)) minClearance = null;

  const { crossings, orderOk: crossingOrderOk } = orderedGateCrossings(spline, samples, gates);
  const orderOk = crossingOrderOk && gateChordOrderOk(points, gates);
  for (const c of crossings) {
    if (!c) continue;
    const { n, axisH, axisV } = gateFrames(c.gate);
    const rel = V.sub(c.pos, c.gate.pos);
    const x = V.dot(rel, axisH);
    const y = V.dot(rel, axisV);
    if (Math.abs(x) > c.gate.halfW - craftRadius || Math.abs(y) > c.gate.halfH - craftRadius) {
      violations.push({ type: 'gate-clearance', severity: 'error', s: c.s, gateId: c.gate.id });
    }
    if (V.dot(c.tan, n) < Math.cos(c.gate.alignLimit ?? Math.PI / 3)) {
      violations.push({ type: 'gate-alignment', severity: 'error', s: c.s, gateId: c.gate.id });
    }
  }
  if (crossings.some((c) => !c)) {
    violations.push({ type: 'gate-missed', severity: 'error', s: 0, gateId: null });
  }

  const overshoots = violations.map((v) => v.overshoot ?? 0);
  const maxOvershoot = overshoots.length ? Math.max(...overshoots) : 0;
  const affectedSegments = countContiguous(violations);
  const maxCurvature = Math.max(0, ...samples.map((s) => s.curvature));
  const maxClimb = Math.max(0, ...samples.map((s) => Math.abs(s.climbAngle)));

  return {
    spline, samples, tangentOverrides: { ...tangentOverrides },
    violations, crossings, gateOrderOk: orderOk,
    feasible: violations.every((v) => v.severity !== 'error') && orderOk,
    metrics: {
      maxCurvature, maxOvershoot, affectedSegments,
      minClearance, maxClimb,
      violationCount: violations.length,
      length: samples.length ? samples[samples.length - 1].s : 0,
      penalty: totalPenalty(violations, orderOk),
    },
  };
}

function countContiguous(violations, gap = 6) {
  if (!violations.length) return 0;
  const ss = violations.map((v) => v.s ?? 0).sort((a, b) => a - b);
  let n = 1;
  for (let i = 1; i < ss.length; i++) if (ss[i] - ss[i - 1] > gap) n++;
  return n;
}

// Scalar objective used to rank candidates. Envelope overshoot dominates,
// gate violations are heavily weighted, and order breakage is disqualifying.
function totalPenalty(violations, orderOk) {
  let penalty = 0;
  for (const v of violations) {
    switch (v.type) {
      case 'clearance': penalty += 0.6 + (v.overshoot ?? 0); break;
      case 'gate-clearance': penalty += 5; break;
      case 'gate-alignment': penalty += 5; break;
      case 'gate-missed': penalty += 20; break;
      default: penalty += v.overshoot ?? 0;
    }
  }
  if (!orderOk) penalty += 1000;
  return penalty;
}

// ---------- gate anchors ----------

function gateAnchorIndices(points, gates) {
  return gates.map((g) => {
    let bi = 0, bd = Infinity;
    points.forEach((p, i) => {
      const d = V.dist2(p, g.pos);
      if (d < bd) { bd = d; bi = i; }
    });
    return bi;
  });
}

// Anchors follow the DECLARED gate order: start, gate0, gate1, ..., end.
// The authored polygon must visit the gate anchors in that same order; if it
// does not, orderOk is false and no construction is allowed to reorder them.
function anchorsFromCourse(points, gates) {
  const idx = gateAnchorIndices(points, gates);
  let orderOk = true;
  for (let i = 1; i < idx.length; i++) if (idx[i] < idx[i - 1]) orderOk = false;
  const anchors = [V.clone(points[0])];
  for (const i of idx) anchors.push(V.clone(points[i]));
  anchors.push(V.clone(points[points.length - 1]));
  return { anchors, gatePointIndices: idx, orderOk };
}

// Geometry-level gate order check independent of the curve's own crossing
// ordering: walk the straight gate-to-gate chords in declared order and flag
// folding. This catches a control polygon that visits gates out of sequence
// even when a smoothed curve happens to pierce each plane forward once.
function gateChordOrderOk(points, gates) {
  const idx = gateAnchorIndices(points, gates);
  for (let i = 1; i < idx.length; i++) if (idx[i] < idx[i - 1]) return false;
  let prevDir = null;
  for (let i = 1; i < gates.length; i++) {
    const dir = V.norm(V.sub(gates[i].pos, gates[i - 1].pos));
    if (prevDir && V.dot(prevDir, dir) < -0.999) return false; // exact fold-back
    prevDir = dir;
  }
  return true;
}

function forceGateTangents(anchorToPoint, gates, overrides) {
  // Anchor k (1-based within construction) is gate k-1.
  gates.forEach((g, k) => { overrides[anchorToPoint[k + 1]] = V.norm(g.normal); });
  return overrides;
}

// ---------- control-point optimisation ----------

export function optimizeControlPoints(input) {
  const {
    points, gates, volumes = [], aircraft, sampleDs = 2,
    direction = 'curve-first', maxIterations = 80,
  } = input;
  const cruise = cruiseSpeedOf(aircraft);
  const overrides = {};
  const gateAnchor = gateAnchorIndices(points, gates);
  const gateIdx = new Set(gateAnchor);
  if (direction === 'gate-reverse') {
    gateAnchor.forEach((pi, k) => { overrides[pi] = V.norm(gates[k].normal); });
  }

  const spline = new Spline(points, overrides);
  let lastMoved = 0;
  for (let it = 0; it < maxIterations; it++) {
    const samples = spline.sample(sampleDs);
    const moves = new Map();
    const push = (i, dir) => {
      if (!moves.has(i)) moves.set(i, []);
      moves.get(i).push(dir);
    };
    for (let s = 2; s < samples.length - 2; s++) {
      const smp = samples[s];
      const idx = nearestInterior(spline, smp.pos, gateIdx);
      if (idx < 0) continue;
      const t0 = spline.tangentAt(idx);
      let t = t0;
      const kMax = budgetAt(smp, gates, aircraft, direction, cruise);
      if (smp.curvature > kMax) {
        const chord = V.norm(V.sub(
          spline.points[Math.min(idx + 1, spline.points.length - 1)],
          spline.points[Math.max(idx - 1, 0)]
        ));
        const blend = clamp((smp.curvature - kMax) / Math.max(smp.curvature, EPS), 0.05, 0.5);
        t = V.norm(V.lerp(t, chord, blend));
      }
      if (direction === 'gate-reverse') {
        const gamma = Math.asin(clamp(t[1], -1, 1));
        if (Math.abs(gamma) > aircraft.gammaMax) {
          const g2 = Math.sign(gamma) * aircraft.gammaMax * 0.9;
          const horiz = V.norm([t[0], 0, t[2]]);
          t = [horiz[0] * Math.cos(g2), Math.sin(g2), horiz[2] * Math.cos(g2)];
        }
      }
      for (const vol of volumes) {
        if (distanceToVolume(smp.pos, vol) < (aircraft.craftRadius ?? 2)) {
          const center = vol.center ?? smp.pos;
          t = V.norm(V.lerp(t, V.norm(V.sub(smp.pos, center)), 0.3));
        }
      }
      if (V.dist2(t, t0) > 1e-10) push(idx, t);
    }
    lastMoved = moves.size;
    if (!lastMoved) break;
    for (const [idx, dirs] of moves) {
      if (gateIdx.has(idx)) continue;
      const avg = V.norm(dirs.reduce((a, d) => V.add(a, d), [0, 0, 0]));
      spline.setTangentOverride(idx, avg);
    }
  }
  const final = new Spline(spline.points, spline.tangentOverrides);
  const evaluation = evaluateGeometry({
    points: final.points, tangentOverrides: final.tangentOverrides,
    gates, volumes, aircraft, sampleDs, direction,
  });
  return {
    ...evaluation,
    points: final.points.map(V.clone),
    tangentOverrides: { ...final.tangentOverrides },
    iterations: lastMoved,
  };
}

function nearestInterior(spline, pos, gateIdx) {
  let best = Infinity, bi = -1;
  for (let i = 1; i < spline.points.length - 1; i++) {
    if (gateIdx.has(i)) continue;
    const d = V.dist2(spline.points[i], pos);
    if (d < best) { best = d; bi = i; }
  }
  return bi;
}

// ---------- segmented candidate search ----------

function basisPerp(chord) {
  const e1 = V.norm(V.cross(chord, [0, 1, 0]));
  const e1s = V.len2(e1) < 1e-8 ? [0, 0, 1] : e1;
  const e2 = V.norm(V.cross(chord, e1s));
  return [e1s, e2];
}

// Candidate interior points for one anchor-to-anchor gap. The set is discrete
// and documented in the comparison's pending assumptions.
function candidateInteriors(a, b, count, step) {
  const chord = V.sub(b, a);
  const L = V.len(chord);
  if (L < 1e-6) return [[]];
  const [e1, e2] = basisPerp(chord);
  const out = [[]];
  const lateral = [0, step, -step, 2 * step, -2 * step];
  const vertical = [0, 0.6 * step, -0.6 * step];
  for (const off of lateral) {
    for (const vy of vertical) {
      if (off === 0 && vy === 0) continue;
      const shift = V.add(V.scale(e1, off), V.scale(e2, vy));
      const pts = [];
      for (let k = 1; k <= count; k++) {
        const t = k / (count + 1);
        pts.push(V.add(V.lerp(a, b, t), shift));
      }
      out.push(pts);
    }
  }
  return out;
}

// Build the full point list with interior candidates inserted per gap.
function assembleAnchors(anchors, inserts) {
  const points = [];
  const anchorToPoint = [];
  for (let k = 0; k < anchors.length - 1; k++) {
    anchorToPoint.push(points.length);
    points.push(V.clone(anchors[k]));
    for (const p of inserts[k] ?? []) points.push(V.clone(p));
  }
  anchorToPoint.push(points.length);
  points.push(V.clone(anchors[anchors.length - 1]));
  return { points, anchorToPoint };
}

// Greedy per-gap search. Each gap's candidates are scored on the FULL
// resulting geometry (coupling through cumulative arc length is real), so a
// locally cheap bend that shoves a later leg into a building is rejected.
// `seedInserts` lets a repair pass keep every gap it is not fixing unchanged.
export function searchSegmented(input, options = {}) {
  const {
    points, gates, volumes = [], aircraft, sampleDs = 2,
    direction = 'curve-first',
  } = input;
  const onlyGaps = options.onlyGaps ?? null;
  const dense = options.dense ?? false;
  const seedInserts = options.seedInserts ?? null;
  const { anchors, orderOk: anchorOrderOk } = anchorsFromCourse(points, gates);

  const inserts = new Array(Math.max(anchors.length - 1, 0)).fill(null).map((_, k) => (
    seedInserts?.[k] ? seedInserts[k].map(V.clone) : []
  ));

  const buildOverrides = (built) => {
    const ov = {};
    if (direction === 'gate-reverse') forceGateTangents(built.anchorToPoint, gates, ov);
    return ov;
  };
  const evaluate = (built) => evaluateGeometry({
    points: built.points, tangentOverrides: buildOverrides(built),
    gates, volumes, aircraft, sampleDs, direction,
  });

  let built = assembleAnchors(anchors, inserts);
  let current = evaluate(built);

  for (let gap = 0; gap < anchors.length - 1; gap++) {
    if (onlyGaps && !onlyGaps.has(gap)) continue;
    const a = anchors[gap];
    const b = anchors[gap + 1];
    const L = V.dist(a, b);
    const step = clamp(L / 6, 5, 35);
    const counts = dense ? [1, 2] : [1];
    let best = { penalty: current.metrics.penalty, candidate: inserts[gap] };
    for (const count of counts) {
      for (const candidate of candidateInteriors(a, b, count, step)) {
        inserts[gap] = candidate;
        const probe = assembleAnchors(anchors, inserts);
        const ev = evaluate(probe);
        // Tiny tie-breaker: prefer fewer inserted points.
        const penalty = ev.metrics.penalty + candidate.length * 1e-3;
        if (penalty < best.penalty - 1e-9) {
          best = { penalty, candidate: candidate.map(V.clone) };
        }
      }
    }
    inserts[gap] = best.candidate;
  }

  built = assembleAnchors(anchors, inserts);
  current = evaluate(built);
  const gateOk = anchorOrderOk && current.gateOrderOk;
  return {
    ...current,
    feasible: current.feasible && gateOk,
    gateOrderOk: gateOk,
    points: built.points.map(V.clone),
    tangentOverrides: buildOverrides(built),
    anchors, inserts, anchorToPoint: built.anchorToPoint,
  };
}

// ---------- repair policy ----------

// Measured policy: local repair when few stretches are affected and the worst
// overshoot is modest; otherwise a full-segment refit. Gate order must already
// be satisfied for local repair to be selectable.
export function chooseRepairStrategy(evaluation, { localMaxSegments = 2, localMaxOvershoot = 0.4 } = {}) {
  const m = evaluation.metrics;
  const localOk = evaluation.gateOrderOk
    && m.affectedSegments > 0 && m.affectedSegments <= localMaxSegments
    && m.maxOvershoot <= localMaxOvershoot;
  return {
    mode: localOk ? 'local' : 'full',
    affectedSegments: m.affectedSegments,
    maxOvershoot: m.maxOvershoot,
    gateOrderOk: evaluation.gateOrderOk,
    rationale: localOk
      ? `仅 ${m.affectedSegments} 段受影响且最大越界 ${(m.maxOvershoot * 100).toFixed(0)}%，局部修整足够`
      : `受影响段 ${m.affectedSegments} 段、最大越界 ${(m.maxOvershoot * 100).toFixed(0)}%，需全段重拟合`,
  };
}

// Which construction gap contains station s, using this result's anchors.
function gapsForViolations(result) {
  const gaps = new Set();
  const bounds = [];
  for (let k = 0; k < result.anchors.length - 1; k++) {
    bounds.push([
      nearestS(result.samples, result.anchors[k]),
      nearestS(result.samples, result.anchors[k + 1]),
    ]);
  }
  for (const v of result.violations) {
    const s = v.s ?? 0;
    let placed = false;
    for (let k = 0; k < bounds.length; k++) {
      const [sA, sB] = bounds[k];
      if (s >= sA - 1e-6 && s <= sB + 1e-6) { gaps.add(k); placed = true; break; }
    }
    if (!placed) gaps.add(0);
  }
  if (!gaps.size) gaps.add(0);
  return gaps;
}

// Apply the chosen repair on top of a construction result. Control-point
// results carry no per-gap inserts, so their repair is a segmented refit over
// the same anchors (full, or only the affected gaps). Both passes are seeded
// from the current inserts; the repaired geometry replaces the original only
// when it actually lowers the measured objective.
export function repairConstruction(input, result, strategy = null) {
  const decision = strategy ?? chooseRepairStrategy(result);
  const anchors = result.anchors ?? anchorsFromCourse(input.points, input.gates).anchors;
  const withAnchors = { ...result, anchors, samples: result.samples };
  const gaps = decision.mode === 'local' ? gapsForViolations(withAnchors) : null;
  const repaired = searchSegmented(input, {
    onlyGaps: gaps,
    dense: true,
    seedInserts: result.inserts ?? null,
  });
  const improved = repaired.metrics.penalty <= result.metrics.penalty + 1e-9;
  return {
    ...(improved ? repaired : result),
    repair: {
      ...decision,
      gaps: gaps ? [...gaps] : null,
      improved,
      beforePenalty: result.metrics.penalty,
      afterPenalty: repaired.metrics.penalty,
    },
  };
}

// ---------- public comparison ----------

export function constructFlightGeometry(input = {}) {
  const {
    strategy = 'control-point', direction = 'curve-first', repair = true,
  } = input;
  let result = strategy === 'segmented'
    ? searchSegmented(input)
    : optimizeControlPoints(input);
  if (repair && !result.feasible) {
    result = repairConstruction(input, result);
  }
  result.strategy = strategy;
  result.direction = direction;
  result.points = result.points.map(V.clone);
  return result;
}

// Run all four family x direction combinations on identical inputs.
export function compareCurveSolvers(input) {
  const variants = [];
  for (const strategy of ['control-point', 'segmented']) {
    for (const direction of ['curve-first', 'gate-reverse']) {
      const r = constructFlightGeometry({ ...input, strategy, direction, repair: true });
      variants.push(summarizeVariant(strategy, direction, r));
    }
  }
  const byFeasible = variants.filter((v) => v.feasible)
    .sort((a, b) => a.maxOvershoot - b.maxOvershoot);
  return {
    variants,
    preferred: byFeasible[0] ?? null,
    // Assumptions that still need a controlled flight test are called out
    // separately instead of being folded into the result.
    pendingAssumptions: [
      '候选点偏移集合仅取横向 ±2 步长、竖向 ±0.6 步长，未穷举连续空间',
      '曲率预算以巡航速度换算，未与逐站实际速度表联合迭代',
      '门反向约束只强制门切向与门速窗预算，未求解门间最优弧长',
      '未在此比较中加入时序（门开放窗）耦合，调度在 course.js 单独校核',
    ],
  };
}

function summarizeVariant(strategy, direction, r) {
  return {
    strategy, direction,
    feasible: r.feasible,
    gateOrderOk: r.gateOrderOk,
    maxOvershoot: r.metrics.maxOvershoot,
    affectedSegments: r.metrics.affectedSegments,
    maxCurvature: r.metrics.maxCurvature,
    minRadius: 1 / Math.max(r.metrics.maxCurvature, 1e-9),
    minClearance: r.metrics.minClearance,
    length: r.metrics.length,
    violationCount: r.metrics.violationCount,
    penalty: r.metrics.penalty,
    repair: r.repair
      ? {
        mode: r.repair.mode, rationale: r.repair.rationale,
        improved: r.repair.improved,
        beforePenalty: r.repair.beforePenalty, afterPenalty: r.repair.afterPenalty,
      }
      : null,
    points: r.points,
    tangentOverrides: r.tangentOverrides,
  };
}
