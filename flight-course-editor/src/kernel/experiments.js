// Reproducible curve-solver experiments on the two shapes the task calls out:
//   sharp  : a 90-degree bend between gates — curvature ceiling vs. clearance
//   narrow : a slot between two forbidden volumes — collision distance limit
//
// Each experiment sweeps one parameter and records, per sweep value, the
// MEASURED maximum curvature / minimum radius and minimum obstacle clearance
// for every construction variant. The numbers come from the same functions the
// editor calls (buildCourse + compareCurveSolvers), so a table in the docs and
// a number on screen can never diverge.

import { rad } from './math.js';
import { buildCourse, DEFAULT_AIRCRAFT } from './course.js';
import { compareCurveSolvers, evaluateGeometry } from './construction.js';

function gate(id, pos, normal, halfW, halfH, extra = {}) {
  return {
    id, label: id, pos, normal, halfW, halfH,
    alignLimit: rad(60), required: true, ...extra,
  };
}

// A 90-degree bend at radius R, authored with a constant arc-length step so
// the Catmull-Rom samples the arc at the same density as its straights. (An
// uneven layout — 2 m arc knots beside 40 m straights — produces genuine C1
// curvature spikes at the junctions and makes the measured radius meaningless.)
// Gate 1 faces +X, gate 2 faces +Z at the exit of the arc, gate 3 sits on the
// straight leg. A building pinches the inside of the corner, so shrinking R
// makes curvature and clearance compete.
function sharpBend({ bendRadius = 34, insideOffset = 4 } = {}) {
  const R = Math.max(bendRadius, 8);
  const aircraft = { ...DEFAULT_AIRCRAFT, cruiseSpeed: 28 };
  const arcLength = (Math.PI / 2) * R;
  const nArc = Math.max(6, Math.round(arcLength / 3));
  const step = arcLength / nArc;
  const points = [];
  // Straight approach: 8 steps before the bend entrance.
  for (let k = 8; k >= 1; k--) points.push([70 - k * step, 14, 0]);
  // Quarter arc, sampled at the same step.
  for (let k = 0; k <= nArc; k++) {
    const th = (k / nArc) * (Math.PI / 2);
    points.push([70 + R * Math.sin(th), 14, R - R * Math.cos(th)]);
  }
  // Straight exit: 12 steps past the bend.
  for (let k = 1; k <= 12; k++) points.push([70 + R, 14, R + k * step]);
  const bx = 70 - (insideOffset + Math.max(0, 20 - R * 0.5));
  const bz = Math.max(8, R * 0.4);
  return {
    id: 'exp-sharp',
    name: '急弯实验',
    aircraft, points,
    gates: [
      gate('S1', [70, 14, 0], [1, 0, 0], 9, 5, { vMax: 16 }),
      gate('S2', [70 + R, 14, R], [0, 0, 1], 9, 5),
      gate('S3', [70 + R, 14, R + 6 * step], [0, 0, 1], 9, 5),
    ],
    volumes: [
      { id: 'B-inside', type: 'box', center: [bx, 14, bz], half: [6, 16, 6], yaw: 0 },
    ],
    minSeparation: 8,
  };
}

// A straight corridor whose two walls leave a slot of half-width `slotHalf`.
// Below the craft radius the corridor itself is unusable; above it the
// clearance margin grows directly with the slot.
function narrowSlot({ slotHalf = 6 } = {}) {
  const hw = Math.max(slotHalf, 0.5);
  const aircraft = { ...DEFAULT_AIRCRAFT, cruiseSpeed: 24 };
  const points = [[0, 14, 0], [200, 14, 0], [260, 14, 0]];
  return {
    id: 'exp-narrow',
    name: '狭窄禁区实验',
    aircraft, points,
    gates: [
      gate('N1', [60, 14, 0], [1, 0, 0], 8, 6),
      gate('N2', [200, 14, 0], [1, 0, 0], 8, 6),
    ],
    volumes: [
      { id: 'V-port', type: 'box', center: [130, 14, -(hw + 6)], half: [50, 18, 6], yaw: 0 },
      { id: 'V-stbd', type: 'box', center: [130, 14, (hw + 6)], half: [50, 18, 6], yaw: 0 },
    ],
    minSeparation: 8,
  };
}

const EXPERIMENTS = {
  sharp: {
    title: '急弯：弯道半径 vs 曲率上限 / 碰撞距离',
    param: 'bendRadius', unit: 'm',
    values: [14, 20, 26, 34, 44, 56, 70],
    make: sharpBend,
  },
  narrow: {
    title: '狭窄禁区：缝隙半宽 vs 碰撞距离 / 可达性',
    param: 'slotHalf', unit: 'm',
    values: [1, 2, 3, 4, 6, 9, 14],
    make: narrowSlot,
  },
};

function sweep(key) {
  const spec = EXPERIMENTS[key];
  const rows = [];
  for (const value of spec.values) {
    const def = spec.make({ [spec.param]: value });
    const built = buildCourse(def);
    const comparison = compareCurveSolvers({
      points: built.points, gates: built.gates, volumes: built.volumes,
      aircraft: built.aircraft,
    });
    const best = comparison.preferred
      ?? [...comparison.variants].sort((a, b) => a.penalty - b.penalty)[0];
    rows.push({
      value,
      authoredFeasible: built.evaluation.feasible,
      schedulerFeasible: built.schedule.feasible,
      authoredMinRadius: built.evaluation.stats.minRadius,
      authoredMaxCurvature: built.evaluation.stats.maxCurvature,
      authoredMinClearance: built.evaluation.stats.minClearance,
      variants: comparison.variants.map((v) => ({
        strategy: v.strategy, direction: v.direction,
        feasible: v.feasible, gateOrderOk: v.gateOrderOk,
        maxOvershoot: v.maxOvershoot,
        maxCurvature: v.maxCurvature,
        minRadius: v.minRadius ?? 1 / Math.max(v.maxCurvature, 1e-9),
        minClearance: v.minClearance,
        affectedSegments: v.affectedSegments,
        penalty: v.penalty,
      })),
      best: best
        ? {
          strategy: best.strategy, direction: best.direction,
          minRadius: 1 / Math.max(best.maxCurvature, 1e-9),
          minClearance: best.minClearance,
        }
        : null,
    });
  }
  return { ...spec, rows };
}

// Recompute the whole experiment set. This is the single source of truth for
// any table in the docs; no figure is typed in by hand.
export function runCurveExperiments() {
  return {
    sharp: sweep('sharp'),
    narrow: sweep('narrow'),
    aircraft: DEFAULT_AIRCRAFT,
  };
}

// Re-check one measured number against the same code path. Used by tests so
// the reported curvature/clearance is proven reproducible, not descriptive.
export function verifyExperimentRow(key, value) {
  const spec = EXPERIMENTS[key];
  const def = spec.make({ [spec.param]: value });
  const built = buildCourse(def);
  const ev = evaluateGeometry({
    points: built.points, gates: built.gates, volumes: built.volumes,
    aircraft: built.aircraft,
  });
  return {
    maxCurvature: ev.metrics.maxCurvature,
    minRadius: 1 / Math.max(ev.metrics.maxCurvature, 1e-9),
    minClearance: ev.metrics.minClearance,
    maxOvershoot: ev.metrics.maxOvershoot,
    feasible: ev.feasible,
    authoredMinRadius: built.evaluation.stats.minRadius,
    authoredMaxCurvature: built.evaluation.stats.maxCurvature,
    authoredMinClearance: built.evaluation.stats.minClearance,
  };
}

export { EXPERIMENTS, sharpBend, narrowSlot };
