// Course orchestration: spline -> samples -> RMF/attitude -> speed -> envelope
// evaluation, plus the two re-planning modes and a full simulation run.

import { Spline } from './spline.js';
import { buildRMF, buildAttitude } from './frame.js';
import { planSpeedProfile, replanAtStation } from './speed.js';
import { evaluateCourse, aggregateSegments } from './constraints.js';
import { reshapeCourse } from './reshape.js';
import { simulate } from './simulation.js';
import { searchSegmented } from './construction.js';
import {
  reachableGateWindows, scheduleGateWindows, mergeScheduleViolation, rejectedLegSpan,
} from './gate-windows.js';
import { V } from './math.js';

// Aircraft envelope used across the shipped scenarios (SI units).
export const DEFAULT_AIRCRAFT = {
  g: 9.81,
  vMin: 8,
  vMax: 45,
  aMax: 8,
  aDecel: 12,
  phiMax: Math.PI / 3,            // 60 deg max bank
  bankRateMax: 2.2,               // rad/s
  gammaMax: Math.PI / 6,          // 30 deg climb/dive
  gammaRateMax: 0.4,              // rad/s
  vzMax: 18,                      // m/s vertical speed
  craftRadius: 2.2,               // collision sphere radius
  guidanceHeadway: 2.5,           // seconds
  trackTolerance: 4.0,
  abortCrossTrack: 22,
  cruiseSpeed: 26,
};

export function buildCourse({
  name, points, gates, volumes = [], aircraft = DEFAULT_AIRCRAFT,
  tangentOverrides = {}, sampleDs = 2, minSeparation, gateOpen = null,
}) {
  const spline = new Spline(points, tangentOverrides);
  const samples = spline.sample(sampleDs);
  // First pass without speeds lets gate crossing stations be located.
  const rmf0 = buildRMF(samples);
  const tmpSpeeds = samples.map(() => aircraft.cruiseSpeed);
  const att0 = buildAttitude(samples, rmf0, aircraft, tmpSpeeds);
  const preEvaluate = evaluateCourse({
    gates, volumes, aircraft, samples, attitudes: att0,
    speeds: tmpSpeeds, craftRadius: aircraft.craftRadius, minSeparation,
  });
  // Locate gate crossings (mutates gate._crossS) even if geometry is bad.
  const plan = planSpeedProfile(samples, aircraft, gates);
  const rmf = buildRMF(samples);
  const attitudes = buildAttitude(samples, rmf, aircraft, plan.speeds);
  const evaluation = evaluateCourse({
    gates, volumes, aircraft, samples, attitudes,
    speeds: plan.speeds, craftRadius: aircraft.craftRadius, minSeparation,
  });
  const sim = simulate({
    samples, speeds: plan.speeds, aircraft,
  });
  // Gate reachable-time intervals come straight from the speed envelope, then
  // the level's opening schedule is allocated inside them. An impossible
  // allocation is an error on the same evaluation the editor renders.
  const gateWindows = reachableGateWindows({ samples, gates, aircraft });
  const schedule = scheduleGateWindows(gateWindows.windows, gateOpen ?? {});
  if (!schedule.feasible) {
    evaluation.samplesLastS = samples.length ? samples[samples.length - 1].s : 0;
    mergeScheduleViolation(evaluation, schedule, aggregateSegments, gateWindows.windows);
  }
  return {
    name,
    spline,
    points: spline.points.map(V.clone),
    gates,
    volumes,
    aircraft,
    samples,
    speeds: plan.speeds,
    targetCeilings: plan.targetCeilings,
    guidance: plan.guidance,
    attitudes,
    rmf,
    evaluation,
    preEvaluate,
    speedConflicts: plan.conflicts,
    simulation: sim,
    gateWindows: gateWindows.windows,
    gateWindowsFeasible: gateWindows.feasible,
    gateWindowsReason: gateWindows.reason,
    schedule,
    params: { sampleDs, minSeparation, gateOpen: gateOpen ?? {} },
  };
}

// Mode A: keep geometry, change one speed target, recompute the whole profile
// and every guidance point before/after it.
export function replanSpeed(course, stationIndex, newSpeed) {
  const plan = replanAtStation(
    course.samples, course.aircraft, course.gates, stationIndex, newSpeed
  );
  const attitudes = buildAttitude(course.samples, course.rmf, course.aircraft, plan.speeds);
  const evaluation = evaluateCourse({
    gates: course.gates, volumes: course.volumes, aircraft: course.aircraft,
    samples: course.samples, attitudes, speeds: plan.speeds,
    craftRadius: course.aircraft.craftRadius, minSeparation: course.params.minSeparation,
  });
  const simulation = simulate({
    samples: course.samples, speeds: plan.speeds, aircraft: course.aircraft,
  });
  return {
    ...course,
    speeds: plan.speeds,
    guidance: plan.guidance,
    attitudes,
    evaluation,
    speedConflicts: plan.conflicts,
    simulation,
  };
}

// Mode B: lock the current speed table, reshape tangents / insert guidance
// points until the same envelope is satisfied.
export function reshapeWithLockedSpeed(course, options = {}) {
  const lockedGateIndices = course.gates
    .map((g) => course.points.findIndex((p) => V.dist2(p, g.pos) < 1e-6))
    .filter((i) => i >= 0);
  const result = reshapeCourse({
    points: course.points,
    gates: course.gates,
    volumes: course.volumes,
    aircraft: course.aircraft,
    craftRadius: course.aircraft.craftRadius,
    lockedSpeed: { samples: course.samples, speeds: course.speeds },
    lockedGateIndices,
    sampleDs: course.params.sampleDs,
    ...options,
  });
  // Rebuild a full course object from the reshaped geometry.
  const rebuilt = buildCourse({
    name: course.name,
    points: result.points,
    gates: course.gates,
    volumes: course.volumes,
    aircraft: course.aircraft,
    tangentOverrides: result.tangentOverrides,
    sampleDs: course.params.sampleDs,
    minSeparation: course.params.minSeparation,
    gateOpen: course.params.gateOpen,
  });
  rebuilt.reshape = result;
  return rebuilt;
}

// Rebuild after the user drags a control point / gate in the editor.
export function rebuildFromPoints(course, points, gates) {
  return buildCourse({
    name: course.name,
    points,
    gates: gates ?? course.gates,
    volumes: course.volumes,
    aircraft: course.aircraft,
    sampleDs: course.params.sampleDs,
    minSeparation: course.params.minSeparation,
    gateOpen: course.params.gateOpen,
  });
}

// Rebuild from a construction/search result (points + tangent overrides),
// keeping the gate set, volumes, aircraft and opening schedule.
export function applyConstruction(course, result) {
  const rebuilt = buildCourse({
    name: course.name,
    points: result.points,
    gates: course.gates,
    volumes: course.volumes,
    aircraft: course.aircraft,
    tangentOverrides: result.tangentOverrides ?? {},
    sampleDs: course.params.sampleDs,
    minSeparation: course.params.minSeparation,
    gateOpen: course.params.gateOpen,
  });
  rebuilt.construction = result;
  return rebuilt;
}

// Apply an opening-time edit (e.g. shrink the middle gate's open window) and
// re-run the scheduler against the SAME geometry. If the edit makes a leg
// unreachable, the course is returned with the rejected leg flagged and
// `requiresRefit` set; the previous feasible geometry is left untouched so the
// UI can offer a refit instead of silently drawing a smooth-but-unflyable line.
export function rescheduleCourse(course, gateOpenChange = {}) {
  const gateOpen = { ...(course.params.gateOpen ?? {}) };
  for (const [id, patch] of Object.entries(gateOpenChange)) {
    gateOpen[id] = { ...(gateOpen[id] ?? {}), ...patch };
  }
  const gateWindows = reachableGateWindows({
    samples: course.samples, gates: course.gates, aircraft: course.aircraft,
  });
  const schedule = scheduleGateWindows(gateWindows.windows, gateOpen);

  // Evaluate the existing geometry with the new schedule attached.
  const evaluation = {
    ...course.evaluation,
    violations: course.evaluation.violations.filter((v) => v.type !== 'gate-schedule'),
  };
  evaluation.violations = evaluation.violations.slice();
  evaluation.segments = aggregateSegments(evaluation.violations);
  const errors = evaluation.violations.filter((v) => v.severity === 'error');
  evaluation.feasible = errors.length === 0;
  evaluation.stats = { ...evaluation.stats, errorCount: errors.length,
    warningCount: evaluation.violations.length - errors.length };
  if (!schedule.feasible) {
    evaluation.samplesLastS = course.samples.length
      ? course.samples[course.samples.length - 1].s : 0;
    mergeScheduleViolation(evaluation, schedule, aggregateSegments, gateWindows.windows);
  }
  return {
    ...course,
    evaluation,
    schedule,
    gateWindows: gateWindows.windows,
    gateWindowsFeasible: gateWindows.feasible,
    gateWindowsReason: gateWindows.reason,
    params: { ...course.params, gateOpen },
    requiresRefit: !schedule.feasible,
    rejectedLeg: schedule.rejectedLeg,
  };
}

// When the scheduler rejects a leg, either re-fit that leg (segmented search
// restricted to the rejected gap, with the gate order locked) or, if no
// candidate geometry can still meet both curvature and the time window, keep
// the old course untouched. The caller gets a definite outcome either way —
// never a smooth-looking line that cannot actually fly the schedule.
export function refitRejectedLeg(course, { direction = 'gate-reverse' } = {}) {
  if (!course.requiresRefit || !course.rejectedLeg) {
    return { applied: false, reason: '没有需要重拟合的被拒航段', course };
  }
  const geometry = {
    points: course.points, gates: course.gates, volumes: course.volumes,
    aircraft: course.aircraft, sampleDs: course.params.sampleDs, direction,
  };
  const rejectedGateId = course.rejectedLeg.to;
  // Search only the gap that ends at the rejected gate; every other gap is
  // held exactly as authored.
  const gapIndex = Math.max(0, course.gates.findIndex((g) => g.id === rejectedGateId));
  const result = searchSegmented(geometry, { onlyGaps: new Set([gapIndex]) });
  const candidate = buildCourse({
    name: course.name,
    points: result.points,
    gates: course.gates,
    volumes: course.volumes,
    aircraft: course.aircraft,
    tangentOverrides: result.tangentOverrides ?? {},
    sampleDs: course.params.sampleDs,
    minSeparation: course.params.minSeparation,
    gateOpen: course.params.gateOpen,
  });
  candidate.construction = result;

  const geometryOk = result.feasible && result.gateOrderOk;
  const scheduleOk = candidate.schedule.feasible;
  if (geometryOk && scheduleOk) {
    return {
      applied: true, course: candidate,
      reason: `已重拟合航段 ${course.rejectedLeg.from ?? '起点'}→${rejectedGateId}，` +
        '曲率与时间窗同时满足。',
    };
  }
  // Cannot meet both: keep the OLD course (with its schedule violation still
  // flagged). Report which constraint blocked the refit.
  const why = !geometryOk
    ? `重拟合后几何仍超包络（最大越界 ${(result.metrics.maxOvershoot * 100).toFixed(0)}%，` +
      `${result.metrics.affectedSegments} 段受影响）`
    : `重拟合后几何可行，但时间窗仍无法分配（${candidate.schedule.rejectedLeg?.reason ?? '无交集'}）`;
  return {
    applied: false, course, reason: `${why}；保留旧关卡，不输出看似平滑却飞不通的新线。`,
    attempted: candidate,
  };
}

export { reshapeCourse, planSpeedProfile, simulate };