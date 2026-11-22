// Course orchestration: spline -> samples -> RMF/attitude -> speed -> envelope
// evaluation, plus the two re-planning modes and a full simulation run.

import { Spline } from './spline.js';
import { buildRMF, buildAttitude } from './frame.js';
import { planSpeedProfile, replanAtStation } from './speed.js';
import { evaluateCourse } from './constraints.js';
import { reshapeCourse } from './reshape.js';
import { simulate } from './simulation.js';
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
  tangentOverrides = {}, sampleDs = 2, minSeparation,
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
    params: { sampleDs, minSeparation },
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
  });
}

export { reshapeCourse, planSpeedProfile, simulate };
