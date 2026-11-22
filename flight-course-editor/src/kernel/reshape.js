// Mode B: locked target speed, reshape the curve locally.
//
// Gate positions stay fixed. The free design variables are (a) tangent
// direction overrides at existing control points and (b) extra guidance
// control points inserted between gates. Each iteration measures curvature /
// climb / clearance violations on the dense arc-length sample and nudges the
// nearest control-point tangent to relieve them. This is a projected-relaxation
// loop, not a cosmetic smoothing pass: convergence is judged against the same
// envelope used by constraints.js, and anything that cannot be removed is
// returned as a hard failure rather than hidden.

import { Spline } from './spline.js';
import { V, clamp, distanceToVolume } from './math.js';

function curvatureCeiling(v, aircraft) {
  const g = aircraft.g ?? 9.81;
  return (g * Math.tan(aircraft.phiMax)) / (v * v);
}

// Map a sampled position back to the nearest internal control-point index.
function nearestControlIndex(spline, pos) {
  let best = Infinity, bi = 0;
  spline.points.forEach((p, i) => {
    const d = V.dist2(p, pos);
    if (d < best) { best = d; bi = i; }
  });
  return clamp(bi, 1, spline.points.length - 2);
}

// One relaxation sweep over the sampled violations.
function relaxationSweep(spline, samples, speeds, aircraft, volumes, locked, craftRadius) {
  const moves = new Map();
  const push = (idx, dir) => {
    if (locked.has(idx)) return;
    if (!moves.has(idx)) moves.set(idx, []);
    moves.get(idx).push(dir);
  };

  for (let i = 2; i < samples.length - 2; i++) {
    const smp = samples[i];
    const v = speeds[i];
    const kMax = curvatureCeiling(v, aircraft);
    const idx = nearestControlIndex(spline, smp.pos);
    const t0 = spline.tangentAt(idx);
    let t = t0;

    if (smp.curvature > kMax) {
      // Straighten: rotate tangent toward the local chord between neighbours.
      const chord = V.norm(V.sub(
        spline.points[Math.min(idx + 1, spline.points.length - 1)],
        spline.points[Math.max(idx - 1, 0)]
      ));
      const blend = clamp((smp.curvature - kMax) / Math.max(smp.curvature, 1e-6), 0.05, 0.5);
      t = V.norm(V.lerp(t, chord, blend));
    }

    // Climb angle / climb rate: adjust the vertical component of the tangent.
    const gamma = Math.asin(clamp(t[1], -1, 1));
    if (Math.abs(gamma) > aircraft.gammaMax) {
      const g2 = Math.sign(gamma) * aircraft.gammaMax * 0.9;
      const horiz = V.norm([t[0], 0, t[2]]);
      t = [horiz[0] * Math.cos(g2), Math.sin(g2), horiz[2] * Math.cos(g2)];
    }
    const vz = v * Math.max(0, t[1]);
    if (vz > aircraft.vzMax) {
      const g2 = Math.asin(clamp((aircraft.vzMax * 0.9) / Math.max(v, 1), -1, 1));
      const horiz = V.norm([t[0], 0, t[2]]);
      t = [horiz[0] * Math.cos(g2), Math.sin(g2), horiz[2] * Math.cos(g2)];
    }

    // Clearance push: steer the tangent away from each violated volume.
    for (const vol of volumes) {
      if (distanceToVolume(smp.pos, vol) < craftRadius) {
        const center = vol.center ?? smp.pos;
        const away = V.norm(V.sub(smp.pos, center));
        t = V.norm(V.lerp(t, away, 0.35));
      }
    }

    if (V.dist2(t, t0) > 1e-10) push(idx, t);
  }

  for (const [idx, dirs] of moves) {
    const avg = V.norm(dirs.reduce((acc, d) => V.add(acc, d), [0, 0, 0]));
    spline.setTangentOverride(idx, avg);
  }
  return moves.size;
}

// Insert an extra control point at the middle of the longest segment and
// rebuild tangent overrides with shifted indices.
function insertMidpoint(spline) {
  const lens = spline.segLengths;
  const seg = lens.indexOf(Math.max(...lens));
  const pos = spline.positionAt(seg + 0.5);
  const tan = V.norm(V.sub(
    spline.positionAt(seg + 0.51), spline.positionAt(seg + 0.49)
  ));
  spline.points.splice(seg + 1, 0, pos);
  const shifted = {};
  for (const [k, dir] of Object.entries(spline.tangentOverrides)) {
    const ki = Number(k);
    shifted[ki >= seg + 1 ? ki + 1 : ki] = dir;
  }
  shifted[seg + 1] = tan;
  const fresh = new Spline(spline.points, shifted, spline.alpha);
  spline.baseTangents = fresh.baseTangents;
  spline.tangentOverrides = fresh.tangentOverrides;
  spline._segLengths = null;
  return seg + 1;
}

// Locked-speed reference is a speed table over arc length; geometry changes
// re-parameterise it by fractional arc position.
function alignSpeeds(samples, lockedSpeed, aircraft) {
  if (!lockedSpeed || !lockedSpeed.samples?.length) {
    return samples.map(() => aircraft.cruiseSpeed ?? aircraft.vMax * 0.6);
  }
  const ref = lockedSpeed.samples;
  const table = lockedSpeed.speeds;
  const total = Math.max(samples[samples.length - 1].s, 1e-6);
  const refTotal = Math.max(ref[ref.length - 1].s, 1e-6);
  return samples.map((smp) => {
    const targetS = (smp.s / total) * refTotal;
    let lo = 0, hi = ref.length - 1;
    while (lo < hi - 1) {
      const mid = (lo + hi) >> 1;
      if (ref[mid].s <= targetS) lo = mid; else hi = mid;
    }
    const span = Math.max(ref[hi].s - ref[lo].s, 1e-6);
    const t = clamp((targetS - ref[lo].s) / span, 0, 1);
    return table[lo] + (table[hi] - table[lo]) * t;
  });
}

function violationScore(samples, speeds, aircraft, volumes, craftRadius) {
  let energy = 0;
  let count = 0;
  for (let i = 0; i < samples.length; i++) {
    const smp = samples[i];
    const kMax = curvatureCeiling(speeds[i], aircraft);
    if (smp.curvature > kMax) {
      energy += (smp.curvature - kMax) / kMax;
      count++;
    }
    if (Math.abs(smp.climbAngle) > aircraft.gammaMax) {
      energy += Math.abs(smp.climbAngle) - aircraft.gammaMax;
      count++;
    }
    const vz = Math.abs(speeds[i] * Math.sin(smp.climbAngle));
    if (vz > aircraft.vzMax) {
      energy += (vz - aircraft.vzMax) / aircraft.vzMax;
      count++;
    }
    for (const vol of volumes) {
      const d = distanceToVolume(smp.pos, vol);
      if (d < craftRadius) {
        energy += (craftRadius - d) / craftRadius;
        count++;
      }
    }
  }
  return { energy, count };
}

function residualViolations(samples, speeds, aircraft, volumes, craftRadius) {
  const out = [];
  for (let i = 0; i < samples.length; i++) {
    const smp = samples[i];
    const kMax = curvatureCeiling(speeds[i], aircraft);
    if (smp.curvature > kMax) {
      out.push({
        type: 'turn-radius', severity: 'error', s: smp.s,
        message: `重塑后 s=${smp.s.toFixed(0)}m 曲率仍超限（R=${(1 / smp.curvature).toFixed(0)}m）`,
      });
    }
    if (Math.abs(smp.climbAngle) > aircraft.gammaMax) {
      out.push({
        type: 'climb-angle', severity: 'error', s: smp.s,
        message: `重塑后 s=${smp.s.toFixed(0)}m 爬升角仍超限`,
      });
    }
    for (const vol of volumes) {
      const d = distanceToVolume(smp.pos, vol);
      if (d < craftRadius) {
        out.push({
          type: 'volume-clearance', severity: 'error', s: smp.s,
          message: `重塑后仍侵入障碍 ${vol.id}（净空 ${d.toFixed(1)}m）`,
        });
      }
    }
  }
  return out;
}

// Main entry.
export function reshapeCourse({
  points, gates, volumes, aircraft, craftRadius,
  lockedSpeed, lockedGateIndices, maxIterations = 60, maxExtraPoints = 6,
  sampleDs = 2,
}) {
  const locked = new Set(lockedGateIndices ?? []);
  const spline = new Spline(points, {});
  const history = [];
  let inserted = 0;

  for (let outer = 0; outer <= maxExtraPoints; outer++) {
    for (let it = 0; it < maxIterations; it++) {
      const samples = spline.sample(sampleDs);
      const speeds = alignSpeeds(samples, lockedSpeed, aircraft);
      const moved = relaxationSweep(
        spline, samples, speeds, aircraft, volumes ?? [], locked, craftRadius
      );
      const score = violationScore(samples, speeds, aircraft, volumes ?? [], craftRadius);
      history.push({ iteration: it, inserted, moved, energy: score.energy, count: score.count });
      if (score.energy < 1e-4) {
        const finalSamples = spline.sample(sampleDs);
        const finalSpeeds = alignSpeeds(finalSamples, lockedSpeed, aircraft);
        return {
          converged: true, spline,
          points: spline.points.map(V.clone),
          tangentOverrides: { ...spline.tangentOverrides },
          samples: finalSamples, speeds: finalSpeeds, history,
          residualViolations: [], insertedPoints: inserted,
        };
      }
      if (moved === 0) break;
    }
    if (inserted >= maxExtraPoints) break;
    insertMidpoint(spline);
    inserted++;
  }

  const samples = spline.sample(sampleDs);
  const speeds = alignSpeeds(samples, lockedSpeed, aircraft);
  return {
    converged: false, spline,
    points: spline.points.map(V.clone),
    tangentOverrides: { ...spline.tangentOverrides },
    samples, speeds, history,
    residualViolations: residualViolations(samples, speeds, aircraft, volumes ?? [], craftRadius),
    insertedPoints: inserted,
  };
}

export { curvatureCeiling };
