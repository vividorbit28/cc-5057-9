// Geometric and dynamic envelope checks over a sampled course.
// Every check produces typed, located violations so the editor can highlight
// exactly which stretches of the corridor are not flyable.

import { V, angleDiff, clamp, distanceToVolume, segmentClearance, segmentSegmentDistance } from './math.js';

// Gate: {id, pos, normal (unit, direction of travel), halfW, halfH,
//        required, vMin?, vMax?, label?}
// Volume: see math.js (box / sphere / cylinder).

function gateFrame(gate) {
  const n = V.norm(gate.normal);
  const worldUp = [0, 1, 0];
  // Right-handed gate frame: axisH points to the RIGHT of travel, axisV up.
  let axisH = V.cross(n, worldUp);
  if (V.len2(axisH) < 1e-8) axisH = [1, 0, 0];
  axisH = V.norm(axisH);
  const axisV = V.norm(V.cross(axisH, n));
  return { n, axisH, axisV };
}

// Locate where the path crosses each gate plane. Folded routes can pierce a
// gate plane several times, so crossings are collected in path order and the
// caller picks the next FORWARD crossing after the previous gate. Each spline
// segment is scanned densely in parameter space (independent of sample ds).
function findGateCrossings(gate, samples, spline) {
  const out = [];
  const consider = (pos, tan, score) => out.push({ pos, tan, score });
  if (spline) {
    const perSeg = 24;
    for (let seg = 0; seg < spline.segmentCount; seg++) {
      let prevP = spline.positionAt(seg);
      let prevD = V.dot(V.sub(prevP, gate.pos), gate.normal);
      for (let k = 1; k <= perSeg; k++) {
        const u = seg + k / perSeg;
        const p = spline.positionAt(u);
        const d = V.dot(V.sub(p, gate.pos), gate.normal);
        if (prevD === 0 || prevD * d <= 0) {
          const t = prevD === d ? 0 : prevD / (prevD - d);
          const tan = V.norm(V.sub(spline.positionAt(u + 0.002),
            spline.positionAt(Math.max(0, u - 0.002))));
          consider(V.lerp(prevP, p, t), tan, Math.abs(d - prevD));
        }
        prevP = p;
        prevD = d;
      }
    }
  }
  for (let i = 1; i < samples.length; i++) {
    const d0 = V.dot(V.sub(samples[i - 1].pos, gate.pos), gate.normal);
    const d1 = V.dot(V.sub(samples[i].pos, gate.pos), gate.normal);
    if (d0 === 0 || d0 * d1 <= 0) {
      const t = d0 === d1 ? 0 : d0 / (d0 - d1);
      const pos = V.lerp(samples[i - 1].pos, samples[i].pos, t);
      const tan = V.norm(V.lerp(samples[i - 1].tan, samples[i].tan, t));
      consider(pos, tan, Math.abs(d1 - d0));
    }
  }
  // Deduplicate near-identical hits from the two scan sources.
  const dedup = [];
  for (const c of out) {
    if (!dedup.some((d) => V.dist2(d.pos, c.pos) < 0.25)) dedup.push(c);
  }
  return dedup;
}

function checkGates(gates, samples, craftRadius, spline) {
  const violations = [];
  const crossings = {};
  let lastS = -Infinity;
  for (const gate of gates) {
    const all = findGateCrossings(gate, samples, spline);
    const { n, axisH, axisV } = gateFrame(gate);
    // Pick the first forward crossing after the previous gate; on folded
    // routes this skips the same plane being pierced on a different leg.
    const ordered = all
      .map((c) => ({ ...c, s: nearestS(samples, c.pos) }))
      .filter((c) => c.s >= lastS - 1e-6)
      .sort((a, b) => a.s - b.s);
    const cross = ordered.find((c) => V.dot(c.tan, n) > 0) ?? ordered[0];
    if (!cross) {
      if (gate.required) {
        violations.push({
          type: 'gate-missed', gateId: gate.id, severity: 'error',
          s0: 0, s1: samples.length ? samples[samples.length - 1].s : 0,
          message: `未穿越门 ${gate.label ?? gate.id}`,
        });
      }
      // Even a missed gate still contributes its speed window at the nearest
      // station, so an impossible vMin/vMax is reported rather than silently
      // dropped from the speed-planning pass.
      if (samples.length) gate._crossS = nearestS(samples, gate.pos);
      continue;
    }
    lastS = cross.s;
    crossings[gate.id] = cross;
    const rel = V.sub(cross.pos, gate.pos);
    const x = V.dot(rel, axisH);
    const y = V.dot(rel, axisV);
    const freeW = gate.halfW - craftRadius;
    const freeH = gate.halfH - craftRadius;
    if (Math.abs(x) > freeW || Math.abs(y) > freeH) {
      violations.push({
        type: 'gate-clearance', gateId: gate.id, severity: 'error',
        s: cross.s,
        message: `门 ${gate.label ?? gate.id} 内超出净空（横向 ${x.toFixed(1)}m / 限 ±${freeW.toFixed(1)}m，` +
          `竖向 ${y.toFixed(1)}m / 限 ±${freeH.toFixed(1)}m）`,
      });
    }
    // Heading alignment: tangent must point through the gate plane.
    const align = V.dot(cross.tan, n);
    if (align < Math.cos(gate.alignLimit ?? Math.PI / 3)) {
      violations.push({
        type: 'gate-alignment', gateId: gate.id, severity: 'error',
        s: cross.s,
        message: `门 ${gate.label ?? gate.id} 穿越航向偏差过大（cos=${align.toFixed(2)}）`,
      });
    }
    // Stored for the speed-profile pass.
    gate._crossS = cross.s;
  }
  return { violations, crossings };
}

function nearestS(samples, pos) {
  let best = Infinity, bs = 0;
  for (const s of samples) {
    const d = V.dist2(s.pos, pos);
    if (d < best) { best = d; bs = s.s; }
  }
  return bs;
}

function checkVolumes(volumes, samples, craftRadius) {
  const violations = [];
  for (const vol of volumes) {
    for (let i = 0; i < samples.length - 1; i++) {
      const clear = segmentClearance(samples[i].pos, samples[i + 1].pos, vol);
      if (clear < craftRadius) {
        violations.push({
          // Clearance below the collision radius IS a collision, not a hint.
          type: 'volume-clearance', severity: 'error',
          volumeId: vol.id, s0: samples[i].s, s1: samples[i + 1].s,
          clearance: clear,
          message: `障碍 ${vol.id} 净空 ${clear.toFixed(2)}m < 碰撞半径 ${craftRadius}m`,
        });
      }
    }
  }
  return violations;
}

// Self-approach: non-adjacent corridor stretches must stay further apart than
// the channel diameter plus a visible separation margin. Stations within
// `ignoreArc` metres of each other along the path are the SAME local stretch
// (the ribbon naturally runs parallel to itself there) and are ignored.
function checkSelfProximity(samples, craftRadius, minSeparation, ignoreArc = 30) {
  const violations = [];
  const need = Math.max(minSeparation, 2 * craftRadius);
  for (let i = 0; i < samples.length - 1; i++) {
    for (let j = i + 1; j < samples.length - 1; j++) {
      if (samples[j].s - samples[i].s < ignoreArc) continue;
      const d = segmentSegmentDistance(
        samples[i].pos, samples[i + 1].pos,
        samples[j].pos, samples[j + 1].pos
      );
      if (d < need) {
        violations.push({
          type: 'self-proximity', severity: 'warning',
          s0: samples[i].s, s1: samples[i + 1].s,
          s2: samples[j].s, s3: samples[j + 1].s,
          distance: d,
          message: `航道在 s=${samples[i].s.toFixed(0)}m 与 s=${samples[j].s.toFixed(0)}m 处相距仅 ${d.toFixed(1)}m（通道需 ${need.toFixed(1)}m）`,
        });
      }
    }
  }
  return violations;
}

// Dynamic envelope against the per-sample commanded speed and attitude.
function checkDynamics(samples, attitudes, speeds, aircraft) {
  const violations = [];
  const g = aircraft.g ?? 9.81;
  let prevClimb = samples[0]?.climbAngle ?? 0;
  let prevS = 0;
  for (let i = 0; i < samples.length; i++) {
    const smp = samples[i];
    const v = speeds[i];
    const att = attitudes[i];
    // 1. Minimum turn radius R = 1/kappa >= v^2 / (g tan phiMax).
    if (smp.curvature > 1e-6) {
      const rMin = (v * v) / (g * Math.tan(aircraft.phiMax));
      const rHave = 1 / smp.curvature;
      if (rHave < rMin) {
        violations.push({
          type: 'turn-radius', severity: 'error', s: smp.s,
          radius: rHave, required: rMin, speed: v,
          message: `转弯半径 ${rHave.toFixed(0)}m < 速度 ${v.toFixed(1)}m/s 下所需 ${rMin.toFixed(0)}m`,
        });
      }
    }
    // 2. Climb angle limit.
    if (Math.abs(smp.climbAngle) > aircraft.gammaMax + 1e-4) {
      violations.push({
        type: 'climb-angle', severity: 'error', s: smp.s,
        value: smp.climbAngle, limit: aircraft.gammaMax,
        message: `爬升角 ${(smp.climbAngle * 180 / Math.PI).toFixed(1)}° 超过 ±${(aircraft.gammaMax * 180 / Math.PI).toFixed(0)}°`,
      });
    }
    // 3. Climb-rate limit: vertical speed v sin gamma <= vzMax.
    const vz = v * Math.sin(smp.climbAngle);
    if (Math.abs(vz) > aircraft.vzMax + 1e-3) {
      violations.push({
        type: 'climb-rate', severity: 'error', s: smp.s,
        value: vz, limit: aircraft.vzMax,
        message: `垂直速率 ${vz.toFixed(1)}m/s 超过 ±${aircraft.vzMax}m/s`,
      });
    }
    // 4. Smoothness of the climb profile (gamma rate of change).
    if (i > 0) {
      const ds = smp.s - prevS;
      const dt = ds / Math.max(v, 1);
      const dGamma = angleDiff(smp.climbAngle, prevClimb);
      if (dt > 0 && Math.abs(dGamma) / dt > aircraft.gammaRateMax) {
        violations.push({
          type: 'climb-rate-change', severity: 'warning', s: smp.s,
          value: Math.abs(dGamma) / dt, limit: aircraft.gammaRateMax,
          message: `爬升角变化率 ${(Math.abs(dGamma) / dt * 180 / Math.PI).toFixed(1)}°/s 超限`,
        });
      }
    }
    prevClimb = smp.climbAngle;
    prevS = smp.s;
    // 5. Bank / bank-rate envelope and straight-section roll stability.
    if (att) {
      if (Math.abs(att.bank) > aircraft.phiMax + 1e-4) {
        violations.push({
          type: 'bank', severity: 'error', s: smp.s,
          value: att.bank, limit: aircraft.phiMax,
          message: `倾斜角 ${(att.bank * 180 / Math.PI).toFixed(1)}° 超限`,
        });
      }
      if (Math.abs(att.bankRate) > aircraft.bankRateMax + 1e-3) {
        violations.push({
          type: 'bank-rate', severity: 'error', s: smp.s,
          value: att.bankRate, limit: aircraft.bankRateMax,
          message: `滚转速率超限`,
        });
      }
      if (att.rollJump) {
        violations.push({
          type: 'frame-jump', severity: 'warning', s: smp.s,
          message: '近直线段姿态框架发生突变（翻滚不连续）',
        });
      }
    }
  }
  return violations;
}

// Merge point/short violations into contiguous s-ranges for ribbon rendering.
export function aggregateSegments(violations, maxGap = 6) {
  const typed = {};
  for (const v of violations) {
    const s0 = v.s0 ?? v.s ?? 0;
    const s1 = v.s1 ?? v.s ?? s0;
    (typed[v.type] ??= []).push({ ...v, s0, s1: Math.max(s1, s0 + 1) });
  }
  const segments = [];
  for (const [type, list] of Object.entries(typed)) {
    list.sort((a, b) => a.s0 - b.s0);
    let cur = null;
    for (const item of list) {
      if (cur && item.s0 <= cur.s1 + maxGap && item.severity === cur.severity) {
        cur.s1 = Math.max(cur.s1, item.s1);
        cur.messages.add(item.message);
      } else {
        if (cur) segments.push(cur);
        cur = {
          type, severity: item.severity, s0: item.s0, s1: item.s1,
          messages: new Set([item.message]),
        };
      }
    }
    if (cur) segments.push(cur);
  }
  return segments.map((s) => ({ ...s, messages: [...s.messages] }))
    .sort((a, b) => a.s0 - b.s0);
}

// Full evaluation. `course` = {gates, volumes, aircraft, minSeparation,
// craftRadius, samples, attitudes, speeds}.
export function evaluateCourse(course) {
  const { gates, volumes, aircraft } = course;
  const craftRadius = course.craftRadius ?? 2;
  const violations = [];
  const gateResult = checkGates(gates, course.samples, craftRadius, course.spline);
  violations.push(...gateResult.violations);
  violations.push(...checkVolumes(volumes ?? [], course.samples, craftRadius));
  violations.push(...checkSelfProximity(
    course.samples, craftRadius, course.minSeparation ?? craftRadius * 2.5,
    course.ignoreArc ?? 30
  ));
  violations.push(...checkDynamics(
    course.samples, course.attitudes ?? [], course.speeds, aircraft
  ));
  const segments = aggregateSegments(violations);
  const errors = violations.filter((v) => v.severity === 'error');
  // Actual minimum obstacle clearance, computed from segment clearances on the
  // sampled path. The editor shows this number directly; it is never inferred
  // from whether a violation was emitted.
  let minClearance = null;
  let minClearanceVolumeId = null;
  for (const vol of volumes ?? []) {
    for (let i = 0; i < course.samples.length - 1; i++) {
      const clear = segmentClearance(course.samples[i].pos, course.samples[i + 1].pos, vol);
      if (minClearance == null || clear < minClearance) {
        minClearance = clear;
        minClearanceVolumeId = vol.id;
      }
    }
  }
  return {
    feasible: errors.length === 0,
    violations,
    segments,
    crossings: gateResult.crossings,
    stats: {
      length: course.samples.length ? course.samples[course.samples.length - 1].s : 0,
      maxCurvature: Math.max(0, ...course.samples.map((s) => s.curvature)),
      minRadius: (() => {
        const k = Math.max(1e-9, ...course.samples.map((s) => s.curvature));
        return 1 / k;
      })(),
      maxClimb: Math.max(0, ...course.samples.map((s) => Math.abs(s.climbAngle))),
      minClearance,
      minClearanceVolumeId,
      craftRadius,
      errorCount: errors.length,
      warningCount: violations.length - errors.length,
    },
  };
}

export { gateFrame, distanceToVolume };
