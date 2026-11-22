// Supported-aircraft motion model and course-following simulation.
//
// The aircraft is a 3D point-mass / kinematic model with FINITE limits:
//   speed      v in [vMin, vMax], longitudinal accel <= aMax/aDecel
//   turn       yaw/flight-path curvature <= g tan(phiMax) / v^2
//   bank       |phi| <= phiMax, roll rate <= bankRateMax
//   climb      |gamma| <= gammaMax, |v sin gamma| <= vzMax
// Guidance feeds a look-ahead target; a PD path-follower commands heading and
// flight-path angle. If the geometry cannot be tracked within these limits the
// run records a failure instead of teleporting the aircraft back onto the line.

import { V, clamp, angleDiff } from './math.js';
import { buildRMF, buildAttitude } from './frame.js';

const gConst = 9.81;

function makeController(samples, speeds, aircraft) {
  const headway = aircraft.guidanceHeadway ?? 2.5;
  return {
    // Arc-length index of the station nearest a position. The window must
    // tolerate speed changes: under heavy deceleration the true nearest
    // station can lag the hinted one by more than a fixed-size window.
    nearestIndex(pos, hint = 0) {
      let best = Infinity, bi = hint;
      for (let i = 0; i < samples.length; i++) {
        const d = V.dist2(samples[i].pos, pos);
        if (d < best) { best = d; bi = i; }
      }
      return bi;
    },
    targetAt(s, v) {
      const sLook = s + v * headway;
      // binary-ish walk forward
      let i = 0;
      while (i < samples.length - 1 && samples[i].s < sLook) i++;
      return { station: i, pos: samples[Math.min(i, samples.length - 1)].pos };
    },
    speedAtStation(i) {
      return speeds[clamp(i, 0, speeds.length - 1)];
    },
  };
}

// One simulation run. dt is the integration step (s); returns the full trace.
export function simulate({ samples, speeds, aircraft, dt = 0.05, maxTime = 600,
  startOffset = null }) {
  const ctrl = makeController(samples, speeds, aircraft);
  const start = samples[0].pos;
  // Optional lateral injection proves the follower must converge, not spawn
  // exactly on the line.
  const pos = startOffset ? V.add(start, startOffset) : V.clone(start);
  let v = speeds[0];
  let heading = Math.atan2(samples[0].tan[2], samples[0].tan[0]);
  let gamma = Math.asin(clamp(samples[0].tan[1], -1, 1));
  let phi = 0;
  let hint = 0;
  const trace = [];
  const failures = [];
  const trackingErrors = [];
  let t = 0;
  let finished = false;
  const endS = samples[samples.length - 1].s;
  const kTurnGain = aircraft.turnGain ?? 1.4;
  const kYawDamping = aircraft.yawDamping ?? 0.6;
  const kGammaGain = aircraft.gammaGain ?? 2.6;
  const kCrosstrack = aircraft.crosstrackGain ?? 0.25;
  const kCrosstrackD = aircraft.crosstrackDamping ?? 0.7;
  let prevOffset = 0;

  while (t <= maxTime) {
    const i = ctrl.nearestIndex(pos, hint);
    hint = i;
    const station = samples[i];
    const target = ctrl.targetAt(station.s, v);
    const desired = V.sub(target.pos, pos);
    const desiredHeading = Math.atan2(desired[2], desired[0]);
    const horizDist = Math.hypot(desired[0], desired[2]);
    const desiredGamma = Math.atan2(desired[1], Math.max(horizDist, 1e-6));

    // --- limited bank / turn dynamics ---
    // Damped cross-track PD law on heading, blended with look-ahead pursuit.
    // Bank itself is damped against the current yaw rate so the aircraft does
    // not slam to phiMax and oscillate across the line.
    const refTan = station.tan;
    const refHeading = Math.atan2(refTan[2], refTan[0]);
    const rightAxis = V.norm(V.cross(refTan, [0, 1, 0]));
    const signedOffset = V.dot(V.sub(pos, station.pos), rightAxis);
    const offsetRate = dt > 0 ? (signedOffset - prevOffset) / dt : 0;
    prevOffset = signedOffset;
    const recovery = -Math.atan(clamp(
      kCrosstrack * signedOffset + (kCrosstrackD * offsetRate) / Math.max(v, 1),
      -1, 1
    ));
    const pursuitW = clamp(1 - Math.min(Math.abs(signedOffset) / 12, 1), 0.2, 1);
    const headingCmd = refHeading + recovery
      + pursuitW * angleDiff(desiredHeading, refHeading);
    const yawRateNow = (gConst * Math.tan(phi)) / Math.max(v, 1);
    const yawErr = angleDiff(headingCmd, heading) - kYawDamping * yawRateNow;
    const phiCmd = clamp(kTurnGain * yawErr, -aircraft.phiMax, aircraft.phiMax);
    const maxRollStep = aircraft.bankRateMax * dt;
    phi += clamp(phiCmd - phi, -maxRollStep, maxRollStep);
    phi = clamp(phi, -aircraft.phiMax, aircraft.phiMax);
    const yawRate = (gConst * Math.tan(phi)) / Math.max(v, 1);
    heading = angleDiff(heading + yawRate * dt, 0);

    // --- limited flight-path-angle dynamics ---
    // Blend the look-ahead target gamma with the local reference tangent so a
    // dive/climb entry is tracked without waiting for the far target point.
    const refGamma = Math.asin(clamp(station.tan[1], -1, 1));
    const gammaCmd = clamp(0.55 * refGamma + 0.45 * desiredGamma,
      -aircraft.gammaMax, aircraft.gammaMax);
    const gammaErr = angleDiff(gammaCmd, gamma);
    const gammaStep = (aircraft.gammaRateMax ?? 0.5) * dt;
    gamma += clamp(gammaErr, -gammaStep, gammaStep);
    gamma = clamp(gamma, -aircraft.gammaMax, aircraft.gammaMax);

    // --- speed control with accel/decel limits ---
    const vCmd = ctrl.speedAtStation(target.station);
    const dv = vCmd - v;
    const lim = dv >= 0 ? aircraft.aMax : aircraft.aDecel;
    v += clamp(dv, -lim * dt, lim * dt);
    v = clamp(v, aircraft.vMin, aircraft.vMax);

    // --- integrate ---
    const horiz = v * Math.cos(gamma);
    pos[0] += horiz * Math.cos(heading) * dt;
    pos[2] += horiz * Math.sin(heading) * dt;
    pos[1] += v * Math.sin(gamma) * dt;

    // --- enforce climb-rate envelope explicitly ---
    const vz = v * Math.sin(gamma);
    if (Math.abs(vz) > aircraft.vzMax + 1e-6) {
      failures.push({ t, s: station.s, type: 'climb-rate',
        message: `仿真中垂直速率 ${vz.toFixed(1)}m/s 超限` });
    }

    // --- tracking error: true minimum distance to the reference polyline.
    // Scanning all segments keeps the metric honest when the aircraft lags the
    // hinted station under heavy deceleration, instead of mis-blaming a chord.
    const crossTrack = crossTrackErrorGlobal(pos, samples);
    const lag = station.s - pathProgress(samples, pos, i);
    trackingErrors.push(crossTrack);
    trace.push({
      t, pos: V.clone(pos), v, heading, gamma, phi,
      stationS: station.s, crossTrack, lag,
    });

    // Failure: aircraft wandered outside the recoverable channel.
    const abort = aircraft.abortCrossTrack ?? Infinity;
    if (crossTrack > abort) {
      failures.push({ t, s: station.s, type: 'lost-track',
        message: `横向偏差 ${crossTrack.toFixed(1)}m 超出可恢复通道，无法继续跟随` });
      break;
    }

    if (V.dist(pos, samples[samples.length - 1].pos) <= Math.max(2, v * dt * 2)) {
      finished = true;
      break;
    }
    t += dt;
  }

  const rmf = buildRMF(samples);
  const attitudes = buildAttitude(samples, rmf, aircraft, speeds);
  const rms = Math.sqrt(trackingErrors.reduce((a, e) => a + e * e, 0) / Math.max(1, trackingErrors.length));
  const peak = Math.max(0, ...trackingErrors);
  return {
    finished,
    time: t,
    trace,
    failures,
    tracking: {
      rms: rms,
      peak,
      final: trackingErrors[trackingErrors.length - 1] ?? 0,
      tolerance: aircraft.trackTolerance ?? craftTolerance(aircraft),
    },
    endS,
    attitudes,
    withinTolerance: peak <= (aircraft.trackTolerance ?? craftTolerance(aircraft)),
  };
}

function craftTolerance(aircraft) {
  return (aircraft.craftRadius ?? 2) * 0.8;
}

function crossTrackError(pos, samples, i) {
  const a = samples[Math.max(0, i - 1)].pos;
  const b = samples[Math.min(samples.length - 1, i)].pos;
  const ab = V.sub(b, a);
  const len2 = V.len2(ab);
  if (len2 < 1e-12) return V.dist(pos, a);
  const t = clamp(V.dot(V.sub(pos, a), ab) / len2, 0, 1);
  const foot = V.add(a, V.scale(ab, t));
  return V.dist(pos, foot);
}

// Global minimum distance to every reference segment (and its sign relative
// to the nearest station's right axis), used for the reported tracking metric.
function crossTrackErrorGlobal(pos, samples) {
  let best = Infinity;
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1].pos;
    const b = samples[i].pos;
    const ab = V.sub(b, a);
    const len2 = V.len2(ab);
    if (len2 < 1e-12) continue;
    const t = clamp(V.dot(V.sub(pos, a), ab) / len2, 0, 1);
    const foot = V.add(a, V.scale(ab, t));
    best = Math.min(best, V.dist(pos, foot));
  }
  return best;
}

function pathProgress(samples, pos, i) {
  return samples[clamp(i, 0, samples.length - 1)].s;
}
