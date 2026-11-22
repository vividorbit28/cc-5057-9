// Rotation-minimising frames along the sampled curve and banked attitude.
// A naive Frenet frame flips on near-straight sections; the double-reflection
// RMF (Wang et al. 2008) stays stable, and bank is layered on top of it.

import { V, clamp, clampAngle } from './math.js';

const WORLD_UP = [0, 1, 0];

function reflect(v, a) {
  // Reflection of v across the plane through the origin with normal a.
  const d = V.dot(v, a);
  return V.sub(v, V.scale(a, 2 * d));
}

// Build an RMF over arc-length samples. Each sample gets {T, N, B} where
// N is the transported normal and B = T x N.
export function buildRMF(samples) {
  if (!samples.length) return [];
  const t0 = samples[0].tan;
  // Pick an initial normal not parallel to the first tangent.
  let ref = Math.abs(V.dot(t0, WORLD_UP)) > 0.9 ? [1, 0, 0] : WORLD_UP;
  let n = V.norm(V.sub(ref, V.scale(t0, V.dot(ref, t0))));
  let frame = [];
  let prevP = samples[0].pos;
  let prevT = t0;
  for (let i = 0; i < samples.length; i++) {
    const { pos, tan } = samples[i];
    if (i > 0) {
      // Double reflection transport from previous station.
      const v1 = V.sub(pos, prevP);
      const c1 = V.scale(v1, 1 / V.len(v1));
      let nl = reflect(n, c1);
      const tl = reflect(prevT, c1);
      const v2 = V.sub(tan, tl);
      if (V.len2(v2) > 1e-14) {
        const c2 = V.scale(v2, 1 / V.len(v2));
        nl = reflect(nl, c2);
      }
      n = V.norm(nl);
    }
    const b = V.norm(V.cross(tan, n));
    n = V.norm(V.cross(b, tan)); // re-orthogonalise against drift
    frame.push({ T: tan, N: n, B: b });
    prevP = pos;
    prevT = tan;
  }
  return frame;
}

// Coordinated-turn bank from curvature and speed, rate-limited in time.
// aircraft: {g, phiMax, bankRateMax, straightKappa}
// Returns per-sample attitude with stability diagnostics.
export function buildAttitude(samples, rmf, aircraft, speeds) {
  const g = aircraft.g ?? 9.81;
  const phiMax = aircraft.phiMax ?? Math.PI / 3;
  const bankRateMax = aircraft.bankRateMax ?? 1.2;
  const straightKappa = aircraft.straightKappa ?? 1e-3;
  const out = [];
  let phiPrev = 0;
  for (let i = 0; i < samples.length; i++) {
    const smp = samples[i];
    const v = speeds[i];
    // Curvature normal: direction the path bends toward.
    const kappa = smp.curvature;
    // Sign of turn: curvature normal projected on the frame's right axis.
    // Approximate curvature normal from change of tangent between neighbours.
    let nCurv = rmf[i].N;
    if (i > 0 && i < samples.length - 1) {
      const dT = V.sub(samples[i + 1].tan, samples[i - 1].tan);
      const ds = samples[i + 1].s - samples[i - 1].s;
      if (ds > 0 && V.len2(dT) > 1e-14) nCurv = V.scale(dT, 1 / ds);
    }
    const turnSign = V.dot(nCurv, rmf[i].B) >= 0 ? 1 : -1;
    const phiDesired = kappa > straightKappa
      ? turnSign * Math.min(Math.atan((v * v * kappa) / g), phiMax)
      : 0;
    // Rate limit in time (dt estimated from spacing and local speed).
    const dt = i === 0
      ? 0
      : (smp.s - samples[i - 1].s) / Math.max(v, 1);
    let phi = phiDesired;
    if (dt > 0) {
      const maxStep = bankRateMax * dt;
      phi = clamp(phiDesired, phiPrev - maxStep, phiPrev + maxStep);
    }
    // Roll the RMF normal about T by phi to get the visual up vector.
    const { T, N, B } = rmf[i];
    const up = V.norm(V.add(
      V.scale(N, Math.cos(phi)),
      V.scale(B, -Math.sin(phi))
    ));
    const right = V.norm(V.cross(T, up));
    out.push({
      up, right, T,
      bank: phi,
      bankRate: dt > 0 ? clampAngle(phi - phiPrev) / dt : 0,
      phiDesired,
      onStraight: kappa <= straightKappa,
    });
    phiPrev = phi;
  }
  // Stability: sudden roll on near-straight sections. Numeric noise at near-zero
  // curvature is ignored; only a real twist while the bank is ~0 is flagged.
  for (let i = 1; i < out.length; i++) {
    const ds = samples[i].s - samples[i - 1].s;
    const dUp = V.len(V.sub(out[i].up, out[i - 1].up)) / Math.max(ds, 1e-6);
    out[i].frameTwistRate = dUp;
    out[i].rollJump = out[i].onStraight
      && Math.abs(out[i].bank) < 0.05
      && dUp > 0.08;
  }
  if (out.length) out[0].frameTwistRate = 0, out[0].rollJump = false;
  return out;
}
