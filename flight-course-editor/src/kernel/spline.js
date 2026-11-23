// Centripetal Catmull-Rom spline with per-point tangent overrides.
// A tangent override lets the reshaper bend the curve without moving a gate.

import { V, clamp } from './math.js';

const eps = 1e-9;

function catmullTangents(points, alpha = 0.5) {
  // Centripetal (alpha=0.5) parameterisation avoids cusps/loops at sharp data.
  const n = points.length;
  const t = new Array(n).fill(0);
  for (let i = 1; i < n; i++) {
    t[i] = t[i - 1] + Math.pow(V.dist(points[i - 1], points[i]), alpha);
  }
  const tan = [];
  for (let i = 0; i < n; i++) {
    if (i === 0) {
      tan.push(V.norm(V.sub(points[1], points[0])));
    } else if (i === n - 1) {
      tan.push(V.norm(V.sub(points[n - 1], points[n - 2])));
    } else {
      const p0 = points[i - 1], p1 = points[i], p2 = points[i + 1];
      const d01 = Math.pow(V.dist(p0, p1), alpha) || eps;
      const d12 = Math.pow(V.dist(p1, p2), alpha) || eps;
      // Standard centripetal Catmull-Rom tangent.
      const left = V.scale(V.sub(p1, p0), 1 / d01);
      const right = V.scale(V.sub(p2, p1), 1 / d12);
      tan.push(V.norm(V.add(left, right)));
    }
  }
  return tan;
}

function hermite(p0, m0, p1, m1, t) {
  const t2 = t * t, t3 = t2 * t;
  const h00 = 2 * t3 - 3 * t2 + 1;
  const h10 = t3 - 2 * t2 + t;
  const h01 = -2 * t3 + 3 * t2;
  const h11 = t3 - t2;
  return [0, 1, 2].map(
    (k) => h00 * p0[k] + h10 * m0[k] + h01 * p1[k] + h11 * m1[k]
  );
}

export class Spline {
  // points: array of [x,y,z]; tangentOverrides: optional map index -> unit vec
  constructor(points, tangentOverrides = {}, alpha = 0.5) {
    this.points = points.map(V.clone);
    this.baseTangents = catmullTangents(this.points, alpha);
    this.tangentOverrides = { ...tangentOverrides };
    this.alpha = alpha;
    this._segLengths = null;
  }

  tangentAt(i) {
    const ov = this.tangentOverrides[i];
    return ov ? V.norm(ov) : this.baseTangents[i];
  }

  setTangentOverride(i, dir) {
    if (dir) this.tangentOverrides[i] = V.norm(dir);
    else delete this.tangentOverrides[i];
    this._segLengths = null;
  }

  // Hermite handles scale with the local chord so overriding only the direction
  // still produces a curve whose magnitude is consistent with the neighbours.
  handleAt(i) {
    const n = this.points.length;
    const prev = this.points[Math.max(0, i - 1)];
    const next = this.points[Math.min(n - 1, i + 1)];
    const chord = V.dist(prev, next) / (i === 0 || i === n - 1 ? 1 : 2);
    return V.scale(this.tangentAt(i), Math.max(chord, eps));
  }

  positionAt(u) {
    // u in [0, n-1], segment-local evaluation.
    const n = this.points.length;
    const uu = clamp(u, 0, n - 1 - eps);
    const i = Math.min(n - 2, Math.floor(uu));
    const t = uu - i;
    return hermite(
      this.points[i], this.handleAt(i),
      this.points[i + 1], this.handleAt(i + 1), t
    );
  }

  // Analytic first and second derivatives of the Hermite segment containing u.
  // Sampling by arc length can land on either side of a knot; a finite
  // difference there straddles a C1 curvature jump and reports a spurious
  // spike. Evaluating the segment's own Hermite polynomials is exact.
  _derivs(u) {
    const n = this.points.length;
    const uu = clamp(u, 0, n - 1 - eps);
    const i = Math.min(n - 2, Math.floor(uu));
    const t = uu - i;
    const p0 = this.points[i], p1 = this.points[i + 1];
    const m0 = this.handleAt(i), m1 = this.handleAt(i + 1);
    const t2 = t * t, t3 = t2 * t;
    const h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t;
    const h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
    const dh00 = 6 * t2 - 6 * t, dh10 = 3 * t2 - 4 * t + 1;
    const dh01 = -6 * t2 + 6 * t, dh11 = 3 * t2 - 2 * t;
    const ddh00 = 12 * t - 6, ddh10 = 6 * t - 4;
    const ddh01 = -12 * t + 6, ddh11 = 6 * t - 2;
    const d1 = [0, 1, 2].map(
      (k) => dh00 * p0[k] + dh10 * m0[k] + dh01 * p1[k] + dh11 * m1[k]
    );
    const d2 = [0, 1, 2].map(
      (k) => ddh00 * p0[k] + ddh10 * m0[k] + ddh01 * p1[k] + ddh11 * m1[k]
    );
    return { d1, d2 };
  }

  get segmentCount() { return this.points.length - 1; }

  // Integrate arc length per segment once; cache until geometry changes.
  get segLengths() {
    if (this._segLengths) return this._segLengths;
    const lens = [];
    const steps = 40;
    for (let i = 0; i < this.segmentCount; i++) {
      let L = 0, prev = this.positionAt(i);
      for (let k = 1; k <= steps; k++) {
        const p = this.positionAt(i + k / steps);
        L += V.dist(prev, p);
        prev = p;
      }
      lens.push(L);
    }
    this._segLengths = lens;
    return lens;
  }

  get totalLength() {
    return this.segLengths.reduce((a, b) => a + b, 0);
  }

  // Map arc-length s (m from start) to parameter u.
  uAtLength(s) {
    const lens = this.segLengths;
    let rem = clamp(s, 0, this.totalLength);
    for (let i = 0; i < lens.length; i++) {
      if (rem <= lens[i] || i === lens.length - 1) {
        // refine within the segment with bisection on local length
        let lo = i, hi = i + 1;
        for (let it = 0; it < 24; it++) {
          const mid = (lo + hi) / 2;
          const local = this._localLength(i, mid - i);
          if (local < rem) lo = mid; else hi = mid;
        }
        return (lo + hi) / 2;
      }
      rem -= lens[i];
    }
    return this.segmentCount;
  }

  _localLength(seg, t) {
    const steps = Math.max(4, Math.ceil(t * 40));
    let L = 0, prev = this.positionAt(seg);
    for (let k = 1; k <= steps; k++) {
      const p = this.positionAt(seg + (t * k) / steps);
      L += V.dist(prev, p);
      prev = p;
    }
    return L;
  }

  // Dense, arc-length-uniform sampling with geometric diagnostics.
  // Returns samples [{s, u, pos, tan, curvature, climbAngle}].
  sample(ds = 2) {
    const total = this.totalLength;
    const out = [];
    for (let s = 0; s <= total + 1e-9; s += ds) {
      const ss = Math.min(s, total);
      const u = this.uAtLength(ss);
      const { d1, d2 } = this._derivs(u);
      const speed = V.len(d1);
      const tan = speed > eps ? V.scale(d1, 1 / speed) : [1, 0, 0];
      // kappa = |d1 x d2| / |d1|^3
      const cross = V.cross(d1, d2);
      const curvature = speed > eps ? V.len(cross) / Math.pow(speed, 3) : 0;
      const climbAngle = Math.asin(clamp(tan[1], -1, 1));
      out.push({ s: ss, u, pos: this.positionAt(u), tan, curvature, climbAngle });
    }
    return out;
  }
}
