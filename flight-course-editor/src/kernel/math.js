// 3D vector math (plain mutable arrays [x, y, z]) and geometric primitives.
// All units are SI: metres, seconds, radians.

export const V = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ],
  len: (a) => Math.hypot(a[0], a[1], a[2]),
  len2: (a) => a[0] * a[0] + a[1] * a[1] + a[2] * a[2],
  dist: (a, b) => V.len(V.sub(a, b)),
  dist2: (a, b) => V.len2(V.sub(a, b)),
  norm(a) {
    const l = V.len(a);
    return l > 1e-12 ? [a[0] / l, a[1] / l, a[2] / l] : [0, 0, 0];
  },
  lerp: (a, b, t) => [
    a[0] + (b[0] - a[0]) * t,
    a[1] + (b[1] - a[1]) * t,
    a[2] + (b[2] - a[2]) * t,
  ],
  clone: (a) => [a[0], a[1], a[2]],
  neg: (a) => [-a[0], -a[1], -a[2]],
};

export const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

// Signed horizontal distance from a point to an oriented vertical box.
// Box: {center:[x,y,z], half:[hx,hy,hz], yaw} (yaw rotates around Y).
function toBoxLocal(p, box) {
  const d = V.sub(p, box.center);
  const c = Math.cos(-box.yaw || 0);
  const s = Math.sin(-box.yaw || 0);
  return [d[0] * c - d[2] * s, d[1], d[0] * s + d[2] * c];
}

// Distance to the surface of a volume (0 if inside). Supported types:
//  box     {type:'box', center, half, yaw}
//  sphere  {type:'sphere', center, radius}
//  cylinder {type:'cylinder', center, radius, height, yaw?} axis along Y
export function distanceToVolume(p, vol) {
  if (vol.type === 'sphere') {
    return Math.max(0, V.dist(p, vol.center) - vol.radius);
  }
  if (vol.type === 'box') {
    const q = toBoxLocal(p, vol);
    const dx = Math.max(Math.abs(q[0]) - vol.half[0], 0);
    const dy = Math.max(Math.abs(q[1]) - vol.half[1], 0);
    const dz = Math.max(Math.abs(q[2]) - vol.half[2], 0);
    return Math.hypot(dx, dy, dz);
  }
  if (vol.type === 'cylinder') {
    const d = V.sub(p, vol.center);
    const radial = Math.hypot(d[0], d[2]) - vol.radius;
    const vert = Math.abs(d[1]) - vol.height / 2;
    const r = Math.max(radial, 0);
    const v = Math.max(vert, 0);
    return Math.hypot(r, v);
  }
  throw new Error(`unknown volume type: ${vol.type}`);
}

export function pointInsideVolume(p, vol, margin = 0) {
  return distanceToVolume(p, vol) <= margin;
}

// Minimum clearance of a segment a-b to a volume, sampled adaptively.
// Returns the smallest surface distance found along the segment.
export function segmentClearance(a, b, vol) {
  let worst = Infinity;
  const stack = [[a, b, 0]];
  while (stack.length) {
    const [p0, p1, depth] = stack.pop();
    const d0 = distanceToVolume(p0, vol);
    const d1 = distanceToVolume(p1, vol);
    const mid = V.scale(V.add(p0, p1), 0.5);
    const dm = distanceToVolume(mid, vol);
    worst = Math.min(worst, d0, d1, dm);
    if (depth < 6) {
      // subdivide while the chord is long relative to current clearance
      const chord = V.dist(p0, p1);
      if (chord > Math.max(1, worst) * 0.5) {
        stack.push([p0, mid, depth + 1], [mid, p1, depth + 1]);
      }
    }
  }
  return worst;
}

// Shortest distance between two finite segments.
export function segmentSegmentDistance(p1, q1, p2, q2) {
  const d1 = V.sub(q1, p1);
  const d2 = V.sub(q2, p2);
  const r = V.sub(p1, p2);
  const a = V.dot(d1, d1);
  const e = V.dot(d2, d2);
  const f = V.dot(d2, r);
  let s, t;
  if (a <= 1e-12 && e <= 1e-12) return V.dist(p1, p2);
  if (a <= 1e-12) {
    s = 0;
    t = clamp(f / e, 0, 1);
  } else {
    const c = V.dot(d1, r);
    if (e <= 1e-12) {
      t = 0;
      s = clamp(-c / a, 0, 1);
    } else {
      const b = V.dot(d1, d2);
      const denom = a * e - b * b;
      s = denom > 1e-12 ? clamp((b * f - c * e) / denom, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); }
      else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
    }
  }
  const c1 = V.add(p1, V.scale(d1, s));
  const c2 = V.add(p2, V.scale(d2, t));
  return V.dist(c1, c2);
}

export function clampAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

// Shortest signed angular difference a-b in (-pi, pi].
export function angleDiff(a, b) {
  return clampAngle(a - b);
}

export const rad = (deg) => (deg * Math.PI) / 180;
