// Three.js scene construction for the course editor. All geometry is rebuilt
// from kernel data structures; this module never invents feasibility on its
// own — red segments come straight from the kernel's violation segments.

import * as THREE from 'three';
import { distanceToVolume } from '../kernel/math.js';

const COLORS = {
  course: 0x38bdf8,
  error: 0xf87171,
  warning: 0xfbbf24,
  gate: 0xfb923c,
  gateBad: 0xef4444,
  control: 0x22d3ee,
  guidance: 0xa78bfa,
  volume: 0x64748b,
  attitude: 0xe2e8f0,
  aircraft: 0xfde047,
};

export class CourseScene {
  // A WebGLRenderer is created from the canvas in the browser. In headless
  // tests a stub renderer (no GL context) can be injected to exercise the full
  // scene-graph construction path.
  constructor(canvas, renderer = null) {
    this.canvas = canvas;
    this.renderer = renderer ?? new THREE.WebGLRenderer({ canvas, antialias: true });
    if (this.renderer.setPixelRatio && typeof devicePixelRatio !== 'undefined') {
      this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    }
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0d1117);
    this.scene.fog = new THREE.Fog(0x0d1117, 250, 900);
    this.camera = new THREE.PerspectiveCamera(55, 1, 0.1, 3000);
    this.camera.position.set(180, 150, 220);

    this._lights();
    this._grid();
    this.root = new THREE.Group();
    this.scene.add(this.root);
    this.dragMeshes = [];
    this.groups = {};
    this.aircraft = null;
    this.clearanceLabels = [];
  }

  _lights() {
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const key = new THREE.DirectionalLight(0xffffff, 1.1);
    key.position.set(120, 220, 160);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x88aaff, 0.35);
    fill.position.set(-150, 80, -120);
    this.scene.add(fill);
  }

  _grid() {
    const grid = new THREE.GridHelper(1200, 60, 0x2a3340, 0x18202b);
    grid.position.y = -0.05;
    this.scene.add(grid);
    const axes = new THREE.AxesHelper(40);
    this.scene.add(axes);
  }

  resize() {
    const w = this.canvas?.clientWidth, h = this.canvas?.clientHeight;
    if (!w || !h || !this.renderer.setSize) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  clearRoot() {
    while (this.root.children.length) {
      const c = this.root.children.pop();
      c.traverse?.((o) => { o.geometry?.dispose?.(); o.material?.dispose?.(); });
    }
    this.dragMeshes = [];
    this.groups = {};
    this.clearanceLabels = [];
    this.aircraft = null;
  }

  // Build the full static representation of a kernel course object.
  renderCourse(course, opts = {}) {
    this.clearRoot();
    this.course = course;
    this._buildRibbon(course, opts);
    this._buildGates(course);
    if (opts.showVolumes !== false) this._buildVolumes(course);
    this._buildControlPoints(course);
    if (opts.showGuidance !== false) this._buildGuidance(course);
    if (opts.showAttitude !== false) this._buildAttitude(course);
    this._buildAircraft(course);
    if (opts.showClearance !== false) this._buildClearanceLabels(course);
    this._frameAll();
  }

  _group(name) {
    const g = new THREE.Group();
    g.name = name;
    this.groups[name] = g;
    this.root.add(g);
    return g;
  }

  // Tubular channel ribbon with violation segments overlaid in red/amber.
  _buildRibbon(course) {
    const g = this._group('ribbon');
    const pts = course.samples.map((s) => new THREE.Vector3(...s.pos));
    const baseCurve = new THREE.CatmullRomCurve3(pts, false, 'catmullrom', 0.2);
    const radius = course.aircraft.craftRadius;
    const tube = new THREE.Mesh(
      new THREE.TubeGeometry(baseCurve, Math.max(120, pts.length * 2), radius * 0.55, 10, false),
      new THREE.MeshStandardMaterial({
        color: COLORS.course, transparent: true, opacity: 0.28,
        emissive: COLORS.course, emissiveIntensity: 0.25,
      })
    );
    g.add(tube);

    // Centre line.
    const lineGeo = new THREE.BufferGeometry().setFromPoints(pts);
    g.add(new THREE.Line(lineGeo,
      new THREE.LineBasicMaterial({ color: 0xbfeaff, transparent: true, opacity: 0.7 })));

    // Violation overlays as short coloured tube spans located by arc length.
    const segs = course.evaluation.segments;
    for (const seg of segs) {
      const sub = ptsOnRange(course.samples, seg.s0, seg.s1);
      if (sub.length < 2) continue;
      const curve = new THREE.CatmullRomCurve3(sub, false, 'catmullrom', 0.2);
      const color = seg.severity === 'error' ? COLORS.error : COLORS.warning;
      const mesh = new THREE.Mesh(
        new THREE.TubeGeometry(curve, Math.max(8, sub.length * 3), radius * 0.72, 10, false),
        new THREE.MeshStandardMaterial({
          color, emissive: color, emissiveIntensity: 0.6,
          transparent: true, opacity: 0.92,
        })
      );
      mesh.userData.violation = seg;
      g.add(mesh);
    }
  }

  _buildGates(course) {
    const g = this._group('gates');
    const violated = new Set(
      course.evaluation.violations
        .filter((v) => v.gateId && v.severity === 'error').map((v) => v.gateId)
    );
    course.gates.forEach((gate, idx) => {
      const frame = gateGroup(gate, violated.has(gate.id));
      frame.userData = { kind: 'gate', index: idx, ref: gate };
      g.add(frame);
      this.dragMeshes.push(frame);
    });
  }

  _buildVolumes(course) {
    const g = this._group('volumes');
    for (const vol of course.volumes ?? []) {
      const mesh = volumeMesh(vol);
      mesh.userData = { kind: 'volume', ref: vol };
      g.add(mesh);
    }
  }

  _buildControlPoints(course) {
    const g = this._group('controls');
    course.points.forEach((p, idx) => {
      const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(2.4, 18, 18),
        new THREE.MeshStandardMaterial({ color: COLORS.control, emissive: COLORS.control, emissiveIntensity: 0.5 })
      );
      sphere.position.set(...p);
      sphere.userData = { kind: 'control', index: idx };
      g.add(sphere);
      this.dragMeshes.push(sphere);
    });
  }

  _buildGuidance(course) {
    const g = this._group('guidance');
    const headway = course.aircraft.guidanceHeadway ?? 2.5;
    // One guidance marker every ~12 m, positioned at the look-ahead point.
    for (let i = 0; i < course.samples.length; i += 6) {
      const s = course.samples[i];
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(course.aircraft.craftRadius * 0.7, course.aircraft.craftRadius, 24),
        new THREE.MeshBasicMaterial({ color: COLORS.guidance, transparent: true, opacity: 0.8, side: THREE.DoubleSide })
      );
      ring.position.set(...s.pos);
      ring.lookAt(new THREE.Vector3(...s.pos).add(new THREE.Vector3(...s.tan)));
      g.add(ring);
    }
  }

  // Sparse attitude triads (T/N/up) along the path prove frame stability.
  _buildAttitude(course) {
    const g = this._group('attitude');
    const len = 6;
    for (let i = 0; i < course.samples.length; i += 12) {
      const att = course.attitudes[i];
      const origin = new THREE.Vector3(...course.samples[i].pos);
      const tri = new THREE.Group();
      tri.add(axisMesh(origin, att.T, len, 0x38bdf8));
      tri.add(axisMesh(origin, att.right, len * 0.7, 0x34d399));
      tri.add(axisMesh(origin, att.up, len * 0.7, 0xf472b6));
      g.add(tri);
    }
  }

  _buildAircraft(course) {
    const g = this._group('aircraft');
    this.aircraft = aircraftMesh(course.aircraft.craftRadius);
    const s0 = course.samples[0];
    this.aircraft.position.set(...s0.pos);
    orientAircraft(this.aircraft, s0.tan, course.attitudes[0]?.up ?? [0, 1, 0]);
    g.add(this.aircraft);
    // Trajectory trail from the simulation.
    const trailPts = course.simulation.trace.map((t) => new THREE.Vector3(...t.pos));
    if (trailPts.length > 1) {
      const geo = new THREE.BufferGeometry().setFromPoints(trailPts);
      this.trail = new THREE.Line(geo,
        new THREE.LineBasicMaterial({ color: COLORS.aircraft, transparent: true, opacity: 0.85 }));
      g.add(this.trail);
    }
  }

  _buildClearanceLabels(course) {
    // Per-sample minimum distance to the nearest volume, shown as small ticks.
    const g = this._group('clearance');
    const mat = new THREE.LineBasicMaterial({ color: 0x94a3b8, transparent: true, opacity: 0.5 });
    for (let i = 0; i < course.samples.length; i += 10) {
      const smp = course.samples[i];
      let nearest = null, nd = Infinity;
      for (const vol of course.volumes ?? []) {
        const d = localDistance(smp.pos, vol);
        if (d < nd) { nd = d; nearest = vol; }
      }
      if (!nearest) continue;
      const target = nearest.center ?? smp.pos;
      const geo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(...smp.pos),
        new THREE.Vector3(...target),
      ]);
      g.add(new THREE.Line(geo, mat));
      this.clearanceLabels.push({ s: smp.s, distance: nd, pos: smp.pos });
    }
  }

  _frameAll() {
    const box = new THREE.Box3().setFromObject(this.root);
    if (!box.isEmpty()) {
      const center = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      const dir = new THREE.Vector3(0.9, 0.55, 1).normalize();
      const dist = Math.max(size.length() * 0.9, 120);
      this.camera.position.copy(center).add(dir.multiplyScalar(dist));
      this.camera.lookAt(center);
      this.camera.near = 0.5;
      this.camera.far = dist * 6;
      this.camera.updateProjectionMatrix();
    }
  }

  setLayerVisible(name, visible) {
    if (this.groups[name]) this.groups[name].visible = visible;
  }

  // Place the aircraft at a simulation trace index.
  updateAircraft(tracePoint, up) {
    if (!this.aircraft || !tracePoint) return;
    this.aircraft.position.set(...tracePoint.pos);
    const tan = [
      Math.cos(tracePoint.gamma) * Math.cos(tracePoint.heading),
      Math.sin(tracePoint.gamma),
      Math.cos(tracePoint.gamma) * Math.sin(tracePoint.heading),
    ];
    orientAircraft(this.aircraft, tan, up);
  }

  render() { this.renderer.render?.(this.scene, this.camera); }
}

// ---------- helpers ----------

function ptsOnRange(samples, s0, s1) {
  const out = [];
  for (const s of samples) {
    if (s.s >= s0 - 1e-6 && s.s <= s1 + 1e-6) out.push(new THREE.Vector3(...s.pos));
  }
  return out;
}

function gateGroup(gate, violated) {
  const w = gate.halfW, h = gate.halfH;
  const color = violated ? COLORS.gateBad : COLORS.gate;
  const mat = new THREE.MeshStandardMaterial({
    color, emissive: color, emissiveIntensity: 0.35, transparent: true, opacity: 0.9,
  });
  const grp = new THREE.Group();
  const t = 0.6;
  const bars = [
    [[0, h, 0], [w * 2 + t, t, t]], [[0, -h, 0], [w * 2 + t, t, t]],
    [[-w, 0, 0], [t, h * 2 + t, t]], [[w, 0, 0], [t, h * 2 + t, t]],
  ];
  for (const [pos, size] of bars) {
    const bar = new THREE.Mesh(new THREE.BoxGeometry(...size), mat);
    bar.position.set(...pos);
    grp.add(bar);
  }
  // Normal arrow indicating required travel direction.
  const arrow = new THREE.ArrowHelper(
    new THREE.Vector3(...gate.normal), new THREE.Vector3(0, 0, 0),
    Math.max(6, w), 0xffffff, 2.2, 1.4
  );
  grp.add(arrow);
  grp.position.set(...gate.pos);
  const n = new THREE.Vector3(...gate.normal).normalize();
  grp.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), n);
  return grp;
}

function volumeMesh(vol) {
  let geo;
  if (vol.type === 'sphere') geo = new THREE.SphereGeometry(vol.radius, 24, 18);
  else if (vol.type === 'cylinder') geo = new THREE.CylinderGeometry(vol.radius, vol.radius, vol.height, 24);
  else geo = new THREE.BoxGeometry(vol.half[0] * 2, vol.half[1] * 2, vol.half[2] * 2);
  const mat = new THREE.MeshStandardMaterial({
    color: COLORS.volume, transparent: true, opacity: 0.32,
    emissive: 0x334155, emissiveIntensity: 0.4,
  });
  const mesh = new THREE.Mesh(geo, mat);
  const wire = new THREE.LineSegments(
    new THREE.EdgesGeometry(geo),
    new THREE.LineBasicMaterial({ color: 0x94a3b8 })
  );
  mesh.add(wire);
  mesh.position.set(...vol.center);
  if (vol.yaw) mesh.rotation.y = vol.yaw;
  return mesh;
}

function axisMesh(origin, dir, len, color) {
  const geo = new THREE.BufferGeometry().setFromPoints([
    origin, origin.clone().add(new THREE.Vector3(...dir).multiplyScalar(len)),
  ]);
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
}

function aircraftMesh(radius) {
  const grp = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.ConeGeometry(radius * 0.9, radius * 3.4, 16),
    new THREE.MeshStandardMaterial({ color: COLORS.aircraft, emissive: COLORS.aircraft, emissiveIntensity: 0.4 })
  );
  body.rotation.x = Math.PI / 2; // cone points along +Z before orientation
  grp.add(body);
  const wing = new THREE.Mesh(
    new THREE.BoxGeometry(radius * 4.2, 0.3, radius * 1.1),
    new THREE.MeshStandardMaterial({ color: 0xf59e0b })
  );
  grp.add(wing);
  return grp;
}

function orientAircraft(mesh, tan, up) {
  const z = new THREE.Vector3(...tan).normalize();
  const y = new THREE.Vector3(...up).normalize();
  const x = new THREE.Vector3().crossVectors(y, z).normalize();
  const yy = new THREE.Vector3().crossVectors(z, x).normalize();
  const m = new THREE.Matrix4().makeBasis(x, yy, z);
  mesh.quaternion.setFromRotationMatrix(m);
}

function localDistance(pos, vol) { return distanceToVolume(pos, vol); }
