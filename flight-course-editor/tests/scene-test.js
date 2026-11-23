// Headless editor test: builds the full Three.js scene graph for every
// scenario with a stub renderer (no WebGL context needed). This exercises the
// same renderCourse path the browser uses and proves violation segments, gates,
// volumes, guidance rings, attitude triads and the aircraft are all produced
// from kernel data rather than hand-waved into existence.
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { CourseScene } from '../src/editor/scene.js';
import {
  buildCourse, reshapeWithLockedSpeed, rescheduleCourse, compareCurveSolvers,
  SCENARIOS, V, distanceToVolume,
} from '../src/kernel/index.js';

let passed = 0;
const test = (name, fn) => {
  try { fn(); passed++; console.log('  ok -', name); }
  catch (e) { console.log('  FAIL -', name, e.message); process.exitCode = 1; }
};

const stubRenderer = {
  setPixelRatio() {}, setSize() {}, render() {},
};
const stubCanvas = { clientWidth: 1600, clientHeight: 1000 };

function countObjects(group, predicate) {
  let n = 0;
  group.traverse((o) => { if (predicate(o)) n++; });
  return n;
}
const isMesh = (o) => o.isMesh;
const isLine = (o) => o.isLine;

console.log('# 场景图构建（无头 three.js）');
const built = SCENARIOS.map((make) => buildCourse(make()));

test('每个场景都生成管带、中心线与控制点', () => {
  for (const course of built) {
    const scene = new CourseScene(stubCanvas, stubRenderer);
    scene.renderCourse(course);
    const ribbon = scene.groups.ribbon;
    assert.ok(ribbon, '缺少 ribbon 组');
    assert.ok(countObjects(ribbon, isMesh) >= 1, '缺少通道管带');
    assert.ok(countObjects(ribbon, isLine) >= 1, '缺少中心航线');
    const controls = countObjects(scene.groups.controls, isMesh);
    assert.equal(controls, course.points.length, '控制点数量不匹配');
  }
});

test('门按数量生成且带方向箭头，违规门存在', () => {
  const course = built[3]; // infeasible
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(course);
  const gateGroup = scene.groups.gates;
  assert.equal(scene.dragMeshes.filter((m) => m.userData.kind === 'gate').length,
    course.gates.length);
  // Each gate group contains an arrow helper child.
  const arrows = countObjects(gateGroup, (o) => o.isArrowHelper || o.type === 'ArrowHelper');
  assert.ok(arrows >= course.gates.length, '门方向箭头缺失');
});

test('不可飞场景的违规被渲染为红/琥珀色管段', () => {
  const course = built[3];
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(course);
  const redSegments = scene.groups.ribbon.children.filter(
    (o) => o.isMesh && o.userData.violation
  );
  assert.ok(redSegments.length > 0, '违规区段未生成高亮管段');
  const errSeg = redSegments.find((o) => o.userData.violation.severity === 'error');
  assert.ok(errSeg, '缺少 error 级别红段');
  const hex = errSeg.material.emissive.getHex();
  assert.equal(hex, 0xf87171, 'error 段颜色不正确');
});

test('禁止体积按类型生成网格与边线', () => {
  const course = built[1]; // sharp-turn with boxes
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(course);
  const meshes = countObjects(scene.groups.volumes, isMesh);
  assert.equal(meshes, course.volumes.length);
  const wires = countObjects(scene.groups.volumes, isLine);
  assert.ok(wires >= course.volumes.length, '障碍边线缺失');
});

test('引导环与姿态三轴沿航线生成', () => {
  const course = built[0];
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(course);
  const rings = countObjects(scene.groups.guidance, isMesh);
  assert.ok(rings > 5, '引导点过少');
  const triadLines = countObjects(scene.groups.attitude, isLine);
  assert.ok(triadLines > 5, '姿态框架过少');
});

test('飞行器与仿真轨迹生成，且可按 trace 定位', () => {
  const course = built[0];
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(course);
  assert.ok(scene.aircraft, '飞行器未创建');
  const mid = course.simulation.trace[Math.floor(course.simulation.trace.length / 2)];
  const up = course.attitudes[0].up;
  scene.updateAircraft(mid, up);
  assert.ok(scene.aircraft.position.distanceTo(new THREE.Vector3(...mid.pos)) < 1e-6);
  assert.ok(scene.trail, '仿真轨迹缺失');
});

test('障碍间距标注按采样生成', () => {
  const course = built[1];
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(course);
  assert.ok(scene.clearanceLabels.length > 0, '缺少净空标注');
  assert.ok(scene.clearanceLabels.every((c) => c.distance >= 0));
});

test('图层可见性开关生效', () => {
  const course = built[0];
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(course);
  scene.setLayerVisible('volumes', false);
  assert.equal(scene.groups.volumes.visible, false);
  scene.setLayerVisible('volumes', true);
  assert.equal(scene.groups.volumes.visible, true);
});

test('模式 B 重塑后的场景可完整重建且门位置不变', () => {
  const base = built[1];
  const reshaped = reshapeWithLockedSpeed(base, { maxIterations: 40, maxExtraPoints: 4 });
  const before = base.gates.map((g) => g.pos.map((x) => x.toFixed(2)).join(','));
  const after = reshaped.gates.map((g) => g.pos.map((x) => x.toFixed(2)).join(','));
  assert.deepEqual(after, before);
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(reshaped);
  assert.ok(countObjects(scene.groups.ribbon, isMesh) >= 1);
  assert.ok(scene.aircraft);
});

test('拖拽语义：移动控制点后内核重建，门与点坐标同步', () => {
  const course = built[0];
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(course);
  // Simulate the kernel-side effect of a drag (mirrors main.js handleDrag).
  const points = course.points.map((p) => V.clone(p));
  points[1] = [points[1][0] + 5, points[1][1] + 3, points[1][2]];
  const rebuilt = buildCourse({ ...course, points });
  assert.equal(rebuilt.points[1][1], points[1][1]);
  assert.ok(rebuilt.samples.length > 10);
  scene.renderCourse(rebuilt);
  assert.ok(scene.aircraft);
});


test('候选搜索航迹可作为虚线叠加层渲染', () => {
  const base = built[1];
  const cmp = compareCurveSolvers({
    points: base.points, gates: base.gates, volumes: base.volumes, aircraft: base.aircraft,
  });
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.candidateDirection = 'curve-first';
  scene.renderCourse(base, { candidateOverlay: cmp });
  const g = scene.groups.candidate;
  assert.ok(g, '缺少候选叠加层组');
  const lines = countObjects(g, isLine);
  assert.ok(lines >= 1, '候选航迹缺少虚线');
  const dots = countObjects(g, (o) => o.isMesh);
  const seg = cmp.variants.find((v) => v.strategy === 'segmented');
  assert.equal(dots, seg.points.length, '候选路点数量不符');
});

test('场景渲染的净空标注与内核最小间距同源', () => {
  const course = built[1];
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(course);
  const nearest = Math.min(...scene.clearanceLabels.map((c) => c.distance));
  // Labels sample every 10th station, so they can only be >= the per-segment
  // minimum, never below it; both come from distanceToVolume.
  assert.ok(nearest >= course.evaluation.stats.minClearance - 1e-6,
    '标注距离小于内核统计，来源不一致');
  assert.ok(nearest - course.evaluation.stats.minClearance < 5,
    '标注与内核统计相差过大，可能来自不同计算');
  for (const label of scene.clearanceLabels) {
    const nearestVol = course.volumes
      .map((v) => ({ id: v.id, d: distanceToVolume(label.pos, v) }))
      .sort((a, b) => a.d - b.d)[0];
    assert.ok(Math.abs(nearestVol.d - label.distance) < 1e-6);
  }
});

test('调度拒绝的航段会作为红色区段渲染', () => {
  const base = built[1];
  const mid = base.gates[Math.floor(base.gates.length / 2)];
  const w = base.gateWindows.find((x) => x.gateId === mid.id);
  const open = Math.ceil(w.tMax + 1);
  const rejected = rescheduleCourse(base, { [mid.id]: { open, close: open + 1 } });
  const scene = new CourseScene(stubCanvas, stubRenderer);
  scene.renderCourse(rejected);
  const reds = scene.groups.ribbon.children.filter(
    (o) => o.isMesh && o.userData.violation?.type === 'gate-schedule'
  );
  assert.ok(reds.length > 0, '调度拒绝未渲染为红色区段');
});

console.log(`\n${passed} passed`);
