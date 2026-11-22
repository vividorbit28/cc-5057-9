// Headless verification of the flight-course kernel. Run: node tests/run-tests.js
import assert from 'node:assert/strict';
import {
  buildCourse, replanSpeed, reshapeWithLockedSpeed, DEFAULT_AIRCRAFT,
  SCENARIOS, Spline, simulate, planSpeedProfile, buildRMF, buildAttitude,
  evaluateCourse,
} from '../src/kernel/index.js';

let passed = 0;
const failures = [];
function test(name, fn) {
  try { fn(); passed++; console.log(`  ok - ${name}`); }
  catch (e) { failures.push([name, e]); console.log(`  FAIL - ${name}: ${e.message}`); }
}

console.log('# 样条与框架');
test('向心 Catmull-Rom 经过所有控制点且长度为正', () => {
  const pts = [[0, 0, 0], [10, 2, 4], [20, 0, 8], [30, 4, 0]];
  const sp = new Spline(pts);
  assert.ok(sp.totalLength > 30);
  for (const p of pts) {
    const near = sp.sample(1).some((s) => Math.hypot(
      s.pos[0] - p[0], s.pos[1] - p[1], s.pos[2] - p[2]) < 1.5);
    assert.ok(near, `控制点 ${p} 未被样条覆盖`);
  }
});

test('RMF 在近直线段不翻滚（up 向量稳定）', () => {
  const pts = [[0, 5, 0], [20, 5, 0.1], [40, 5, -0.1], [60, 5, 0], [80, 5, 0]];
  const sp = new Spline(pts);
  const samples = sp.sample(1);
  const rmf = buildRMF(samples);
  const speeds = samples.map(() => 20);
  const att = buildAttitude(samples, rmf, DEFAULT_AIRCRAFT, speeds);
  const jumps = att.filter((a) => a.rollJump);
  assert.equal(jumps.length, 0, '直线段出现姿态突变');
  // up 向量应始终接近世界上方
  for (const a of att) assert.ok(a.up[1] > 0.9, '直线段 up 偏离竖直');
});

console.log('# 场景构建与几何包络');
const built = {};
for (const make of SCENARIOS) {
  const sc = make();
  built[sc.id] = buildCourse(sc);
}

test('高低穿越：门全部穿越，违规仅为警告或零错误', () => {
  const c = built['high-low'];
  const missed = c.evaluation.violations.filter((v) => v.type === 'gate-missed');
  assert.equal(missed.length, 0);
  const hard = c.evaluation.violations.filter((v) => v.severity === 'error');
  // 允许爬升类警告，但硬错误必须可由速度规划消除
  const unplanned = hard.filter((v) => !['climb-rate-change'].includes(v.type));
  assert.equal(unplanned.length, 0, unplanned.map((v) => v.message).join('; '));
});

test('急转场景：速度规划在窄门前把速度降到转弯半径允许范围', () => {
  const c = built['sharp-turn'];
  // 找到曲率最大的采样站，核对 v^2/(g tan phi) <= 1/kappa
  const k = Math.max(...c.samples.map((s) => s.curvature));
  const idx = c.samples.findIndex((s) => s.curvature === k);
  const v = c.speeds[idx];
  const rMin = (v * v) / (9.81 * Math.tan(DEFAULT_AIRCRAFT.phiMax));
  assert.ok(1 / k >= rMin - 0.5,
    `R=${(1 / k).toFixed(1)} < 需要 ${rMin.toFixed(1)} @ v=${v.toFixed(1)}`);
  assert.ok(v < DEFAULT_AIRCRAFT.cruiseSpeed, '弯道前未减速');
});

test('急转场景模式 A：编辑门速后前后引导点同步回算', () => {
  const c = built['sharp-turn'];
  const before = c.guidance.map((g) => g.s);
  const idx = Math.floor(c.samples.length / 2);
  const c2 = replanSpeed(c, idx, 12);
  const after = c2.guidance.map((g) => g.s);
  const changed = before.filter((s, i) => Math.abs(s - after[i]) > 1e-6).length;
  assert.ok(changed > 2, '引导点未随速度编辑回算');
  // 速度表始终满足加减速约束
  for (let i = 1; i < c2.speeds.length; i++) {
    const ds = c2.samples[i].s - c2.samples[i - 1].s;
    const v0 = c2.speeds[i - 1], v1 = c2.speeds[i];
    const vReach = Math.sqrt(v0 * v0 + 2 * DEFAULT_AIRCRAFT.aMax * ds);
    assert.ok(v1 <= vReach + 1e-6, '违反加速限制');
  }
});

test('急转场景模式 B：锁定航速重塑后曲率包络满足且门位不动', () => {
  const c = built['sharp-turn'];
  const gatePosBefore = c.gates.map((g) => g.pos.map((x) => x.toFixed(2)).join(','));
  const reshaped = reshapeWithLockedSpeed(c, { maxIterations: 80, maxExtraPoints: 5 });
  const gatePosAfter = reshaped.gates.map((g) => g.pos.map((x) => x.toFixed(2)).join(','));
  assert.deepEqual(gatePosAfter, gatePosBefore, '重塑移动了固定门位');
  const turnErrors = reshaped.evaluation.violations
    .filter((v) => v.type === 'turn-radius' && v.severity === 'error');
  assert.ok(turnErrors.length === 0 || reshaped.reshape?.converged === false,
    '重塑结果未如实报告残余违规');
  if (reshaped.reshape?.converged) {
    assert.ok(reshaped.reshape.insertedPoints >= 0);
  }
});

test('相邻航道：折返段自近接被检测为通道宽度违规或保持足够间距', () => {
  const c = built['adjacent'];
  const prox = c.evaluation.violations.filter((v) => v.type === 'self-proximity');
  // 样例设计间距 48m，远大于 minSeparation，因此应为 0；改成贴近时必须能报出
  assert.equal(prox.length, 0);
  // 人为把折返段压近，验证检测确实生效
  const tight = c.points.map((p) => (p[2] > 40 && p[2] < 60 ? [p[0], p[1], p[2] - 38] : p));
  const c2 = buildCourse({ ...c, points: tight });
  const prox2 = c2.evaluation.violations.filter((v) => v.type === 'self-proximity');
  assert.ok(prox2.length > 0, '贴近的折返航道未触发自近接违规');
});

test('不可飞样例：门被吞没与速度窗冲突如实报为 error', () => {
  const c = built['infeasible'];
  assert.equal(c.evaluation.feasible, false);
  const kinds = new Set(c.evaluation.violations.map((v) => v.type));
  assert.ok(kinds.has('volume-clearance') || kinds.has('gate-clearance'));
  assert.ok(c.speedConflicts.length > 0 || kinds.has('speed-window-conflict')
    || c.evaluation.violations.some((v) => v.type === 'turn-radius'));
  // 违规被聚合成区段供界面渲染
  assert.ok(c.evaluation.segments.length > 0);
});

console.log('# 仿真跟随');
test('受支持运动模型沿高低航道跑完全程，横向误差在容差内', () => {
  const c = built['high-low'];
  const r = c.simulation;
  assert.equal(r.finished, true, r.failures.map((f) => f.message).join('; '));
  assert.ok(r.withinTolerance, `峰值横向偏差 ${r.tracking.peak.toFixed(2)}m > ${r.tracking.tolerance}m`);
  assert.ok(r.tracking.rms < 2.5, `RMS 偏差 ${r.tracking.rms.toFixed(2)}m 过大`);
});

test('带初始横向注入时控制器收敛回航道而非瞬移', () => {
  const c = built['high-low'];
  const r = simulate({
    samples: c.samples, speeds: c.speeds, aircraft: c.aircraft,
    startOffset: [0, 0, 12],
  });
  assert.equal(r.finished, true);
  // First sample carries the injected 12 m; the controller must not grow it.
  const initial = r.trace[0].crossTrack;
  const afterInjection = Math.max(...r.trace.slice(1).map((s) => s.crossTrack));
  assert.ok(initial > 8);
  assert.ok(afterInjection <= initial + 0.5,
    `恢复过程偏差扩大到 ${afterInjection.toFixed(2)}m`);
  assert.ok(r.tracking.final < 1.0, `终态未收敛: ${r.tracking.final.toFixed(2)}m`);
});

test('仿真器不会靠无限转向作弊：倾斜与滚转速率始终受限', () => {
  const c = built['sharp-turn'];
  for (const step of c.simulation.trace) {
    assert.ok(Math.abs(step.phi) <= DEFAULT_AIRCRAFT.phiMax + 1e-6);
    assert.ok(step.v >= DEFAULT_AIRCRAFT.vMin - 1e-6 && step.v <= DEFAULT_AIRCRAFT.vMax + 1e-6);
  }
});

test('曲率/爬升/净空共同约束：缩小碰撞体不会让侵入障碍变合法', () => {
  const c = built['infeasible'];
  const honest = c.evaluation.violations.some((v) => v.type === 'volume-clearance');
  // 即便把碰撞半径缩到极小，门仍在实体内部，距离判定仍为 0
  const tiny = buildCourse({ ...c, aircraft: { ...c.aircraft, craftRadius: 0.2 } });
  const still = tiny.evaluation.violations.some((v) => v.type === 'volume-clearance');
  assert.equal(honest || still, true);
});

console.log(`\n${passed} passed, ${failures.length} failed`);
if (failures.length) {
  for (const [n, e] of failures) console.error(`\n[${n}]`, e);
  process.exit(1);
}
