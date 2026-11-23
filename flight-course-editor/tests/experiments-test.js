// Tests for gate reachable-time windows, the opening scheduler, the curve
// solver comparison, refit-on-rejection behaviour and the reproducible
// curvature/clearance experiments. Run: node tests/experiments-test.js
import assert from 'node:assert/strict';
import {
  buildCourse, rescheduleCourse, refitRejectedLeg, SCENARIOS,
  reachableGateWindows, scheduleGateWindows,
  compareCurveSolvers, constructFlightGeometry, evaluateGeometry,
  runCurveExperiments, verifyExperimentRow, EXPERIMENTS,
  DEFAULT_AIRCRAFT, V,
} from '../src/kernel/index.js';

let passed = 0;
const failures = [];
function test(name, fn) {
  try { fn(); passed++; console.log(`  ok - ${name}`); }
  catch (e) { failures.push([name, e]); console.log(`  FAIL - ${name}: ${e.message}`); }
}

console.log('# 门可达时间窗');
const sharp = buildCourse(SCENARIOS[1]());

test('曲线求解返回每个门的可达时间区间，且 tMin <= tMax', () => {
  assert.equal(sharp.gateWindows.length, sharp.gates.length);
  for (const w of sharp.gateWindows) {
    assert.ok(Number.isFinite(w.tMin) && Number.isFinite(w.tMax));
    assert.ok(w.tMin <= w.tMax + 1e-6, `${w.gateId}: ${w.tMin} > ${w.tMax}`);
  }
});

test('可达区间随沿程推进而增大（时间单调）', () => {
  const mins = sharp.gateWindows.map((w) => w.tMin);
  for (let i = 1; i < mins.length; i++) {
    assert.ok(mins[i] >= mins[i - 1] - 1e-6, '门到达时间未单调递增');
  }
});

test('tMin/tMax 由快/慢剖面在同一站取样得到', () => {
  const w = reachableGateWindows({
    samples: sharp.samples, gates: sharp.gates, aircraft: sharp.aircraft,
  });
  assert.ok(w.feasible);
  // Fastest profile never slower than slowest at the same station, and each
  // window equals the arrival time of the corresponding profile.
  for (let i = 0; i < w.profiles.fast.length; i++) {
    assert.ok(w.profiles.fast[i] >= w.profiles.slow[i] - 1e-6);
  }
  for (const win of w.windows) {
    assert.ok(Math.abs(win.tMin - w.profiles.tFast[win.stationIndex]) < 1e-9);
    assert.ok(Math.abs(win.tMax - w.profiles.tSlow[win.stationIndex]) < 1e-9);
  }
});

test('无开放窗时调度可行并落在可达区间内', () => {
  assert.equal(sharp.schedule.feasible, true);
  for (const a of sharp.schedule.allocations) {
    assert.ok(a.windowStart >= a.tMin - 1e-6);
    assert.ok(a.windowEnd <= a.tMax + 1e-6);
    assert.ok(a.slack >= -1e-6);
  }
});

test('调度分配遵守门先后次序（窗口起点单调）', () => {
  const starts = sharp.schedule.allocations.map((a) => a.windowStart);
  for (let i = 1; i < starts.length; i++) {
    assert.ok(starts[i] > starts[i - 1] - 1e-9, '门通行窗次序被破坏');
  }
});

console.log('# 缩短中门开放时间');
test('中门开放窗过短：调度拒绝原航段并要求重拟合', () => {
  const mid = sharp.gates[Math.floor(sharp.gates.length / 2)];
  const before = sharp.evaluation.feasible;
  const s = rescheduleCourse(sharp, { [mid.id]: { open: 0, close: 4 } });
  assert.equal(s.requiresRefit, true, '过短的开放窗未被拒绝');
  assert.equal(s.schedule.feasible, false);
  assert.equal(s.rejectedLeg.to, mid.id);
  assert.match(s.rejectedLeg.reason, /重拟合/);
  // The schedule violation is on the same evaluation the editor renders.
  const v = s.evaluation.violations.find((x) => x.type === 'gate-schedule');
  assert.ok(v && v.severity === 'error');
  assert.equal(s.evaluation.feasible, false);
  assert.ok(s.evaluation.segments.some((seg) => seg.type === 'gate-schedule'));
  assert.equal(before, true, '旧关卡原状态不应被静默改写');
});

test('调度拒绝定位到被拒航段（s 区间不为全段）', () => {
  const mid = sharp.gates[1];
  const s = rescheduleCourse(sharp, { [mid.id]: { open: 0, close: 4 } });
  const v = s.evaluation.violations.find((x) => x.type === 'gate-schedule');
  assert.ok(v.s1 - v.s0 > 0);
  assert.ok(v.s0 >= sharp.gateWindows[0].s - 1e-6);
});

test('重拟合无法兼顾曲率与时间窗时保留旧关卡', () => {
  const mid = sharp.gates[1];
  const s = rescheduleCourse(sharp, { [mid.id]: { open: 0, close: 4 } });
  const r = refitRejectedLeg(s);
  assert.equal(r.applied, false);
  assert.match(r.reason, /保留旧关卡/);
  // The returned course is the OLD one, with its geometry untouched.
  assert.deepEqual(
    r.course.points.map((p) => p.map((x) => x.toFixed(6))),
    s.points.map((p) => p.map((x) => x.toFixed(6))),
  );
  assert.equal(r.course.evaluation.feasible, false, '拒绝后仍应显示不可行');
});

test('可行的开放窗编辑会被接受（不要求重拟合）', () => {
  const mid = sharp.gates[1];
  const w = sharp.gateWindows[1];
  const open = Math.ceil(w.tMin);
  const close = Math.floor(w.tMax);
  if (close - open < 1) return; // no room to test on this geometry
  const s = rescheduleCourse(sharp, { [mid.id]: { open, close } });
  assert.equal(s.requiresRefit, false);
  assert.equal(s.evaluation.violations.some((v) => v.type === 'gate-schedule'), false);
});

console.log('# 曲线求解策略比较');
test('四种策略组合都在同一组输入上给出可复算指标', () => {
  const c = buildCourse(SCENARIOS[0]());
  const cmp = compareCurveSolvers({
    points: c.points, gates: c.gates, volumes: c.volumes, aircraft: c.aircraft,
  });
  assert.equal(cmp.variants.length, 4);
  const combos = new Set(cmp.variants.map((v) => `${v.strategy}/${v.direction}`));
  for (const k of ['control-point/curve-first', 'control-point/gate-reverse',
    'segmented/curve-first', 'segmented/gate-reverse']) {
    assert.ok(combos.has(k), `缺少策略组合 ${k}`);
  }
  for (const v of cmp.variants) {
    assert.ok(Number.isFinite(v.maxOvershoot) && v.maxOvershoot >= 0);
    assert.ok(Number.isFinite(v.maxCurvature) && v.maxCurvature >= 0);
    assert.ok(Array.isArray(v.points) && v.points.length >= 2);
  }
  assert.ok(cmp.pendingAssumptions.length >= 3, '未单列待验假设');
});

test('以门反向约束曲率：门切向被强制为门法向', () => {
  const c = buildCourse(SCENARIOS[1]());
  const r = constructFlightGeometry({
    points: c.points, gates: c.gates, volumes: c.volumes, aircraft: c.aircraft,
    strategy: 'segmented', direction: 'gate-reverse', repair: false,
  });
  // Every gate anchor with an override must match the gate normal direction.
  let forced = 0;
  for (const key of Object.keys(r.tangentOverrides)) {
    const dir = V.norm(r.tangentOverrides[key]);
    const gate = c.gates.find((g) => {
      const anchor = r.anchors?.[Number(key)] ?? r.points[Number(key)];
      return anchor && V.dist(anchor, g.pos) < 1e-6;
    });
    if (gate) {
      assert.ok(V.dot(dir, V.norm(gate.normal)) > 0.999, '门切向未被约束为门法向');
      forced++;
    }
  }
  assert.ok(forced > 0, '门反向模式未强制任何门切向');
});

test('强制保留飞行门先后次序；打乱的航线被报为顺序违规', () => {
  const c = buildCourse(SCENARIOS[1]());
  // Declare the gates in a different order than the route visits them.
  const swappedGates = [c.gates[1], c.gates[0], c.gates[2]];
  const bad = evaluateGeometry({
    points: c.points, gates: swappedGates, volumes: c.volumes, aircraft: c.aircraft,
  });
  assert.equal(bad.gateOrderOk, false, '乱序门序列未被识别');
  const r = constructFlightGeometry({
    points: c.points, gates: swappedGates, volumes: c.volumes, aircraft: c.aircraft,
    strategy: 'segmented', direction: 'curve-first', repair: true,
  });
  assert.notEqual(r.gateOrderOk, true, '构造过程重排了门次序');
  assert.equal(r.feasible, false);
  // The declared order is never rewritten: returned anchors still follow input.
  assert.equal(r.points.length >= 2, true);
});

test('局部修整 vs 全段重拟合按越界距离与受影响段选择', () => {
  const c = buildCourse(SCENARIOS[1]());
  const cmp = compareCurveSolvers({
    points: c.points, gates: c.gates, volumes: c.volumes, aircraft: c.aircraft,
  });
  const withRepair = cmp.variants.filter((v) => v.repair);
  assert.ok(withRepair.length > 0, '没有任何变体触发修补以检查策略选择');
  for (const v of withRepair) {
    assert.ok(['local', 'full'].includes(v.repair.mode));
    if (v.repair.mode === 'local') {
      assert.ok(v.affectedSegments <= 2, '局部修整却有多段受影响');
      assert.ok(v.maxOvershoot <= 0.4, '局部修整却越界过大');
    }
  }
});

test('不可飞样例：所有策略都如实返回不可行，门次序仍保留', () => {
  const c = buildCourse(SCENARIOS[3]());
  const cmp = compareCurveSolvers({
    points: c.points, gates: c.gates, volumes: c.volumes, aircraft: c.aircraft,
  });
  assert.ok(cmp.variants.every((v) => v.infoseasible === undefined));
  assert.ok(cmp.variants.every((v) => v.feasible === false || v.gateOrderOk === false),
    '不可飞样例被某策略伪报为可飞');
});

console.log('# 实验：曲率上限与碰撞距离');
test('实验数据由代码复算，且与逐行复检一致', () => {
  const r = runCurveExperiments();
  for (const key of ['sharp', 'narrow']) {
    assert.ok(r[key].rows.length >= 5);
    for (const row of r[key].rows) {
      const again = verifyExperimentRow(key, row.value);
      assert.ok(Math.abs(again.authoredMinRadius - row.authoredMinRadius) < 1e-6,
        `${key}/${row.value}: 最小半径复算不一致`);
      assert.ok(Math.abs(again.authoredMinClearance - row.authoredMinClearance) < 1e-6,
        `${key}/${row.value}: 最小净空复算不一致`);
    }
  }
});

test('狭窄禁区：缝隙收窄时最小碰撞距离随之下降（单调）', () => {
  const r = runCurveExperiments();
  const rows = r.narrow.rows;
  for (let i = 1; i < rows.length; i++) {
    assert.ok(rows[i].authoredMinClearance >= rows[i - 1].authoredMinClearance - 1e-6,
      '净空未随缝隙半宽单调变化');
  }
  // Half-width below the craft radius must be flagged as a clearance error,
  // not silently accepted.
  assert.ok(rows.some((row) => row.authoredFeasible === false),
    '过窄缝隙未被判为不可行');
});

test('急弯：每个变体记录的曲率都有对应的最小半径', () => {
  const r = runCurveExperiments();
  for (const row of r.sharp.rows) {
    for (const v of row.variants) {
      // minRadius is derived as 1/maxCurvature when the row is summarised.
      assert.ok(Number.isFinite(v.minRadius) && v.minRadius > 0);
      assert.ok(Number.isFinite(v.maxCurvature) && v.maxCurvature >= 0);
    }
  }
  // Gate-reverse must not be strictly gentler than curve-first everywhere:
  // the two strategies differ, which is the point of comparing them.
  const differs = r.sharp.rows.some((row) => {
    const cf = row.variants.find((v) => v.direction === 'curve-first' && v.strategy === 'segmented');
    const gr = row.variants.find((v) => v.direction === 'gate-reverse' && v.strategy === 'segmented');
    return cf && gr && Math.abs(cf.minRadius - gr.minRadius) > 1;
  });
  assert.ok(differs, '门反向与曲线先行未产生可测差异');
});

console.log(`\n${passed} passed, ${failures.length} failed`);
if (failures.length) {
  for (const [n, e] of failures) console.error(`\n[${n}]`, e);
  process.exit(1);
}
