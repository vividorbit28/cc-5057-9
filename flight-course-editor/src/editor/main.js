// Editor controller: the kernel owns all feasibility logic; the UI only renders
// kernel results and sends user edits back. Every drag triggers a kernel
// rebuild, so the view, violation list and simulation always agree.

import { CourseScene } from './scene.js';
import { Interaction } from './interaction.js';
import {
  buildCourse, rebuildFromPoints, replanSpeed, reshapeWithLockedSpeed,
  applyConstruction, rescheduleCourse, refitRejectedLeg,
  constructFlightGeometry, compareCurveSolvers, runCurveExperiments,
  SCENARIOS, V,
} from '../kernel/index.js';

const $ = (id) => document.getElementById(id);

class Editor {
  // sceneFactory lets headless tests inject a GL-free CourseScene; the browser
  // default constructs one against the real canvas and WebGLRenderer.
  constructor({ sceneFactory = null, startLoop = true } = {}) {
    this.canvas = $('three-canvas');
    this.scene = sceneFactory ? sceneFactory(this.canvas) : new CourseScene(this.canvas);
    this.scenarioIndex = 0;
    this.course = null;
    this.mode = 'A';
    this.playing = false;
    this.playT = 0;
    this.playbackRate = 1;
    this.layerOpts = {
      showRibbon: true, showGuidance: true, showAttitude: true,
      showVolumes: true, showClearance: true,
    };
    this.compareResult = null;
    this.showCandidateOverlay = false;
    this.constructDirection = 'curve-first';
    this._bindUI();
    this.scene.candidateDirection = this.constructDirection;
    this.loadScenario(0);
    this.interaction = new Interaction(this.scene, this.canvas, {
      onDrag: (kind, index, pos, end) => this.handleDrag(kind, index, pos, end),
      onSelect: (ref) => this.handleSelect(ref),
    });
    this._loop = this._loop.bind(this);
    this._lastFrame = performance.now();
    if (startLoop) requestAnimationFrame(this._loop);
    window.addEventListener('resize', () => this.scene.resize());
  }

  _bindUI() {
    const box = $('scenario-buttons');
    SCENARIOS.forEach((make, i) => {
      const b = document.createElement('button');
      b.className = 'scenario-btn';
      b.textContent = make().name;
      b.onclick = () => this.loadScenario(i);
      box.appendChild(b);
    });
    document.querySelectorAll('input[name=mode]').forEach((r) => {
      r.onchange = () => { this.mode = r.value; this._modeHint(); };
    });
    $('btn-run-mode').onclick = () => this.runMode();
    $('btn-play').onclick = () => { this.playing = true; };
    $('btn-pause').onclick = () => { this.playing = false; };
    $('btn-reset').onclick = () => { this.playing = false; this.playT = 0; this._seekAircraft(); };
    $('seek').oninput = (e) => {
      this.playing = false;
      this.playT = (e.target.value / 1000) * this.course.simulation.time;
      this._seekAircraft();
    };
    $('rate').oninput = (e) => {
      this.playbackRate = parseFloat(e.target.value);
      $('rate-val').textContent = this.playbackRate.toFixed(2) + '×';
    };
    const layers = [
      ['show-ribbon', 'ribbon'], ['show-guidance', 'guidance'],
      ['show-attitude', 'attitude'], ['show-volumes', 'volumes'],
      ['show-clearance', 'clearance'],
    ];
    layers.forEach(([id, group]) => {
      $(id).onchange = (e) => this.scene.setLayerVisible(group, e.target.checked);
    });
    $('btn-shrink-mid').onclick = () => this.shrinkMiddleGateWindow();
    $('btn-refit-leg').onclick = () => this.refitRejectedLeg();
    $('btn-construct').onclick = () => this.reconstruct();
    $('btn-compare').onclick = () => this.compareStrategies();
    $('btn-experiments').onclick = () => this.runExperiments();
    $('cs-candidates').onchange = (e) => {
      this.showCandidateOverlay = e.target.checked;
      if (this.showCandidateOverlay && !this.compareResult) this.compareStrategies();
      else this.refresh();
    };
    $('cs-reverse').onchange = (e) => {
      this.constructDirection = e.target.checked ? 'gate-reverse' : 'curve-first';
      if (this.scene) this.scene.candidateDirection = this.constructDirection;
    };
  }

  // Shorten the middle gate's opening window so it no longer intersects the
  // reachable arrival interval. The kernel rejects the affected leg; the UI
  // must show that, not a smooth line that pretends the schedule still works.
  shrinkMiddleGateWindow() {
    if (!this.course?.gateWindows?.length) return;
    const mid = this.course.gates[Math.floor(this.course.gates.length / 2)];
    const w = this.course.gateWindows.find((x) => x.gateId === mid.id);
    // Keep a one-second window that opens after the latest reachable arrival:
    // the gate is physically unreachable within it, so the leg must be refit.
    const open = Math.ceil((w?.tMax ?? 0) + 1);
    this.course = rescheduleCourse(this.course, { [mid.id]: { open, close: open + 1 } });
    $('schedule-status').textContent = this.course.schedule.feasible
      ? '缩短后的开放窗仍可调度（无需重拟合）。'
      : `已拒绝：${this.course.rejectedLeg?.reason ?? '调度不可行'}。可点击“重拟合被拒航段”。`;
    this.refresh();
  }

  refitRejectedLeg() {
    if (!this.course?.requiresRefit) {
      $('schedule-status').textContent = '当前没有需要重拟合的被拒航段。';
      return;
    }
    const r = refitRejectedLeg(this.course, { direction: this.constructDirection });
    if (r.applied) {
      this.course = r.course;
      $('schedule-status').textContent = r.reason;
    } else {
      // Keep old course; surface which constraint blocked the refit.
      $('schedule-status').textContent = r.reason;
    }
    this.refresh();
  }

  reconstruct() {
    if (!this.course) return;
    const result = constructFlightGeometry({
      points: this.course.points,
      gates: this.course.gates,
      volumes: this.course.volumes,
      aircraft: this.course.aircraft,
      strategy: this.showCandidateOverlay ? 'segmented' : 'control-point',
      direction: this.constructDirection,
      repair: true,
    });
    if (result.feasible) {
      this.course = applyConstruction(this.course, result);
      $('construct-status').textContent =
        `已重建（${result.strategy} / ${result.direction}）：几何与门序均满足。`;
    } else {
      // Do not replace a flyable course with a failed construction.
      $('construct-status').textContent =
        `该策略未通过校核（越界 ${(result.metrics.maxOvershoot * 100).toFixed(0)}%，` +
        `${result.metrics.affectedSegments} 段受影响${result.gateOrderOk ? '' : '，门序被破坏'}），保留当前航道。`;
    }
    this.refresh();
  }

  compareStrategies() {
    if (!this.course) return;
    this.compareResult = compareCurveSolvers({
      points: this.course.points,
      gates: this.course.gates,
      volumes: this.course.volumes,
      aircraft: this.course.aircraft,
    });
    this._renderCompare();
    const p = this.compareResult.preferred;
    $('construct-status').textContent = p
      ? `首选：${p.strategy} / ${p.direction}（最大越界 ${(p.maxOvershoot * 100).toFixed(0)}%）。` +
        '待验假设已单列，未并入结论。'
      : '所有策略均不可行；保留原航道，不伪报可飞。';
    this.refresh();
  }

  runExperiments() {
    const r = runCurveExperiments();
    this.experimentResult = r;
    this._renderExperiments();
    $('experiment-status').textContent =
      `已用代码复算：急弯 ${r.sharp.rows.length} 点、狭窄禁区 ${r.narrow.rows.length} 点。` +
      '表中数值为实际运算结果。';
  }

  loadScenario(i) {
    this.scenarioIndex = i;
    const def = SCENARIOS[i]();
    this.course = buildCourse(def);
    this.playT = 0;
    this.playing = false;
    document.querySelectorAll('.scenario-btn').forEach((b, k) =>
      b.classList.toggle('active', k === i));
    $('scenario-desc').textContent = def.description;
    this._modeHint();
    this.refresh();
  }

  _modeHint() {
    $('mode-status').textContent = this.mode === 'A'
      ? '门位与曲线固定，按曲率/爬升/门速窗重算沿程航速，并回算前后引导点。'
      : '锁定当前航速表，调整切线控制柄并在门间插入引导点，直到同一包络被满足。';
  }

  // Rebuild the scene + every panel from kernel data.
  refresh() {
    this.scene.resize();
    this.scene.renderCourse(this.course, {
      ...this.layerOpts,
      candidateOverlay: this.showCandidateOverlay ? this.compareResult : null,
    });
    this._renderEnvelope();
    this._renderResults();
    this._renderViolations();
    this._renderHud();
    this._renderGateWindows();
    this._renderAlerts();
    this._seekAircraft();
  }

  _renderGateWindows() {
    const rows = this.course.gateWindows ?? [];
    const alloc = new Map((this.course.schedule?.allocations ?? []).map((a) => [a.gateId, a]));
    const rejectedId = this.course.schedule?.rejectedLeg?.to ?? null;
    if (!rows.length) { $('gate-window-table').innerHTML = '<p class="muted">无门。</p>'; return; }
    const head = '<div class="gt-head"><span>门</span><span>可达 [tMin, tMax] s</span>' +
      '<span>分配通行窗 s</span></div>';
    const body = rows.map((w) => {
      const a = alloc.get(w.gateId);
      // Only the gate whose leg the scheduler rejected is marked; gates whose
      // window was never closed by the backward pass show their forward start.
      const cls = w.gateId === rejectedId ? 'gt-row rejected' : 'gt-row';
      let win;
      if (!a) win = '<span class="feasible-no">未分配</span>';
      else if (a.windowEnd == null) win = `${a.assigned.toFixed(2)} – 待定`;
      else win = `${a.windowStart.toFixed(2)} – ${a.windowEnd.toFixed(2)}`;
      return `<div class="${cls}"><span>${w.label ?? w.gateId}</span>` +
        `<span>${w.tMin.toFixed(2)} – ${w.tMax.toFixed(2)}</span><span>${win}</span></div>`;
    }).join('');
    $('gate-window-table').innerHTML = head + body;
  }

  // Alerts are derived from real kernel output: schedule rejection, clearance
  // below the collision radius, and tracking failure. Nothing is invented here.
  _renderAlerts() {
    const alerts = [];
    const ev = this.course.evaluation;
    const stats = ev.stats;
    if (this.course.requiresRefit || this.course.schedule?.feasible === false) {
      alerts.push({
        severity: 'error',
        text: `门开放窗调度不可行：${this.course.rejectedLeg?.reason ?? '航段需重拟合'}`,
      });
    }
    if (stats.minClearance != null && stats.minClearance < stats.craftRadius) {
      alerts.push({
        severity: 'error',
        text: `最小障碍间距 ${stats.minClearance.toFixed(2)}m < 碰撞半径 ` +
          `${stats.craftRadius}m（障碍 ${stats.minClearanceVolumeId ?? '?'}）`,
      });
    } else if (stats.minClearance != null) {
      alerts.push({
        severity: 'info',
        text: `最小障碍间距 ${stats.minClearance.toFixed(2)}m（障碍 ` +
          `${stats.minClearanceVolumeId ?? '无'}），碰撞半径 ${stats.craftRadius}m。`,
      });
    }
    const sim = this.course.simulation;
    if (sim && !sim.finished) {
      alerts.push({ severity: 'error', text: '仿真未能跑完全程：' + (sim.failures[0]?.message ?? '跟随失败') });
    } else if (sim && !sim.withinTolerance) {
      alerts.push({
        severity: 'warn',
        text: `横向峰值偏差 ${sim.tracking.peak.toFixed(2)}m 超容差 ${sim.tracking.tolerance.toFixed(1)}m`,
      });
    }
    if (this.course.gateWindowsFeasible === false) {
      alerts.push({ severity: 'error', text: `门可达区间不可行：${this.course.gateWindowsReason}` });
    }
    const box = $('alert-box');
    if (!alerts.length) {
      box.innerHTML = '<p class="muted">无异常：调度、净空、跟随均在包络内。</p>';
      return;
    }
    box.innerHTML = alerts.map((a) =>
      `<div class="alert ${a.severity}">${a.text}</div>`).join('');
  }

  _renderCompare() {
    const res = this.compareResult;
    if (!res) { $('compare-table').innerHTML = ''; return; }
    const head = '<div class="ct-head"><span>策略</span><span>可行</span><span>曲率上限</span>' +
      '<span>碰撞距离</span><span>门序</span></div>';
    const body = res.variants.map((v) => {
      const cls = v.feasible ? 'ct-row ok' : 'ct-row bad';
      const curv = `R≥${v.minRadius.toFixed(0)}m`;
      const clear = v.minClearance == null ? '—' : `${v.minClearance.toFixed(1)}m`;
      return `<div class="${cls}"><span>${v.strategy}/${v.direction}</span>` +
        `<span>${v.feasible ? '是' : '否'}</span><span>${curv}</span>` +
        `<span>${clear}</span><span>${v.gateOrderOk ? '✓' : '✗'}</span></div>`;
    }).join('');
    const note = res.pendingAssumptions.length
      ? `<div class="ct-note">比较按巡航速度换算曲率预算；本航道可飞性另由逐站速度规划判定` +
        `（弯道处会减速）。两者口径不同，故本表可能显示“否”而结果区仍“可行”。</div>` +
        `<div class="ct-note">待验假设：${res.pendingAssumptions.join('；')}</div>` : '';
    $('compare-table').innerHTML = head + body + note;
  }

  _renderExperiments() {
    const r = this.experimentResult;
    if (!r) { $('experiment-table').innerHTML = ''; return; }
    // A straight segment has zero sampled curvature, so 1/kappa overflows.
    // Show it as "直" (unbounded radius) instead of a fake 1e9.
    const curvatureLabel = (radius, kappa) => {
      if (kappa != null && kappa <= 1e-6) return '直';
      if (!Number.isFinite(radius) || radius >= 1e6) return '直';
      return radius.toFixed(0);
    };
    const block = (spec) => {
      const head = `<div class="et-title">${spec.title}</div>` +
        `<div class="et-head"><span>${spec.param}${spec.unit ? ' / ' + spec.unit : ''}</span>` +
        '<span>最小半径 m</span><span>最小间距 m</span><span>首选</span></div>';
      const rows = spec.rows.map((row) => `<div class="et-row"><span>${row.value}</span>` +
        `<span>${curvatureLabel(row.authoredMinRadius, row.authoredMaxCurvature)}</span>` +
        `<span>${row.authoredMinClearance == null ? '—' : row.authoredMinClearance.toFixed(2)}</span>` +
        `<span>${row.best ? row.best.strategy + '/' + row.best.direction : '—'}</span></div>`).join('');
      return head + rows;
    };
    $('experiment-table').innerHTML =
      block(r.sharp) + block(r.narrow) +
      `<div class="et-note">数值由 runCurveExperiments() 实时计算，可在测试中逐行复检。</div>`;
  }

  handleDrag(kind, index, pos, end) {
    if (!end) return; // live visual move is handled by the scene already
    if (kind === 'control') {
      const points = this.course.points.map((p) => V.clone(p));
      points[index] = pos;
      this.course = rebuildFromPoints(this.course, points);
    } else if (kind === 'gate') {
      const gates = this.course.gates.map((g) => ({ ...g, pos: V.clone(g.pos) }));
      gates[index].pos = pos;
      // Keep the matching control point co-located so the spline threads it.
      const points = this.course.points.map((p) => V.clone(p));
      const gate = this.course.gates[index];
      const ci = points.findIndex((p) => V.dist2(p, gate.pos) < 25);
      if (ci >= 0) points[ci] = pos;
      this.course = rebuildFromPoints(this.course, points, gates);
    }
    this.playT = 0;
    this.refresh();
  }

  handleSelect(ref) {
    if (!ref) { $('inspector').innerHTML = '<p class="muted">未选中。在视窗中点击控制点或门。</p>'; return; }
    if (ref.kind === 'control') this._inspectControl(ref.index);
    else if (ref.kind === 'gate') this._inspectGate(ref.index);
  }

  _inspectControl(index) {
    const p = this.course.points[index];
    // Find nearest sample station for the speed handle.
    let si = 0, bd = Infinity;
    this.course.samples.forEach((s, i) => {
      const d = V.dist2(s.pos, p);
      if (d < bd) { bd = d; si = i; }
    });
    const v = this.course.speeds[si];
    $('inspector').innerHTML = `
      <div class="field"><label>控制点 #${index} 位置 (x, y, z) m</label>
        <input id="insp-pos" value="${p.map((x) => x.toFixed(1)).join(', ')}" /></div>
      <div class="field"><label>该站指令航速 (m/s) — 触发模式 A 回算</label>
        <input id="insp-speed" type="number" step="0.5" min="${this.course.aircraft.vMin}"
          max="${this.course.aircraft.vMax}" value="${v.toFixed(1)}" /></div>
      <button class="primary" id="insp-apply">应用航速并重算引导点</button>`;
    $('insp-apply').onclick = () => {
      const nv = parseFloat($('insp-speed').value);
      this.course = replanSpeed(this.course, si, nv);
      this.refresh();
      this.handleSelect({ kind: 'control', index });
    };
  }

  _inspectGate(index) {
    const g = this.course.gates[index];
    $('inspector').innerHTML = `
      <p>门 <b>${g.label ?? g.id}</b></p>
      <div class="field"><label>半宽 / 半高 (m)</label>
        <input id="gate-size" value="${g.halfW}, ${g.halfH}" /></div>
      <div class="field"><label>速度窗 vMin / vMax (m/s，留空表示不限)</label>
        <input id="gate-speed" value="${g.vMin ?? ''}, ${g.vMax ?? ''}" /></div>
      <button class="primary" id="gate-apply">应用并重新校核</button>`;
    $('gate-apply').onclick = () => {
      const [w, h] = $('gate-size').value.split(',').map(parseFloat);
      const [mn, mx] = $('gate-speed').value.split(',').map((s) => parseFloat(s.trim()));
      const gates = this.course.gates.map((x) => ({ ...x }));
      gates[index].halfW = w; gates[index].halfH = h;
      gates[index].vMin = Number.isFinite(mn) ? mn : undefined;
      gates[index].vMax = Number.isFinite(mx) ? mx : undefined;
      this.course = rebuildFromPoints(this.course, this.course.points, gates);
      this.refresh();
      this.handleSelect({ kind: 'gate', index });
    };
  }

  runMode() {
    if (this.mode === 'A') {
      // Re-derive the speed profile for the current geometry (full re-plan).
      this.course = rebuildFromPoints(this.course, this.course.points);
      $('mode-status').textContent = '模式 A 完成：航速与全部引导点已按当前几何重算。';
    } else {
      const before = this.course.evaluation.feasible;
      this.course = reshapeWithLockedSpeed(this.course, {
        maxIterations: 60, maxExtraPoints: 6,
      });
      const r = this.course.reshape;
      $('mode-status').textContent = r.converged
        ? `模式 B 收敛：插入 ${r.insertedPoints} 个引导点，包络满足，门位未移动。`
        : `模式 B 未能完全收敛（插入 ${r.insertedPoints} 点），残余 ${r.residualViolations.length} 项违规已在下方标红。`;
      if (!before && r.converged) $('mode-status').textContent += ' 原不可飞段已被重塑修复。';
    }
    this.playT = 0;
    this.refresh();
  }

  _renderEnvelope() {
    const a = this.course.aircraft;
    const rows = [
      ['速度范围', `${a.vMin}–${a.vMax} m/s`],
      ['最大倾斜', `${(a.phiMax * 180 / Math.PI).toFixed(0)}°`],
      ['滚转速率', `${a.bankRateMax} rad/s`],
      ['爬升角', `±${(a.gammaMax * 180 / Math.PI).toFixed(0)}°`],
      ['垂直速率', `±${a.vzMax} m/s`],
      ['碰撞半径', `${a.craftRadius} m`],
      ['加/减速', `${a.aMax} / ${a.aDecel} m/s²`],
    ];
    $('envelope').innerHTML = rows.map(([k, v]) =>
      `<span class="k">${k}</span><span class="val">${v}</span>`).join('');
  }

  _renderResults() {
    const e = this.course.evaluation;
    const sim = this.course.simulation;
    const data = [
      ['航道总长', `${e.stats.length.toFixed(0)} m`],
      ['最小转弯半径', `${e.stats.minRadius.toFixed(1)} m`],
      ['最大爬升角', `${(e.stats.maxClimb * 180 / Math.PI).toFixed(1)}°`],
      ['最小障碍间距', e.stats.minClearance == null ? '—'
        : `${e.stats.minClearance.toFixed(2)} m <span class="muted">(${e.stats.minClearanceVolumeId ?? '—'})</span>`],
      ['碰撞半径', `${e.stats.craftRadius} m`],
      ['可飞性', e.feasible ? '<span class="feasible-yes">可行</span>' : '<span class="feasible-no">不可行</span>'],
      ['仿真完成', sim.finished ? '<span class="feasible-yes">是</span>' : '<span class="feasible-no">否</span>'],
      ['仿真耗时', `${sim.time.toFixed(1)} s`],
      ['横向 RMS 偏差', `${sim.tracking.rms.toFixed(2)} m`],
      ['横向峰值偏差', `${sim.tracking.peak.toFixed(2)} m`],
      ['跟随容差', `${sim.tracking.tolerance.toFixed(1)} m`],
    ];
    $('result-stats').innerHTML = data.map(([k, v]) =>
      `<span class="k">${k}</span><span class="v">${v}</span>`).join('');
  }

  _renderViolations() {
    const segs = this.course.evaluation.segments;
    const errors = segs.filter((s) => s.severity === 'error').length;
    const warns = segs.length - errors;
    const badge = $('violation-count');
    badge.className = 'badge ' + (errors ? '' : warns ? 'warn' : 'zero');
    badge.textContent = segs.length ? `${errors} 错 / ${warns} 警` : '0';
    const list = $('violation-list');
    if (!segs.length) {
      list.innerHTML = '<p class="muted">无违规：曲率、爬升变化与空间净空全部满足。</p>';
      return;
    }
    list.innerHTML = '';
    for (const seg of segs) {
      const div = document.createElement('div');
      div.className = `violation clickable ${seg.severity}`;
      div.innerHTML = `<div>${seg.messages[0]}</div>
        <div class="meta">s = ${seg.s0.toFixed(0)}–${seg.s1.toFixed(1)} m · ${seg.type} · 共 ${seg.messages.length} 条</div>`;
      div.onclick = () => {
        const smp = this.course.samples.find((s) => s.s >= seg.s0) ?? this.course.samples[0];
        this.interaction.focusOn(smp.pos);
      };
      list.appendChild(div);
    }
  }

  _renderHud() {
    const e = this.course.evaluation;
    $('hud-stats').innerHTML = `
      <div class="row"><span>状态</span><span class="${e.feasible ? 'feasible-yes' : 'feasible-no'}">${e.feasible ? '可飞' : '违规'}</span></div>
      <div class="row"><span>错误 / 警告</span><span>${e.stats.errorCount} / ${e.stats.warningCount}</span></div>
      <div class="row"><span>航道长度</span><span>${e.stats.length.toFixed(0)} m</span></div>`;
    const sim = this.course.simulation;
    $('hud-tracking').innerHTML = `
      <div class="row"><span>横向 RMS</span><span>${sim.tracking.rms.toFixed(2)} m</span></div>
      <div class="row"><span>峰值偏差</span><span>${sim.tracking.peak.toFixed(2)} m</span></div>
      <div class="row"><span>完成</span><span>${sim.finished ? '是' : '否'}</span></div>`;
  }

  _seekAircraft() {
    const trace = this.course.simulation.trace;
    if (!trace?.length || !this.scene.aircraft) return;
    const idx = this._traceIndexAt(this.playT);
    const tp = trace[idx];
    const att = this.course.attitudes[Math.min(idx, this.course.attitudes.length - 1)];
    this.scene.updateAircraft(tp, att?.up ?? [0, 1, 0]);
    $('seek').value = Math.round((this.playT / Math.max(this.course.simulation.time, 1e-6)) * 1000);
  }

  _traceIndexAt(t) {
    const trace = this.course.simulation.trace;
    let lo = 0, hi = trace.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (trace[mid].t < t) lo = mid + 1; else hi = mid;
    }
    return lo;
  }

  _loop(now) {
    const dt = Math.min((now - this._lastFrame) / 1000, 0.05);
    this._lastFrame = now;
    if (this.playing && this.course) {
      this.playT += dt * this.playbackRate;
      if (this.playT >= this.course.simulation.time) {
        this.playT = this.course.simulation.time;
        this.playing = false;
      }
      this._seekAircraft();
    }
    this.interaction?.orbit.update();
    this.scene.render();
    requestAnimationFrame(this._loop);
  }
}

// Boot in the browser. Tests import { Editor } and inject a GL-free scene.
export function bootEditor(options) {
  const editor = new Editor(options);
  if (typeof window !== 'undefined') window.__editor = editor;
  return editor;
}
export { Editor };

if (typeof document !== 'undefined' && !globalThis.__FLIGHT_EDITOR_NO_AUTOBOOT__) {
  bootEditor();
}
