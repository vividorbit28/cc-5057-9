// Editor controller: the kernel owns all feasibility logic; the UI only renders
// kernel results and sends user edits back. Every drag triggers a kernel
// rebuild, so the view, violation list and simulation always agree.

import { CourseScene } from './scene.js';
import { Interaction } from './interaction.js';
import {
  buildCourse, rebuildFromPoints, replanSpeed, reshapeWithLockedSpeed,
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
    this._bindUI();
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
    this.scene.renderCourse(this.course, this.layerOpts);
    this._renderEnvelope();
    this._renderResults();
    this._renderViolations();
    this._renderHud();
    this._seekAircraft();
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
export function bootEditor(options) { return new Editor(options); }
export { Editor };

if (typeof document !== 'undefined' && !globalThis.__FLIGHT_EDITOR_NO_AUTOBOOT__) {
  bootEditor();
}
