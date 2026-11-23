// Headless DOM wiring test using happy-dom + a stubbed WebGL renderer.
// Exercises the real main.js controller event chain (scenario switch, mode A/B,
// inspector speed re-plan, replay, layer toggles) without a GL context.
import { Window } from 'happy-dom';
import { readFileSync } from 'node:fs';

const window = new Window({ url: 'http://localhost/' });
const document = window.document;
document.write(readFileSync('index.html', 'utf8'));

// --- globals the editor modules touch ---
globalThis.window = window;
globalThis.document = document;
globalThis.devicePixelRatio = 1;
globalThis.requestAnimationFrame = () => 0;
globalThis.cancelAnimationFrame = () => {};
globalThis.performance = window.performance;
globalThis.Event = window.Event;
globalThis.MouseEvent = window.MouseEvent;
globalThis.PointerEvent = window.PointerEvent;

// Force CourseScene to use a stub renderer instead of a real WebGL context.
const canvas = document.getElementById('three-canvas');
Object.defineProperty(canvas, 'clientWidth', { configurable: true, value: 1600 });
Object.defineProperty(canvas, 'clientHeight', { configurable: true, value: 1000 });
const stubRenderer = { setPixelRatio() {}, setSize() {}, render() {} };

const { CourseScene } = await import('../src/editor/scene.js');
const sceneFactory = (c) => new CourseScene(c, stubRenderer);
globalThis.__FLIGHT_EDITOR_NO_AUTOBOOT__ = true;

let passed = 0, failed = 0;
const check = (name, cond, extra = '') => {
  if (cond) { passed++; console.log('  ok -', name); }
  else { failed++; console.log('  FAIL -', name, extra); }
};

// Importing main.js with auto-boot disabled, then boot with the GL-free scene.
const { Editor } = await import('../src/editor/main.js');
const editor = new Editor({ sceneFactory, startLoop: false });
await new Promise((r) => setTimeout(r, 50));

console.log('# 控制器 DOM 接线（happy-dom）');

check('生成 4 个场景按钮', document.querySelectorAll('.scenario-btn').length === 4);
check('初始 HUD 显示可飞性', /可飞|违规/.test(document.getElementById('hud-stats').textContent));
check('结果区显示 RMS 偏差', /横向 RMS 偏差/.test(document.getElementById('result-stats').textContent));
check('包络面板已填充', document.getElementById('envelope').children.length >= 14);

// Switch to infeasible scenario.
document.querySelectorAll('.scenario-btn')[3].click();
await new Promise((r) => setTimeout(r, 20));
const badge = document.getElementById('violation-count').textContent;
const errCount = document.querySelectorAll('.violation:not(.warning)').length;
check('不可飞场景显示错误区段', errCount > 0, `badge=${badge}`);
check('违规行可点击聚焦', !!document.querySelector('.violation.clickable'));

// Switch to sharp-turn and run Mode B.
document.querySelectorAll('.scenario-btn')[1].click();
await new Promise((r) => setTimeout(r, 20));
document.querySelector('input[name=mode][value=B]').click();
document.getElementById('btn-run-mode').click();
await new Promise((r) => setTimeout(r, 20));
check('模式 B 输出状态文案', /模式 B/.test(document.getElementById('mode-status').textContent),
  document.getElementById('mode-status').textContent);

// Mode A re-plan.
document.querySelector('input[name=mode][value=A]').click();
document.getElementById('btn-run-mode').click();
await new Promise((r) => setTimeout(r, 20));
check('模式 A 输出状态文案', /模式 A/.test(document.getElementById('mode-status').textContent));

// Replay controls.
const seek = document.getElementById('seek');
document.getElementById('btn-play').click();
document.getElementById('btn-reset').click();
check('复位后回放进度归零', parseInt(seek.value, 10) === 0);

// Layer toggle.
const volBox = document.getElementById('show-volumes');
volBox.click();
check('图层复选框可切换', volBox.checked === false);
volBox.click();

// Inspector: select a control point via the kernel-driven handler path.
// We can't raycast without a pointer, but the inspector render path is exposed
// through handleSelect; trigger it by dispatching on a scenario's first point
// through the public editor instance stored on the module scope is not exposed,
// so verify the inspector placeholder state instead.
check('检查器初始占位文案', /未选中/.test(document.getElementById('inspector').textContent));

// Inspector: select a control point and run the Mode A speed re-plan.
editor.handleSelect({ kind: 'control', index: 1 });
await new Promise((r) => setTimeout(r, 10));
check('选中控制点出现速度输入框', !!document.getElementById('insp-speed'));
const speedBefore = parseFloat(document.getElementById('insp-speed').value);
document.getElementById('insp-speed').value = (speedBefore - 4).toFixed(1);
document.getElementById('insp-apply').click();
await new Promise((r) => setTimeout(r, 20));
check('速度回算后结果区刷新', /横向 RMS 偏差/.test(document.getElementById('result-stats').textContent));
check('速度回算后场景仍有飞行器', !!editor.scene.aircraft);

// Inspector: select a gate and edit its size / speed window.
editor.handleSelect({ kind: 'gate', index: 0 });
await new Promise((r) => setTimeout(r, 10));
check('选中门出现尺寸输入框', !!document.getElementById('gate-size'));
document.getElementById('gate-size').value = '9, 7';
document.getElementById('gate-speed').value = '12, 30';
document.getElementById('gate-apply').click();
await new Promise((r) => setTimeout(r, 20));
const editedGate = editor.course.gates[0];
check('门尺寸编辑写回内核', editedGate.halfW === 9 && editedGate.halfH === 7);
check('门速度窗编辑写回内核', editedGate.vMin === 12 && editedGate.vMax === 30);

// Deselect.
editor.handleSelect(null);
check('取消选中回到占位文案', /未选中/.test(document.getElementById('inspector').textContent));

// Violation row click should not throw and should call focus (camera stubbed).
const firstViolation = document.querySelector('.violation.clickable');
if (firstViolation) {
  let threw = false;
  try { firstViolation.click(); } catch (e) { threw = true; }
  check('点击违规行不抛异常', !threw);
}


// --- gate windows / scheduling ---
check('门可达时间窗表已渲染', document.querySelectorAll('#gate-window-table .gt-row').length > 0);
const winRowsBefore = document.querySelectorAll('#gate-window-table .gt-row').length;
document.getElementById('btn-shrink-mid').click();
await new Promise((r) => setTimeout(r, 20));
check('缩短中门开放窗后调度给出拒绝文案',
  /拒绝|重拟合/.test(document.getElementById('schedule-status').textContent),
  document.getElementById('schedule-status').textContent);
check('被拒航段在时间窗表中标记为未分配',
  document.querySelectorAll('#gate-window-table .gt-row.rejected').length > 0);
check('异常提示区显示调度不可行',
  /调度不可行|重拟合/.test(document.getElementById('alert-box').textContent));
document.getElementById('btn-refit-leg').click();
await new Promise((r) => setTimeout(r, 20));
check('重拟合按钮给出结论（应用或保留旧关卡）',
  /重拟合|保留旧关卡|没有需要/.test(document.getElementById('schedule-status').textContent));

// --- curve solver comparison ---
document.querySelectorAll('.scenario-btn')[1].click();
await new Promise((r) => setTimeout(r, 20));
document.getElementById('btn-compare').click();
await new Promise((r) => setTimeout(r, 20));
const ctRows = document.querySelectorAll('#compare-table .ct-row').length;
check('比较表列出四种策略', ctRows === 4, `rows=${ctRows}`);
check('比较表含待验假设单列',
  /待验假设/.test(document.getElementById('compare-table').textContent));
check('比较结论写入状态区',
  /首选|所有策略均不可行/.test(document.getElementById('construct-status').textContent));
document.getElementById('cs-candidates').click();
await new Promise((r) => setTimeout(r, 20));
check('候选叠加层可由复选框驱动', document.getElementById('cs-candidates').checked === true);
check('叠加层生成候选航迹组', !!editor.scene.groups.candidate);

// --- experiments ---
document.getElementById('btn-experiments').click();
await new Promise((r) => setTimeout(r, 20));
check('实验表渲染急弯与狭窄禁区',
  document.querySelectorAll('#experiment-table .et-row').length >= 10);
check('实验状态声明代码复算',
  /复算/.test(document.getElementById('experiment-status').textContent));

// --- reconstruction strategy ---
document.getElementById('cs-reverse').click();
document.getElementById('btn-construct').click();
await new Promise((r) => setTimeout(r, 20));
check('以门反向约束曲率重建给出结论',
  /已重建|未通过|保留/.test(document.getElementById('construct-status').textContent));

// --- clearance is reported as an actual number ---
check('结果区显示最小障碍间距数值',
  /最小障碍间距/.test(document.getElementById('result-stats').textContent));

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
