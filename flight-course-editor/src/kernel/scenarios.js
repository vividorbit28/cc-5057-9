// Shipped example courses. Each stresses a different part of the envelope:
//   high-low   -> climb-angle, climb-rate and vertical gate threading
//   sharp-turn -> minimum turn radius vs. speed, Mode A/B comparison
//   adjacent   -> self-proximity, channel width and visible separation
// Plus one deliberately infeasible course used to demonstrate violation UI.

import { rad } from './math.js';

const baseAircraft = {
  g: 9.81,
  vMin: 8,
  vMax: 45,
  aMax: 8,
  aDecel: 12,
  phiMax: Math.PI / 3,
  bankRateMax: 2.2,
  gammaMax: Math.PI / 6,
  gammaRateMax: 0.4,
  vzMax: 18,
  craftRadius: 2.2,
  guidanceHeadway: 2.5,
  trackTolerance: 4.0,
  abortCrossTrack: 22,
  cruiseSpeed: 26,
};

function gate(id, pos, normal, halfW, halfH, extra = {}) {
  return {
    id, label: id, pos, normal, halfW, halfH,
    alignLimit: rad(55), required: true, ...extra,
  };
}

// 1. High-low crossing: dive under an overhead box, then climb through a high
// gate, then descend. Climb rate and gamma envelope are the binding limits.
export function highLowScenario() {
  const aircraft = { ...baseAircraft };
  const points = [
    [0, 12, 0],
    [60, 12, 5],
    [120, 6, 10],
    [180, 6, 14],
    [240, 26, 18],
    [300, 26, 22],
    [360, 10, 26],
    [420, 10, 30],
    [470, 10, 33],
  ];
  const gates = [
    gate('G1', [60, 12, 5], [1, 0, 0.08], 7, 5),
    gate('G2', [180, 6, 14], [1, 0, 0.07], 6, 4, { vMax: 24 }),
    gate('G3', [300, 26, 22], [1, 0, 0.07], 6, 5),
    gate('G4', [420, 10, 30], [1, 0, 0.07], 8, 6),
  ];
  const volumes = [
    { id: 'V-overhead', type: 'box', center: [150, 20, 12], half: [26, 6, 10], yaw: 0 },
    { id: 'V-floor', type: 'box', center: [360, -2, 28], half: [30, 2, 10], yaw: 0 },
  ];
  return {
    id: 'high-low',
    name: '高低穿越',
    description: '从高位下潜穿过架空障碍下方，再爬升至高位门，最后俯冲入低位门。约束重点：爬升角 ±30°、垂直速率 ±18m/s、竖向门净空。',
    aircraft, points, gates, volumes,
    minSeparation: 8,
  };
}

// 2. Sharp-turn restriction: a 90-degree bend between two narrow gates. At
// cruise speed the required radius exceeds what 60-degree bank allows, so
// Mode A must slow down; Mode B keeps speed and reshapes a wider arc.
export function sharpTurnScenario() {
  const aircraft = { ...baseAircraft, cruiseSpeed: 34 };
  const points = [
    [0, 14, 0],
    [70, 14, 0],
    [110, 14, 0],
    [125, 14, 35],
    [125, 14, 90],
    [125, 14, 150],
    [180, 14, 170],
    [240, 14, 170],
    [290, 14, 170],
  ];
  const gates = [
    gate('T1', [70, 14, 0], [1, 0, 0], 6, 5, { vMax: 20 }),
    gate('T2', [125, 14, 90], [0, 0, 1], 6, 5),
    gate('T3', [240, 14, 170], [1, 0, 0], 7, 5),
  ];
  const volumes = [
    // Building pinching the inside of the corner.
    { id: 'B-inside', type: 'box', center: [98, 14, 58], half: [10, 16, 10], yaw: 0 },
    { id: 'B-outside', type: 'box', center: [158, 14, 62], half: [8, 16, 12], yaw: 0 },
  ];
  return {
    id: 'sharp-turn',
    name: '急转限制',
    description: '两座窄门之间是 90° 急弯，内侧有建筑。巡航速度下所需转弯半径超过 60° 倾斜极限。模式 A 在门前减速通过；模式 B 锁定航速、重塑更大半径的弧线。',
    aircraft, points, gates, volumes,
    minSeparation: 8,
  };
}

// 3. Adjacent channels: the route folds back parallel to itself inside a
// hangar corridor. Self-proximity and channel width dominate; the visual
// guide must keep the two passes distinguishable.
export function adjacentScenario() {
  const aircraft = { ...baseAircraft, cruiseSpeed: 22 };
  const points = [
    [0, 12, 0],
    [50, 12, 0],
    [90, 12, 0],
    [100, 12, 24],
    [90, 12, 48],
    [50, 12, 48],
    [10, 12, 48],
    [0, 12, 72],
    [10, 12, 96],
    [60, 12, 96],
    [110, 12, 96],
  ];
  const gates = [
    gate('A1', [50, 12, 0], [1, 0, 0], 5, 4),
    gate('A2', [50, 12, 48], [-1, 0, 0], 5, 4),
    gate('A3', [60, 12, 96], [1, 0, 0], 5, 4),
  ];
  const volumes = [
    // Outer hangar walls flanking the folded channel.
    { id: 'W-north', type: 'box', center: [55, 12, -10], half: [60, 14, 3], yaw: 0 },
    { id: 'W-south', type: 'box', center: [60, 12, 110], half: [60, 14, 3], yaw: 0 },
    { id: 'W-east', type: 'box', center: [125, 12, 48], half: [3, 14, 62], yaw: 0 },
  ];
  return {
    id: 'adjacent',
    name: '相邻航道',
    description: '航道在机库走廊内折返，与自身平行贴近。约束重点：自近接距离、通道直径与可见引导分离，折返段净距不足时直接报违规而非让两条管道重叠。',
    aircraft, points, gates, volumes,
    minSeparation: 12,
  };
}

// 4. Deliberately infeasible: a gate inside a box and a corner tighter than
// any speed in the envelope allows. Used to prove failures are surfaced.
export function infeasibleScenario() {
  const aircraft = { ...baseAircraft, cruiseSpeed: 40 };
  const points = [
    [0, 14, 0],
    [50, 14, 0],
    [60, 14, 12],
    [50, 14, 24],
    [50, 14, 70],
    [120, 14, 80],
    [180, 14, 95],
    [230, 14, 95],
  ];
  const gates = [
    // Force a slow pass through the block, then demand high speed almost
    // immediately: the short gap cannot bridge 10 -> 44 m/s under aMax.
    gate('X1', [50, 14, 12], [0, 0, 1], 4, 3, { vMax: 10 }),
    gate('X2', [120, 14, 80], [1, 0, 0], 5, 4, { vMin: 44, vMax: 45 }),
  ];
  const volumes = [
    // Swallows the first gate entirely.
    { id: 'F-block', type: 'box', center: [50, 14, 12], half: [12, 10, 8], yaw: 0 },
  ];
  return {
    id: 'infeasible',
    name: '不可飞样例（违规演示）',
    description: '门被障碍整体吞没且弯道超过最小转弯半径，同时门速下限与减速能力冲突。用于验证违规区段标红与失败反馈，而不是假装可飞。',
    aircraft, points, gates, volumes,
    minSeparation: 10,
  };
}

export const SCENARIOS = [
  highLowScenario,
  sharpTurnScenario,
  adjacentScenario,
  infeasibleScenario,
];
