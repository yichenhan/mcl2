#!/usr/bin/env node
// Replay analysis script — parses compact [T=N||key]=value lines
// (same format as the frontend replayParser) and produces a diagnostic report.

import { readFileSync } from "fs";

const logFile = process.argv[2] || "/Users/alexanderhan/Downloads/output (20).txt";
let text = readFileSync(logFile);
// Auto-detect UTF-16LE (BOM FF FE) and convert
if (text[0] === 0xFF && text[1] === 0xFE) {
  text = text.subarray(2).toString("utf16le");
} else {
  text = text.toString("utf-8");
}
// Strip null bytes from UTF-16 leftovers
text = text.replace(/\0/g, "");

const COMPACT_KV_RE = /^\[T=(\d+)\|\|([^\]]+)\]=(.*)$/;

function parseTicks(text) {
  const tickMap = new Map();
  for (const line of text.split("\n")) {
    const m = COMPACT_KV_RE.exec(line.trim());
    if (!m) continue;
    const tc = Number(m[1]);
    const key = m[2].trim();
    const val = m[3];
    if (!tickMap.has(tc)) tickMap.set(tc, { tc });
    const t = tickMap.get(tc);

    if (key === "oh") t.oh = Number(val);
    else if (key === "vs") t.vs = Number(val);
    else if (key === "or") {
      const inner = val.replace(/^\[|\]$/g, "").split(",").map(Number);
      t.or = inner;
    } else if (key === "op") {
      const inner = val.replace(/^\(|\)$/g, "").split(",").map(Number);
      t.op = { x: inner[0], y: inner[1], theta: inner[2] };
    } else if (key === "re") {
      const inner = val.replace(/^\(|\)$/g, "").split(",").map(Number);
      t.re = { x: inner[0], y: inner[1], theta: inner[2] };
    } else if (key === "cp") {
      const inner = val.replace(/^\(|\)$/g, "").split(",").map(Number);
      t.cp = { x: inner[0], y: inner[1], theta: inner[2] };
    } else if (key === "g") {
      const parts = val.split("|");
      t.accepted = parts[0] === "1";
      t.reason = parts[1] || "";
      const msrRt = (parts[2] || "0/0").split("/");
      t.maxResidual = Number(msrRt[0]);
      t.residualThresh = Number(msrRt[1]);
      t.r90 = Number((parts[3] || "r90=0").replace("r90=", ""));
      t.jump = Number((parts[4] || "j=0").replace("j=", ""));
      t.cd = Number((parts[5] || "cd=0").replace("cd=", ""));
    }
  }
  return [...tickMap.entries()].sort((a, b) => a[0] - b[0]).map(([, v]) => v);
}

const ticks = parseTicks(text);
console.log(`\n${"=".repeat(72)}`);
console.log(`  MCL REPLAY ANALYSIS — output (20).txt`);
console.log(`  Total ticks parsed: ${ticks.length} (tc=${ticks[0]?.tc} .. ${ticks[ticks.length - 1]?.tc})`);
console.log(`${"=".repeat(72)}\n`);

// ── Intended route (from skills_auton in main.cpp) ──
console.log("INTENDED ROUTE (skills_auton):");
console.log("─".repeat(60));
const route = [
  { action: "setPose(20, 66, 270)", note: "Start pose" },
  { action: "startMclLocTask()", note: "MCL begins, 300 particles" },
  { action: "moveToPoint(-28, 66)", note: "Drive left across loader zone, intake running" },
  { action: "turnToHeading(340)", note: "Rotate to face ~NNW" },
  { action: "moveToPose(7.5, 19.5, 330)", note: "Reverse toward middle goal area" },
  { action: "swingToHeading(45, LEFT)", note: "Swing to face middle goal" },
  { action: "moveDistance(3)", note: "Nudge forward for alignment" },
  { action: "middle_goal_scoring(-68)", note: "Score middle goal" },
  { action: "moveToPoint(48, 48)", note: "Drive to right loader, matchloader on" },
  { action: "turnToHeading(0)", note: "Face north" },
  { action: "moveToPoint(48, 34) [reverse]", note: "Back toward right long goal" },
  { action: "long_goal_scoring(-70)", note: "Score right long goal #1" },
  { action: "moveToPoint(48, 52) → (48, 72)", note: "Collect from right loader" },
  { action: "follow(Part1_txt) pursuit", note: "Pure pursuit path to far side" },
  { action: "setPose(61, -20, θ)", note: "Manual pose correction at path end" },
  { action: "moveToPoint(48, -48) [reverse]", note: "Approach lower-right goal area" },
  { action: "turnToHeading(180)", note: "Face south" },
  { action: "moveToPoint(48, -34) [reverse]", note: "Back into right long goal south" },
  { action: "long_goal_scoring(-65)", note: "Score right long goal south #1" },
  { action: "collect loader → (48, -72)", note: "Right south loader" },
  { action: "moveToPoint(48, -34) [reverse]", note: "Back into right long goal south again" },
  { action: "long_goal_scoring(-65)", note: "Score right long goal south #2" },
  { action: "follow(ParkBarrier_txt)", note: "Traverse to left side" },
  { action: "setPose(-24, -67.5, θ)", note: "Manual pose correction" },
  { action: "turnToHeading(0) → descore sequence", note: "Approach & descore middle goal" },
  { action: "moveToPoint(-48, -48)", note: "Drive to left-south goal area" },
  { action: "turnToHeading(180)", note: "Face south" },
  { action: "long_goal_scoring(-70)", note: "Score left long goal south #1" },
  { action: "collect loader → (-48, -72)", note: "Left south loader" },
  { action: "follow(Part2ofPart1_txt)", note: "Traverse to left-north side" },
  { action: "setPose(-61, 18, θ)", note: "Manual pose correction" },
  { action: "moveToPoint(-48, 48) [reverse]", note: "Approach left long goal north" },
  { action: "long_goal_scoring(-70)", note: "Score left long goal north #1" },
  { action: "collect loader → (-48, 72)", note: "Left north loader" },
  { action: "moveToPoint(-48, 34) [reverse]", note: "Back into left long goal north again" },
  { action: "long_goal_scoring(-70)", note: "Score left long goal north #2" },
  { action: "follow(ParkBarrier2_txt)", note: "Park on barrier" },
];
route.forEach((r, i) => console.log(`  ${String(i + 1).padStart(2)}. ${r.action.padEnd(42)} ${r.note}`));

// ── Summary statistics ──
console.log(`\n\nGATE STATISTICS:`);
console.log("─".repeat(60));
const accepted = ticks.filter(t => t.accepted);
const rejected = ticks.filter(t => !t.accepted);
console.log(`  Accepted: ${accepted.length}/${ticks.length} (${(accepted.length / ticks.length * 100).toFixed(1)}%)`);
console.log(`  Rejected: ${rejected.length}/${ticks.length} (${(rejected.length / ticks.length * 100).toFixed(1)}%)`);

const reasons = {};
rejected.forEach(t => {
  const r = t.reason || "unknown";
  reasons[r] = (reasons[r] || 0) + 1;
});
console.log(`\n  Rejection reasons:`);
Object.entries(reasons).sort((a, b) => b[1] - a[1]).forEach(([r, c]) => {
  console.log(`    ${String(c).padStart(4)}× ${r}`);
});

// ── Sensor availability ──
console.log(`\n\nSENSOR AVAILABILITY:`);
console.log("─".repeat(60));
const vsCounts = [0, 0, 0, 0, 0];
ticks.forEach(t => { if (t.vs >= 0 && t.vs <= 4) vsCounts[t.vs]++; });
vsCounts.forEach((c, i) => console.log(`  ${i} valid sensors: ${c} ticks (${(c / ticks.length * 100).toFixed(1)}%)`));

const sensorNames = ["Left", "Right", "Front", "Back"];
const sensorValid = [0, 0, 0, 0];
ticks.forEach(t => {
  if (!t.or) return;
  t.or.forEach((v, i) => { if (v >= 0) sensorValid[i]++; });
});
console.log(`\n  Per-sensor valid rate:`);
sensorNames.forEach((n, i) => console.log(`    ${n}: ${sensorValid[i]}/${ticks.length} (${(sensorValid[i] / ticks.length * 100).toFixed(1)}%)`));

// ── Odom drift analysis ──
console.log(`\n\nODOM DRIFT ANALYSIS (op vs cp):`);
console.log("─".repeat(60));
const drifts = ticks.filter(t => t.op && t.cp).map(t => {
  const dx = t.op.x - t.cp.x;
  const dy = t.op.y - t.cp.y;
  return { tc: t.tc, dist: Math.sqrt(dx * dx + dy * dy), dx, dy, op: t.op, cp: t.cp };
});
if (drifts.length > 0) {
  const maxDrift = drifts.reduce((a, b) => a.dist > b.dist ? a : b);
  const finalDrift = drifts[drifts.length - 1];
  console.log(`  Max odom-chassis divergence: ${maxDrift.dist.toFixed(1)} in at tc=${maxDrift.tc}`);
  console.log(`    op=(${maxDrift.op.x.toFixed(1)}, ${maxDrift.op.y.toFixed(1)})  cp=(${maxDrift.cp.x.toFixed(1)}, ${maxDrift.cp.y.toFixed(1)})`);
  console.log(`  Final divergence: ${finalDrift.dist.toFixed(1)} in at tc=${finalDrift.tc}`);
  console.log(`    op=(${finalDrift.op.x.toFixed(1)}, ${finalDrift.op.y.toFixed(1)})  cp=(${finalDrift.cp.x.toFixed(1)}, ${finalDrift.cp.y.toFixed(1)})`);
}

// ── Odom theta analysis: detect wrapping issue ──
console.log(`\n\nHEADING ANALYSIS:`);
console.log("─".repeat(60));
const thetaIssues = ticks.filter(t => t.op && t.op.theta > 360);
if (thetaIssues.length > 0) {
  console.log(`  *** ODOM THETA EXCEEDS 360°: ${thetaIssues.length} ticks ***`);
  console.log(`  First occurrence: tc=${thetaIssues[0].tc} theta=${thetaIssues[0].op.theta.toFixed(1)}°`);
  console.log(`  Max theta: ${Math.max(...thetaIssues.map(t => t.op.theta)).toFixed(1)}°`);
  console.log(`  This means odom heading is NOT wrapped — it accumulates turns.`);
  console.log(`  IMU (oh) vs odom theta divergence grows without bound.`);
} else {
  console.log(`  Odom theta stays within [0, 360] — no wrapping issues detected.`);
}

// ── Correction distance analysis ──
console.log(`\n\nCORRECTION DISTANCE (cd) ANALYSIS:`);
console.log("─".repeat(60));
const cdData = ticks.filter(t => t.cd !== undefined);
const highCd = cdData.filter(t => t.cd > 12);
console.log(`  Ticks where cd > 12 in: ${highCd.length}/${cdData.length}`);
if (highCd.length > 0) {
  const maxCd = highCd.reduce((a, b) => a.cd > b.cd ? a : b);
  console.log(`  Max cd: ${maxCd.cd.toFixed(1)} in at tc=${maxCd.tc} (${maxCd.reason || "accepted"})`);
}

// ── Rejection streaks ──
console.log(`\n\nREJECTION STREAKS:`);
console.log("─".repeat(60));
let streaks = [];
let curStreak = 0;
let streakStart = 0;
ticks.forEach((t, i) => {
  if (!t.accepted) {
    if (curStreak === 0) streakStart = i;
    curStreak++;
  } else {
    if (curStreak > 0) {
      streaks.push({ start: ticks[streakStart].tc, end: ticks[i - 1].tc, len: curStreak });
    }
    curStreak = 0;
  }
});
if (curStreak > 0) {
  streaks.push({ start: ticks[ticks.length - curStreak].tc, end: ticks[ticks.length - 1].tc, len: curStreak });
}
streaks.sort((a, b) => b.len - a.len);
const top5 = streaks.slice(0, 5);
top5.forEach((s, i) => {
  console.log(`  ${i + 1}. ${s.len} consecutive rejections: tc=${s.start}..${s.end}`);
});

// ── R90 spikes ──
console.log(`\n\nR90 CONVERGENCE:`);
console.log("─".repeat(60));
const r90Data = ticks.filter(t => t.r90 !== undefined);
const converged = r90Data.filter(t => t.r90 < 8.5);
const unconverged = r90Data.filter(t => t.r90 >= 8.5);
console.log(`  Converged (r90 < 8.5 in): ${converged.length}/${r90Data.length} (${(converged.length / r90Data.length * 100).toFixed(1)}%)`);
console.log(`  Unconverged: ${unconverged.length}/${r90Data.length}`);
const r90Spikes = r90Data.filter(t => t.r90 > 30);
if (r90Spikes.length > 0) {
  console.log(`  Major R90 spikes (>30 in): ${r90Spikes.length} ticks`);
  r90Spikes.slice(0, 5).forEach(t => console.log(`    tc=${t.tc} r90=${t.r90.toFixed(1)}`));
}

// ── Phase analysis: what happened when ──
console.log(`\n\nPHASE-BY-PHASE TIMELINE:`);
console.log("─".repeat(60));

function describePhase(startIdx, endIdx) {
  const slice = ticks.slice(startIdx, endIdx + 1);
  const acc = slice.filter(t => t.accepted).length;
  const rej = slice.length - acc;
  const avgR90 = slice.reduce((s, t) => s + (t.r90 || 0), 0) / slice.length;
  const avgVs = slice.reduce((s, t) => s + (t.vs || 0), 0) / slice.length;
  const cpStart = slice[0]?.cp;
  const cpEnd = slice[slice.length - 1]?.cp;
  const opEnd = slice[slice.length - 1]?.op;
  return { acc, rej, avgR90, avgVs, cpStart, cpEnd, opEnd, count: slice.length };
}

// Identify key transitions based on heading and position changes
const phases = [];
let phaseStart = 0;
for (let i = 1; i < ticks.length; i++) {
  const prev = ticks[i - 1];
  const cur = ticks[i];
  const headingDelta = Math.abs((cur.oh || 0) - (prev.oh || 0));
  const bigTurn = headingDelta > 15;
  const bigGap = (cur.tc - prev.tc) > 4;
  if (bigTurn || bigGap || i === ticks.length - 1) {
    if (i - phaseStart >= 3) {
      phases.push({ startIdx: phaseStart, endIdx: i - 1 });
    }
    phaseStart = i;
  }
}

phases.forEach((p, i) => {
  const d = describePhase(p.startIdx, p.endIdx);
  const tcRange = `tc=${ticks[p.startIdx].tc}..${ticks[p.endIdx].tc}`;
  const cp = d.cpEnd ? `cp=(${d.cpEnd.x.toFixed(1)},${d.cpEnd.y.toFixed(1)})` : "cp=?";
  const op = d.opEnd ? `op=(${d.opEnd.x.toFixed(1)},${d.opEnd.y.toFixed(1)})` : "op=?";
  console.log(`  Phase ${String(i + 1).padStart(2)}: ${tcRange.padEnd(16)} ${String(d.count).padStart(3)} ticks | acc=${String(d.acc).padStart(3)} rej=${String(d.rej).padStart(3)} | r90=${d.avgR90.toFixed(1).padStart(5)} vs=${d.avgVs.toFixed(1)} | ${cp} ${op}`);
});

// ── KEY FINDINGS ──
console.log(`\n\n${"=".repeat(72)}`);
console.log(`  KEY FINDINGS`);
console.log(`${"=".repeat(72)}\n`);

// Check odom theta wrapping
if (thetaIssues.length > 0) {
  console.log(`1. CRITICAL: ODOM HEADING NOT WRAPPED`);
  console.log(`   Odom theta reaches ${Math.max(...ticks.filter(t => t.op).map(t => t.op.theta)).toFixed(1)}° (should stay 0-360).`);
  console.log(`   MCL uses IMU heading (oh, correctly 0-360) but odom deltas are computed`);
  console.log(`   from raw odom pose. If LemLib's theta accumulates (e.g., 720° after two`);
  console.log(`   full turns), the delta calculation in compute_odom_deltas() uses prev_odom.theta`);
  console.log(`   which is also unwrapped. The sin/cos decomposition still works for deltas,`);
  console.log(`   BUT the logged op.theta diverges from oh, making diagnostics confusing.`);
  console.log(`   More critically: if any code path uses op.theta instead of oh for heading,`);
  console.log(`   the robot's sense of direction is wrong.\n`);
}

// Check left sensor
const leftValidPct = (sensorValid[0] / ticks.length * 100);
if (leftValidPct < 30) {
  console.log(`2. LEFT SENSOR MOSTLY DEAD`);
  console.log(`   Left sensor valid on only ${leftValidPct.toFixed(1)}% of ticks.`);
  console.log(`   With 3-sensor max most of the time, MCL has fewer constraints.`);
  console.log(`   Sensor might be obstructed by mechanism, out of range, or faulty.\n`);
}

// Check acceptance rate
const accRate = accepted.length / ticks.length * 100;
if (accRate < 40) {
  console.log(`3. LOW GATE ACCEPTANCE RATE (${accRate.toFixed(1)}%)`);
  console.log(`   The MCL filter was mostly rejected. Primary reasons:`);
  Object.entries(reasons).sort((a, b) => b[1] - a[1]).forEach(([r, c]) => {
    console.log(`     ${r}: ${c}×`);
  });
  console.log(`   The robot navigated primarily on dead-reckoned odom + sparse corrections.\n`);
}

// Check max correction distance gate
const maxCorrGate = rejected.filter(t => t.reason === "max correction gate");
if (maxCorrGate.length > 10) {
  console.log(`4. MAX CORRECTION GATE LOCKING OUT MCL`);
  console.log(`   ${maxCorrGate.length} rejections by max correction gate.`);
  console.log(`   Once odom drifts far enough from MCL estimate, cd exceeds max_correction_in (12 in)`);
  console.log(`   and MCL can never recover — every subsequent tick is also rejected.`);
  console.log(`   This creates a terminal state: MCL converges correctly but can't apply.`);
  const firstMaxCorr = maxCorrGate[0];
  console.log(`   First occurrence: tc=${firstMaxCorr.tc} cd=${firstMaxCorr.cd.toFixed(1)} in\n`);
}

// Check final state
const last = ticks[ticks.length - 1];
if (last) {
  console.log(`5. FINAL STATE (tc=${last.tc}):`);
  console.log(`   cp=(${last.cp?.x.toFixed(1)}, ${last.cp?.y.toFixed(1)}) — where robot thinks it is`);
  console.log(`   op=(${last.op?.x.toFixed(1)}, ${last.op?.y.toFixed(1)}) — raw odom`);
  console.log(`   re=(${last.re?.x.toFixed(1)}, ${last.re?.y.toFixed(1)}) — MCL estimate`);
  console.log(`   oh=${last.oh?.toFixed(1)}° op.θ=${last.op?.theta.toFixed(1)}°`);
  console.log(`   gate: ${last.accepted ? "ACCEPTED" : `REJECTED (${last.reason})`}`);
  console.log(`   cd=${last.cd?.toFixed(1)} in, r90=${last.r90?.toFixed(1)} in`);
  const opCpDrift = last.op && last.cp ? Math.sqrt((last.op.x - last.cp.x) ** 2 + (last.op.y - last.cp.y) ** 2) : 0;
  console.log(`   odom↔chassis drift: ${opCpDrift.toFixed(1)} in\n`);
}

console.log(`\n${"=".repeat(72)}`);
console.log(`  ROOT CAUSE SUMMARY`);
console.log(`${"=".repeat(72)}\n`);
console.log(`The skills auton failed because MCL corrections stopped being applied`);
console.log(`partway through the run. The cascade:`);
console.log(``);
console.log(`  1. Left sensor was invalid for ~${(100 - leftValidPct).toFixed(0)}% of ticks, leaving MCL with`);
console.log(`     only 2-3 sensors. Fewer sensors → weaker particle constraints → `);
console.log(`     higher R90 → more gate rejections.`);
console.log(``);
console.log(`  2. During turning/traversal phases, R90 spiked above the 8.5 in gate`);
console.log(`     threshold (${unconverged.length} ticks unconverged). The robot drove on pure odom`);
console.log(`     during these rejection streaks, accumulating drift.`);
console.log(``);
console.log(`  3. Odom accumulated enough drift that when MCL did reconverge,`);
console.log(`     the correction distance exceeded max_correction_in (12 in).`);
console.log(`     The max correction gate then permanently locked out MCL —`);
console.log(`     ${maxCorrGate.length} rejections by "max correction gate" in the tail of the run.`);
console.log(``);
console.log(`  4. Without MCL corrections, odom drift grew to ${drifts.length > 0 ? drifts[drifts.length - 1].dist.toFixed(0) : '?'} in by end of run.`);
console.log(`     moveToPoint() targets were relative to chassis_pose (which was`);
console.log(`     stuck on stale corrections), causing the robot to drive to the`);
console.log(`     wrong physical locations in the second half of the routine.`);
console.log(``);
console.log(`RECOMMENDATIONS:`);
console.log(`  a. Implement slew-rate correction (cap correction to 1-2 in/tick)`);
console.log(`     instead of the max_correction_in hard gate. This lets MCL gradually`);
console.log(`     correct large drifts instead of permanently locking out.`);
console.log(`  b. Fix or replace the left distance sensor.`);
console.log(`  c. Lower r90 gate to 6.5 in (currently 8.5 on robot) for tighter`);
console.log(`     convergence requirement, but combine with the slew approach.`);
console.log(`  d. Add kidnap/lost recovery: if rejected for >20 consecutive ticks`);
console.log(`     with vs>=3, re-init particles to break the deadlock.`);
