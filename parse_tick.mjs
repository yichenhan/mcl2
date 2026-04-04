#!/usr/bin/env node
/**
 * Parse a VEX serial log .txt the same way the frontend does,
 * extract a specific tick, and print a gate-failure analysis.
 *
 * Usage: node parse_tick.mjs <file> <tick_number>
 */
import { readFileSync as readBuf } from "fs";

const COMPACT_KV_RE = /^\[T=(\d+)\|\|([^\]]+)\]=(.*)$/;
const TICK_KV_RE = /^\[TICK=(\d+)\s*\|\|\s*([^\]]+)\]=(.*)$/;

const COMPACT_KEY_MAP = {
  tc: "tick_count", oh: "observed_heading", pg: "pose_gated",
  ts: "timestamp_iso", vs: "valid_sensor_count", us: "update_skipped",
  ne: "n_eff",
  "or.0": "observed_readings.0", "or.1": "observed_readings.1",
  "or.2": "observed_readings.2", "or.3": "observed_readings.3",
  "op.x": "odom_pose.x", "op.y": "odom_pose.y", "op.t": "odom_pose.theta",
  "re.x": "raw_estimate.x", "re.y": "raw_estimate.y", "re.t": "raw_estimate.theta",
  "cp.x": "chassis_pose.x", "cp.y": "chassis_pose.y", "cp.t": "chassis_pose.theta",
  "cs.x": "cluster_stats.centroid.x", "cs.y": "cluster_stats.centroid.y",
  "cs.r": "cluster_stats.radius_90", "cs.s": "cluster_stats.spread",
  "g.a": "gate.accepted", "g.rsn": "gate.reason",
  "g.r": "gate.radius_90_in", "g.j": "gate.jump_in", "g.s": "gate.spread_in",
  "g.fcj": "gate.failed_centroid_jump", "g.fr": "gate.failed_r90",
  "g.fp": "gate.failed_passability", "g.fres": "gate.failed_residual",
  "g.wcj": "gate.would_fail_centroid_jump", "g.wr": "gate.would_fail_r90",
  "g.wp": "gate.would_fail_passability", "g.wres": "gate.would_fail_residual",
  "g.msr": "gate.max_sensor_residual_in", "g.rt": "gate.residual_threshold_in",
  "g.cj": "gate.centroid_jump_ft_per_s", "g.ne": "gate.n_eff_at_gate",
  "pr.0": "mcl_predicted_readings.0", "pr.1": "mcl_predicted_readings.1",
  "pr.2": "mcl_predicted_readings.2", "pr.3": "mcl_predicted_readings.3",
  "sr.0": "mcl_sensor_residuals.0", "sr.1": "mcl_sensor_residuals.1",
  "sr.2": "mcl_sensor_residuals.2", "sr.3": "mcl_sensor_residuals.3",
  "pp.x": "post_predict.estimate.x", "pp.y": "post_predict.estimate.y",
  "pp.n": "post_predict.n_eff", "pp.s": "post_predict.spread", "pp.r": "post_predict.radius_90",
  "pu.x": "post_update.estimate.x", "pu.y": "post_update.estimate.y",
  "pu.n": "post_update.n_eff", "pu.s": "post_update.spread", "pu.r": "post_update.radius_90",
  "ps.x": "post_resample.estimate.x", "ps.y": "post_resample.estimate.y",
  "ps.n": "post_resample.n_eff", "ps.s": "post_resample.spread", "ps.r": "post_resample.radius_90",
};

const BOOL_PATHS = new Set([
  "pose_gated", "update_skipped", "gate.accepted",
  "gate.failed_centroid_jump", "gate.failed_r90", "gate.failed_passability",
  "gate.failed_residual", "gate.would_fail_centroid_jump", "gate.would_fail_r90",
  "gate.would_fail_passability", "gate.would_fail_residual",
]);

function setNested(obj, path, raw) {
  const keys = path.split(".");
  let cur = obj;
  for (let i = 0; i < keys.length - 1; i++) {
    const k = keys[i];
    if (!(k in cur) || typeof cur[k] !== "object" || cur[k] === null)
      cur[k] = /^\d+$/.test(keys[i + 1]) ? [] : {};
    cur = cur[k];
  }
  const leaf = keys[keys.length - 1];
  let value = raw;
  if (raw === "true") value = true;
  else if (raw === "false") value = false;
  else if (raw === "null") value = null;
  else { const n = Number(raw); if (!Number.isNaN(n)) value = n; }
  if (Array.isArray(cur)) cur[Number(leaf)] = value;
  else cur[leaf] = value;
}

function parseTicks(text) {
  const tickMap = new Map();
  for (const line of text.split("\n")) {
    const trimmed = line.trim();
    let tickNum, shortKey, rawVal;

    const compact = COMPACT_KV_RE.exec(trimmed);
    if (compact) {
      tickNum = Number(compact[1]);
      shortKey = compact[2].trim();
      const fullKey = COMPACT_KEY_MAP[shortKey] ?? shortKey;
      rawVal = compact[3];
      if (BOOL_PATHS.has(fullKey)) rawVal = rawVal === "1" ? "true" : "false";
      if (!tickMap.has(tickNum)) tickMap.set(tickNum, { tick_count: tickNum });
      setNested(tickMap.get(tickNum), fullKey, rawVal);
      continue;
    }
    const old = TICK_KV_RE.exec(trimmed);
    if (old) {
      tickNum = Number(old[1]);
      if (!tickMap.has(tickNum)) tickMap.set(tickNum, {});
      setNested(tickMap.get(tickNum), old[2].trim(), old[3]);
    }
  }
  return tickMap;
}

// --- main ---
const [,, file, tickArg] = process.argv;
if (!file || !tickArg) { console.error("Usage: node parse_tick.mjs <file> <tick>"); process.exit(1); }

const buf = readBuf(file);
const text = (buf[0] === 0xff && buf[1] === 0xfe) ? buf.toString("utf16le") : buf.toString("utf-8");
const ticks = parseTicks(text);
const tickNum = Number(tickArg);
const t = ticks.get(tickNum);

if (!t) { console.error(`Tick ${tickNum} not found. Available: ${[...ticks.keys()].join(", ")}`); process.exit(1); }

console.log(`\n=== TICK ${tickNum} — Full Parsed Data ===\n`);
console.log(JSON.stringify(t, null, 2));

console.log(`\n=== GATE ANALYSIS ===\n`);
const g = t.gate ?? {};
console.log(`  Result:         ${g.accepted ? "ACCEPTED" : "REJECTED"}`);
console.log(`  Reason:         ${g.reason ?? "—"}`);
console.log(`  ---`);
console.log(`  r90:            ${g.radius_90_in}  (threshold: 4.0)`);
console.log(`  failed_r90:     ${g.failed_r90}`);
console.log(`  would_fail_r90: ${g.would_fail_r90}`);
console.log(`  ---`);
console.log(`  max_sensor_residual: ${g.max_sensor_residual_in}  (threshold: ${g.residual_threshold_in})`);
console.log(`  failed_residual:     ${g.failed_residual}`);
console.log(`  would_fail_residual: ${g.would_fail_residual}`);
console.log(`  ---`);
console.log(`  jump:              ${g.jump_in}  (threshold: 12.0)`);
console.log(`  centroid_jump_ft/s: ${g.centroid_jump_ft_per_s}  (threshold: 30.0)`);
console.log(`  failed_centroid_jump: ${g.failed_centroid_jump}`);
console.log(`  would_fail_centroid_jump: ${g.would_fail_centroid_jump}`);
console.log(`  ---`);
console.log(`  failed_passability:     ${g.failed_passability}`);
console.log(`  would_fail_passability: ${g.would_fail_passability}`);

console.log(`\n=== SENSOR DIAGNOSTICS ===\n`);
const labels = ["L (sensor 0)", "R (sensor 1)", "F (sensor 2)", "B (sensor 3)"];
for (let i = 0; i < 4; i++) {
  const reading = t.observed_readings?.[i] ?? -1;
  const predicted = t.mcl_predicted_readings?.[i] ?? -1;
  const residual = t.mcl_sensor_residuals?.[i] ?? 0;
  const status = reading < 0 ? "INVALID" : "ok";
  console.log(`  ${labels[i]}: reading=${reading}  predicted=${predicted}  residual=${residual}  ${status}`);
}

console.log(`\n=== POSES ===\n`);
console.log(`  Odom (chassis):  (${t.odom_pose?.x}, ${t.odom_pose?.y}) θ=${t.odom_pose?.theta}`);
console.log(`  MCL estimate:    (${t.raw_estimate?.x}, ${t.raw_estimate?.y}) θ=${t.raw_estimate?.theta}`);
console.log(`  Cluster centroid:(${t.cluster_stats?.centroid?.x}, ${t.cluster_stats?.centroid?.y})`);
console.log(`  Cluster r90:     ${t.cluster_stats?.radius_90}`);
console.log(`  Cluster spread:  ${t.cluster_stats?.spread}`);
