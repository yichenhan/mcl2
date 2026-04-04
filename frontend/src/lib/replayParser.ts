import { toTickState } from "@/lib/tick";
import type { TickState } from "@/lib/types";

const MCL_JSON_SEGMENT_RE = /\[MCL_JSON_START\]\s*([\s\S]*?)\s*\[MCL_JSON_FINISH\]/g;
const TICK_KV_RE = /^\[TICK=(\d+)\s*\|\|\s*([^\]]+)\]=(.*)$/;
const COMPACT_KV_RE = /^\[T=(\d+)\|\|([^\]]+)\]=(.*)$/;

const COMPACT_KEY_MAP: Record<string, string> = {
  tc: "tick_count",
  oh: "observed_heading",
  pg: "pose_gated",
  ts: "timestamp_iso",
  vs: "valid_sensor_count",
  us: "update_skipped",
  ne: "n_eff",
  "or.0": "observed_readings.0",
  "or.1": "observed_readings.1",
  "or.2": "observed_readings.2",
  "or.3": "observed_readings.3",
  "op.x": "raw_odom.x",
  "op.y": "raw_odom.y",
  "op.t": "raw_odom.theta",
  "re.x": "raw_estimate.x",
  "re.y": "raw_estimate.y",
  "re.t": "raw_estimate.theta",
  "cs.x": "cluster_stats.centroid.x",
  "cs.y": "cluster_stats.centroid.y",
  "cs.r": "cluster_stats.radius_90",
  "cs.s": "cluster_stats.spread",
  "g.a": "gate.accepted",
  "g.rsn": "gate.reason",
  "g.r": "gate.radius_90_in",
  "g.j": "gate.jump_in",
  "g.s": "gate.spread_in",
  "g.fcj": "gate.failed_centroid_jump",
  "g.fr": "gate.failed_r90",
  "g.fp": "gate.failed_passability",
  "g.fres": "gate.failed_residual",
  "g.wcj": "gate.would_fail_centroid_jump",
  "g.wr": "gate.would_fail_r90",
  "g.wp": "gate.would_fail_passability",
  "g.wres": "gate.would_fail_residual",
  "g.msr": "gate.max_sensor_residual_in",
  "g.rt": "gate.residual_threshold_in",
  "g.cj": "gate.centroid_jump_ft_per_s",
  "g.ne": "gate.n_eff_at_gate",
  "pr.0": "mcl_predicted_readings.0",
  "pr.1": "mcl_predicted_readings.1",
  "pr.2": "mcl_predicted_readings.2",
  "pr.3": "mcl_predicted_readings.3",
  "sr.0": "mcl_sensor_residuals.0",
  "sr.1": "mcl_sensor_residuals.1",
  "sr.2": "mcl_sensor_residuals.2",
  "sr.3": "mcl_sensor_residuals.3",
  "pp.x": "post_predict.estimate.x",
  "pp.y": "post_predict.estimate.y",
  "pp.n": "post_predict.n_eff",
  "pp.s": "post_predict.spread",
  "pp.r": "post_predict.radius_90",
  "pu.x": "post_update.estimate.x",
  "pu.y": "post_update.estimate.y",
  "pu.n": "post_update.n_eff",
  "pu.s": "post_update.spread",
  "pu.r": "post_update.radius_90",
  "ps.x": "post_resample.estimate.x",
  "ps.y": "post_resample.estimate.y",
  "ps.n": "post_resample.n_eff",
  "ps.s": "post_resample.spread",
  "ps.r": "post_resample.radius_90",
};

const COMPACT_BOOL_PATHS = new Set([
  "pose_gated",
  "update_skipped",
  "gate.accepted",
  "gate.failed_centroid_jump",
  "gate.failed_r90",
  "gate.failed_passability",
  "gate.failed_residual",
  "gate.would_fail_centroid_jump",
  "gate.would_fail_r90",
  "gate.would_fail_passability",
  "gate.would_fail_residual",
]);

function setNestedValue(obj: Record<string, unknown>, path: string, raw: string): void {
  const keys = path.split(".");
  let cur: Record<string, unknown> = obj;
  for (let i = 0; i < keys.length - 1; i++) {
    const k = keys[i];
    if (!(k in cur) || typeof cur[k] !== "object" || cur[k] === null) {
      const nextKey = keys[i + 1];
      cur[k] = /^\d+$/.test(nextKey) ? [] : {};
    }
    cur = cur[k] as Record<string, unknown>;
  }
  const leaf = keys[keys.length - 1];
  let value: unknown = raw;
  if (raw === "true") value = true;
  else if (raw === "false") value = false;
  else if (raw === "null") value = null;
  else if (raw.startsWith("\"") && raw.endsWith("\"")) {
    try {
      value = JSON.parse(raw);
    } catch {
      value = raw;
    }
  } else {
    const n = Number(raw);
    if (!Number.isNaN(n)) value = n;
  }
  if (Array.isArray(cur)) {
    cur[Number(leaf)] = value;
  } else {
    cur[leaf] = value;
  }
}

function parseTickKVLines(text: string): unknown[] {
  const tickMap = new Map<number, Record<string, unknown>>();
  for (const line of text.split("\n")) {
    const trimmed = line.trim();
    const old = TICK_KV_RE.exec(trimmed);
    if (old) {
      const tickNum = Number(old[1]);
      if (!tickMap.has(tickNum)) tickMap.set(tickNum, {});
      setNestedValue(tickMap.get(tickNum)!, old[2].trim(), old[3]);
      continue;
    }
    const compact = COMPACT_KV_RE.exec(trimmed);
    if (compact) {
      const tickNum = Number(compact[1]);
      const shortKey = compact[2].trim();
      const fullKey = COMPACT_KEY_MAP[shortKey] ?? shortKey;
      let rawVal = compact[3];
      if (COMPACT_BOOL_PATHS.has(fullKey)) {
        rawVal = rawVal === "1" ? "true" : "false";
      }
      if (!tickMap.has(tickNum)) tickMap.set(tickNum, {});
      setNestedValue(tickMap.get(tickNum)!, fullKey, rawVal);
    }
  }
  return [...tickMap.entries()].sort((a, b) => a[0] - b[0]).map(([, obj]) => obj);
}

export function parseReplayText(text: string): TickState[] {
  try {
    const parsed = JSON.parse(text) as unknown;
    if (Array.isArray(parsed)) return parsed.map(toTickState);
    if (parsed && typeof parsed === "object" && Array.isArray((parsed as { ticks?: unknown[] }).ticks)) {
      return ((parsed as { ticks: unknown[] }).ticks).map(toTickState);
    }
  } catch {
    // Fall through to text formats.
  }

  const kvTicks = parseTickKVLines(text);
  if (kvTicks.length > 0) return kvTicks.map(toTickState);

  const segmentTicks: unknown[] = [];
  for (const match of text.matchAll(MCL_JSON_SEGMENT_RE)) {
    const raw = (match[1] ?? "").trim();
    if (!raw) continue;
    try {
      segmentTicks.push(JSON.parse(raw) as unknown);
    } catch {
      // Ignore malformed segment.
    }
  }
  if (segmentTicks.length > 0) return segmentTicks.map(toTickState);

  throw new Error(
    "Could not parse replay. Supported formats: JSON ticks, [T=N||k]=v compact lines, [TICK=..||..]=.. lines, or [MCL_JSON_START] segments.",
  );
}
