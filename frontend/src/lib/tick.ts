import type { MCLClusterStats, MCLSnapshot, TickState } from "@/lib/types";

function num(v: unknown, fallback = 0): number {
  return typeof v === "number" && Number.isFinite(v) ? v : fallback;
}

function snapshotFromCluster(cs: MCLClusterStats, nEff: number): MCLSnapshot {
  return {
    estimate: { x: cs.centroid?.x ?? 0, y: cs.centroid?.y ?? 0 },
    n_eff: nEff,
    spread: cs.spread,
    radius_90: cs.radius_90,
  };
}

export function toTickState(raw: unknown): TickState {
  const r = (raw ?? {}) as Partial<TickState>;
  const legacy = raw as Record<string, unknown>;
  const gateDecision = (r.gate_decision ?? legacy.gate) as TickState["gate_decision"];
  const sensorResiduals = (r.sensor_residuals ?? legacy.mcl_sensor_residuals) as TickState["sensor_residuals"];
  const predictedReadings = (r.mcl_predicted_readings ?? legacy.mcl_predicted_readings) as TickState["mcl_predicted_readings"];

  const cs = r.cluster_stats as MCLClusterStats | undefined;
  const nEff = num((legacy as Record<string, unknown>).n_eff);
  const clusterFallback = cs ? snapshotFromCluster(cs, nEff) : undefined;
  const defaultSnapshot: MCLSnapshot = { estimate: { x: 0, y: 0 }, n_eff: 0, spread: 0, radius_90: 0 };

  const postPredict = r.post_predict ?? clusterFallback ?? defaultSnapshot;
  const postUpdate = r.post_update ?? clusterFallback ?? defaultSnapshot;
  const postResample = r.post_resample ?? clusterFallback ?? defaultSnapshot;

  if (cs && postResample.radius_90 === undefined) {
    postResample.radius_90 = cs.radius_90;
  }
  if (cs && postResample.spread === undefined) {
    postResample.spread = cs.spread;
  }

  const rawEstimate = r.raw_estimate ?? (cs?.centroid ? { x: cs.centroid.x, y: cs.centroid.y, theta: r.observed_heading ?? 0 } : undefined);

  const odomPose = r.odom_pose
    && typeof r.odom_pose.x === "number"
    && typeof r.odom_pose.y === "number"
    ? r.odom_pose
    : undefined;

  return {
    tick: r.tick ?? (typeof legacy.tick_count === "number" ? legacy.tick_count : 0),
    timestamp_iso: r.timestamp_iso ?? "",
    ground_truth: r.ground_truth ?? { x: 0, y: 0, heading_deg: 0 },
    observed_readings: r.observed_readings ?? [-1, -1, -1, -1],
    observed_heading: r.observed_heading ?? 0,
    active_failures: r.active_failures ?? [],
    post_predict: postPredict,
    post_update: postUpdate,
    post_resample: postResample,
    mcl_error: r.mcl_error ?? 0,
    accepted_error: r.accepted_error ?? 0,
    odom_error: r.odom_error ?? 0,
    valid_sensor_count: r.valid_sensor_count ?? 0,
    update_skipped: r.update_skipped ?? false,
    pose_gated: r.pose_gated ?? false,
    odom_pose: odomPose,
    raw_estimate: rawEstimate,
    accepted_estimate: r.accepted_estimate,
    gate_decision: gateDecision,
    cluster_stats: cs,
    mcl_predicted_readings: predictedReadings,
    sensor_residuals: sensorResiduals,
  };
}
