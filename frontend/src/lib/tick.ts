import type { TickState } from "@/lib/types";

export function toTickState(raw: unknown): TickState {
  const r = (raw ?? {}) as Partial<TickState>;
  const legacy = raw as Record<string, unknown>;
  const gateDecision = (r.gate_decision ?? legacy.gate) as TickState["gate_decision"];
  const sensorResiduals = (r.sensor_residuals ?? legacy.mcl_sensor_residuals) as TickState["sensor_residuals"];
  const predictedReadings = (r.mcl_predicted_readings ?? legacy.mcl_predicted_readings) as TickState["mcl_predicted_readings"];
  return {
    tick: r.tick ?? (typeof legacy.tick_count === "number" ? legacy.tick_count : 0),
    timestamp_iso: r.timestamp_iso ?? "",
    ground_truth: r.ground_truth ?? { x: 0, y: 0, heading_deg: 0 },
    observed_readings: r.observed_readings ?? [-1, -1, -1, -1],
    observed_heading: r.observed_heading ?? 0,
    active_failures: r.active_failures ?? [],
    post_predict: r.post_predict ?? { estimate: { x: 0, y: 0 }, n_eff: 0, spread: 0, radius_90: 0 },
    post_update: r.post_update ?? { estimate: { x: 0, y: 0 }, n_eff: 0, spread: 0, radius_90: 0 },
    post_resample: r.post_resample ?? { estimate: { x: 0, y: 0 }, n_eff: 0, spread: 0, radius_90: 0 },
    mcl_error: r.mcl_error ?? 0,
    accepted_error: r.accepted_error ?? 0,
    odom_error: r.odom_error ?? 0,
    valid_sensor_count: r.valid_sensor_count ?? 0,
    update_skipped: r.update_skipped ?? false,
    pose_gated: r.pose_gated ?? false,
    odom_pose: r.odom_pose,
    raw_estimate: r.raw_estimate,
    accepted_estimate: r.accepted_estimate,
    gate_decision: gateDecision,
    cluster_stats: r.cluster_stats,
    mcl_predicted_readings: predictedReadings,
    sensor_residuals: sensorResiduals,
  };
}
