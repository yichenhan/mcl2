export type Action = "forward" | "backward" | "cw" | "ccw" | "none";
export type SessionMode = "live" | "route" | "replay";

export interface AABB {
  min_x: number;
  min_y: number;
  max_x: number;
  max_y: number;
}

export interface RectObstacle extends AABB {
  type: "rect";
  color?: string;
}

export interface CircleObstacle {
  type: "circle";
  cx: number;
  cy: number;
  radius: number;
  color?: string;
}

export interface TriangleObstacle {
  type: "triangle";
  x0: number;
  y0: number;
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  color?: string;
}

// Keep bare AABB in the union for backward compatibility.
export type Obstacle = RectObstacle | CircleObstacle | TriangleObstacle | AABB;

export interface Particle {
  x: number;
  y: number;
  weight: number;
}

export interface Estimate {
  x: number;
  y: number;
}

export interface Pose {
  x: number;
  y: number;
  theta: number;
}

export interface MCLSnapshot {
  particles?: Particle[];
  estimate: Estimate;
  n_eff: number;
  spread?: number;
  radius_90?: number;
}

export interface RobotState {
  x: number;
  y: number;
  heading_deg: number;
}

export interface TickState {
  tick: number;
  timestamp_iso?: string;
  ground_truth?: RobotState;
  observed_readings: [number, number, number, number];
  observed_heading: number;
  active_failures: string[];
  post_predict: MCLSnapshot;
  post_update: MCLSnapshot;
  post_resample: MCLSnapshot;
  mcl_error?: number;
  accepted_error?: number;
  odom_error?: number;
  valid_sensor_count: number;
  update_skipped?: boolean;
  pose_gated?: boolean;
  raw_odom?: Pose;
  raw_estimate?: Pose;
  accepted_estimate?: Estimate;
  /** MCL-corrected pose used for all movement decisions (Phase 4+). */
  chassis_pose?: Pose;
  gate_decision?: GateDecision;
  cluster_stats?: MCLClusterStats;
  mcl_predicted_readings?: [number, number, number, number];
  sensor_residuals?: [number, number, number, number];
  nav_status?: NavStatus;
}

export interface NavStatus {
  status: "idle" | "navigating" | "manual_override";
  current_waypoint_index: number;
  waypoints_remaining: number;
  completed: boolean;
}

export interface NavResponse {
  status: string;
  total_waypoints: number;
}

export interface CancelNavResponse {
  cancelled: boolean;
  waypoints_remaining: number;
}

export interface GateDecision {
  accepted: boolean;
  failed_centroid_jump: boolean;
  failed_r90: boolean;
  failed_passability: boolean;
  failed_residual: boolean;
  jump_in: number;
  radius_90_in: number;
  spread_in: number;
  reason: string;
  would_fail_centroid_jump?: boolean;
  would_fail_r90?: boolean;
  would_fail_passability?: boolean;
  would_fail_residual?: boolean;
  max_sensor_residual_in?: number;
  residual_threshold_in?: number;
  centroid_jump_ft_per_s?: number;
  n_eff_at_gate?: number;
}

/** Mirrors backend `state::SessionAnalytics` (replay `analytics` JSON block). */
export interface SessionAnalytics {
  peak_accepted_error: number;
  peak_accepted_error_tick: number;
  mean_accepted_error: number;
  p95_accepted_error: number;
  oracle_threshold: number;
  gate_precision: number;
  gate_recall: number;
  false_accept_count: number;
  false_accept_threshold: number;
  longest_false_accept_streak: number;
  mean_false_accept_duration: number;
  peak_error_during_false_accept: number;
  false_accept_nearest_gate_r90: number;
  false_accept_nearest_gate_residual: number;
  false_accept_nearest_gate_centroid_jump: number;
  false_reject_count: number;
  false_reject_threshold: number;
  r90_reject_count: number;
  residual_reject_count: number;
  centroid_jump_reject_count: number;
  passability_reject_count: number;
  r90_false_reject_count: number;
  residual_false_reject_count: number;
  centroid_jump_false_reject_count: number;
  passability_false_reject_count: number;
  gate_availability: number;
  longest_rejection_streak: number;
  cold_start_convergence_ticks: number;
  cold_start_convergence_error: number;
  worst_reacquisition_ticks: number;
  mean_reacquisition_ticks: number;
  reacquisition_count: number;
  post_convergence_error_stddev: number;
  peak_r90_at_acceptance: number;
  neff_collapse_count: number;
  sensor_dropout_rate: number;
  total_ticks: number;
}

export type ScrubberAnnotationType = "error_peak" | "false_accept" | "rejection_streak" | "convergence";

export interface ScrubberAnnotation {
  tick: number;
  /** When set, scrubber draws a span (e.g. rejection streak). */
  endTick?: number;
  type: ScrubberAnnotationType;
  label: string;
}

export interface PhaseSnapshot {
  particles?: Particle[];
  estimate: Estimate;
  n_eff: number;
  spread: number;
  radius_90: number;
}

export interface MCLClusterStats {
  spread: number;
  radius_90: number;
  centroid: Estimate;
}

export interface MCLTickResult {
  tick_count?: number;
  raw_odom?: Pose;
  raw_estimate: Pose;
  observed_readings?: [number, number, number, number];
  mcl_sensor_residuals?: [number, number, number, number];
  mcl_predicted_readings?: [number, number, number, number];
  gate: GateDecision;
  valid_sensor_count: number;
  update_skipped: boolean;
  cluster_stats: MCLClusterStats;
  n_eff: number;
  post_predict: PhaseSnapshot;
  post_update: PhaseSnapshot;
  post_resample: PhaseSnapshot;
}

export interface MCLReplayFile {
  session_id?: string;
  total_ticks?: number;
  field_half?: number;
  obstacles?: Obstacle[];
  ticks: MCLTickResult[];
}

export type AnyTick = TickState | MCLTickResult;

export function isTickState(tick: AnyTick): tick is TickState {
  return "post_resample" in tick;
}

export interface SessionStartRequest {
  mode?: SessionMode;
  session_name?: string;
  map_name?: string;
  algorithm?: "arc_nav" | "turn_and_go";
  seed?: number;
  num_particles?: number;
  field_half?: number;
  initial_state?: RobotState;
  max_ticks?: number;
  failure_config?: FailureConfig;
  obstacles?: Obstacle[];
}

export interface SessionStartResponse {
  session_id: string;
  session_name?: string;
}

export interface SessionConfigResponse {
  seed: number;
  num_particles: number;
  field_half: number;
  robot_radius: number;
  obstacles: Obstacle[];
}

export interface Waypoint {
  x: number;
  y: number;
}

export interface FollowerConfig {
  linear_velocity: number;
  waypoint_tolerance: number;
  max_angular_velocity_deg: number;
  turn_in_place_threshold_deg: number;
}

export interface FailureConfig {
  sensor_dead_prob: number;
  sensor_stuck_prob: number;
  odom_spike_prob: number;
  heading_bias_prob: number;
  spurious_reflection_prob: number;
  kidnap_prob: number;
  /** Shared minimum duration (ticks) for multi-tick failures. */
  min_duration_ticks: number;
  /** Legacy fallback when a per-type max is not set (backend uses 0 = unset). */
  max_duration_ticks: number;
  sensor_dead_max_duration_ticks: number;
  sensor_stuck_max_duration_ticks: number;
  odom_spike_max_duration_ticks: number;
  heading_bias_max_duration_ticks: number;
  spurious_reflection_max_duration_ticks: number;
  odom_spike_range: [number, number];
  heading_bias_range: [number, number];
  spurious_reflection_range: [number, number];
}

export interface OverlayFlags {
  robotTruth: boolean;
  rawOdom: boolean;
  chassisPose: boolean;
  mclEstimate: boolean;
  acceptedEstimate: boolean;
  r90Circle: boolean;
  particles: boolean;
  heatmap: boolean;
  sensorReadings: boolean;
  sensorResiduals: boolean;
  diffMclPose: boolean;
  diffMclTruth: boolean;
  diffPoseTruth: boolean;
}

export interface MapPreset {
  name: string;
  obstacles: Obstacle[];
}

export interface KidnapEvent {
  tick: number;
  target: "random" | { x: number; y: number };
}

export interface RouteDefinition {
  name: string;
  description: string;
  waypoints: Waypoint[];
  obstacles: Obstacle[];
  initial_heading_deg: number;
  follower: FollowerConfig;
  failure_seed: number;
  failure_config: FailureConfig;
  kidnap_events: KidnapEvent[];
  max_ticks: number;
}

export interface RouteRunResult {
  route_name: string;
  seed: number;
  total_ticks: number;
  waypoints_reached: number;
  completed: boolean;
  final_mcl_error: number;
  mean_mcl_error: number;
}

export interface ParsedFailure {
  type:
    | "sensor_dead"
    | "sensor_stuck"
    | "odom_spike"
    | "heading_bias"
    | "spurious_reflection"
    | "kidnap"
    | "unknown";
  sensor?: number;
  sensorName?: string;
  param?: number;
  targetX?: number;
  targetY?: number;
  label: string;
  severity: "info" | "warning" | "critical";
  color: string;
}
