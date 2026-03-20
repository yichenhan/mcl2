export type Action = "forward" | "backward" | "cw" | "ccw" | "none";

export interface AABB {
  min_x: number;
  min_y: number;
  max_x: number;
  max_y: number;
}

export interface Particle {
  x: number;
  y: number;
  weight: number;
}

export interface Estimate {
  x: number;
  y: number;
}

export interface MCLSnapshot {
  particles: Particle[];
  estimate: Estimate;
  n_eff: number;
}

export interface RobotState {
  x: number;
  y: number;
  heading_deg: number;
}

export interface TickState {
  tick: number;
  ground_truth: RobotState;
  observed_readings: [number, number, number, number];
  observed_heading: number;
  active_failures: string[];
  post_predict: MCLSnapshot;
  post_update: MCLSnapshot;
  post_resample: MCLSnapshot;
  mcl_error: number;
  odom_error: number;
  valid_sensor_count: number;
}

export interface SessionStartRequest {
  seed?: number;
  num_particles?: number;
  obstacles?: AABB[];
}

export interface SessionStartResponse {
  session_id: string;
}

export interface SessionConfigResponse {
  seed: number;
  num_particles: number;
  field_half: number;
  robot_radius: number;
  obstacles: AABB[];
}

export interface Waypoint {
  x: number;
  y: number;
}

export interface PurePursuitConfig {
  lookahead_distance: number;
  linear_velocity: number;
  waypoint_tolerance: number;
  max_angular_velocity_deg?: number;
}

export interface FailureConfig {
  sensor_dead_prob: number;
  sensor_stuck_prob: number;
  odom_spike_prob: number;
  heading_bias_prob: number;
  spurious_reflection_prob: number;
  kidnap_prob: number;
  min_duration_ticks: number;
  max_duration_ticks: number;
  odom_spike_range: [number, number];
  heading_bias_range: [number, number];
  spurious_reflection_range: [number, number];
}

export interface KidnapEvent {
  tick: number;
  target: "random" | { x: number; y: number };
}

export interface RouteDefinition {
  name: string;
  description: string;
  waypoints: Waypoint[];
  obstacles: AABB[];
  initial_heading_deg: number;
  pure_pursuit: PurePursuitConfig;
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
