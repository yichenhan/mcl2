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
