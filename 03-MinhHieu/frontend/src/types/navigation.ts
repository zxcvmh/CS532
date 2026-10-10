export type NavigationState = 'IDLE' | 'PLANNING' | 'NAVIGATING' | 'REACHED' | 'BLOCKED' | 'CANCELLED' | 'ERROR';

export interface NavigationStatus {
  status: NavigationState;
  goalX: number | null;
  goalY: number | null;
  distanceRemaining: number;
  progress: number;
  velocity: number;
}

export interface GoalPosition {
  x: number;
  y: number;
}

export interface ObstacleCluster {
  id: number;
  centroid: [number, number];
  centroid_world?: [number, number];
  closest_point: [number, number];
  min_distance: number;
  azimuth_deg: number;
  length_m: number;
  width_m: number;
  orientation_rad: number;
  points_count: number;
  bounding_box_2d: [number, number][];
  bounding_box_world?: [number, number][];
  obstacle_type: 'CYLINDER_LEG' | 'BOX_OBSTACLE' | 'WALL_SURFACE' | string;
  threat_level: 'CRITICAL' | 'WARNING' | 'SAFE' | 'CRITICAL_ONCOMING' | string;
  speed: number;
  closing_speed: number;
  ttc: number;
  is_dynamic: boolean;
}

export interface SocialBubble {
  x: number;
  y: number;
  radius: number;
  label?: string;
}

export interface MotionXaiInfo {
  status: string;
  speed_scale: number;
  curvature_w: number;
  evasion_active: boolean;
  evasion_reason: string;
  lookahead_target: [number, number] | null;
  clusters: ObstacleCluster[];
  low_obstacles: [number, number][];
  social_bubbles: SocialBubble[];
}
