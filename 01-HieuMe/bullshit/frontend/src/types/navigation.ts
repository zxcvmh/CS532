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
