export interface RobotPose {
  x: number;
  y: number;
  theta: number;
  linearSpeed?: number;
  angularSpeed?: number;
  nearestObstacle?: number | null;
  cpuTemp?: number;
}

export interface RobotStatus {
  connected: boolean;
  battery: number;
  mode: 'MANUAL' | 'AUTONOMOUS';
  pose: RobotPose;
  linearVelocity: number;
  angularVelocity: number;
  sensors: SensorHealth[];
  cpuTemp?: number;
  nearestObstacle?: number | null;
}

export interface SensorHealth {
  name: string;
  status: 'ACTIVE' | 'WARNING' | 'OFFLINE';
  lastUpdate: number;
}
