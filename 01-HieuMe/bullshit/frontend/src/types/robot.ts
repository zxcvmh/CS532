export interface RobotPose {
  x: number;
  y: number;
  theta: number;
}

export interface RobotStatus {
  connected: boolean;
  battery: number;
  mode: 'MANUAL' | 'AUTONOMOUS';
  pose: RobotPose;
  linearVelocity: number;
  angularVelocity: number;
  sensors: SensorHealth[];
}

export interface SensorHealth {
  name: string;
  status: 'ACTIVE' | 'WARNING' | 'OFFLINE';
  lastUpdate: number;
}
