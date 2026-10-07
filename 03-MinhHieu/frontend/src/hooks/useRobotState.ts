import { useState, useEffect, useRef, useCallback } from 'react';
import { useWebSocket } from './useWebSocket';
import { RobotPose, SensorHealth } from '../types/robot';
import { MapData, LidarPoint } from '../types/map';
import { Detection } from '../types/detection';
import { NavigationStatus } from '../types/navigation';
import { EventLogEntry } from '../types/events';

export function useRobotState() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  // If running on Vite dev server (port 5173), connect to 8000; otherwise connect to same origin host
  const wsHost = window.location.port === '5173'
    ? (window.location.hostname === 'localhost' ? '127.0.0.1:8000' : `${window.location.hostname}:8000`)
    : (window.location.host || '127.0.0.1:8000');
  const wsUrl = `${protocol}//${wsHost}/ws`;
  const { connected, lastMessage, sendMessage, rtt } = useWebSocket(wsUrl);

  // React state for things that trigger UI updates
  const [battery, setBattery] = useState<number>(100);
  const [cpuTemp, setCpuTemp] = useState<number>(43.5);
  const [linearSpeed, setLinearSpeed] = useState<number>(0);
  const [angularSpeed, setAngularSpeed] = useState<number>(0);
  const [nearestObstacle, setNearestObstacle] = useState<number | null>(null);
  const [mode, setMode] = useState<'MANUAL' | 'AUTONOMOUS'>('MANUAL');
  const [isRealConnected, setIsRealConnected] = useState<boolean>(false);
  const [navigationStatus, setNavigationStatus] = useState<NavigationStatus>({
    status: 'IDLE', goalX: null, goalY: null, distanceRemaining: 0, progress: 0, velocity: 0
  });
  const [sensors, setSensors] = useState<SensorHealth[]>([]);
  const [events, setEvents] = useState<EventLogEntry[]>([]);
  const [cameraFrame, setCameraFrame] = useState<string>('');
  
  // Refs for high-frequency data (avoids re-rendering the whole app)
  const pose = useRef<RobotPose>({ x: 0, y: 0, theta: 0 });
  const lidarPoints = useRef<LidarPoint[]>([]);
  const mapData = useRef<MapData | null>(null);
  const detections = useRef<Detection[]>([]);
  const trajectory = useRef<LidarPoint[]>([]);
  const plannedPath = useRef<LidarPoint[]>([]);

  // Trigger for components that need to know detections changed (for CameraView)
  const [detectionsVersion, setDetectionsVersion] = useState(0);
  
  useEffect(() => {
    if (!lastMessage) return;
    
    // Messages from backend have flat structure: { type: "robot_pose", x: ..., y: ..., theta: ... }
    switch (lastMessage.type) {
      case 'robot_pose':
        pose.current = {
          x: lastMessage.x,
          y: lastMessage.y,
          theta: lastMessage.theta,
          linearSpeed: lastMessage.linear_speed,
          angularSpeed: lastMessage.angular_speed,
          nearestObstacle: lastMessage.nearest_obstacle,
          cpuTemp: lastMessage.cpu_temp
        };
        if (lastMessage.linear_speed !== undefined) setLinearSpeed(lastMessage.linear_speed);
        if (lastMessage.angular_speed !== undefined) setAngularSpeed(lastMessage.angular_speed);
        if (lastMessage.nearest_obstacle !== undefined) setNearestObstacle(lastMessage.nearest_obstacle);
        if (lastMessage.cpu_temp !== undefined) setCpuTemp(lastMessage.cpu_temp);
        break;
      case 'battery':
        setBattery(lastMessage.percentage);
        break;
      case 'lidar_scan':
        lidarPoints.current = lastMessage.points;
        break;
      case 'map_update':
        mapData.current = {
          resolution: lastMessage.resolution,
          width: lastMessage.width,
          height: lastMessage.height,
          originX: lastMessage.origin_x,
          originY: lastMessage.origin_y,
          data: lastMessage.data,
          version: (mapData.current?.version ?? 0) + 1
        };
        break;
      case 'detections':
        detections.current = lastMessage.objects.map((d: any) => ({
          className: d.class_name,
          confidence: d.confidence,
          bbox: d.bbox,
          distance: d.distance,
          azimuth_deg: d.azimuth_deg,
          azimuthDeg: d.azimuth_deg
        }));
        setDetectionsVersion(v => v + 1);
        break;
      case 'navigation_status':
        setNavigationStatus({
          status: lastMessage.status,
          goalX: lastMessage.goal_x ?? null,
          goalY: lastMessage.goal_y ?? null,
          distanceRemaining: lastMessage.distance_remaining ?? 0,
          progress: lastMessage.progress ?? 0,
          velocity: lastMessage.velocity ?? 0
        });
        break;
      case 'sensor_health':
        if (lastMessage.is_real_connected !== undefined) {
          setIsRealConnected(!!lastMessage.is_real_connected);
        }
        setSensors(lastMessage.sensors.map((s: any) => ({
          name: s.name,
          status: s.status,
          lastUpdate: s.last_update
        })));
        break;
      case 'event':
        setEvents(prev => [{
          timestamp: lastMessage.timestamp,
          level: lastMessage.level,
          message: lastMessage.message
        }, ...prev].slice(0, 100));
        break;
      case 'planned_path':
        plannedPath.current = lastMessage.points;
        break;
      case 'trajectory':
        trajectory.current = lastMessage.points;
        break;
      case 'camera_frame':
        setCameraFrame(lastMessage.data);
        break;
    }
  }, [lastMessage]);

  // Backend expects flat messages: { type: "set_goal", x: ..., y: ... }
  const sendGoal = useCallback((x: number, y: number) => {
    sendMessage({ type: 'set_goal', x, y });
  }, [sendMessage]);

  const cancelGoal = useCallback(() => {
    sendMessage({ type: 'cancel_goal' });
  }, [sendMessage]);

  const emergencyStop = useCallback(() => {
    sendMessage({ type: 'emergency_stop' });
  }, [sendMessage]);

  const setModeAction = useCallback((newMode: 'MANUAL' | 'AUTONOMOUS') => {
    sendMessage({ type: 'set_mode', mode: newMode });
    setMode(newMode); // Optimistic update
  }, [sendMessage]);

  const sendManualCommand = useCallback((command: string, speed?: number) => {
    sendMessage({ type: 'manual_control', command, ...(speed !== undefined ? { speed } : {}) });
  }, [sendMessage]);

  const sendVelocity = useCallback((linear: number, angular: number) => {
    sendMessage({ type: 'manual_control', linear, angular });
  }, [sendMessage]);

  const resetMap = useCallback(() => {
    sendMessage({ type: 'reset_map' });
    trajectory.current = [];
  }, [sendMessage]);

  const resetPose = useCallback(() => {
    sendMessage({ type: 'reset_pose' });
    trajectory.current = [];
  }, [sendMessage]);

  return {
    connected,
    rtt,
    isRealConnected,
    battery,
    cpuTemp,
    linearSpeed,
    angularSpeed,
    nearestObstacle,
    mode,
    navigationStatus,
    sensors,
    events,
    cameraFrame,
    pose,
    lidarPoints,
    mapData,
    detections,
    detectionsVersion,
    trajectory,
    plannedPath,
    sendGoal,
    cancelGoal,
    emergencyStop,
    setMode: setModeAction,
    sendManualCommand,
    sendVelocity,
    resetMap,
    resetPose
  };
}
