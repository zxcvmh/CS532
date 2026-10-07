import { useState, useEffect, useRef, useCallback } from 'react';
import { useWebSocket } from './useWebSocket';
import { RobotPose, SensorHealth } from '../types/robot';
import { MapData, LidarPoint } from '../types/map';
import { Detection } from '../types/detection';
import { NavigationStatus } from '../types/navigation';
import { EventLogEntry } from '../types/events';

export function useRobotState() {
  const wsUrl = `ws://${window.location.hostname}:8000/ws`;
  const { connected, lastMessage, sendMessage } = useWebSocket(wsUrl);

  // React state for things that trigger UI updates
  const [battery, setBattery] = useState<number>(100);
  const [mode, setMode] = useState<'MANUAL' | 'AUTONOMOUS'>('MANUAL');
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
        pose.current = { x: lastMessage.x, y: lastMessage.y, theta: lastMessage.theta };
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
          data: lastMessage.data
        };
        break;
      case 'detections':
        detections.current = lastMessage.objects.map((d: any) => ({
          className: d.class_name,
          confidence: d.confidence,
          bbox: d.bbox,
          distance: d.distance
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

  const sendManualCommand = useCallback((command: string) => {
    sendMessage({ type: 'manual_control', command });
  }, [sendMessage]);

  return {
    connected,
    battery,
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
    sendManualCommand
  };
}
