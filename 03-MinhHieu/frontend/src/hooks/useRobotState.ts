import { useState, useEffect, useRef, useCallback } from 'react';
import { useWebSocket } from './useWebSocket';
import { RobotPose, SensorHealth } from '../types/robot';
import { MapData, LidarPoint } from '../types/map';
import { Detection } from '../types/detection';
import { NavigationStatus, MotionXaiInfo } from '../types/navigation';
import { EventLogEntry } from '../types/events';

const defaultXaiInfo: MotionXaiInfo = {
  status: 'IDLE',
  speed_scale: 1.0,
  curvature_w: 0.0,
  evasion_active: false,
  evasion_reason: '',
  lookahead_target: null,
  clusters: [],
  low_obstacles: [],
  social_bubbles: []
};

export function useRobotState() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  // If running on Vite dev server (port 5173), connect to 8000; otherwise connect to same origin host
  const wsHost = window.location.port === '5173'
    ? (window.location.hostname === 'localhost' ? '127.0.0.1:8000' : `${window.location.hostname}:8000`)
    : (window.location.host || '127.0.0.1:8000');
  const wsUrl = `${protocol}//${wsHost}/ws`;
  const { connected, lastMessage, lastBinaryMessage, sendMessage, rtt } = useWebSocket(wsUrl);

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
  const [motionXaiState, setMotionXaiState] = useState<MotionXaiInfo>(defaultXaiInfo);
  
  // Refs for high-frequency data (avoids re-rendering the whole app)
  const pose = useRef<RobotPose>({ x: 0, y: 0, theta: 0 });
  const lidarPoints = useRef<LidarPoint[]>([]);
  const rangesMm = useRef<Uint16Array | null>(null);
  const mapData = useRef<MapData | null>(null);
  const detections = useRef<Detection[]>([]);
  const trajectory = useRef<LidarPoint[]>([]);
  const plannedPath = useRef<LidarPoint[]>([]);
  const motionXai = useRef<MotionXaiInfo>(defaultXaiInfo);

  // Trigger for components that need to know detections changed (for CameraView)
  const [detectionsVersion, setDetectionsVersion] = useState(0);

  // ─── ZERO-COPY ULTRA-FAST BINARY TELEMETRY UNPACKER (0.002ms) ───
  useEffect(() => {
    if (!lastBinaryMessage || lastBinaryMessage.byteLength < 1130) return;
    const view = new DataView(lastBinaryMessage);
    // Check Magic Header 'JB' (0x4A, 0x42)
    if (view.getUint8(0) !== 0x4A || view.getUint8(1) !== 0x42) return;

    const x = view.getFloat32(18, true);
    const y = view.getFloat32(22, true);
    const theta = view.getFloat32(26, true);
    const v = view.getFloat32(30, true);
    const w = view.getFloat32(34, true);
    const nearObs = view.getFloat32(38, true);
    const bat = view.getFloat32(42, true);
    const cpu = view.getFloat32(46, true);

    // Update high-rate pose ref immediately
    pose.current = {
      x,
      y,
      theta,
      linearSpeed: v,
      angularSpeed: w,
      nearestObstacle: nearObs > 0.001 ? nearObs : null,
      cpuTemp: cpu
    };

    // Zero-copy Uint16Array view directly into received ArrayBuffer
    const rMm = new Uint16Array(lastBinaryMessage, 50, 360);
    rangesMm.current = rMm;

    // Fast coordinate projection for MapViewer canvas
    const pts: LidarPoint[] = [];
    const cosTh = Math.cos(theta);
    const sinTh = Math.sin(theta);
    const lx = x + 0.08 * cosTh;
    const ly = y + 0.08 * sinTh;
    const deg2rad = Math.PI / 180.0;
    for (let i = 0; i < 360; i++) {
      const dMm = rMm[i];
      if (dMm >= 50 && dMm <= 12000) {
        const dM = dMm / 1000.0;
        const rad = i * deg2rad + theta;
        pts.push({ x: lx + dM * Math.cos(rad), y: ly + dM * Math.sin(rad) });
      }
    }
    lidarPoints.current = pts;

    // Delta-checked state updates
    setLinearSpeed(prev => Math.abs(prev - v) > 0.005 ? v : prev);
    setAngularSpeed(prev => Math.abs(prev - w) > 0.01 ? w : prev);
    if (nearObs > 0.001) {
      setNearestObstacle(prev => prev === null || Math.abs(prev - nearObs) > 0.02 ? nearObs : prev);
    }
    if (bat > 0) {
      setBattery(prev => Math.abs(prev - bat) > 0.5 ? bat : prev);
    }
    setCpuTemp(prev => Math.abs(prev - cpu) > 0.5 ? cpu : prev);
  }, [lastBinaryMessage]);
  
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
        if (lastMessage.linear_speed !== undefined) {
          setLinearSpeed(prev => Math.abs(prev - lastMessage.linear_speed) > 0.005 ? lastMessage.linear_speed : prev);
        }
        if (lastMessage.angular_speed !== undefined) {
          setAngularSpeed(prev => Math.abs(prev - lastMessage.angular_speed) > 0.01 ? lastMessage.angular_speed : prev);
        }
        if (lastMessage.nearest_obstacle !== undefined) {
          setNearestObstacle(prev => {
            const next = lastMessage.nearest_obstacle;
            if (prev === null && next === null) return prev;
            if (prev === null || next === null) return next;
            return Math.abs(prev - next) > 0.02 ? next : prev;
          });
        }
        if (lastMessage.cpu_temp !== undefined) {
          setCpuTemp(prev => Math.abs(prev - lastMessage.cpu_temp) > 0.5 ? lastMessage.cpu_temp : prev);
        }
        break;
      case 'battery':
        setBattery(lastMessage.percentage);
        break;
      case 'lidar_scan':
        if (Array.isArray(lastMessage.points) && lastMessage.points.length > 0) {
          lidarPoints.current = lastMessage.points;
        }
        break;
      case 'map_update': {
        let gridData = lastMessage.data;
        if (lastMessage.data_b64) {
          try {
            const binaryStr = atob(lastMessage.data_b64);
            const len = binaryStr.length;
            const u8 = new Uint8Array(len);
            for (let i = 0; i < len; i++) {
              u8[i] = binaryStr.charCodeAt(i);
            }
            gridData = new Int8Array(u8.buffer);
          } catch (e) {
            console.error('Failed to decode base64 map data', e);
          }
        }
        mapData.current = {
          resolution: lastMessage.resolution,
          width: lastMessage.width,
          height: lastMessage.height,
          originX: lastMessage.origin_x,
          originY: lastMessage.origin_y,
          data: gridData,
          version: (mapData.current?.version ?? 0) + 1
        };
        break;
      }
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
      case 'motion_xai': {
        const nextXai: MotionXaiInfo = {
          status: lastMessage.status || 'IDLE',
          speed_scale: lastMessage.speed_scale ?? 1.0,
          curvature_w: lastMessage.curvature_w ?? 0.0,
          evasion_active: !!lastMessage.evasion_active,
          evasion_reason: lastMessage.evasion_reason || '',
          lookahead_target: lastMessage.lookahead_target || null,
          clusters: lastMessage.clusters || [],
          low_obstacles: lastMessage.low_obstacles || [],
          social_bubbles: lastMessage.social_bubbles || []
        };
        motionXai.current = nextXai;
        setMotionXaiState(nextXai);
        break;
      }
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
    motionXai,
    motionXaiState,
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
