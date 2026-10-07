import React, { useRef, useEffect, useState, useCallback } from 'react';
import { RobotPose } from '../types/robot';
import { MapData, LidarPoint, MapTransform } from '../types/map';
import { NavigationStatus } from '../types/navigation';
import { mapToScreen, screenToMap } from '../utils/mapTransform';
import { 
  Plus, Minus, RotateCcw, Target, X, Trash2, 
  Compass, Maximize2, Minimize2, Navigation2, Layers, Info, HelpCircle
} from 'lucide-react';
import { Theme } from '../hooks/useUiPreferences';
import { translations, Language } from '../i18n/translations';

interface MapViewerProps {
  pose: React.MutableRefObject<RobotPose>;
  mapData: React.MutableRefObject<MapData | null>;
  lidarPoints: React.MutableRefObject<LidarPoint[]>;
  trajectory: React.MutableRefObject<LidarPoint[]>;
  plannedPath: React.MutableRefObject<LidarPoint[]>;
  navigationStatus: NavigationStatus;
  mode: 'MANUAL' | 'AUTONOMOUS';
  sendGoal: (x: number, y: number) => void;
  resetMap?: () => void;
  resetPose?: () => void;
  showLidarScan?: boolean;
  showTrajectory?: boolean;
  showPlannedPath?: boolean;
  showFovCone?: boolean;
  showGrid?: boolean;
  isMaximized?: boolean;
  onToggleMaximize?: () => void;
  theme?: Theme;
  language?: Language;
}

export default function MapViewer({ 
  pose, 
  mapData, 
  lidarPoints, 
  trajectory, 
  plannedPath, 
  navigationStatus, 
  mode, 
  sendGoal, 
  resetMap, 
  resetPose,
  showLidarScan = true,
  showTrajectory = true,
  showPlannedPath = true,
  showFovCone = true,
  showGrid = true,
  isMaximized = false,
  onToggleMaximize,
  theme = 'light',
  language = 'vi'
}: MapViewerProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const requestRef = useRef<number>(0);
  
  const [transform, setTransform] = useState<MapTransform>({ scale: 45, offsetX: 0, offsetY: 0 });
  const isDragging = useRef(false);
  const dragStart = useRef({ x: 0, y: 0 });
  const lastMousePos = useRef({ x: 0, y: 0 });
  const [cursorMapPos, setCursorMapPos] = useState({ x: 0, y: 0 });
  
  // Legend state
  const [showLegend, setShowLegend] = useState(true);

  // Pending Goal marker before confirmation
  const [pendingGoal, setPendingGoal] = useState<{ x: number; y: number; screenX: number; screenY: number } | null>(null);

  // Cached off-screen canvas to avoid expensive image data rebuild every frame (Ticket TK-02)
  const offscreenCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const lastMapVersionRef = useRef(-1);
  const lastThemeRef = useRef(theme);

  const t = translations[language];
  const isDark = theme === 'dark' || (typeof document !== 'undefined' && document.documentElement.classList.contains('dark'));

  // Main 60fps render loop with Apple Light & Dark theme styling
  const renderFrame = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;
    
    // Clear canvas with distinct unexplored space color
    // Light mode: technical soft blueprint gray #e2e8f0; Dark mode: Pure obsidian black #0d0c0c
    ctx.fillStyle = isDark ? '#0d0c0c' : '#e2e8f0';
    ctx.fillRect(0, 0, width, height);

    const mData = mapData.current;
    
    // ── 1. Render Occupancy Grid Map (Off-screen Canvas Cached) ──
    if (mData && mData.data) {
      const currentVersion = mData.version ?? 0;
      const currentTheme = isDark ? 'dark' : 'light';
      const themeChanged = lastThemeRef.current !== currentTheme;
      
      if (currentVersion !== lastMapVersionRef.current || themeChanged || !offscreenCanvasRef.current) {
        if (!offscreenCanvasRef.current) {
          offscreenCanvasRef.current = document.createElement('canvas');
        }
        const offscreen = offscreenCanvasRef.current;
        if (offscreen.width !== mData.width || offscreen.height !== mData.height) {
          offscreen.width = mData.width;
          offscreen.height = mData.height;
        }
        
        const offCtx = offscreen.getContext('2d');
        if (offCtx) {
          const imgData = offCtx.createImageData(mData.width, mData.height);
          for (let i = 0; i < mData.data.length; i++) {
            const val = mData.data[i];
            const idx = i * 4;
            if (val === -1) {
              // Unknown space - transparent so canvas background (#0d0c0c) shows through
              imgData.data[idx] = 0; imgData.data[idx+1] = 0; imgData.data[idx+2] = 0; imgData.data[idx+3] = 0;
            } else if (val >= 50) {
              // Occupied Obstacle - High Contrast
              if (isDark) {
                // Crisp white chalk in dark mode
                imgData.data[idx] = 244; imgData.data[idx+1] = 244; imgData.data[idx+2] = 245; imgData.data[idx+3] = 255;
              } else {
                // Pitch dark charcoal black walls in light mode
                imgData.data[idx] = 9; imgData.data[idx+1] = 13; imgData.data[idx+2] = 22; imgData.data[idx+3] = 255;
              }
            } else {
              // Free space (Safe navigable area) - Clear high contrast against unexplored void
              // Light mode: Clean bright white; Dark mode: High-contrast technical slate #2d3442
              if (isDark) {
                imgData.data[idx] = 45; imgData.data[idx+1] = 52; imgData.data[idx+2] = 66; imgData.data[idx+3] = 255;
              } else {
                imgData.data[idx] = 255; imgData.data[idx+1] = 255; imgData.data[idx+2] = 255; imgData.data[idx+3] = 255;
              }
            }
          }
          offCtx.putImageData(imgData, 0, 0);
        }
        lastMapVersionRef.current = currentVersion;
        lastThemeRef.current = currentTheme;
      }

      if (offscreenCanvasRef.current) {
        const originScreen = mapToScreen(mData.originX, mData.originY, transform, width, height, mData);
        const drawWidth = mData.width * mData.resolution * transform.scale;
        const drawHeight = mData.height * mData.resolution * transform.scale;
        
        ctx.save();
        ctx.translate(originScreen.x, originScreen.y);
        ctx.scale(1, -1);
        ctx.drawImage(offscreenCanvasRef.current, 0, 0, drawWidth, drawHeight);
        ctx.restore();
      }
    }

    // ── 2. Coordinate Grid Lines (1 meter spacing) ──
    if (showGrid) {
      ctx.strokeStyle = isDark ? 'rgba(255, 255, 255, 0.08)' : 'rgba(0, 0, 0, 0.07)';
      ctx.lineWidth = 1;
      const gridSpacing = transform.scale;
      const centerX = width / 2 + transform.offsetX;
      const centerY = height / 2 + transform.offsetY;
      
      for (let x = centerX % gridSpacing; x < width; x += gridSpacing) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke();
      }
      for (let y = centerY % gridSpacing; y < height; y += gridSpacing) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
      }

      // Origin Axes (0, 0)
      const origin = mapToScreen(0, 0, transform, width, height, mapData.current);
      ctx.strokeStyle = isDark ? 'rgba(255, 255, 255, 0.25)' : 'rgba(0, 0, 0, 0.2)';
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(origin.x - 24, origin.y); ctx.lineTo(origin.x + 24, origin.y); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(origin.x, origin.y - 24); ctx.lineTo(origin.x, origin.y + 24); ctx.stroke();
    }

    // ── 3. Robot Odometry Trajectory ──
    if (showTrajectory) {
      const traj = trajectory.current;
      if (traj.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = isDark ? 'rgba(249, 115, 22, 0.40)' : 'rgba(79, 70, 229, 0.45)';
        ctx.lineWidth = 2.5;
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        for (let i = 0; i < traj.length; i++) {
          const p = mapToScreen(traj[i].x, traj[i].y, transform, width, height, mData);
          if (i === 0) ctx.moveTo(p.x, p.y);
          else ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
      }
    }

    // ── 4. A* Planned Path (Apple Blue / Orange Navigation Dash Line) ──
    if (showPlannedPath) {
      const path = plannedPath.current;
      if (path.length > 1) {
        // Glowing path aura
        ctx.beginPath();
        ctx.strokeStyle = isDark ? 'rgba(249, 115, 22, 0.20)' : 'rgba(0, 113, 227, 0.18)';
        ctx.lineWidth = 8;
        for (let i = 0; i < path.length; i++) {
          const p = mapToScreen(path[i].x, path[i].y, transform, width, height, mData);
          if (i === 0) ctx.moveTo(p.x, p.y);
          else ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();

        // Foreground dotted route
        ctx.beginPath();
        ctx.strokeStyle = isDark ? '#f97316' : '#0071e3';
        ctx.setLineDash([7, 6]);
        ctx.lineWidth = 3;
        for (let i = 0; i < path.length; i++) {
          const p = mapToScreen(path[i].x, path[i].y, transform, width, height, mData);
          if (i === 0) ctx.moveTo(p.x, p.y);
          else ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }

    // ── 5. LiDAR 360 Scan Points (Apple Emerald Dots) ──
    if (showLidarScan) {
      const pts = lidarPoints.current;
      ctx.fillStyle = isDark ? '#4ade80' : '#10b981';
      for (let i = 0; i < pts.length; i++) {
        const p = mapToScreen(pts[i].x, pts[i].y, transform, width, height, mData);
        ctx.fillRect(p.x - 1.25, p.y - 1.25, 2.5, 2.5);
      }
    }

    // ── 6. Active Navigation Goal Marker (Apple Maps Pulse Pin) ──
    const navGoalX = navigationStatus.goalX;
    const navGoalY = navigationStatus.goalY;
    if (navGoalX !== null && navGoalY !== null && 
        ['NAVIGATING', 'PLANNING'].includes(navigationStatus.status)) {
      const gPos = mapToScreen(navGoalX, navGoalY, transform, width, height, mData);
      
      // Animated Expanding Pulse Ring
      const pulse = (Date.now() % 1600) / 1600;
      const radius = 10 + pulse * 22;
      const alpha = 1.0 - pulse;
      
      ctx.beginPath();
      ctx.arc(gPos.x, gPos.y, radius, 0, Math.PI * 2);
      ctx.strokeStyle = isDark ? `rgba(249, 115, 22, ${alpha})` : `rgba(0, 113, 227, ${alpha})`;
      ctx.lineWidth = 2.5;
      ctx.stroke();

      // Pin center dot
      ctx.beginPath();
      ctx.arc(gPos.x, gPos.y, 7, 0, Math.PI * 2);
      ctx.fillStyle = isDark ? '#f97316' : '#0071e3';
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2.5;
      ctx.stroke();

      // Goal Label Pill (Generous padding & clear text)
      ctx.font = '600 12px -apple-system, BlinkMacSystemFont, "SF Pro Text", sans-serif';
      const label = `Đích: (${navGoalX.toFixed(2)}, ${navGoalY.toFixed(2)}m)`;
      const textWidth = ctx.measureText(label).width;
      
      const pillWidth = textWidth + 24;
      const pillHeight = 26;
      const pillX = gPos.x - pillWidth / 2;
      const pillY = gPos.y - 38;

      ctx.fillStyle = isDark ? '#141414' : '#ffffff';
      ctx.beginPath();
      ctx.roundRect ? ctx.roundRect(pillX, pillY, pillWidth, pillHeight, 8) : ctx.rect(pillX, pillY, pillWidth, pillHeight);
      ctx.fill();
      ctx.strokeStyle = isDark ? '#262626' : 'rgba(0, 113, 227, 0.4)';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      ctx.fillStyle = isDark ? '#f4f4f5' : '#0071e3';
      ctx.fillText(label, pillX + 12, pillY + 18);
    }

    // ── 7. Pending Goal Marker (When user clicks on map) ──
    if (pendingGoal) {
      const pgPos = mapToScreen(pendingGoal.x, pendingGoal.y, transform, width, height, mData);
      ctx.beginPath();
      ctx.arc(pgPos.x, pgPos.y, 10, 0, Math.PI * 2);
      ctx.strokeStyle = isDark ? '#f97316' : '#0071e3';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
      
      ctx.beginPath();
      ctx.arc(pgPos.x, pgPos.y, 5, 0, Math.PI * 2);
      ctx.fillStyle = isDark ? '#f97316' : '#0071e3';
      ctx.fill();
    }

    // ── 8. Robot Vehicle & Orientation (Apple Minimal Hardware Style) ──
    const rPose = pose.current;
    const rPos = mapToScreen(rPose.x, rPose.y, transform, width, height, mData);
    
    ctx.save();
    ctx.translate(rPos.x, rPos.y);
    ctx.rotate(-rPose.theta); // Invert Y in canvas

    // 8a. Camera FOV Cone (60 degrees)
    if (showFovCone) {
      const fovLength = Math.max(55, 1.8 * transform.scale);
      const halfFov = (30 * Math.PI) / 180;
      
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(fovLength * Math.cos(-halfFov), fovLength * Math.sin(-halfFov));
      ctx.arc(0, 0, fovLength, -halfFov, halfFov);
      ctx.closePath();
      
      const fovGrad = ctx.createRadialGradient(0, 0, 5, 0, 0, fovLength);
      fovGrad.addColorStop(0, 'rgba(0, 113, 227, 0.22)');
      fovGrad.addColorStop(0.7, 'rgba(0, 113, 227, 0.05)');
      fovGrad.addColorStop(1, 'rgba(0, 113, 227, 0.0)');
      ctx.fillStyle = fovGrad;
      ctx.fill();
      ctx.strokeStyle = 'rgba(0, 113, 227, 0.35)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // 8b. JetBot Chassis (Rounded Squircle & Track Treads)
    // Left & Right treads
    ctx.fillStyle = '#334155';
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    // Left tread
    ctx.beginPath();
    ctx.roundRect ? ctx.roundRect(-16, -15, 32, 7, 3) : ctx.rect(-16, -15, 32, 7);
    ctx.fill(); ctx.stroke();
    // Right tread
    ctx.beginPath();
    ctx.roundRect ? ctx.roundRect(-16, 8, 32, 7, 3) : ctx.rect(-16, 8, 32, 7);
    ctx.fill(); ctx.stroke();

    // Main Chassis Body (Aluminium Silver)
    ctx.fillStyle = '#e2e8f0';
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect ? ctx.roundRect(-14, -11, 28, 22, 6) : ctx.rect(-14, -11, 28, 22);
    ctx.fill();
    ctx.stroke();

    // LiDAR D500 Sensor Puck (Centered Blue Turret)
    ctx.fillStyle = '#0071e3';
    ctx.beginPath();
    ctx.arc(0, 0, 6, 0, Math.PI * 2);
    ctx.fill();

    // Direction Heading Triangle (Front Arrow)
    ctx.fillStyle = '#0071e3';
    ctx.beginPath();
    ctx.moveTo(17, 0);
    ctx.lineTo(9, -5);
    ctx.lineTo(9, 5);
    ctx.closePath();
    ctx.fill();

    ctx.restore();

    requestRef.current = requestAnimationFrame(renderFrame);
  }, [transform, navigationStatus, pendingGoal, showLidarScan, showTrajectory, showPlannedPath, showFovCone, showGrid, pose, mapData, lidarPoints, trajectory, plannedPath]);

  // Start 60fps render loop
  useEffect(() => {
    requestRef.current = requestAnimationFrame(renderFrame);
    return () => cancelAnimationFrame(requestRef.current);
  }, [renderFrame]);

  // Canvas resize listener
  useEffect(() => {
    const handleResize = () => {
      const container = containerRef.current;
      const canvas = canvasRef.current;
      if (!container || !canvas) return;
      canvas.width = container.clientWidth;
      canvas.height = container.clientHeight;
    };
    handleResize();
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  // Mouse pan/zoom and click-to-goal interactions
  const handleMouseDown = (e: React.MouseEvent) => {
    if (e.button === 0) { // Left click
      isDragging.current = true;
      dragStart.current = { x: e.clientX - transform.offsetX, y: e.clientY - transform.offsetY };
      lastMousePos.current = { x: e.clientX, y: e.clientY };
    }
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;

    // Track cursor metric coordinates
    const mapPos = screenToMap(mouseX, mouseY, transform, canvas.width, canvas.height, mapData.current);
    setCursorMapPos({ x: Math.round(mapPos.x * 100) / 100, y: Math.round(mapPos.y * 100) / 100 });

    if (isDragging.current) {
      setTransform(prev => ({
        ...prev,
        offsetX: e.clientX - dragStart.current.x,
        offsetY: e.clientY - dragStart.current.y
      }));
    }
  };

  const handleMouseUp = (e: React.MouseEvent) => {
    if (isDragging.current) {
      const dist = Math.hypot(e.clientX - lastMousePos.current.x, e.clientY - lastMousePos.current.y);
      isDragging.current = false;
      
      // If was just a short click without drag (< 4px), set goal!
      if (dist < 4 && canvasRef.current) {
        const rect = canvasRef.current.getBoundingClientRect();
        const clickX = e.clientX - rect.left;
        const clickY = e.clientY - rect.top;
        const target = screenToMap(clickX, clickY, transform, canvasRef.current.width, canvasRef.current.height, mapData.current);
        
        setPendingGoal({
          x: Math.round(target.x * 100) / 100,
          y: Math.round(target.y * 100) / 100,
          screenX: clickX,
          screenY: clickY
        });
      }
    }
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
    setTransform(prev => ({
      ...prev,
      scale: Math.max(10, Math.min(250, prev.scale * zoomFactor))
    }));
  };

  // Center view on robot
  const centerRobot = () => {
    setTransform(prev => ({
      ...prev,
      offsetX: -pose.current.x * prev.scale,
      offsetY: pose.current.y * prev.scale
    }));
  };

  // Zoom helpers
  const zoomIn = () => setTransform(prev => ({ ...prev, scale: Math.min(250, prev.scale * 1.25) }));
  const zoomOut = () => setTransform(prev => ({ ...prev, scale: Math.max(10, prev.scale * 0.8) }));
  const resetView = () => setTransform({ scale: 45, offsetX: 0, offsetY: 0 });

  // Confirm target goal
  const confirmGoal = () => {
    if (pendingGoal) {
      sendGoal(pendingGoal.x, pendingGoal.y);
      setPendingGoal(null);
    }
  };

  return (
    <div 
      ref={containerRef}
      className="relative w-full h-full flex flex-col bg-white dark:bg-[#141414] overflow-hidden rounded-2xl border border-gray-200/80 dark:border-[#262626] shadow-[0_2px_12px_rgba(0,0,0,0.04)] dark:shadow-[0_4px_20px_rgba(0,0,0,0.6)] select-none group"
    >
      {/* Map Header Bar */}
      <div className="absolute top-0 inset-x-0 h-12 px-4 z-30 flex items-center justify-between bg-white/85 dark:bg-[#141414]/90 backdrop-blur-md border-b border-gray-100 dark:border-[#262626]">
        <div className="flex items-center gap-2.5">
          <div className="flex items-center gap-2 px-3 py-1 rounded-full bg-blue-50 dark:bg-[#1c1c1c] text-[#0071e3] dark:text-[#f4f4f5] border border-blue-200/60 dark:border-[#262626] text-xs font-semibold shadow-xs">
            <Compass className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#f97316]" />
            <span>{t.mapOccupancyGrid}</span>
          </div>

          <div className="hidden sm:flex items-center gap-1.5 px-3 py-1 rounded-full bg-gray-50 dark:bg-[#1c1c1c] border border-gray-200/60 dark:border-[#262626] text-xs font-mono text-[#6e6e73] dark:text-[#9ca3af]">
            <span>{t.cursor}:</span>
            <span className="text-[#1d1d1f] dark:text-[#f4f4f5] font-semibold">{cursorMapPos.x.toFixed(2)}, {cursorMapPos.y.toFixed(2)}m</span>
          </div>

          {/* Toggle Legend Button */}
          <button
            onClick={() => setShowLegend(prev => !prev)}
            className={`flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs font-semibold border transition-all apple-btn shadow-xs ${
              showLegend
                ? 'bg-[#0071e3]/10 dark:bg-[#262626] text-[#0071e3] dark:text-[#f4f4f5] border-[#0071e3]/20 dark:border-[#333333]'
                : 'bg-gray-50 dark:bg-[#1c1c1c] text-[#6e6e73] dark:text-[#9ca3af] border-gray-200/60 dark:border-[#262626] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5]'
            }`}
          >
            <Info className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#f97316]" />
            <span className="hidden md:inline">{t.legendTitle}</span>
          </button>
        </div>

        {/* Maximize Toggle */}
        <div className="flex items-center gap-2">
          {onToggleMaximize && (
            <button
              onClick={onToggleMaximize}
              title={isMaximized ? "Thu nhỏ về Cockpit" : "Phóng to bản đồ"}
              className="w-8 h-8 rounded-xl bg-gray-50 dark:bg-[#1c1c1c] hover:bg-gray-150 dark:hover:bg-[#262626] border border-gray-200 dark:border-[#262626] flex items-center justify-center text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5] transition-all apple-btn"
            >
              {isMaximized ? <Minimize2 className="w-4 h-4" /> : <Maximize2 className="w-4 h-4" />}
            </button>
          )}
        </div>
      </div>

      {/* Main Canvas Area */}
      <canvas
        ref={canvasRef}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onWheel={handleWheel}
        className="w-full h-full cursor-crosshair"
      />

      {/* Map Legend Overlay Card */}
      {showLegend && (
        <div className="absolute top-14 left-4 z-20 w-64 p-3 rounded-2xl bg-white/95 dark:bg-[#141414]/95 backdrop-blur-xl border border-gray-200/90 dark:border-[#262626] shadow-[0_8px_30px_rgba(0,0,0,0.6)] flex flex-col gap-2 animate-fadeIn text-xs">
          <div className="flex items-center justify-between pb-1.5 border-b border-gray-100 dark:border-[#262626]">
            <span className="font-bold text-[#1d1d1f] dark:text-[#f4f4f5] flex items-center gap-1.5 text-[11px] uppercase tracking-wider">
              <Info className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#f97316]" />
              {t.legendTitle}
            </span>
            <button
              onClick={() => setShowLegend(false)}
              className="w-5 h-5 rounded-full hover:bg-gray-100 dark:hover:bg-[#262626] flex items-center justify-center text-gray-400 dark:text-[#9ca3af] hover:text-gray-700 dark:hover:text-white"
            >
              <X className="w-3 h-3" />
            </button>
          </div>

          <div className="flex flex-col gap-1.5 text-[11px] text-[#6e6e73] dark:text-[#9ca3af]">
            {/* Free space */}
            <div className="flex items-start gap-2">
              <span className="w-3.5 h-3.5 rounded-sm bg-white dark:bg-[#2d3442] border border-gray-400 dark:border-[#475569] shadow-xs shrink-0 mt-0.5"></span>
              <div className="flex flex-col">
                <strong className="text-[#1d1d1f] dark:text-[#f4f4f5]">{t.freeSpace}</strong>
                <span className="text-[10px] text-gray-500 dark:text-[#9ca3af]">{t.freeSpaceDesc}</span>
              </div>
            </div>

            {/* Obstacles / Walls */}
            <div className="flex items-start gap-2">
              <span className="w-3.5 h-3.5 rounded-sm bg-[#090d16] dark:bg-[#f4f4f5] border border-gray-800 dark:border-[#f4f4f5] shadow-xs shrink-0 mt-0.5"></span>
              <div className="flex flex-col">
                <strong className="text-[#1d1d1f] dark:text-[#f4f4f5]">{t.obstacle}</strong>
                <span className="text-[10px] text-gray-500 dark:text-[#9ca3af]">{t.obstacleDesc}</span>
              </div>
            </div>

            {/* Unknown */}
            <div className="flex items-start gap-2">
              <span className="w-3.5 h-3.5 rounded-sm bg-[#e2e8f0] dark:bg-[#0d0c0c] border border-gray-300 dark:border-[#262626] shrink-0 mt-0.5"></span>
              <div className="flex flex-col">
                <strong className="text-[#1d1d1f] dark:text-[#f4f4f5]">{t.unknown}</strong>
                <span className="text-[10px] text-gray-500 dark:text-[#9ca3af]">{t.unknownDesc}</span>
              </div>
            </div>

            {/* LiDAR Points */}
            <div className="flex items-start gap-2">
              <span className="w-3.5 h-3.5 rounded-full bg-[#10b981] dark:bg-[#4ade80] shadow-xs shrink-0 mt-0.5"></span>
              <div className="flex flex-col">
                <strong className="text-[#1d1d1f] dark:text-[#f4f4f5]">{t.lidarRays}</strong>
                <span className="text-[10px] text-gray-500 dark:text-[#9ca3af]">{t.lidarRaysDesc}</span>
              </div>
            </div>

            {/* Planned Path */}
            <div className="flex items-start gap-2">
              <span className="w-3.5 h-1.5 rounded-full bg-[#0071e3] dark:bg-[#f97316] shadow-xs shrink-0 mt-1"></span>
              <div className="flex flex-col">
                <strong className="text-[#1d1d1f] dark:text-[#f4f4f5]">{t.plannedRoute}</strong>
                <span className="text-[10px] text-gray-500 dark:text-[#9ca3af]">{t.plannedRouteDesc}</span>
              </div>
            </div>

            {/* Robot */}
            <div className="flex items-start gap-2">
              <span className="w-3.5 h-3.5 rounded-sm bg-blue-100 dark:bg-[#1c1c1c] border border-[#0071e3] dark:border-[#262626] shrink-0 mt-0.5 flex items-center justify-center text-[8px] font-bold text-[#0071e3] dark:text-[#f97316]">▲</span>
              <div className="flex flex-col">
                <strong className="text-[#1d1d1f] dark:text-[#f4f4f5]">{t.robotHeading}</strong>
                <span className="text-[10px] text-gray-500 dark:text-[#9ca3af]">{t.robotHeadingDesc}</span>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Floating Apple Maps Style Toolbar */}
      <div className="absolute top-16 right-4 z-20 flex flex-col gap-1.5 p-1.5 rounded-2xl bg-white/90 dark:bg-[#141414]/90 backdrop-blur-xl border border-gray-200 dark:border-[#262626] shadow-lg">
        <button
          onClick={zoomIn}
          title="Phóng to (+)"
          className="w-8 h-8 rounded-xl bg-gray-50 dark:bg-[#1c1c1c] hover:bg-gray-100 dark:hover:bg-[#262626] flex items-center justify-center text-[#1d1d1f] dark:text-[#f4f4f5] transition-colors apple-btn"
        >
          <Plus className="w-4 h-4" />
        </button>
        <button
          onClick={zoomOut}
          title="Thu nhỏ (-)"
          className="w-8 h-8 rounded-xl bg-gray-50 dark:bg-[#1c1c1c] hover:bg-gray-100 dark:hover:bg-[#262626] flex items-center justify-center text-[#1d1d1f] dark:text-[#f4f4f5] transition-colors apple-btn"
        >
          <Minus className="w-4 h-4" />
        </button>
        <div className="w-full h-[1px] bg-gray-200 dark:bg-[#262626] my-0.5"></div>
        <button
          onClick={centerRobot}
          title="Căn giữa Robot"
          className="w-8 h-8 rounded-xl bg-blue-50 dark:bg-[#1c1c1c] hover:bg-blue-100 dark:hover:bg-[#262626] flex items-center justify-center text-[#0071e3] dark:text-[#f97316] transition-colors apple-btn"
        >
          <Navigation2 className="w-4 h-4" />
        </button>
        <button
          onClick={resetView}
          title="Khôi phục góc nhìn"
          className="w-8 h-8 rounded-xl bg-gray-50 dark:bg-[#1c1c1c] hover:bg-gray-100 dark:hover:bg-[#262626] flex items-center justify-center text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5] transition-colors apple-btn"
        >
          <RotateCcw className="w-4 h-4" />
        </button>
      </div>

      {/* Target Confirmation Pill (When user clicks map to set goal) */}
      {pendingGoal && (
        <div className="absolute bottom-5 left-1/2 -translate-x-1/2 z-30 flex items-center gap-3 p-2 px-4 rounded-full bg-white/95 dark:bg-[#141414]/95 backdrop-blur-2xl border border-[#0071e3]/30 dark:border-[#262626] shadow-[0_12px_36px_rgba(0,0,0,0.5)] animate-scaleUp">
          <div className="w-2.5 h-2.5 rounded-full bg-[#0071e3] dark:bg-[#f97316] animate-ping"></div>
          <span className="text-xs text-[#1d1d1f] dark:text-[#f4f4f5] font-medium">
            Điểm đích: <strong className="font-mono text-[#0071e3] dark:text-[#f97316] font-bold">{pendingGoal.x}, {pendingGoal.y}m</strong>
          </span>
          <button
            onClick={confirmGoal}
            className="px-4 py-1.5 rounded-full bg-[#0071e3] dark:bg-[#f97316] hover:bg-[#0077ed] dark:hover:bg-[#ea580c] text-white dark:text-[#ffffff] font-semibold text-xs shadow-sm transition-all apple-btn"
          >
            {t.confirmGoal}
          </button>
          <button
            onClick={() => setPendingGoal(null)}
            className="w-6 h-6 rounded-full bg-gray-100 dark:bg-[#1c1c1c] hover:bg-gray-200 dark:hover:bg-[#262626] flex items-center justify-center text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-white transition-colors"
          >
            <X className="w-3.5 h-3.5" />
          </button>
        </div>
      )}

      {/* Bottom Scale Indicator */}
      <div className="absolute bottom-3 right-4 z-20 pointer-events-none">
        <div className="flex items-center gap-2 px-3 py-1 rounded-full bg-white/90 dark:bg-[#141414]/90 backdrop-blur-md border border-gray-200/80 dark:border-[#262626] text-xs font-mono text-[#6e6e73] dark:text-[#9ca3af] shadow-xs">
          <div className="w-10 h-[2px] bg-[#0071e3] dark:bg-[#f97316]"></div>
          <span>1.0m ({transform.scale}px)</span>
        </div>
      </div>
    </div>
  );
}
