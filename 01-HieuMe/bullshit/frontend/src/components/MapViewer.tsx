import React, { useRef, useEffect, useState, useCallback } from 'react';
import { RobotPose } from '../types/robot';
import { MapData, LidarPoint, MapTransform } from '../types/map';
import { NavigationStatus } from '../types/navigation';
import { mapToScreen, screenToMap } from '../utils/mapTransform';
import GoalMarker from './GoalMarker';
import { Plus, Minus, RotateCcw, Target, X } from 'lucide-react';

interface MapViewerProps {
  pose: React.MutableRefObject<RobotPose>;
  mapData: React.MutableRefObject<MapData | null>;
  lidarPoints: React.MutableRefObject<LidarPoint[]>;
  trajectory: React.MutableRefObject<LidarPoint[]>;
  plannedPath: React.MutableRefObject<LidarPoint[]>;
  navigationStatus: NavigationStatus;
  mode: 'MANUAL' | 'AUTONOMOUS';
  sendGoal: (x: number, y: number) => void;
}

export default function MapViewer({ pose, mapData, lidarPoints, trajectory, plannedPath, navigationStatus, mode, sendGoal }: MapViewerProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const requestRef = useRef<number>(0);
  
  const [transform, setTransform] = useState<MapTransform>({ scale: 40, offsetX: 0, offsetY: 0 });
  const isDragging = useRef(false);
  const dragStart = useRef({ x: 0, y: 0 });
  const lastMousePos = useRef({ x: 0, y: 0 });
  const [cursorMapPos, setCursorMapPos] = useState({ x: 0, y: 0 });
  
  // Goal setting
  const [pendingGoal, setPendingGoal] = useState<{x: number, y: number, screenX: number, screenY: number} | null>(null);

  // Cached map ImageData to avoid redrawing every frame
  const mapImageRef = useRef<ImageData | null>(null);
  const mapVersionRef = useRef(0);
  const lastMapVersionRef = useRef(-1);

  const renderFrame = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;
    
    // Clear background
    ctx.fillStyle = '#0a0a1a';
    ctx.fillRect(0, 0, width, height);

    // Draw grid lines
    ctx.strokeStyle = 'rgba(50, 50, 70, 0.3)';
    ctx.lineWidth = 0.5;
    const gridSpacing = transform.scale; // 1m grid
    const centerX = width / 2 + transform.offsetX;
    const centerY = height / 2 + transform.offsetY;
    
    for (let x = centerX % gridSpacing; x < width; x += gridSpacing) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke();
    }
    for (let y = centerY % gridSpacing; y < height; y += gridSpacing) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
    }

    const mData = mapData.current;
    
    // 1. Render occupancy grid map
    if (mData && mData.data) {
      // Check if map has changed by comparing a hash-like value
      const currentVersion = mData.data.length > 0 ? 
        mData.data[0] + mData.data[Math.floor(mData.data.length / 2)] + mData.data[mData.data.length - 1] + mData.data.length : 0;
      
      if (currentVersion !== lastMapVersionRef.current || !mapImageRef.current) {
        // Rebuild cached image
        const imgData = new ImageData(mData.width, mData.height);
        for (let i = 0; i < mData.data.length; i++) {
          const val = mData.data[i];
          const idx = i * 4;
          if (val === -1) {
            // Unknown - transparent
            imgData.data[idx] = 0; imgData.data[idx+1] = 0; imgData.data[idx+2] = 0; imgData.data[idx+3] = 0;
          } else if (val >= 50) {
            // Occupied - bright green
            imgData.data[idx] = 0; imgData.data[idx+1] = 255; imgData.data[idx+2] = 100; imgData.data[idx+3] = 220;
          } else {
            // Free - dark blue
            imgData.data[idx] = 15; imgData.data[idx+1] = 20; imgData.data[idx+2] = 35; imgData.data[idx+3] = 180;
          }
        }
        mapImageRef.current = imgData;
        lastMapVersionRef.current = currentVersion;
      }

      if (mapImageRef.current) {
        // Draw map via offscreen canvas
        const offscreen = document.createElement('canvas');
        offscreen.width = mData.width;
        offscreen.height = mData.height;
        const offCtx = offscreen.getContext('2d')!;
        offCtx.putImageData(mapImageRef.current, 0, 0);
        
        // Calculate where to draw on main canvas
        const originScreen = mapToScreen(mData.originX, mData.originY, transform, width, height, mData);
        const drawWidth = mData.width * mData.resolution * transform.scale;
        const drawHeight = mData.height * mData.resolution * transform.scale;
        
        ctx.save();
        ctx.translate(originScreen.x, originScreen.y);
        ctx.scale(1, -1);
        ctx.drawImage(offscreen, 0, 0, drawWidth, drawHeight);
        ctx.restore();
      }
    }

    // 2. Robot trajectory - cyan/teal polyline
    const traj = trajectory.current;
    if (traj.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = '#0088aa';
      ctx.lineWidth = 2;
      ctx.globalAlpha = 0.6;
      for (let i = 0; i < traj.length; i++) {
        const p = mapToScreen(traj[i].x, traj[i].y, transform, width, height, mData);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
      ctx.globalAlpha = 1.0;
    }

    // 3. Planned path - yellow dashed line
    const path = plannedPath.current;
    if (path.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = '#ffcc00';
      ctx.setLineDash([6, 4]);
      ctx.lineWidth = 2;
      for (let i = 0; i < path.length; i++) {
        const p = mapToScreen(path[i].x, path[i].y, transform, width, height, mData);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 4. LiDAR points - small bright dots
    const pts = lidarPoints.current;
    ctx.fillStyle = '#00ffaa';
    for (let i = 0; i < pts.length; i++) {
      const p = mapToScreen(pts[i].x, pts[i].y, transform, width, height, mData);
      ctx.fillRect(p.x - 1, p.y - 1, 2, 2);
    }

    // 5. Goal marker
    const navGoalX = navigationStatus.goalX;
    const navGoalY = navigationStatus.goalY;
    if (navGoalX !== null && navGoalY !== null && 
        ['NAVIGATING', 'PLANNING'].includes(navigationStatus.status)) {
      const gPos = mapToScreen(navGoalX, navGoalY, transform, width, height, mData);
      
      // Outer ring (animated)
      const pulseSize = 12 + 3 * Math.sin(Date.now() * 0.005);
      ctx.beginPath();
      ctx.arc(gPos.x, gPos.y, pulseSize, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(255, 100, 50, 0.5)';
      ctx.lineWidth = 2;
      ctx.stroke();
      
      // Inner dot
      ctx.beginPath();
      ctx.arc(gPos.x, gPos.y, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#ff6432';
      ctx.fill();
      
      // Crosshair
      ctx.strokeStyle = '#ff6432';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(gPos.x - 15, gPos.y); ctx.lineTo(gPos.x - 7, gPos.y);
      ctx.moveTo(gPos.x + 7, gPos.y); ctx.lineTo(gPos.x + 15, gPos.y);
      ctx.moveTo(gPos.x, gPos.y - 15); ctx.lineTo(gPos.x, gPos.y - 7);
      ctx.moveTo(gPos.x, gPos.y + 7); ctx.lineTo(gPos.x, gPos.y + 15);
      ctx.stroke();
    }

    // 6. Pending goal marker
    if (pendingGoal) {
      const pgPos = mapToScreen(pendingGoal.x, pendingGoal.y, transform, width, height, mData);
      ctx.beginPath();
      ctx.arc(pgPos.x, pgPos.y, 8, 0, Math.PI * 2);
      ctx.strokeStyle = '#00d4ff';
      ctx.lineWidth = 2;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
      
      ctx.beginPath();
      ctx.arc(pgPos.x, pgPos.y, 3, 0, Math.PI * 2);
      ctx.fillStyle = '#00d4ff';
      ctx.fill();
    }

    // 7. Robot - triangle with direction
    const rPose = pose.current;
    const rPos = mapToScreen(rPose.x, rPose.y, transform, width, height, mData);
    
    ctx.save();
    ctx.translate(rPos.x, rPos.y);
    ctx.rotate(-rPose.theta); // Y is inverted in canvas
    
    // Glow effect
    ctx.shadowColor = '#00d4ff';
    ctx.shadowBlur = 10;
    
    // Draw triangle
    ctx.beginPath();
    ctx.moveTo(12, 0);
    ctx.lineTo(-8, 7);
    ctx.lineTo(-5, 0);
    ctx.lineTo(-8, -7);
    ctx.closePath();
    ctx.fillStyle = '#00d4ff';
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Direction line
    ctx.beginPath();
    ctx.moveTo(12, 0);
    ctx.lineTo(25, 0);
    ctx.strokeStyle = 'rgba(0, 212, 255, 0.4)';
    ctx.lineWidth = 2;
    ctx.stroke();
    
    ctx.restore();

    // 8. Label
    ctx.fillStyle = 'rgba(0, 0, 0, 0.6)';
    ctx.fillRect(5, 5, 110, 22);
    ctx.fillStyle = '#00d4ff';
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.fillText('● 2D SLAM MAP', 10, 18);

    requestRef.current = requestAnimationFrame(renderFrame);
  }, [transform, mapData, trajectory, plannedPath, lidarPoints, pose, navigationStatus, pendingGoal]);

  useEffect(() => {
    requestRef.current = requestAnimationFrame(renderFrame);
    return () => {
      if (requestRef.current) cancelAnimationFrame(requestRef.current);
    };
  }, [renderFrame]);

  // Handle Resize
  useEffect(() => {
    const handleResize = () => {
      if (containerRef.current && canvasRef.current) {
        canvasRef.current.width = containerRef.current.clientWidth;
        canvasRef.current.height = containerRef.current.clientHeight;
      }
    };
    handleResize();
    const observer = new ResizeObserver(handleResize);
    if (containerRef.current) observer.observe(containerRef.current);
    return () => observer.disconnect();
  }, []);

  // Event Handlers
  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const zoomFactor = 1.1;
    setTransform(prev => {
      let newScale = e.deltaY < 0 ? prev.scale * zoomFactor : prev.scale / zoomFactor;
      newScale = Math.max(5, Math.min(newScale, 500));
      return { ...prev, scale: newScale };
    });
  };

  const handleMouseDown = (e: React.MouseEvent) => {
    isDragging.current = true;
    dragStart.current = { x: e.clientX, y: e.clientY };
    lastMousePos.current = { x: e.clientX, y: e.clientY };
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    // Update cursor map pos
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const mapPos = screenToMap(x, y, transform, canvas.width, canvas.height, mapData.current);
    setCursorMapPos(mapPos);

    if (isDragging.current) {
      const dx = e.clientX - lastMousePos.current.x;
      const dy = e.clientY - lastMousePos.current.y;
      setTransform(prev => ({
        ...prev,
        offsetX: prev.offsetX + dx,
        offsetY: prev.offsetY + dy
      }));
      lastMousePos.current = { x: e.clientX, y: e.clientY };
    }
  };

  const handleMouseUp = (e: React.MouseEvent) => {
    const wasDragging = isDragging.current;
    isDragging.current = false;
    
    // Only trigger click-to-set-goal if it wasn't a drag
    if (wasDragging && mode === 'AUTONOMOUS') {
      const dx = e.clientX - dragStart.current.x;
      const dy = e.clientY - dragStart.current.y;
      if (Math.abs(dx) < 5 && Math.abs(dy) < 5) {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const rect = canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        const mapPos = screenToMap(x, y, transform, canvas.width, canvas.height, mapData.current);
        
        setPendingGoal({
          x: mapPos.x,
          y: mapPos.y,
          screenX: x,
          screenY: y
        });
      }
    }
  };

  const confirmGoal = () => {
    if (pendingGoal) {
      sendGoal(pendingGoal.x, pendingGoal.y);
      setPendingGoal(null);
    }
  };

  return (
    <div ref={containerRef} className="w-full h-full relative" onWheel={handleWheel}>
      <canvas
        ref={canvasRef}
        className={`w-full h-full ${mode === 'AUTONOMOUS' ? 'cursor-crosshair' : 'cursor-grab active:cursor-grabbing'}`}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={() => { isDragging.current = false; }}
      />
      
      {/* Pending Goal Marker Popup */}
      {pendingGoal && (
        <div 
          className="absolute z-20"
          style={{ 
            left: `${Math.min(pendingGoal.screenX, (containerRef.current?.clientWidth || 300) - 200)}px`, 
            top: `${Math.max(0, pendingGoal.screenY - 120)}px` 
          }}
        >
          <GoalMarker 
            x={pendingGoal.x} 
            y={pendingGoal.y} 
            onConfirm={confirmGoal} 
            onCancel={() => setPendingGoal(null)} 
          />
        </div>
      )}

      {/* Toolbar */}
      <div className="absolute top-2 right-2 flex flex-col gap-1 z-10">
        <button className="bg-gray-800/90 border border-gray-700 text-gray-300 hover:text-white hover:border-cyan-600 p-1.5 rounded transition-all" onClick={() => setTransform(p => ({...p, scale: Math.min(500, p.scale * 1.3)}))}>
          <Plus size={14} />
        </button>
        <button className="bg-gray-800/90 border border-gray-700 text-gray-300 hover:text-white hover:border-cyan-600 p-1.5 rounded transition-all" onClick={() => setTransform(p => ({...p, scale: Math.max(5, p.scale / 1.3)}))}>
          <Minus size={14} />
        </button>
        <button className="bg-gray-800/90 border border-gray-700 text-gray-300 hover:text-white hover:border-cyan-600 p-1.5 rounded transition-all mt-1" onClick={() => setTransform({scale: 40, offsetX: 0, offsetY: 0})}>
          <RotateCcw size={14} />
        </button>
        <button className="bg-gray-800/90 border border-gray-700 text-gray-300 hover:text-white hover:border-cyan-600 p-1.5 rounded transition-all" onClick={() => setTransform(prev => ({...prev, offsetX: 0, offsetY: 0}))}>
          <Target size={14} />
        </button>
        <button className="bg-gray-800/90 border border-gray-700 text-gray-300 hover:text-white hover:border-red-500 p-1.5 rounded transition-all mt-1" onClick={() => setPendingGoal(null)}>
          <X size={14} />
        </button>
      </div>

      {/* Coordinate display */}
      <div className="absolute bottom-2 right-2 bg-gray-900/80 backdrop-blur px-2 py-1 rounded text-[10px] font-mono text-gray-400 pointer-events-none border border-gray-700/50">
        X: {cursorMapPos.x.toFixed(2)}m  Y: {cursorMapPos.y.toFixed(2)}m
      </div>
      
      {/* Scale bar */}
      <div className="absolute bottom-2 left-2 bg-gray-900/80 backdrop-blur px-2 py-1 rounded flex items-center gap-2 pointer-events-none border border-gray-700/50">
        <div className="text-[10px] text-gray-400 font-mono">1m</div>
        <div style={{ width: `${transform.scale}px` }} className="h-0.5 bg-gray-400"></div>
      </div>

      {/* Mode indicator */}
      {mode === 'AUTONOMOUS' && (
        <div className="absolute top-2 left-1/2 -translate-x-1/2 bg-cyan-900/50 border border-cyan-700/50 px-3 py-1 rounded-full text-[10px] font-mono text-cyan-400 pointer-events-none">
          Click to set goal
        </div>
      )}
    </div>
  );
}
