import React, { useRef, useEffect, useState, useCallback } from 'react';
import { Detection } from '../types/detection';
import { 
  Camera, Maximize2, Minimize2, Download, Eye, Sparkles, Video, RefreshCw 
} from 'lucide-react';

interface CameraViewProps {
  cameraFrame?: string;
  detections: React.MutableRefObject<Detection[]>;
  detectionsVersion: number;
  showYoloBoxes?: boolean;
  showDistanceTags?: boolean;
  isMaximized?: boolean;
  isRealConnected?: boolean;
  onToggleMaximize?: () => void;
}

export default function CameraView({
  cameraFrame,
  detections,
  detectionsVersion,
  showYoloBoxes = true,
  showDistanceTags = true,
  isMaximized = false,
  isRealConnected = false,
  onToggleMaximize
}: CameraViewProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imgRef = useRef<HTMLImageElement>(null);

  const [fps, setFps] = useState<number>(15);
  const [streamError, setStreamError] = useState(false);
  const [retryKey, setRetryKey] = useState<number>(0);
  const [viewMode, setViewMode] = useState<'AUTO' | 'RAW'>('AUTO');
  const [snapshotFlash, setSnapshotFlash] = useState(false);

  // Auto-retry reconnecting to /video_feed if stream error occurs
  useEffect(() => {
    if (streamError) {
      const timer = setTimeout(() => {
        setRetryKey(k => k + 1);
        setStreamError(false);
      }, 2000);
      return () => clearTimeout(timer);
    }
  }, [streamError]);

  // Render YOLO Bounding Boxes onto transparent overlay Canvas
  const renderOverlay = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    if (viewMode === 'RAW' || !showYoloBoxes) return;

    const dets = detections.current;
    const width = canvas.width;
    const height = canvas.height;

    dets.forEach(det => {
      const [x1, y1, x2, y2] = det.bbox;
      const scaleX = width / 640;
      const scaleY = height / 480;
      const cx1 = x1 * scaleX;
      const cy1 = y1 * scaleY;
      const cw = (x2 - x1) * scaleX;
      const ch = (y2 - y1) * scaleY;

      // Color scheme according to class
      const isPerson = det.className.toLowerCase() === 'person';
      const strokeColor = isPerson ? '#10b981' : '#0071e3';
      const fillColor = isPerson ? 'rgba(16, 185, 129, 0.12)' : 'rgba(0, 113, 227, 0.12)';

      // 1. Box with soft rounded corners
      ctx.beginPath();
      const r = Math.min(8, cw / 4, ch / 4);
      ctx.moveTo(cx1 + r, cy1);
      ctx.lineTo(cx1 + cw - r, cy1);
      ctx.quadraticCurveTo(cx1 + cw, cy1, cx1 + cw, cy1 + r);
      ctx.lineTo(cx1 + cw, cy1 + ch - r);
      ctx.quadraticCurveTo(cx1 + cw, cy1 + ch, cx1 + cw - r, cy1 + ch);
      ctx.lineTo(cx1 + r, cy1 + ch);
      ctx.quadraticCurveTo(cx1, cy1 + ch, cx1, cy1 + ch - r);
      ctx.lineTo(cx1, cy1 + r);
      ctx.quadraticCurveTo(cx1, cy1, cx1 + r, cy1);
      ctx.closePath();

      ctx.fillStyle = fillColor;
      ctx.fill();
      ctx.strokeStyle = strokeColor;
      ctx.lineWidth = 2.5;
      ctx.stroke();

      // 2. Corner highlights (Apple Vision style)
      const cornerLen = Math.min(16, cw / 3, ch / 3);
      ctx.lineWidth = 3.5;
      ctx.strokeStyle = isPerson ? '#059669' : '#0071e3';
      // Top-Left
      ctx.beginPath();
      ctx.moveTo(cx1, cy1 + cornerLen); ctx.lineTo(cx1, cy1); ctx.lineTo(cx1 + cornerLen, cy1);
      ctx.stroke();
      // Top-Right
      ctx.beginPath();
      ctx.moveTo(cx1 + cw - cornerLen, cy1); ctx.lineTo(cx1 + cw, cy1); ctx.lineTo(cx1 + cw, cy1 + cornerLen);
      ctx.stroke();
      // Bottom-Left
      ctx.beginPath();
      ctx.moveTo(cx1, cy1 + ch - cornerLen); ctx.lineTo(cx1, cy1); ctx.lineTo(cx1 + cornerLen, cy1);
      ctx.stroke();
      // Bottom-Right
      ctx.beginPath();
      ctx.moveTo(cx1 + cw - cornerLen, cy1 + ch); ctx.lineTo(cx1 + cw, cy1 + ch); ctx.lineTo(cx1 + cw, cy1 + ch - cornerLen);
      ctx.stroke();

      // 3. Label Pill (Apple White Glass Card with generous padding)
      let label = `${det.className} ${Math.round(det.confidence * 100)}%`;
      if (showDistanceTags && det.distance) {
        label += ` • ${det.distance.toFixed(2)}m`;
        if (isPerson) {
          label += ` (Bubble: 0.9m)`;
        }
      }
      if (det.azimuth_deg) {
        label += ` • ${det.azimuth_deg > 0 ? '+' : ''}${det.azimuth_deg.toFixed(0)}°`;
      }

      ctx.font = '600 12px -apple-system, BlinkMacSystemFont, "SF Pro Text", "Inter", sans-serif';
      const textMetrics = ctx.measureText(label);
      const pillWidth = textMetrics.width + 24;
      const pillHeight = 26;
      const pillX = cx1;
      const pillY = Math.max(6, cy1 - pillHeight - 6);

      // Pill background: Clean white with drop shadow
      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.roundRect ? ctx.roundRect(pillX, pillY, pillWidth, pillHeight, 7) : ctx.rect(pillX, pillY, pillWidth, pillHeight);
      ctx.fill();
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.12)';
      ctx.lineWidth = 1;
      ctx.stroke();

      // Status Accent dot
      ctx.fillStyle = isPerson ? '#10b981' : '#0071e3';
      ctx.beginPath();
      ctx.arc(pillX + 10, pillY + pillHeight / 2, 3.5, 0, Math.PI * 2);
      ctx.fill();

      // Text
      ctx.fillStyle = '#1d1d1f';
      ctx.fillText(label, pillX + 20, pillY + 17);
    });
  }, [detections, viewMode, showYoloBoxes, showDistanceTags]);

  useEffect(() => {
    renderOverlay();
  }, [detectionsVersion, renderOverlay]);

  // Handle Resize of canvas to match container
  useEffect(() => {
    const handleResize = () => {
      const container = containerRef.current;
      const canvas = canvasRef.current;
      if (!container || !canvas) return;
      canvas.width = container.clientWidth;
      canvas.height = container.clientHeight;
      renderOverlay();
    };

    handleResize();
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, [renderOverlay]);

  // Take high quality snapshot with overlays
  const takeSnapshot = () => {
    const container = containerRef.current;
    if (!container) return;

    setSnapshotFlash(true);
    setTimeout(() => setSnapshotFlash(false), 250);

    const exportCanvas = document.createElement('canvas');
    exportCanvas.width = 640;
    exportCanvas.height = 480;
    const ctx = exportCanvas.getContext('2d');
    if (!ctx) return;

    // Draw video feed
    const img = imgRef.current;
    if (img && !streamError) {
      try {
        ctx.drawImage(img, 0, 0, 640, 480);
      } catch {
        ctx.fillStyle = '#f1f5f9';
        ctx.fillRect(0, 0, 640, 480);
      }
    } else {
      ctx.fillStyle = '#f1f5f9';
      ctx.fillRect(0, 0, 640, 480);
    }

    // Draw overlay
    const overlay = canvasRef.current;
    if (overlay) {
      ctx.drawImage(overlay, 0, 0, 640, 480);
    }

    // Trigger download
    const link = document.createElement('a');
    link.download = `jetbot-snapshot-${new Date().toISOString().replace(/[:.]/g, '-')}.png`;
    link.href = exportCanvas.toDataURL('image/png');
    link.click();
  };

  return (
    <div 
      ref={containerRef}
      className="relative w-full h-full flex flex-col bg-white dark:bg-[#141414] overflow-hidden rounded-2xl border border-gray-200/80 dark:border-[#262626] shadow-[0_2px_12px_rgba(0,0,0,0.04)] dark:shadow-[0_4px_20px_rgba(0,0,0,0.6)] select-none group"
    >
      {/* Camera Header Bar */}
      <div className="absolute top-0 inset-x-0 h-12 px-4 z-30 flex items-center justify-between bg-white/85 dark:bg-[#141414]/90 backdrop-blur-md border-b border-gray-100 dark:border-[#262626]">
        <div className="flex items-center gap-2.5">
          <div className="flex items-center gap-2 px-3 py-1 rounded-full bg-blue-50 dark:bg-[#1c1c1c] text-[#0071e3] dark:text-[#f4f4f5] border border-blue-200/60 dark:border-[#262626] text-xs font-semibold shadow-xs">
            <Video className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#f97316]" />
            <span>Camera CSI (IMX219)</span>
          </div>

          <div className="flex items-center gap-1.5 px-3 py-1 rounded-full bg-gray-50 dark:bg-[#1c1c1c] border border-gray-200/60 dark:border-[#262626] text-xs font-mono text-[#6e6e73] dark:text-[#9ca3af]">
            <span>YOLOv8n:</span>
            <span className="text-[#34c759] dark:text-[#4ade80] font-bold">~15 FPS</span>
          </div>
        </div>

        {/* Action Controls */}
        <div className="flex items-center gap-2">
          {/* View mode toggle */}
          <div className="flex bg-gray-100 dark:bg-[#1c1c1c] p-0.5 rounded-xl border border-gray-200/60 dark:border-[#262626]">
            <button
              onClick={() => setViewMode('AUTO')}
              className={`px-3 py-1 text-xs rounded-lg font-semibold transition-all ${
                viewMode === 'AUTO' 
                  ? 'bg-white dark:bg-[#262626] text-[#1d1d1f] dark:text-[#f4f4f5] shadow-xs' 
                  : 'text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5]'
              }`}
            >
              AI HUD
            </button>
            <button
              onClick={() => setViewMode('RAW')}
              className={`px-3 py-1 text-xs rounded-lg font-semibold transition-all ${
                viewMode === 'RAW' 
                  ? 'bg-white dark:bg-[#262626] text-[#1d1d1f] dark:text-[#f4f4f5] shadow-xs' 
                  : 'text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5]'
              }`}
            >
              Gốc (Raw)
            </button>
          </div>

          {/* Snapshot Button */}
          <button
            onClick={takeSnapshot}
            title="Chụp ảnh màn hình"
            className="w-8 h-8 rounded-xl bg-gray-50 dark:bg-[#1c1c1c] hover:bg-gray-150 dark:hover:bg-[#262626] border border-gray-200 dark:border-[#262626] flex items-center justify-center text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5] transition-all apple-btn"
          >
            <Download className="w-4 h-4" />
          </button>

          {/* Maximize Button */}
          {onToggleMaximize && (
            <button
              onClick={onToggleMaximize}
              title={isMaximized ? "Thu nhỏ về Cockpit" : "Phóng to camera"}
              className="w-8 h-8 rounded-xl bg-gray-50 dark:bg-[#1c1c1c] hover:bg-gray-150 dark:hover:bg-[#262626] border border-gray-200 dark:border-[#262626] flex items-center justify-center text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5] transition-all apple-btn"
            >
              {isMaximized ? <Minimize2 className="w-4 h-4" /> : <Maximize2 className="w-4 h-4" />}
            </button>
          )}
        </div>
      </div>

      {/* Main Video Area: Native MJPEG stream or Fallback Standby */}
      <div className="relative flex-1 w-full h-full flex items-center justify-center bg-gray-900 overflow-hidden">
        {/* Stream Image */}
        {!streamError ? (
          <img
            ref={imgRef}
            src={`/video_feed?t=${retryKey}`}
            alt="JetBot Camera Stream"
            onLoad={() => setStreamError(false)}
            onError={() => setStreamError(true)}
            className="w-full h-full object-contain pointer-events-none select-none"
          />
        ) : cameraFrame ? (
          <img
            src={`data:image/jpeg;base64,${cameraFrame}`}
            alt="WebSocket Fallback Frame"
            className="w-full h-full object-contain pointer-events-none select-none"
          />
        ) : (
          /* Standby State */
          <div className="flex flex-col items-center justify-center gap-3 text-gray-400">
            <div className="relative w-16 h-16 rounded-full border border-blue-400/30 flex items-center justify-center animate-apple-pulse">
              <Camera className="w-8 h-8 text-blue-400" />
            </div>
            <div className="flex flex-col items-center gap-1 text-center">
              <span className="text-sm font-semibold text-gray-200">Đang tự động kết nối luồng Camera CSI IMX219...</span>
              <span className="text-xs text-gray-400 font-mono">Endpoint: /video_feed (15 FPS)</span>
            </div>
            <button
              onClick={() => { setRetryKey(k => k + 1); setStreamError(false); }}
              className="flex items-center gap-2 px-4 py-1.5 rounded-full bg-white/10 hover:bg-white/20 text-xs text-white transition-colors apple-btn"
            >
              <RefreshCw className="w-3.5 h-3.5" />
              Thử lại kết nối
            </button>
          </div>
        )}

        {/* Transparent Canvas Overlay for YOLO Detections */}
        <canvas
          ref={canvasRef}
          className="absolute inset-0 w-full h-full pointer-events-none z-10"
        />

        {/* Flash effect on snapshot */}
        {snapshotFlash && (
          <div className="absolute inset-0 bg-white/70 z-50 pointer-events-none animate-fadeOut"></div>
        )}
      </div>

      {/* Bottom Status Info */}
      <div className="absolute bottom-3 left-4 z-30 pointer-events-none">
        <div className="flex items-center gap-2 px-3 py-1 rounded-full bg-white/90 dark:bg-[#141414]/90 backdrop-blur-md border border-gray-200/80 dark:border-[#262626] text-xs font-mono text-[#1d1d1f] dark:text-[#f4f4f5] shadow-xs">
          <span className="w-2 h-2 rounded-full bg-[#34c759] dark:bg-[#4ade80] animate-pulse"></span>
          <span>IMX219 CSI 160° FOV</span>
        </div>
      </div>
    </div>
  );
}
