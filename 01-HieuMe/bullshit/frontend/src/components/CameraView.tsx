import { useRef, useEffect, useState } from 'react';
import { Detection } from '../types/detection';

interface CameraViewProps {
  cameraFrame: string;
  detections: React.MutableRefObject<Detection[]>;
  detectionsVersion: number;
}

export default function CameraView({ cameraFrame, detections, detectionsVersion }: CameraViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [viewMode, setViewMode] = useState<'ORIGINAL' | 'YOLO' | 'YOLO_DIST'>('YOLO_DIST');

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    if (!cameraFrame) {
      // Draw placeholder
      ctx.fillStyle = '#111827';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = '#374151';
      ctx.lineWidth = 1;
      
      // Grid pattern
      for (let i = 0; i < canvas.width; i += 40) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, canvas.height); ctx.stroke();
      }
      for (let i = 0; i < canvas.height; i += 40) {
        ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(canvas.width, i); ctx.stroke();
      }
      
      // Crosshair
      ctx.strokeStyle = '#4B5563';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(canvas.width / 2, 0);
      ctx.lineTo(canvas.width / 2, canvas.height);
      ctx.moveTo(0, canvas.height / 2);
      ctx.lineTo(canvas.width, canvas.height / 2);
      ctx.stroke();
      
      ctx.fillStyle = '#6B7280';
      ctx.font = '16px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('NO CAMERA SIGNAL', canvas.width / 2, canvas.height / 2 - 10);
      ctx.font = '12px monospace';
      ctx.fillStyle = '#4B5563';
      ctx.fillText('Waiting for camera stream...', canvas.width / 2, canvas.height / 2 + 15);
      return;
    }

    const img = new Image();
    img.src = `data:image/jpeg;base64,${cameraFrame}`;

    img.onload = () => {
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);

      if (viewMode === 'ORIGINAL') return;

      const dets = detections.current;
      dets.forEach(det => {
        const [x1, y1, x2, y2] = det.bbox;
        // bbox is in pixel coordinates from backend (640x480)
        const scaleX = canvas.width / 640;
        const scaleY = canvas.height / 480;
        const cx1 = x1 * scaleX;
        const cy1 = y1 * scaleY;
        const cw = (x2 - x1) * scaleX;
        const ch = (y2 - y1) * scaleY;

        // Color by class
        const colors: Record<string, string> = {
          'person': '#00ff88',
          'chair': '#ffaa00',
          'bottle': '#00aaff',
          'laptop': '#ff66cc',
          'dog': '#ffcc00',
          'default': '#ff0055'
        };
        const color = colors[det.className.toLowerCase()] || colors.default;

        // Bounding box
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.strokeRect(cx1, cy1, cw, ch);

        // Corner accents (cyberpunk style)
        const cornerSize = 8;
        ctx.lineWidth = 3;
        // Top-left
        ctx.beginPath();
        ctx.moveTo(cx1, cy1 + cornerSize); ctx.lineTo(cx1, cy1); ctx.lineTo(cx1 + cornerSize, cy1);
        ctx.stroke();
        // Top-right
        ctx.beginPath();
        ctx.moveTo(cx1 + cw - cornerSize, cy1); ctx.lineTo(cx1 + cw, cy1); ctx.lineTo(cx1 + cw, cy1 + cornerSize);
        ctx.stroke();
        // Bottom-left
        ctx.beginPath();
        ctx.moveTo(cx1, cy1 + ch - cornerSize); ctx.lineTo(cx1, cy1 + ch); ctx.lineTo(cx1 + cornerSize, cy1 + ch);
        ctx.stroke();
        // Bottom-right
        ctx.beginPath();
        ctx.moveTo(cx1 + cw - cornerSize, cy1 + ch); ctx.lineTo(cx1 + cw, cy1 + ch); ctx.lineTo(cx1 + cw, cy1 + ch - cornerSize);
        ctx.stroke();

        // Label background
        let label = `${det.className} ${Math.round(det.confidence * 100)}%`;
        if (viewMode === 'YOLO_DIST' && det.distance) {
          label += ` | ${det.distance.toFixed(2)}m`;
        }

        ctx.font = '11px "JetBrains Mono", monospace';
        const textWidth = ctx.measureText(label).width;
        
        ctx.fillStyle = 'rgba(0, 0, 0, 0.75)';
        ctx.fillRect(cx1, cy1 - 22, textWidth + 12, 20);
        ctx.fillStyle = color;
        ctx.fillRect(cx1, cy1 - 22, 3, 20); // Color accent bar
        
        ctx.fillStyle = '#ffffff';
        ctx.fillText(label, cx1 + 8, cy1 - 7);
      });
    };
  }, [cameraFrame, detectionsVersion, viewMode, detections]);

  return (
    <div className="flex flex-col h-full relative">
      {/* View mode buttons */}
      <div className="absolute top-2 left-2 right-2 flex justify-between z-10 pointer-events-none">
        <div className="bg-gray-900/80 text-cyan-400 text-[10px] font-bold font-mono px-2 py-1 rounded backdrop-blur border border-cyan-900/50">
          ● CAMERA FEED
        </div>
        <div className="flex gap-1 pointer-events-auto">
          {(['ORIGINAL', 'YOLO', 'YOLO_DIST'] as const).map(m => (
            <button
              key={m}
              onClick={() => setViewMode(m)}
              className={`text-[10px] px-2 py-1 rounded backdrop-blur font-mono transition-all ${
                viewMode === m
                  ? 'bg-cyan-600 text-white shadow-[0_0_8px_rgba(8,145,178,0.4)]'
                  : 'bg-gray-900/80 text-gray-400 hover:text-white border border-gray-700'
              }`}
            >
              {m === 'YOLO_DIST' ? 'YOLO+DIST' : m}
            </button>
          ))}
        </div>
      </div>

      {/* Canvas */}
      <div className="flex-1 bg-black relative">
        <canvas 
          ref={canvasRef} 
          className="absolute inset-0 w-full h-full"
          width={640}
          height={480}
        />
      </div>

      {/* Status bar */}
      <div className="h-8 bg-gray-900/95 border-t border-gray-700 flex items-center justify-between px-3 shrink-0">
        <div className="flex items-center gap-4 text-[10px] text-gray-400 font-mono">
          <span>FPS: <span className="text-gray-200">24</span></span>
          <span>LATENCY: <span className="text-gray-200">82ms</span></span>
          <span>RES: <span className="text-gray-200">640×480</span></span>
        </div>
        <div className="flex items-center gap-3 text-[10px] font-mono">
          <div className="flex items-center gap-1.5">
            <div className="w-1.5 h-1.5 bg-green-500 rounded-full animate-pulse"></div>
            <span className="text-gray-300">YOLO</span>
          </div>
          <div className="flex items-center gap-1.5">
            <div className="w-1.5 h-1.5 bg-green-500 rounded-full"></div>
            <span className="text-gray-300">CAM</span>
          </div>
        </div>
      </div>
    </div>
  );
}
