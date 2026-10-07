import { useState, useRef, useEffect, useCallback } from 'react';
import { Compass, Navigation } from 'lucide-react';

interface VirtualJoystickProps {
  onVelocityChange: (linear: number, angular: number) => void;
  onStop: () => void;
  speedLimit?: number;
  disabled?: boolean;
}

export default function VirtualJoystick({
  onVelocityChange,
  onStop,
  speedLimit = 0.25,
  disabled = false
}: VirtualJoystickProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const [knobPos, setKnobPos] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [telemetry, setTelemetry] = useState<{ angleDeg: number; powerPct: number }>({ angleDeg: 0, powerPct: 0 });
  
  const lastSendTimeRef = useRef(0);
  const radius = 54; // Max travel radius in px
  const maxAngularSpeed = 1.5; // rad/s

  const processCoord = useCallback((clientX: number, clientY: number) => {
    if (!containerRef.current || disabled) return;
    const rect = containerRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    const dx = clientX - centerX;
    const dy = clientY - centerY;
    const distance = Math.hypot(dx, dy);
    const clampedDist = Math.min(distance, radius);
    const angleRad = Math.atan2(dy, dx);

    const normX = clampedDist === 0 ? 0 : (clampedDist * Math.cos(angleRad)) / radius;
    const normY = clampedDist === 0 ? 0 : (clampedDist * Math.sin(angleRad)) / radius;

    setKnobPos({
      x: normX * radius,
      y: normY * radius
    });

    // In robot coordinates: forward is -Y, backward is +Y, right is +X, left is -X
    const forward = -normY; // [-1, 1]
    const turn = normX;     // [-1, 1]

    const linear = forward * speedLimit;
    const angular = -turn * maxAngularSpeed;

    // Angle calculation: 0° is North (forward), 90° East (right), 180° South (back), 270° West (left)
    let deg = Math.round((Math.atan2(normX, -normY) * 180) / Math.PI);
    if (deg < 0) deg += 360;

    const power = Math.round((clampedDist / radius) * 100);
    setTelemetry({ angleDeg: deg, powerPct: power });

    // Throttle commands to 20Hz (50ms)
    const now = Date.now();
    if (now - lastSendTimeRef.current >= 45) {
      lastSendTimeRef.current = now;
      onVelocityChange(linear, angular);
    }
  }, [disabled, onVelocityChange, speedLimit, radius]);

  const handlePointerDown = (e: React.PointerEvent) => {
    if (disabled) return;
    e.preventDefault();
    (e.target as HTMLElement).setPointerCapture?.(e.pointerId);
    setIsDragging(true);
    processCoord(e.clientX, e.clientY);
  };

  const handlePointerMove = (e: React.PointerEvent) => {
    if (!isDragging || disabled) return;
    e.preventDefault();
    processCoord(e.clientX, e.clientY);
  };

  const handlePointerUp = (e: React.PointerEvent) => {
    if (!isDragging) return;
    try {
      (e.target as HTMLElement).releasePointerCapture?.(e.pointerId);
    } catch {}
    setIsDragging(false);
    setKnobPos({ x: 0, y: 0 });
    setTelemetry({ angleDeg: 0, powerPct: 0 });
    onVelocityChange(0, 0);
    onStop();
  };

  return (
    <div className="flex flex-col items-center gap-1 select-none">
      {/* Joystick Base Pad */}
      <div
        ref={containerRef}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
        className={`relative w-32 h-32 rounded-full border flex items-center justify-center cursor-grab active:cursor-grabbing transition-colors ${
          disabled 
            ? 'opacity-40 cursor-not-allowed bg-gray-100 dark:bg-[#141414] border-gray-200 dark:border-[#262626]' 
            : isDragging
              ? 'bg-blue-50/60 dark:bg-[#f97316]/15 border-[#0071e3] dark:border-[#f97316] shadow-[0_0_20px_rgba(0,113,227,0.3)] dark:shadow-[0_0_20px_rgba(249,115,22,0.25)]'
              : 'bg-gradient-to-b from-gray-50 to-gray-100/80 dark:from-[#1c1c1c] dark:to-[#141414] border-gray-200/90 dark:border-[#262626] shadow-inner'
        }`}
        style={{ touchAction: 'none' }}
      >
        {/* Cardinal Grid Crosshairs */}
        <div className="absolute w-full h-[1px] bg-gray-300/60 dark:bg-[#262626] pointer-events-none"></div>
        <div className="absolute h-full w-[1px] bg-gray-300/60 dark:bg-[#262626] pointer-events-none"></div>

        {/* Center Resting Ring */}
        <div className="absolute w-8 h-8 rounded-full border border-dashed border-gray-300 dark:border-[#262626] pointer-events-none"></div>

        {/* Direction Indicators */}
        <span className="absolute top-1 text-[9px] font-bold text-gray-400 dark:text-[#9ca3af] pointer-events-none">▲</span>
        <span className="absolute bottom-1 text-[9px] font-bold text-gray-400 dark:text-[#9ca3af] pointer-events-none">▼</span>
        <span className="absolute left-1 text-[9px] font-bold text-gray-400 dark:text-[#9ca3af] pointer-events-none">◀</span>
        <span className="absolute right-1 text-[9px] font-bold text-gray-400 dark:text-[#9ca3af] pointer-events-none">▶</span>

        {/* Thumb Nipple */}
        <div
          className={`absolute w-12 h-12 rounded-full shadow-[0_4px_12px_rgba(0,0,0,0.18)] flex items-center justify-center transition-transform ${
            isDragging
              ? 'bg-[#0071e3] dark:bg-[#f97316] text-white dark:text-white scale-105 shadow-[0_6px_20px_rgba(249,115,22,0.5)]'
              : 'bg-white dark:bg-[#1c1c1c] text-gray-700 dark:text-[#f4f4f5] hover:scale-102 border border-gray-200 dark:border-[#262626]'
          }`}
          style={{
            transform: `translate(${knobPos.x}px, ${knobPos.y}px)`,
            transition: isDragging ? 'none' : 'transform 0.2s cubic-bezier(0.16, 1, 0.3, 1)'
          }}
        >
          {isDragging ? (
            <Navigation 
              className="w-5 h-5 text-white dark:text-white transition-transform" 
              style={{ transform: `rotate(${telemetry.angleDeg}deg)` }}
            />
          ) : (
            <div className="w-3.5 h-3.5 rounded-full bg-[#0071e3]/30 dark:bg-[#f97316]/30 border border-[#0071e3] dark:border-[#f97316]"></div>
          )}
        </div>
      </div>

      {/* Angle & Power Metric Readout */}
      <div className="flex items-center justify-between w-full px-2 text-[11px] font-mono text-[#6e6e73] dark:text-[#9ca3af]">
        <span>Góc: <strong className="text-[#0071e3] dark:text-[#f97316] font-bold">{telemetry.powerPct > 0 ? `${telemetry.angleDeg}°` : '--'}</strong></span>
        <span>Lực: <strong className={telemetry.powerPct > 0 ? 'text-emerald-500 dark:text-[#4ade80] font-bold' : 'text-gray-400 dark:text-[#9ca3af]'}>{telemetry.powerPct}%</strong></span>
      </div>
    </div>
  );
}
