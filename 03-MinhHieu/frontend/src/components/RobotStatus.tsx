import { RobotPose } from '../types/robot';
import React, { useState, useEffect } from 'react';
import { Wifi, WifiOff, Battery } from 'lucide-react';

interface RobotStatusProps {
  connected: boolean;
  battery: number;
  pose: React.MutableRefObject<RobotPose>;
  mode: 'MANUAL' | 'AUTONOMOUS';
}

export default function RobotStatus({ connected, battery, pose, mode }: RobotStatusProps) {
  const [currentPose, setCurrentPose] = useState(pose.current);

  // Update display periodically rather than on every pose change
  useEffect(() => {
    const interval = setInterval(() => {
      setCurrentPose({ ...pose.current });
    }, 100);
    return () => clearInterval(interval);
  }, [pose]);

  const getBatteryColor = (level: number) => {
    if (level > 50) return 'bg-green-500';
    if (level > 20) return 'bg-yellow-500';
    return 'bg-red-500';
  };

  const getBatteryTextColor = (level: number) => {
    if (level > 50) return 'text-green-400';
    if (level > 20) return 'text-yellow-400';
    return 'text-red-400';
  };

  return (
    <div className="flex flex-col gap-2">
      <h3 className="text-xs font-bold text-gray-400 uppercase tracking-wider">Robot Status</h3>
      
      {/* Connection */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          {connected ? (
            <Wifi className="w-3.5 h-3.5 text-green-400" />
          ) : (
            <WifiOff className="w-3.5 h-3.5 text-red-400" />
          )}
          <span className={`text-xs font-mono font-bold ${connected ? 'text-green-400' : 'text-red-400'}`}>
            {connected ? 'ONLINE' : 'OFFLINE'}
          </span>
        </div>
        <span className={`text-[10px] font-mono px-2 py-0.5 rounded ${
          mode === 'AUTONOMOUS' 
            ? 'bg-cyan-900/50 text-cyan-400 border border-cyan-800' 
            : 'bg-gray-700 text-gray-300 border border-gray-600'
        }`}>
          {mode}
        </span>
      </div>

      {/* Battery */}
      <div className="bg-gray-900/80 p-2 rounded border border-gray-700/50">
        <div className="flex items-center justify-between mb-1">
          <div className="flex items-center gap-1.5">
            <Battery className="w-3.5 h-3.5 text-gray-400" />
            <span className="text-[10px] text-gray-400">Battery</span>
          </div>
          <span className={`text-xs font-mono font-bold ${getBatteryTextColor(battery)}`}>
            {battery.toFixed(1)}%
          </span>
        </div>
        <div className="w-full h-1.5 bg-gray-700 rounded-full overflow-hidden">
          <div 
            className={`h-full ${getBatteryColor(battery)} transition-all duration-500`} 
            style={{ width: `${Math.max(0, Math.min(100, battery))}%` }}
          />
        </div>
      </div>

      {/* Position */}
      <div className="grid grid-cols-3 gap-1.5">
        <div className="bg-gray-900/80 p-1.5 rounded border border-gray-700/50 flex flex-col items-center">
          <span className="text-[9px] text-gray-500">X (m)</span>
          <span className="text-xs font-mono text-cyan-400">{currentPose.x.toFixed(2)}</span>
        </div>
        <div className="bg-gray-900/80 p-1.5 rounded border border-gray-700/50 flex flex-col items-center">
          <span className="text-[9px] text-gray-500">Y (m)</span>
          <span className="text-xs font-mono text-cyan-400">{currentPose.y.toFixed(2)}</span>
        </div>
        <div className="bg-gray-900/80 p-1.5 rounded border border-gray-700/50 flex flex-col items-center">
          <span className="text-[9px] text-gray-500">θ (deg)</span>
          <span className="text-xs font-mono text-cyan-400">{(currentPose.theta * 180 / Math.PI).toFixed(0)}°</span>
        </div>
      </div>
    </div>
  );
}
