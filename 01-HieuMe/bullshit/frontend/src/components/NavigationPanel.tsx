import { NavigationStatus, NavigationState } from '../types/navigation';
import { Ban, Navigation, MapPin } from 'lucide-react';

interface NavigationPanelProps {
  status: NavigationStatus;
  mode: 'MANUAL' | 'AUTONOMOUS';
  cancelGoal: () => void;
  emergencyStop: () => void;
}

const statusConfig: Record<NavigationState, { color: string; bg: string; text: string }> = {
  IDLE: { color: 'bg-gray-500', bg: 'bg-gray-500/10', text: 'text-gray-400' },
  PLANNING: { color: 'bg-blue-500 animate-pulse', bg: 'bg-blue-500/10', text: 'text-blue-400' },
  NAVIGATING: { color: 'bg-green-500 animate-pulse', bg: 'bg-green-500/10', text: 'text-green-400' },
  REACHED: { color: 'bg-emerald-400', bg: 'bg-emerald-500/10', text: 'text-emerald-400' },
  BLOCKED: { color: 'bg-orange-500 animate-pulse', bg: 'bg-orange-500/10', text: 'text-orange-400' },
  CANCELLED: { color: 'bg-gray-500', bg: 'bg-gray-500/10', text: 'text-gray-400' },
  ERROR: { color: 'bg-red-500', bg: 'bg-red-500/10', text: 'text-red-400' },
};

export default function NavigationPanel({ status, mode, cancelGoal, emergencyStop }: NavigationPanelProps) {
  const config = statusConfig[status.status] || statusConfig.IDLE;

  return (
    <div className="flex flex-col h-full gap-2">
      <div className="flex justify-between items-center shrink-0">
        <div className="flex items-center gap-2">
          <Navigation className="w-3.5 h-3.5 text-gray-400" />
          <h3 className="text-xs font-bold text-gray-400 uppercase tracking-wider">Navigation</h3>
        </div>
        <div className={`flex items-center gap-1.5 ${config.bg} px-2 py-1 rounded border border-gray-700/50`}>
          <div className={`w-2 h-2 rounded-full ${config.color}`}></div>
          <span className={`text-[10px] font-bold font-mono ${config.text}`}>{status.status}</span>
        </div>
      </div>

      <div className="flex-1 flex flex-col gap-2 overflow-auto">
        {/* Goal */}
        <div className="bg-gray-900/80 p-2 rounded border border-gray-700/50">
          <div className="flex items-center gap-1.5 mb-1">
            <MapPin className="w-3 h-3 text-gray-500" />
            <span className="text-[10px] text-gray-500">Goal Position</span>
          </div>
          <div className="text-xs font-mono">
            {status.goalX !== null && status.goalY !== null ? (
              <div className="flex gap-3">
                <span className="text-cyan-400">X: {status.goalX.toFixed(2)}m</span>
                <span className="text-cyan-400">Y: {status.goalY.toFixed(2)}m</span>
              </div>
            ) : (
              <span className="text-gray-600 italic">No goal set</span>
            )}
          </div>
        </div>

        {/* Distance & Progress */}
        <div className="bg-gray-900/80 p-2 rounded border border-gray-700/50">
          <div className="flex justify-between text-[10px] mb-1">
            <span className="text-gray-500">Distance</span>
            <span className="text-gray-300 font-mono">{status.distanceRemaining.toFixed(2)}m</span>
          </div>
          <div className="w-full h-1.5 bg-gray-700 rounded-full overflow-hidden mb-2">
            <div 
              className="h-full bg-gradient-to-r from-cyan-600 to-cyan-400 transition-all duration-300" 
              style={{ width: `${Math.max(0, Math.min(100, status.progress))}%` }}
            />
          </div>
          <div className="flex justify-between text-[10px]">
            <span className="text-gray-500">Progress</span>
            <span className="text-cyan-400 font-mono font-bold">{Math.round(status.progress)}%</span>
          </div>
        </div>

        {/* Velocity */}
        <div className="bg-gray-900/80 p-2 rounded border border-gray-700/50">
          <div className="flex justify-between text-[10px]">
            <span className="text-gray-500">Velocity</span>
            <span className="text-gray-300 font-mono">{status.velocity.toFixed(2)} m/s</span>
          </div>
        </div>
      </div>

      {/* Buttons */}
      <div className="flex gap-2 shrink-0">
        <button 
          onClick={cancelGoal}
          disabled={status.status !== 'NAVIGATING' && status.status !== 'PLANNING'}
          className="flex-1 bg-gray-700 hover:bg-gray-600 disabled:opacity-30 disabled:cursor-not-allowed text-white text-[11px] font-bold py-2 rounded transition-colors"
        >
          Cancel Goal
        </button>
        <button 
          onClick={emergencyStop}
          className="flex-1 bg-red-600 hover:bg-red-500 text-white text-[11px] font-bold py-2 rounded flex justify-center items-center gap-1 shadow-[0_0_12px_rgba(220,38,38,0.3)] transition-colors"
        >
          <Ban className="w-3.5 h-3.5" /> E-STOP
        </button>
      </div>
    </div>
  );
}
