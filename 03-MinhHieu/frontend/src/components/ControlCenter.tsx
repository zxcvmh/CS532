import { useRef, useEffect } from 'react';
import { UiPreferences } from '../hooks/useUiPreferences';
import { 
  Sliders, Eye, Layers, Gauge, Video, Map, 
  Compass, Activity, Terminal, ShieldAlert, 
  RotateCcw, Trash2, X, Sparkles
} from 'lucide-react';

interface ControlCenterProps {
  isOpen: boolean;
  onClose: () => void;
  prefs: UiPreferences;
  toggle: (key: keyof UiPreferences) => void;
  setSpeedLimit: (speed: number) => void;
  resetMap?: () => void;
  resetPose?: () => void;
}

export default function ControlCenter({
  isOpen,
  onClose,
  prefs,
  toggle,
  setSpeedLimit,
  resetMap,
  resetPose
}: ControlCenterProps) {
  const panelRef = useRef<HTMLDivElement>(null);

  // Close when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (panelRef.current && !panelRef.current.contains(event.target as Node)) {
        onClose();
      }
    }
    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen, onClose]);

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-start justify-end p-4 pt-16 bg-black/30 backdrop-blur-[2px] animate-fadeIn">
      <div 
        ref={panelRef}
        className="w-96 glass-panel rounded-3xl p-5 flex flex-col gap-4 text-gray-100 shadow-[0_24px_50px_rgba(0,0,0,0.7)] border border-white/15 animate-scaleUp"
      >
        {/* Header */}
        <div className="flex items-center justify-between border-b border-white/10 pb-3">
          <div className="flex items-center gap-2.5">
            <div className="w-8 h-8 rounded-xl bg-gradient-to-tr from-cyan-500/20 to-blue-500/30 flex items-center justify-center border border-white/10">
              <Sliders className="w-4 h-4 text-cyan-300" />
            </div>
            <div>
              <h2 className="text-sm font-semibold tracking-tight text-white flex items-center gap-1.5">
                Control Center
                <span className="text-[10px] px-1.5 py-0.5 rounded-full bg-cyan-500/20 text-cyan-300 font-mono">macOS</span>
              </h2>
              <p className="text-[11px] text-gray-400">Workspace toggles & robot safety</p>
            </div>
          </div>
          <button 
            onClick={onClose}
            className="w-7 h-7 rounded-full bg-white/5 hover:bg-white/15 flex items-center justify-center text-gray-400 hover:text-white transition-colors"
          >
            <X className="w-4 h-4" />
          </button>
        </div>

        {/* Speed Limit Slider */}
        <div className="glass-card rounded-2xl p-3.5 flex flex-col gap-2">
          <div className="flex justify-between items-center text-xs">
            <span className="font-medium text-gray-300 flex items-center gap-1.5">
              <Gauge className="w-3.5 h-3.5 text-amber-400" />
              Safety Speed Limit
            </span>
            <span className="font-mono font-semibold text-amber-400">
              {prefs.speedLimit.toFixed(2)} m/s
            </span>
          </div>
          <input 
            type="range"
            min="0.10"
            max="0.35"
            step="0.05"
            value={prefs.speedLimit}
            onChange={(e) => setSpeedLimit(parseFloat(e.target.value))}
            className="w-full h-1.5 bg-gray-700/60 rounded-lg appearance-none cursor-pointer accent-amber-400"
          />
          <div className="flex justify-between text-[10px] text-gray-400 font-mono">
            <span>Slow (0.10)</span>
            <span>Balanced (0.25)</span>
            <span>Max (0.35)</span>
          </div>
        </div>

        {/* Section 1: Widget Visibility */}
        <div className="flex flex-col gap-2">
          <div className="flex items-center gap-1.5 text-[11px] font-semibold text-gray-400 uppercase tracking-wider px-1">
            <Eye className="w-3 h-3 text-cyan-400" />
            <span>Dashboard Widgets</span>
          </div>
          
          <div className="grid grid-cols-2 gap-2">
            <button
              onClick={() => toggle('showCamera')}
              className={`p-2.5 rounded-xl border flex items-center gap-2 text-xs font-medium transition-all ${
                prefs.showCamera 
                  ? 'bg-cyan-500/15 border-cyan-500/30 text-cyan-200' 
                  : 'bg-white/[0.02] border-white/5 text-gray-400 hover:bg-white/5'
              }`}
            >
              <Video className="w-3.5 h-3.5" />
              <span>Camera Stream</span>
            </button>

            <button
              onClick={() => toggle('showMap')}
              className={`p-2.5 rounded-xl border flex items-center gap-2 text-xs font-medium transition-all ${
                prefs.showMap 
                  ? 'bg-cyan-500/15 border-cyan-500/30 text-cyan-200' 
                  : 'bg-white/[0.02] border-white/5 text-gray-400 hover:bg-white/5'
              }`}
            >
              <Map className="w-3.5 h-3.5" />
              <span>2D SLAM Map</span>
            </button>

            <button
              onClick={() => toggle('showTelemetry')}
              className={`p-2.5 rounded-xl border flex items-center gap-2 text-xs font-medium transition-all ${
                prefs.showTelemetry 
                  ? 'bg-cyan-500/15 border-cyan-500/30 text-cyan-200' 
                  : 'bg-white/[0.02] border-white/5 text-gray-400 hover:bg-white/5'
              }`}
            >
              <Activity className="w-3.5 h-3.5" />
              <span>Telemetry</span>
            </button>

            <button
              onClick={() => toggle('showNavigation')}
              className={`p-2.5 rounded-xl border flex items-center gap-2 text-xs font-medium transition-all ${
                prefs.showNavigation 
                  ? 'bg-cyan-500/15 border-cyan-500/30 text-cyan-200' 
                  : 'bg-white/[0.02] border-white/5 text-gray-400 hover:bg-white/5'
              }`}
            >
              <Compass className="w-3.5 h-3.5" />
              <span>Navigation</span>
            </button>

            <button
              onClick={() => toggle('showControls')}
              className={`p-2.5 rounded-xl border flex items-center gap-2 text-xs font-medium transition-all ${
                prefs.showControls 
                  ? 'bg-cyan-500/15 border-cyan-500/30 text-cyan-200' 
                  : 'bg-white/[0.02] border-white/5 text-gray-400 hover:bg-white/5'
              }`}
            >
              <ShieldAlert className="w-3.5 h-3.5" />
              <span>Teleop Pad</span>
            </button>

            <button
              onClick={() => toggle('showLogs')}
              className={`p-2.5 rounded-xl border flex items-center gap-2 text-xs font-medium transition-all ${
                prefs.showLogs 
                  ? 'bg-cyan-500/15 border-cyan-500/30 text-cyan-200' 
                  : 'bg-white/[0.02] border-white/5 text-gray-400 hover:bg-white/5'
              }`}
            >
              <Terminal className="w-3.5 h-3.5" />
              <span>System Log</span>
            </button>
          </div>
        </div>

        {/* Section 2: Map & Camera Overlays */}
        <div className="flex flex-col gap-2">
          <div className="flex items-center gap-1.5 text-[11px] font-semibold text-gray-400 uppercase tracking-wider px-1">
            <Layers className="w-3 h-3 text-emerald-400" />
            <span>Map & Vision Layers</span>
          </div>

          <div className="glass-card rounded-2xl p-2.5 flex flex-col gap-1.5">
            <label className="flex items-center justify-between text-xs py-1 px-1 cursor-pointer">
              <span className="text-gray-300">LiDAR 360° Scan Points</span>
              <input 
                type="checkbox"
                checked={prefs.showLidarScan}
                onChange={() => toggle('showLidarScan')}
                className="w-4 h-4 rounded accent-emerald-500 cursor-pointer"
              />
            </label>

            <label className="flex items-center justify-between text-xs py-1 px-1 cursor-pointer">
              <span className="text-gray-300">A* Planned Path</span>
              <input 
                type="checkbox"
                checked={prefs.showPlannedPath}
                onChange={() => toggle('showPlannedPath')}
                className="w-4 h-4 rounded accent-emerald-500 cursor-pointer"
              />
            </label>

            <label className="flex items-center justify-between text-xs py-1 px-1 cursor-pointer">
              <span className="text-gray-300">Robot Odometry Trajectory</span>
              <input 
                type="checkbox"
                checked={prefs.showTrajectory}
                onChange={() => toggle('showTrajectory')}
                className="w-4 h-4 rounded accent-emerald-500 cursor-pointer"
              />
            </label>

            <label className="flex items-center justify-between text-xs py-1 px-1 cursor-pointer">
              <span className="text-gray-300">Camera FOV Cone</span>
              <input 
                type="checkbox"
                checked={prefs.showFovCone}
                onChange={() => toggle('showFovCone')}
                className="w-4 h-4 rounded accent-emerald-500 cursor-pointer"
              />
            </label>

            <label className="flex items-center justify-between text-xs py-1 px-1 cursor-pointer">
              <span className="text-gray-300">YOLO 3D Distance Tags</span>
              <input 
                type="checkbox"
                checked={prefs.showDistanceTags}
                onChange={() => toggle('showDistanceTags')}
                className="w-4 h-4 rounded accent-emerald-500 cursor-pointer"
              />
            </label>
          </div>
        </div>

        {/* Section 3: Reset Actions */}
        <div className="flex gap-2 pt-1">
          {resetMap && (
            <button
              onClick={() => { resetMap(); onClose(); }}
              className="flex-1 py-2 px-3 rounded-xl bg-white/5 hover:bg-white/10 text-xs text-gray-300 hover:text-white border border-white/10 flex items-center justify-center gap-1.5 transition-colors"
            >
              <Trash2 className="w-3.5 h-3.5 text-rose-400" />
              Reset Map
            </button>
          )}

          {resetPose && (
            <button
              onClick={() => { resetPose(); onClose(); }}
              className="flex-1 py-2 px-3 rounded-xl bg-white/5 hover:bg-white/10 text-xs text-gray-300 hover:text-white border border-white/10 flex items-center justify-center gap-1.5 transition-colors"
            >
              <RotateCcw className="w-3.5 h-3.5 text-cyan-400" />
              Reset Pose
            </button>
          )}
        </div>

      </div>
    </div>
  );
}
