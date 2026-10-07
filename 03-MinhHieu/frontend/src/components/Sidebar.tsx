import { 
  LayoutDashboard, Map, Video, ShieldAlert, Cpu, 
  Layers, Gauge, ChevronRight, 
  RotateCcw, Trash2, Ban, Eye, Radio, Sparkles
} from 'lucide-react';
import { UiPreferences } from '../hooks/useUiPreferences';
import { translations, Language } from '../i18n/translations';

export type ActiveView = 'OVERVIEW' | 'MAP' | 'CAMERA' | 'TELEOP' | 'DIAGNOSTICS';

interface SidebarProps {
  isOpen: boolean;
  activeView: ActiveView;
  setActiveView: (view: ActiveView) => void;
  prefs: UiPreferences;
  toggle: (key: keyof UiPreferences) => void;
  setSpeedLimit: (speed: number) => void;
  onOpenDeviceInspector: () => void;
  battery: number;
  cpuTemp?: number;
  isRealConnected?: boolean;
  emergencyStop: () => void;
  resetMap?: () => void;
  resetPose?: () => void;
  language?: Language;
}

export default function Sidebar({
  isOpen,
  activeView,
  setActiveView,
  prefs,
  toggle,
  setSpeedLimit,
  onOpenDeviceInspector,
  battery,
  cpuTemp = 42.5,
  isRealConnected = false,
  emergencyStop,
  resetMap,
  resetPose,
  language = 'vi'
}: SidebarProps) {
  if (!isOpen) return null;

  const t = translations[language];

  const menuItems = [
    { id: 'OVERVIEW', label: t.overview, icon: LayoutDashboard, desc: t.overviewDesc },
    { id: 'MAP', label: t.mapSlam, icon: Map, desc: t.mapSlamDesc },
    { id: 'CAMERA', label: t.cameraAi, icon: Video, desc: t.cameraAiDesc },
    { id: 'TELEOP', label: t.teleop, icon: ShieldAlert, desc: t.teleopDesc },
  ];

  return (
    <aside className="w-64 h-full bg-white dark:bg-[#141414] border-r border-gray-200/80 dark:border-[#262626] flex flex-col justify-between shrink-0 select-none z-30 shadow-[4px_0_24px_rgba(0,0,0,0.02)] transition-colors">
      {/* Top Section: Brand & Navigation Menu */}
      <div className="flex flex-col gap-4 p-4 overflow-y-auto custom-scrollbar">
        
        {/* Brand Header */}
        <div className="flex items-center gap-3 px-1 pb-2 border-b border-gray-100 dark:border-[#262626]">
          <div className="w-9 h-9 rounded-2xl bg-[#0071e3] dark:bg-[#1c1c1c] dark:border dark:border-[#262626] flex items-center justify-center shadow-sm">
            <Radio className="w-5 h-5 text-white dark:text-[#f97316]" />
          </div>
          <div className="flex flex-col">
            <span className="text-sm font-bold text-[#1d1d1f] dark:text-[#f4f4f5] tracking-tight">
              {t.appSubtitle}
            </span>
            <span className="text-xs text-[#6e6e73] dark:text-[#9ca3af]">
              {t.autonomousNav}
            </span>
          </div>
        </div>

        {/* Device Health Inspector Card - Truthful Real vs SIL status */}
        <button
          onClick={onOpenDeviceInspector}
          className={`p-3 rounded-2xl border transition-all text-left flex items-center justify-between group shadow-sm apple-btn ${
            isRealConnected
              ? 'bg-gradient-to-br from-blue-50/80 to-indigo-50/80 dark:from-[#1c1c1c] dark:to-[#141414] border-blue-100 dark:border-emerald-500/40'
              : 'bg-gradient-to-br from-amber-50/80 to-orange-50/80 dark:from-[#1c1c1c] dark:to-[#141414] border-amber-200/80 dark:border-[#262626]'
          }`}
        >
          <div className="flex items-center gap-2.5">
            <div className="w-8 h-8 rounded-xl bg-white dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex items-center justify-center shadow-xs">
              <Cpu className={`w-4 h-4 ${isRealConnected ? 'text-[#0071e3] dark:text-emerald-400' : 'text-amber-500 dark:text-[#fbbf24]'}`} />
            </div>
            <div className="flex flex-col">
              <span className="text-xs font-bold text-[#1d1d1f] dark:text-[#f4f4f5] group-hover:text-[#0071e3] dark:group-hover:text-[#f97316] transition-colors">
                {t.checkDevices}
              </span>
              <span className={`text-[11px] font-semibold flex items-center gap-1 ${
                isRealConnected ? 'text-emerald-600 dark:text-emerald-400' : 'text-amber-700 dark:text-[#fbbf24]'
              }`}>
                <span className={`w-1.5 h-1.5 rounded-full ${
                  isRealConnected ? 'bg-emerald-500 dark:bg-emerald-400 animate-pulse' : 'bg-amber-500 dark:bg-[#fbbf24]'
                }`}></span>
                {isRealConnected ? t.realConnectedCount : t.simConnectedCount}
              </span>
            </div>
          </div>
          <ChevronRight className="w-4 h-4 text-gray-400 group-hover:text-[#0071e3] dark:group-hover:text-[#f97316] transition-transform group-hover:translate-x-0.5" />
        </button>

        {/* Section 1: Navigation Menu */}
        <div className="flex flex-col gap-1.5">
          <span className="text-xs font-semibold text-[#86868b] dark:text-[#71717a] uppercase tracking-wider px-1 mb-0.5">
            {t.menuCategories}
          </span>

          {menuItems.map(item => {
            const Icon = item.icon;
            const isActive = activeView === item.id;

            return (
              <button
                key={item.id}
                onClick={() => setActiveView(item.id as ActiveView)}
                className={`w-full p-2.5 rounded-2xl flex items-center justify-start gap-3 transition-all text-left group cursor-pointer active:scale-[0.98] ${
                  isActive
                    ? 'bg-[#0071e3] dark:bg-[#f97316] text-white dark:text-[#ffffff] shadow-sm border border-[#0071e3] dark:border-[#f97316]'
                    : 'bg-gray-50/60 dark:bg-[#1c1c1c] hover:bg-gray-100/90 dark:hover:bg-[#262626] border border-gray-200/60 dark:border-[#262626] text-[#1d1d1f] dark:text-[#f4f4f5]'
                }`}
              >
                <div className={`w-9 h-9 rounded-xl flex items-center justify-center shrink-0 transition-colors ${
                  isActive 
                    ? 'bg-white/20 dark:bg-white/20 text-white dark:text-[#ffffff]' 
                    : 'bg-white dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] text-[#0071e3] dark:text-[#9ca3af]'
                }`}>
                  <Icon className="w-4.5 h-4.5" />
                </div>
                <div className="flex flex-col min-w-0 text-left items-start">
                  <span className="text-xs font-bold truncate block text-left w-full">
                    {item.label}
                  </span>
                  <span className={`text-[11px] truncate block text-left w-full ${isActive ? 'text-blue-100 dark:text-white/80' : 'text-[#86868b] dark:text-[#71717a]'}`}>
                    {item.desc}
                  </span>
                </div>
              </button>
            );
          })}
        </div>

        {/* Section 2: Map & AI Layers Toggle */}
        <div className="flex flex-col gap-2 pt-2 border-t border-gray-100 dark:border-[#262626]">
          <span className="text-xs font-semibold text-[#86868b] dark:text-[#71717a] uppercase tracking-wider px-2 flex items-center gap-1.5">
            <Layers className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#9ca3af]" />
            <span>{t.mapLayers}</span>
          </span>

          <div className="flex flex-col gap-1.5 px-2">
            {[
              { key: 'showLidarScan', label: t.lidarScan },
              { key: 'showPlannedPath', label: t.plannedPath },
              { key: 'showTrajectory', label: t.odometry },
              { key: 'showGrid', label: t.grid1m },
              { key: 'showDistanceTags', label: t.yoloDist },
            ].map(layer => (
              <label 
                key={layer.key} 
                className="flex items-center justify-between text-xs text-[#1d1d1f] dark:text-[#f4f4f5] cursor-pointer hover:text-[#0071e3] dark:hover:text-[#f97316] transition-colors py-0.5"
              >
                <span>{layer.label}</span>
                <input
                  type="checkbox"
                  checked={prefs[layer.key as keyof UiPreferences] as boolean}
                  onChange={() => toggle(layer.key as keyof UiPreferences)}
                  className="w-4 h-4 rounded-md accent-[#0071e3] dark:accent-[#f97316] cursor-pointer"
                />
              </label>
            ))}
          </div>
        </div>

        {/* Section 3: Safe Speed Limit Slider */}
        <div className="flex flex-col gap-2 pt-2 border-t border-gray-100 dark:border-[#262626]">
          <div className="flex items-center justify-between px-2 text-xs">
            <span className="font-semibold text-[#86868b] dark:text-[#71717a] uppercase tracking-wider flex items-center gap-1.5">
              <Gauge className="w-3.5 h-3.5 text-amber-500 dark:text-[#fbbf24]" />
              <span>{t.speedLimit}</span>
            </span>
            <span className="font-mono font-bold text-[#0071e3] dark:text-[#f97316]">
              {prefs.speedLimit.toFixed(2)} m/s
            </span>
          </div>

          <div className="px-2">
            <input
              type="range"
              min="0.10"
              max="0.35"
              step="0.05"
              value={prefs.speedLimit}
              onChange={(e) => setSpeedLimit(parseFloat(e.target.value))}
              className="w-full h-1.5 bg-gray-200 dark:bg-[#1c1c1c] rounded-lg appearance-none cursor-pointer accent-[#0071e3] dark:accent-[#f97316]"
            />
            <div className="flex justify-between text-[10px] text-[#86868b] dark:text-[#71717a] mt-1 font-mono">
              <span>0.10m/s ({t.slow})</span>
              <span>0.35m/s ({t.fast})</span>
            </div>
          </div>
        </div>

        {/* Map Reset Actions */}
        <div className="grid grid-cols-2 gap-2 pt-1">
          {resetMap && (
            <button
              onClick={resetMap}
              title="Xóa bản đồ lưới và khôi phục ô trống"
              className="py-1.5 px-2 rounded-xl bg-gray-50 dark:bg-[#1c1c1c] hover:bg-rose-50 dark:hover:bg-rose-500/20 border border-gray-200 dark:border-[#262626] hover:border-rose-200 dark:hover:border-rose-500/40 text-xs font-semibold text-[#6e6e73] dark:text-[#9ca3af] hover:text-rose-600 dark:hover:text-rose-400 transition-colors flex items-center justify-center gap-1.5 apple-btn shadow-xs"
            >
              <Trash2 className="w-3.5 h-3.5 text-rose-500 dark:text-rose-400" />
              <span>{t.clearMap}</span>
            </button>
          )}

          {resetPose && (
            <button
              onClick={resetPose}
              title="Đặt lại tọa độ robot về (0,0)"
              className="py-1.5 px-2 rounded-xl bg-gray-50 dark:bg-[#1c1c1c] hover:bg-gray-100 dark:hover:bg-[#262626] border border-gray-200 dark:border-[#262626] hover:border-gray-300 dark:hover:border-[#333333] text-xs font-semibold text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#0071e3] dark:hover:text-[#f4f4f5] transition-colors flex items-center justify-center gap-1.5 apple-btn shadow-xs"
            >
              <RotateCcw className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#9ca3af]" />
              <span>{t.resetOrigin}</span>
            </button>
          )}
        </div>

      </div>

      {/* Bottom Footer Section: Quick Telemetry & Emergency Stop */}
      <div className="p-3 border-t border-gray-200/80 dark:border-[#262626] bg-gray-50/70 dark:bg-[#141414] flex flex-col gap-2">
        <div className="flex items-center justify-between text-xs px-1 text-[#6e6e73] dark:text-[#9ca3af] font-mono">
          <span>{t.batteryLabel}: <strong className="text-[#1d1d1f] dark:text-[#22c55e] font-bold">{battery.toFixed(0)}%</strong></span>
          <span>{t.tempLabel}: <strong className="text-[#1d1d1f] dark:text-[#fbbf24] font-bold">{cpuTemp.toFixed(1)}°C</strong></span>
        </div>

        <button
          onClick={emergencyStop}
          className="w-full py-2.5 rounded-xl bg-[#ff3b30] hover:bg-[#ff453a] dark:bg-[#ef4444] dark:hover:bg-[#dc2626] text-white font-bold text-xs shadow-sm transition-all apple-btn flex items-center justify-center gap-2"
        >
          <Ban className="w-4 h-4 animate-pulse" />
          <span>{t.emergencyStop} (SPACE)</span>
        </button>
      </div>
    </aside>
  );
}
