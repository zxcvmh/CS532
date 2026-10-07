import { NavigationStatus, NavigationState } from '../types/navigation';
import { Navigation, MapPin, Clock, XCircle } from 'lucide-react';
import { Language, translations } from '../i18n/translations';

interface NavigationPanelProps {
  status: NavigationStatus;
  mode: 'MANUAL' | 'AUTONOMOUS';
  cancelGoal: () => void;
  emergencyStop: () => void;
  language?: Language;
}

export default function NavigationPanel({
  status,
  mode,
  cancelGoal,
  emergencyStop,
  language = 'vi'
}: NavigationPanelProps) {
  const t = translations[language];

  const getStatusConfig = (st: NavigationState) => {
    switch (st) {
      case 'PLANNING':
        return {
          label: t.navStatusPlanning,
          color: 'text-[#0071e3] dark:text-[#f97316]',
          bg: 'bg-blue-50 dark:bg-[#1c1c1c]',
          border: 'border-blue-200 dark:border-[#262626]'
        };
      case 'NAVIGATING':
        return {
          label: t.navStatusNavigating,
          color: 'text-emerald-700 dark:text-[#4ade80]',
          bg: 'bg-emerald-50 dark:bg-emerald-950/30',
          border: 'border-emerald-200 dark:border-[#4ade80]/30'
        };
      case 'REACHED':
        return {
          label: t.navStatusReached,
          color: 'text-emerald-800 dark:text-[#4ade80]',
          bg: 'bg-emerald-100 dark:bg-emerald-950/40',
          border: 'border-emerald-300 dark:border-[#4ade80]/40'
        };
      case 'BLOCKED':
        return {
          label: t.navStatusBlocked,
          color: 'text-amber-700 dark:text-[#fbbf24]',
          bg: 'bg-amber-50 dark:bg-amber-950/30',
          border: 'border-amber-200 dark:border-[#fbbf24]/30'
        };
      case 'CANCELLED':
        return {
          label: t.navStatusCancelled,
          color: 'text-[#6e6e73] dark:text-[#9ca3af]',
          bg: 'bg-gray-100 dark:bg-[#141414]',
          border: 'border-gray-200 dark:border-[#262626]'
        };
      case 'ERROR':
        return {
          label: t.navStatusError,
          color: 'text-rose-700 dark:text-[#f87171]',
          bg: 'bg-rose-50 dark:bg-rose-950/30',
          border: 'border-rose-200 dark:border-[#f87171]/30'
        };
      case 'IDLE':
      default:
        return {
          label: t.navStatusIdle,
          color: 'text-[#1d1d1f] dark:text-[#9ca3af]',
          bg: 'bg-gray-100 dark:bg-[#141414]',
          border: 'border-gray-200 dark:border-[#262626]'
        };
    }
  };

  const currentStatus = getStatusConfig(status.status);
  const progressPercent = Math.min(100, Math.max(0, Math.round(status.progress * 100)));
  const isNavigating = ['NAVIGATING', 'PLANNING'].includes(status.status);

  // Approximate ETA
  const etaSeconds = status.velocity > 0.05 
    ? Math.round(status.distanceRemaining / status.velocity) 
    : 0;

  return (
    <div className="apple-card p-4 h-full flex flex-col justify-between select-none">
      {/* Panel Header */}
      <div className="flex items-center justify-between border-b border-gray-100 dark:border-[#262626] pb-2">
        <div className="flex items-center gap-2.5">
          <div className="w-7 h-7 rounded-xl bg-blue-50 dark:bg-[#1c1c1c] border border-blue-100 dark:border-[#262626] flex items-center justify-center">
            <Navigation className="w-4 h-4 text-[#0071e3] dark:text-[#f97316]" />
          </div>
          <h3 className="text-sm font-bold text-[#1d1d1f] dark:text-[#f4f4f5] tracking-tight">{t.navTitle}</h3>
        </div>

        {/* State Badge with generous padding */}
        <div className={`flex items-center gap-2 px-3 py-1 rounded-full text-xs font-semibold border transition-all ${currentStatus.bg} ${currentStatus.color} ${currentStatus.border}`}>
          <span className={`w-2 h-2 rounded-full ${isNavigating ? 'bg-[#34c759] dark:bg-[#4ade80] animate-ping' : 'bg-current'}`}></span>
          <span>{currentStatus.label}</span>
        </div>
      </div>

      {/* Target Coordinates & Metric Info */}
      <div className="grid grid-cols-2 gap-2.5 my-2">
        {/* Goal Metric */}
        <div className="p-3 rounded-2xl bg-gray-50/80 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col justify-center">
          <div className="flex items-center gap-1.5 text-xs text-[#86868b] dark:text-[#9ca3af] mb-1">
            <MapPin className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#f97316]" />
            <span className="font-medium">{t.targetCoord}</span>
          </div>
          <div className="text-sm font-mono font-bold text-[#1d1d1f] dark:text-[#f4f4f5]">
            {status.goalX !== null && status.goalY !== null ? (
              <span className="text-[#0071e3] dark:text-[#f97316]">({status.goalX.toFixed(2)}, {status.goalY.toFixed(2)}m)</span>
            ) : (
              <span className="text-[#86868b] dark:text-[#9ca3af] font-normal italic">{t.noGoalSelected}</span>
            )}
          </div>
        </div>

        {/* Distance Remaining & ETA */}
        <div className="p-3 rounded-2xl bg-gray-50/80 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col justify-center">
          <div className="flex items-center justify-between text-xs text-[#86868b] dark:text-[#9ca3af] mb-1">
            <span className="flex items-center gap-1.5 font-medium">
              <Clock className="w-3.5 h-3.5 text-indigo-500 dark:text-[#9ca3af]" /> {t.distanceRemaining}
            </span>
            <span className="font-mono text-[#1d1d1f] dark:text-[#f4f4f5] font-bold">
              {status.distanceRemaining.toFixed(2)}m
            </span>
          </div>
          <div className="text-xs text-[#6e6e73] dark:text-[#9ca3af] font-mono flex justify-between">
            <span>{t.eta}</span>
            <span className="text-[#1d1d1f] dark:text-[#f4f4f5] font-bold">
              {etaSeconds > 0 ? `~${etaSeconds} ${t.seconds}` : '--'}
            </span>
          </div>
        </div>
      </div>

      {/* Progress Bar */}
      <div className="flex flex-col gap-1.5 pt-1">
        <div className="flex justify-between items-center text-xs text-[#6e6e73] dark:text-[#9ca3af] font-semibold">
          <span>{t.progress}</span>
          <span className="font-mono text-[#0071e3] dark:text-[#f97316] font-bold">{progressPercent}%</span>
        </div>
        <div className="w-full h-2 rounded-full bg-gray-100 dark:bg-[#262626] overflow-hidden">
          <div
            className="h-full rounded-full bg-gradient-to-r from-[#0071e3] to-[#34c759] dark:from-[#f97316] dark:to-[#4ade80] transition-all duration-500"
            style={{ width: `${progressPercent}%` }}
          />
        </div>
      </div>

      {/* Action Footer */}
      <div className="flex items-center justify-between pt-2 border-t border-gray-100 dark:border-[#262626]">
        <span className="text-xs text-[#6e6e73] dark:text-[#9ca3af]">
          {t.modeLabel}: <strong className={mode === 'AUTONOMOUS' ? 'text-[#0071e3] dark:text-[#f97316]' : 'text-[#1d1d1f] dark:text-[#f4f4f5]'}>{mode}</strong>
        </span>

        {isNavigating && (
          <button
            onClick={cancelGoal}
            className="flex items-center gap-1.5 px-3 py-1.5 rounded-full bg-rose-50 dark:bg-[#f87171]/20 hover:bg-rose-100 dark:hover:bg-[#f87171]/30 border border-rose-200 dark:border-[#f87171]/40 text-xs font-semibold text-[#ff3b30] dark:text-[#f87171] transition-all apple-btn"
          >
            <XCircle className="w-3.5 h-3.5" />
            <span>{t.cancelGoal}</span>
          </button>
        )}
      </div>
    </div>
  );
}
