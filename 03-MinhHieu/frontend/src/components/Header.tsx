import { useState, useEffect } from 'react';
import { 
  Bot, PanelLeft, Cpu, Activity, Clock, ShieldCheck, 
  Sun, Moon, Globe, AlertTriangle
} from 'lucide-react';
import { translations, Language } from '../i18n/translations';
import { Theme } from '../hooks/useUiPreferences';

interface HeaderProps {
  connected: boolean;
  rtt?: number | null;
  isRealConnected?: boolean;
  mode: 'MANUAL' | 'AUTONOMOUS';
  setMode: (mode: 'MANUAL' | 'AUTONOMOUS') => void;
  onToggleSidebar: () => void;
  onOpenDeviceInspector: () => void;
  isSidebarOpen: boolean;
  theme?: Theme;
  onToggleTheme?: () => void;
  language?: Language;
  onToggleLanguage?: () => void;
}

export default function Header({ 
  connected, 
  rtt, 
  isRealConnected = false, 
  mode, 
  setMode,
  onToggleSidebar,
  onOpenDeviceInspector,
  isSidebarOpen,
  theme = 'light',
  onToggleTheme,
  language = 'vi',
  onToggleLanguage
}: HeaderProps) {
  const [time, setTime] = useState(new Date());

  const t = translations[language];

  useEffect(() => {
    const timer = setInterval(() => setTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  return (
    <header className="h-16 px-5 bg-white/80 dark:bg-[#141414]/90 backdrop-blur-xl border-b border-gray-200/80 dark:border-[#262626] flex items-center justify-between select-none shrink-0 z-40 relative shadow-[0_1px_8px_rgba(0,0,0,0.03)] transition-colors">
      {/* Left: Sidebar Toggle, Brand & Status Badges */}
      <div className="flex items-center gap-4">
        {/* Sidebar Toggle Button */}
        <button
          onClick={onToggleSidebar}
          title={isSidebarOpen ? "Thu gọn danh mục" : "Mở danh mục"}
          className={`w-9 h-9 rounded-xl border flex items-center justify-center transition-all apple-btn ${
            isSidebarOpen 
              ? 'bg-gray-100 dark:bg-[#262626] border-gray-300 dark:border-[#333333] text-[#1d1d1f] dark:text-[#f4f4f5]' 
              : 'bg-white dark:bg-[#1c1c1c] border-gray-200 dark:border-[#262626] text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5] hover:border-gray-300 shadow-sm'
          }`}
        >
          <PanelLeft className="w-4 h-4" />
        </button>

        {/* Brand Title: JetBot Studio v1.0 / CS532 */}
        <div className="flex items-center gap-2.5">
          <div className="w-8 h-8 rounded-xl bg-[#0071e3]/10 dark:bg-[#1c1c1c] border border-[#0071e3]/20 dark:border-[#262626] flex items-center justify-center">
            <Bot className="w-4 h-4 text-[#0071e3] dark:text-[#f4f4f5]" />
          </div>
          <div className="flex flex-col">
            <span className="text-sm font-bold text-[#1d1d1f] dark:text-[#f4f4f5] tracking-tight flex items-center gap-2">
              {t.appTitle}
              <span className="text-[11px] font-mono px-2 py-0.5 rounded-full bg-gray-100 dark:bg-[#1c1c1c] text-[#6e6e73] dark:text-[#9ca3af] font-semibold border border-gray-200/60 dark:border-[#262626]">
                {t.appVersion}
              </span>
            </span>
            <span className="text-xs font-semibold text-[#6e6e73] dark:text-[#9ca3af]">
              {t.appSubtitle}
            </span>
          </div>
        </div>

        {/* Vertical Divider */}
        <div className="h-5 w-[1px] bg-gray-200 dark:bg-[#262626] hidden md:block"></div>

        {/* Connection Status Pill */}
        <div className="flex items-center gap-2.5">
          <div className={`flex items-center gap-2 px-3.5 py-1.5 rounded-full text-xs font-semibold border transition-all ${
            connected 
              ? 'bg-emerald-50 dark:bg-emerald-500/15 text-emerald-700 dark:text-emerald-400 border-emerald-200 dark:border-emerald-500/30' 
              : 'bg-rose-50 dark:bg-rose-500/15 text-rose-700 dark:text-rose-400 border-rose-200 dark:border-rose-500/30'
          }`}>
            <span className={`w-2 h-2 rounded-full ${
              connected ? 'bg-[#34c759] dark:bg-emerald-400 animate-pulse' : 'bg-[#ff3b30] dark:bg-rose-400'
            }`}></span>
            <span>{connected ? t.connected : t.disconnected}</span>
          </div>

          {/* RTT Ping Latency */}
          {connected && rtt !== undefined && rtt !== null && (
            <div className="flex items-center gap-1.5 px-3 py-1.5 rounded-full bg-gray-50 dark:bg-[#1c1c1c] border border-gray-200/80 dark:border-[#262626] text-xs font-mono text-[#1d1d1f] dark:text-[#f4f4f5]">
              <span className="text-[#86868b] dark:text-[#9ca3af] font-sans">{t.latency}:</span>
              <span className={rtt < 50 ? 'text-[#34c759] dark:text-emerald-400 font-bold' : 'text-[#ff9500] dark:text-[#fbbf24] font-bold'}>
                {rtt} ms
              </span>
            </div>
          )}

          {/* Hardware Source (Physical JetBot vs Truthful SIL Simulation) */}
          <div className={`hidden sm:flex items-center gap-1.5 px-3.5 py-1.5 rounded-full border text-xs font-semibold transition-all ${
            isRealConnected
              ? 'bg-emerald-50 dark:bg-emerald-500/15 text-emerald-700 dark:text-emerald-400 border-emerald-200 dark:border-emerald-500/30'
              : 'bg-gray-50 dark:bg-[#1c1c1c] text-[#1d1d1f] dark:text-[#fbbf24] border-gray-200/80 dark:border-[#262626]'
          }`}>
            <Cpu className={`w-3.5 h-3.5 ${isRealConnected ? 'text-[#34c759] dark:text-emerald-400' : 'text-amber-500 dark:text-[#fbbf24]'}`} />
            <span>
              {isRealConnected 
                ? t.physicalHardware 
                : `${t.silSimulation} (${t.realRobotDisconnected})`}
            </span>
          </div>
        </div>
      </div>

      {/* Right: Theme Toggle, Language Toggle, Diagnostic Trigger, Mode Switcher, Digital Clock */}
      <div className="flex items-center gap-2.5">
        {/* Theme Toggle (Light / Dark) */}
        {onToggleTheme && (
          <button
            onClick={onToggleTheme}
            data-testid="theme-toggle"
            title={theme === 'light' ? t.darkMode : t.lightMode}
            className="w-9 h-9 rounded-xl border border-gray-200 dark:border-[#262626] bg-white dark:bg-[#1c1c1c] flex items-center justify-center text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5] dark:hover:bg-[#262626] transition-all apple-btn shadow-xs"
          >
            {theme === 'light' ? (
              <Moon className="w-4 h-4 text-indigo-600" />
            ) : (
              <Sun className="w-4 h-4 text-[#f97316]" />
            )}
          </button>
        )}

        {/* Language Toggle (VI / EN) */}
        {onToggleLanguage && (
          <button
            onClick={onToggleLanguage}
            data-testid="language-toggle"
            title="Đổi ngôn ngữ / Change language"
            className="h-9 px-3 rounded-xl border border-gray-200 dark:border-[#262626] bg-white dark:bg-[#1c1c1c] flex items-center gap-1.5 text-xs font-semibold text-[#1d1d1f] dark:text-[#f4f4f5] hover:border-gray-300 dark:hover:border-[#333333] dark:hover:bg-[#262626] transition-all apple-btn shadow-xs"
          >
            <Globe className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#9ca3af]" />
            <span className="font-mono uppercase">{language}</span>
          </button>
        )}

        {/* Quick Device Inspector Button */}
        <button
          onClick={onOpenDeviceInspector}
          className={`flex items-center gap-1.5 px-3.5 py-1.5 rounded-full border text-xs font-semibold transition-all apple-btn ${
            isRealConnected
              ? 'bg-[#0071e3]/10 dark:bg-emerald-500/15 hover:bg-[#0071e3]/15 text-[#0071e3] dark:text-emerald-400 border-[#0071e3]/20 dark:border-emerald-500/30'
              : 'bg-amber-50 dark:bg-[#1c1c1c] hover:bg-amber-100 dark:hover:bg-[#262626] text-amber-700 dark:text-[#fbbf24] border-amber-200 dark:border-[#262626]'
          }`}
        >
          <ShieldCheck className="w-3.5 h-3.5" />
          <span>{t.checkDevices}</span>
        </button>

        {/* Apple Segmented Control for Mode */}
        <div className="flex p-1 rounded-xl bg-gray-100 dark:bg-[#0d0c0c] border border-gray-200/80 dark:border-[#262626]">
          <button
            onClick={() => setMode('MANUAL')}
            className={`px-3.5 py-1 text-xs font-semibold rounded-lg transition-all ${
              mode === 'MANUAL'
                ? 'bg-white dark:bg-[#262626] text-[#1d1d1f] dark:text-[#ffffff] shadow-sm'
                : 'text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5]'
            }`}
          >
            {t.manual}
          </button>
          <button
            onClick={() => setMode('AUTONOMOUS')}
            className={`px-3.5 py-1 text-xs font-semibold rounded-lg transition-all ${
              mode === 'AUTONOMOUS'
                ? 'bg-[#0071e3] dark:bg-[#f97316] text-white dark:text-[#ffffff] shadow-sm'
                : 'text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5]'
            }`}
          >
            {t.autonomous}
          </button>
        </div>

        {/* Digital Clock */}
        <div className="text-xs font-mono font-medium text-[#6e6e73] dark:text-[#9ca3af] px-1 hidden xl:block">
          {time.toLocaleTimeString()}
        </div>
      </div>
    </header>
  );
}
