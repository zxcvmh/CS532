import { useEffect, useState, useRef, useCallback } from 'react';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Ban, ShieldAlert, Zap, Gamepad2, Keyboard } from 'lucide-react';
import VirtualJoystick from './VirtualJoystick';
import { translations, Language } from '../i18n/translations';
import { ControlInputMode } from '../hooks/useUiPreferences';

interface RobotControlProps {
  mode: 'MANUAL' | 'AUTONOMOUS';
  sendManualCommand: (cmd: string, speed?: number) => void;
  sendVelocity?: (linear: number, angular: number) => void;
  emergencyStop: () => void;
  speedLimit?: number;
  language?: Language;
  controlInput?: ControlInputMode;
  onToggleControlInput?: (mode: ControlInputMode) => void;
}

export default function RobotControl({
  mode,
  sendManualCommand,
  sendVelocity,
  emergencyStop,
  speedLimit = 0.25,
  language = 'vi',
  controlInput = 'WASD',
  onToggleControlInput
}: RobotControlProps) {
  const [activeKey, setActiveKey] = useState<string | null>(null);
  const [localInputMode, setLocalInputMode] = useState<ControlInputMode>(controlInput);
  const driveIntervalRef = useRef<number | null>(null);

  const t = translations[language];
  const activeInputMode = onToggleControlInput ? controlInput : localInputMode;

  const handleSetInputMode = (newMode: ControlInputMode) => {
    if (onToggleControlInput) {
      onToggleControlInput(newMode);
    } else {
      setLocalInputMode(newMode);
    }
  };

  const startDriving = useCallback((cmd: string, keyName: string) => {
    setActiveKey(keyName);
    sendManualCommand(cmd, speedLimit);
    if (driveIntervalRef.current) clearInterval(driveIntervalRef.current);
    driveIntervalRef.current = window.setInterval(() => {
      sendManualCommand(cmd, speedLimit);
    }, 120);
  }, [sendManualCommand, speedLimit]);

  const stopDriving = useCallback(() => {
    if (driveIntervalRef.current) {
      clearInterval(driveIntervalRef.current);
      driveIntervalRef.current = null;
    }
    setActiveKey(null);
    sendManualCommand('STOP');
  }, [sendManualCommand]);

  // Keyboard Event Listeners (W, A, S, D, Spacebar)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Spacebar always triggers emergency stop
      if (e.code === 'Space') {
        e.preventDefault();
        emergencyStop();
        stopDriving();
        return;
      }

      if (mode !== 'MANUAL' || e.repeat) return;
      
      switch (e.key.toLowerCase()) {
        case 'w': startDriving('FORWARD', 'W'); break;
        case 's': startDriving('BACKWARD', 'S'); break;
        case 'a': startDriving('ROTATE_LEFT', 'A'); break;
        case 'd': startDriving('ROTATE_RIGHT', 'D'); break;
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (['w', 'a', 's', 'd'].includes(k)) {
        stopDriving();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      stopDriving();
    };
  }, [mode, startDriving, stopDriving, emergencyStop]);

  const handleJoystickVelocity = useCallback((linear: number, angular: number) => {
    if (sendVelocity) {
      sendVelocity(linear, angular);
    } else {
      // Fallback to discrete commands if sendVelocity is not provided
      if (linear > 0.05) sendManualCommand('FORWARD', Math.abs(linear));
      else if (linear < -0.05) sendManualCommand('BACKWARD', Math.abs(linear));
      else if (angular > 0.1) sendManualCommand('ROTATE_LEFT');
      else if (angular < -0.1) sendManualCommand('ROTATE_RIGHT');
      else sendManualCommand('STOP');
    }
  }, [sendVelocity, sendManualCommand]);

  return (
    <div className="apple-card p-3.5 h-full flex flex-col justify-between select-none">
      {/* Header with Title and Mode Switcher (WASD vs Joystick) */}
      <div className="flex items-center justify-between border-b border-gray-100 dark:border-[#262626] pb-2">
        <div className="flex items-center gap-2">
          <div className="w-6 h-6 rounded-lg bg-blue-50 dark:bg-[#1c1c1c] border border-blue-100 dark:border-[#262626] flex items-center justify-center">
            <ShieldAlert className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#f97316]" />
          </div>
          <h3 className="text-xs font-bold text-[#1d1d1f] dark:text-[#f4f4f5] tracking-tight">
            {t.manualControl}
          </h3>
        </div>

        {/* Input Toggle (WASD vs Joystick) */}
        <div className="flex p-0.5 rounded-lg bg-gray-100 dark:bg-[#0d0c0c] border border-gray-200/60 dark:border-[#262626]">
          <button
            data-testid="control-tab-wasd"
            onClick={() => handleSetInputMode('WASD')}
            title="Điều khiển bằng phím WASD"
            className={`px-2 py-0.5 rounded-md text-[11px] font-semibold flex items-center gap-1 transition-all ${
              activeInputMode === 'WASD'
                ? 'bg-white dark:bg-[#262626] text-[#1d1d1f] dark:text-[#f4f4f5] shadow-xs'
                : 'text-gray-500 dark:text-[#9ca3af] hover:text-gray-900 dark:hover:text-[#f4f4f5]'
            }`}
          >
            <Keyboard className="w-3 h-3" />
            <span>WASD</span>
          </button>
          <button
            data-testid="control-tab-joystick"
            onClick={() => handleSetInputMode('JOYSTICK')}
            title="Điều khiển bằng cần gạt cảm ứng 360°"
            className={`px-2 py-0.5 rounded-md text-[11px] font-semibold flex items-center gap-1 transition-all ${
              activeInputMode === 'JOYSTICK'
                ? 'bg-[#0071e3] dark:bg-[#f97316] text-white dark:text-[#ffffff] shadow-xs'
                : 'text-gray-500 dark:text-[#9ca3af] hover:text-gray-900 dark:hover:text-[#f4f4f5]'
            }`}
          >
            <Gamepad2 className="w-3 h-3" />
            <span>Joystick</span>
          </button>
        </div>
      </div>

      {/* Main Body: Either Compact WASD or Virtual Joystick */}
      <div className="my-auto py-1 flex items-center justify-center">
        {activeInputMode === 'WASD' ? (
          <div className="flex flex-col items-center gap-1">
            {/* Key W */}
            <button
              onMouseDown={() => startDriving('FORWARD', 'W')}
              onMouseUp={stopDriving}
              onMouseLeave={stopDriving}
              onTouchStart={() => startDriving('FORWARD', 'W')}
              onTouchEnd={stopDriving}
              disabled={mode !== 'MANUAL'}
              className={`w-10 h-10 rounded-xl border flex flex-col items-center justify-center transition-all apple-btn ${
                activeKey === 'W'
                  ? 'bg-[#0071e3] dark:bg-[#f97316] border-[#0071e3] dark:border-[#f97316] text-white dark:text-[#ffffff] shadow-inner scale-95'
                  : 'bg-white dark:bg-[#1c1c1c] hover:bg-gray-50 dark:hover:bg-[#262626] border-gray-200/90 dark:border-[#262626] shadow-xs text-[#1d1d1f] dark:text-[#f4f4f5]'
              } ${mode !== 'MANUAL' ? 'opacity-40 cursor-not-allowed' : 'cursor-pointer'}`}
            >
              <ArrowUp className="w-3.5 h-3.5" />
              <span className="text-[9px] font-mono font-bold -mt-0.5">W</span>
            </button>

            {/* Row A, S, D */}
            <div className="flex gap-1">
              <button
                onMouseDown={() => startDriving('ROTATE_LEFT', 'A')}
                onMouseUp={stopDriving}
                onMouseLeave={stopDriving}
                onTouchStart={() => startDriving('ROTATE_LEFT', 'A')}
                onTouchEnd={stopDriving}
                disabled={mode !== 'MANUAL'}
                className={`w-10 h-10 rounded-xl border flex flex-col items-center justify-center transition-all apple-btn ${
                  activeKey === 'A'
                    ? 'bg-[#0071e3] dark:bg-[#f97316] border-[#0071e3] dark:border-[#f97316] text-white dark:text-[#ffffff] shadow-inner scale-95'
                    : 'bg-white dark:bg-[#1c1c1c] hover:bg-gray-50 dark:hover:bg-[#262626] border-gray-200/90 dark:border-[#262626] shadow-xs text-[#1d1d1f] dark:text-[#f4f4f5]'
                } ${mode !== 'MANUAL' ? 'opacity-40 cursor-not-allowed' : 'cursor-pointer'}`}
              >
                <ArrowLeft className="w-3.5 h-3.5" />
                <span className="text-[9px] font-mono font-bold -mt-0.5">A</span>
              </button>

              <button
                onMouseDown={() => startDriving('BACKWARD', 'S')}
                onMouseUp={stopDriving}
                onMouseLeave={stopDriving}
                onTouchStart={() => startDriving('BACKWARD', 'S')}
                onTouchEnd={stopDriving}
                disabled={mode !== 'MANUAL'}
                className={`w-10 h-10 rounded-xl border flex flex-col items-center justify-center transition-all apple-btn ${
                  activeKey === 'S'
                    ? 'bg-[#0071e3] dark:bg-[#f97316] border-[#0071e3] dark:border-[#f97316] text-white dark:text-[#ffffff] shadow-inner scale-95'
                    : 'bg-white dark:bg-[#1c1c1c] hover:bg-gray-50 dark:hover:bg-[#262626] border-gray-200/90 dark:border-[#262626] shadow-xs text-[#1d1d1f] dark:text-[#f4f4f5]'
                } ${mode !== 'MANUAL' ? 'opacity-40 cursor-not-allowed' : 'cursor-pointer'}`}
              >
                <ArrowDown className="w-3.5 h-3.5" />
                <span className="text-[9px] font-mono font-bold -mt-0.5">S</span>
              </button>

              <button
                onMouseDown={() => startDriving('ROTATE_RIGHT', 'D')}
                onMouseUp={stopDriving}
                onMouseLeave={stopDriving}
                onTouchStart={() => startDriving('ROTATE_RIGHT', 'D')}
                onTouchEnd={stopDriving}
                disabled={mode !== 'MANUAL'}
                className={`w-10 h-10 rounded-xl border flex flex-col items-center justify-center transition-all apple-btn ${
                  activeKey === 'D'
                    ? 'bg-[#0071e3] dark:bg-[#f97316] border-[#0071e3] dark:border-[#f97316] text-white dark:text-[#ffffff] shadow-inner scale-95'
                    : 'bg-white dark:bg-[#1c1c1c] hover:bg-gray-50 dark:hover:bg-[#262626] border-gray-200/90 dark:border-[#262626] shadow-xs text-[#1d1d1f] dark:text-[#f4f4f5]'
                } ${mode !== 'MANUAL' ? 'opacity-40 cursor-not-allowed' : 'cursor-pointer'}`}
              >
                <ArrowRight className="w-3.5 h-3.5" />
                <span className="text-[9px] font-mono font-bold -mt-0.5">D</span>
              </button>
            </div>
          </div>
        ) : (
          <VirtualJoystick
            onVelocityChange={handleJoystickVelocity}
            onStop={stopDriving}
            speedLimit={speedLimit}
            disabled={mode !== 'MANUAL'}
          />
        )}
      </div>

      {/* Emergency Stop Button Underneath WASD/Joystick */}
      <div className="pt-2 border-t border-gray-100 dark:border-[#262626] flex flex-col gap-1.5">
        <button
          onClick={() => {
            emergencyStop();
            stopDriving();
          }}
          title={t.spacebarHint}
          className="w-full py-2.5 px-3 rounded-xl bg-[#ff3b30] hover:bg-[#ff453a] dark:bg-[#ef4444] dark:hover:bg-[#dc2626] text-white font-bold text-xs shadow-[0_2px_10px_rgba(255,59,48,0.25)] hover:shadow-[0_4px_16px_rgba(255,59,48,0.35)] transition-all apple-btn flex items-center justify-center gap-2"
        >
          <Ban className="w-4 h-4 text-white animate-pulse" />
          <span>{t.emergencyStop}</span>
          <span className="text-[10px] font-mono bg-black/20 px-1.5 py-0.5 rounded text-white/90">
            SPACE
          </span>
        </button>

        <div className="flex justify-between items-center text-[10px] text-[#6e6e73] dark:text-[#9ca3af] px-1">
          <span>{activeInputMode === 'WASD' ? t.wasdHint : t.joystickHint}</span>
          <span className="font-mono text-[#0071e3] dark:text-[#f97316] font-semibold">{speedLimit.toFixed(2)}m/s</span>
        </div>
      </div>
    </div>
  );
}
