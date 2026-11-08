import { useState, useEffect } from 'react';
import { Language } from '../i18n/translations';

export type Theme = 'light' | 'dark';
export type ControlInputMode = 'WASD' | 'JOYSTICK';

export interface UiPreferences {
  // Theme & Language
  theme: Theme;
  language: Language;
  controlInput: ControlInputMode;

  // Widget toggles
  showCamera: boolean;
  showMap: boolean;
  showTelemetry: boolean;
  showNavigation: boolean;
  showControls: boolean;
  showLogs: boolean;

  // Map layer toggles
  showGrid: boolean;
  showLidarScan: boolean;
  showTrajectory: boolean;
  showPlannedPath: boolean;
  showFovCone: boolean;

  // Camera settings
  showYoloBoxes: boolean;
  showDistanceTags: boolean;

  // Speed limit for teleop (m/s)
  speedLimit: number;

  // Focus / Maximize mode
  maximizedPanel: 'CAMERA' | 'MAP' | null;
}

const DEFAULT_PREFS: UiPreferences = {
  theme: 'light',
  language: 'vi',
  controlInput: 'WASD',
  showCamera: true,
  showMap: true,
  showTelemetry: true,
  showNavigation: true,
  showControls: true,
  showLogs: true,
  showGrid: true,
  showLidarScan: true,
  showTrajectory: true,
  showPlannedPath: true,
  showFovCone: true,
  showYoloBoxes: true,
  showDistanceTags: true,
  speedLimit: 0.25,
  maximizedPanel: null,
};

export function useUiPreferences() {
  const [prefs, setPrefs] = useState<UiPreferences>(() => {
    try {
      const saved = localStorage.getItem('cs532_ui_prefs');
      if (saved) {
        return { ...DEFAULT_PREFS, ...JSON.parse(saved) };
      }
    } catch {
      // ignore
    }
    return DEFAULT_PREFS;
  });

  // Apply theme to document element
  useEffect(() => {
    const isDark = prefs.theme === 'dark';
    document.documentElement.classList.toggle('dark', isDark);
    document.documentElement.setAttribute('data-theme', prefs.theme);
  }, [prefs.theme]);

  useEffect(() => {
    try {
      localStorage.setItem('cs532_ui_prefs', JSON.stringify(prefs));
    } catch {
      // ignore
    }
  }, [prefs]);

  const toggle = (key: keyof UiPreferences) => {
    setPrefs(prev => ({
      ...prev,
      [key]: !prev[key]
    }));
  };

  const setTheme = (theme: Theme) => {
    setPrefs(prev => ({ ...prev, theme }));
  };

  const toggleTheme = () => {
    setPrefs(prev => ({ ...prev, theme: prev.theme === 'light' ? 'dark' : 'light' }));
  };

  const setLanguage = (language: Language) => {
    setPrefs(prev => ({ ...prev, language }));
  };

  const toggleLanguage = () => {
    setPrefs(prev => ({ ...prev, language: prev.language === 'vi' ? 'en' : 'vi' }));
  };

  const setControlInput = (controlInput: ControlInputMode) => {
    setPrefs(prev => ({ ...prev, controlInput }));
  };

  const setSpeedLimit = (speed: number) => {
    setPrefs(prev => ({ ...prev, speedLimit: speed }));
  };

  const setMaximizedPanel = (panel: 'CAMERA' | 'MAP' | null) => {
    setPrefs(prev => ({ ...prev, maximizedPanel: panel }));
  };

  return {
    prefs,
    setPrefs,
    toggle,
    setTheme,
    toggleTheme,
    setLanguage,
    toggleLanguage,
    setControlInput,
    setSpeedLimit,
    setMaximizedPanel
  };
}
