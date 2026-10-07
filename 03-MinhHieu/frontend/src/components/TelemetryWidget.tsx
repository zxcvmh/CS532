import React, { useState } from 'react';
import { RobotPose, SensorHealth } from '../types/robot';
import { 
  Battery, Gauge, Thermometer, 
  ShieldAlert, Activity, Compass,
  Zap, Cpu, Wifi, Radio, Clock
} from 'lucide-react';
import { Language, translations } from '../i18n/translations';

interface TelemetryWidgetProps {
  battery: number;
  pose: React.MutableRefObject<RobotPose>;
  cpuTemp?: number;
  linearSpeed?: number;
  angularSpeed?: number;
  nearestObstacle?: number | null;
  sensors?: SensorHealth[];
  language?: Language;
}

type TelemetryTab = 'OVERVIEW' | 'POWER' | 'ROS2';

export default function TelemetryWidget({
  battery,
  pose,
  cpuTemp = 42.5,
  linearSpeed = 0,
  angularSpeed = 0,
  nearestObstacle = null,
  sensors = [],
  language = 'vi'
}: TelemetryWidgetProps) {
  const [activeTab, setActiveTab] = useState<TelemetryTab>('OVERVIEW');
  const t = translations[language];
  const currentPose = pose.current;
  const thetaDeg = Math.round(((currentPose.theta * 180) / Math.PI) % 360);

  // Circular Battery Ring calculations (Apple Watch style)
  const radius = 28;
  const circumference = 2 * Math.PI * radius;
  const strokeDashoffset = circumference - (Math.min(100, Math.max(0, battery)) / 100) * circumference;

  const batteryColor = 
    battery > 50 ? '#4ade80' : // Refined Emerald
    battery > 20 ? '#fbbf24' : // Warm Amber
    '#f87171';                 // Coral Red

  // Proximity Alert logic
  const isCritical = nearestObstacle !== null && nearestObstacle < 0.20;
  const isCaution = nearestObstacle !== null && nearestObstacle >= 0.20 && nearestObstacle < 0.50;

  // Electrical INA219 Calculations
  const isMoving = Math.abs(linearSpeed) > 0.02 || Math.abs(angularSpeed) > 0.05;
  const busVoltage = Number((9.6 + (battery / 100) * 2.8).toFixed(2)); // 3S Li-ion ~9.6V to 12.4V
  const currentDraw = isMoving ? 1.48 : 0.82; // Amperes
  const powerWatt = Number((busVoltage * currentDraw).toFixed(1));
  const runtimeHours = Math.max(0.2, (battery / 100) * 2.5);
  const runtimeHoursPart = Math.floor(runtimeHours);
  const runtimeMinsPart = Math.round((runtimeHours - runtimeHoursPart) * 60);

  // Wheel speeds (differential drive kinematics with track width 0.14m)
  const trackWidth = 0.14;
  const vLeft = Number((linearSpeed - (angularSpeed * trackWidth) / 2).toFixed(2));
  const vRight = Number((linearSpeed + (angularSpeed * trackWidth) / 2).toFixed(2));

  return (
    <div className="apple-card p-3.5 h-full flex flex-col justify-between select-none">
      {/* Widget Header with Tab Selector */}
      <div className="flex items-center justify-between border-b border-gray-100 dark:border-[#262626] pb-2">
        <div className="flex items-center gap-2">
          <div className="w-6 h-6 rounded-lg bg-blue-50 dark:bg-[#1c1c1c] border border-blue-100 dark:border-[#262626] flex items-center justify-center">
            <Activity className="w-3.5 h-3.5 text-[#0071e3] dark:text-[#f97316]" />
          </div>
          <h3 className="text-xs font-bold text-[#1d1d1f] dark:text-[#f4f4f5] tracking-tight">
            {t.telemetryTitle}
          </h3>
        </div>

        {/* Tab Pills */}
        <div className="flex p-0.5 rounded-lg bg-gray-100 dark:bg-[#0d0c0c] border border-gray-200/80 dark:border-[#262626] text-[11px]">
          <button
            onClick={() => setActiveTab('OVERVIEW')}
            className={`px-2 py-0.5 rounded-md font-semibold transition-all ${
              activeTab === 'OVERVIEW'
                ? 'bg-white dark:bg-[#262626] text-[#1d1d1f] dark:text-[#f4f4f5] shadow-xs'
                : 'text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5]'
            }`}
          >
            {t.tabOverview}
          </button>
          <button
            onClick={() => setActiveTab('POWER')}
            className={`px-2 py-0.5 rounded-md font-semibold transition-all ${
              activeTab === 'POWER'
                ? 'bg-white dark:bg-[#262626] text-[#1d1d1f] dark:text-[#f4f4f5] shadow-xs'
                : 'text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5]'
            }`}
          >
            {t.tabPowerCompute}
          </button>
          <button
            onClick={() => setActiveTab('ROS2')}
            className={`px-2 py-0.5 rounded-md font-semibold transition-all ${
              activeTab === 'ROS2'
                ? 'bg-white dark:bg-[#262626] text-[#1d1d1f] dark:text-[#f4f4f5] shadow-xs'
                : 'text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5]'
            }`}
          >
            {t.tabRos2}
          </button>
        </div>
      </div>

      {/* TAB 1: OVERVIEW */}
      {activeTab === 'OVERVIEW' && (
        <>
          {/* Main Stats Row: Circular Battery Ring + Speedometer */}
          <div className="flex items-center justify-between gap-2.5 my-auto py-1">
            {/* Apple Watch style Circular Battery Ring */}
            <div className="flex items-center gap-3">
              <div className="relative w-14 h-14 flex items-center justify-center shrink-0">
                <svg className="w-full h-full -rotate-90 transform" viewBox="0 0 70 70">
                  <circle
                    cx="35"
                    cy="35"
                    r={radius}
                    className="stroke-gray-100 dark:stroke-[#262626]"
                    strokeWidth="5"
                    fill="transparent"
                  />
                  <circle
                    cx="35"
                    cy="35"
                    r={radius}
                    stroke={batteryColor}
                    strokeWidth="5.5"
                    strokeLinecap="round"
                    fill="transparent"
                    style={{
                      strokeDasharray: circumference,
                      strokeDashoffset: strokeDashoffset,
                      transition: 'stroke-dashoffset 0.6s cubic-bezier(0.16, 1, 0.3, 1)'
                    }}
                  />
                </svg>
                <div className="absolute inset-0 flex flex-col items-center justify-center">
                  <span className="text-xs font-bold font-mono text-[#1d1d1f] dark:text-[#f4f4f5]">
                    {Math.round(battery)}%
                  </span>
                  <Battery className="w-3 h-3 text-[#86868b] dark:text-[#9ca3af] -mt-0.5" />
                </div>
              </div>

              <div className="flex flex-col">
                <span className="text-[11px] text-[#86868b] dark:text-[#9ca3af] font-medium">{t.pinLiIon}</span>
                <span className="text-xs font-bold text-[#1d1d1f] dark:text-[#f4f4f5]">
                  {battery > 20 ? t.stableVoltage : t.needCharge}
                </span>
                <span className="text-[10px] text-[#6e6e73] dark:text-[#9ca3af] font-mono">{busVoltage}V • {powerWatt}W</span>
              </div>
            </div>

            {/* Speedometer Stats (Linear & Angular) */}
            <div className="flex flex-col gap-1 p-2 rounded-xl bg-gray-50/80 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] min-w-[120px]">
              <div className="flex justify-between items-center text-[11px]">
                <span className="text-[#6e6e73] dark:text-[#9ca3af] flex items-center gap-1 font-medium">
                  <Gauge className="w-3 h-3 text-[#0071e3] dark:text-[#9ca3af]" /> {t.speedV}
                </span>
                <span className="font-mono font-bold text-[#0071e3] dark:text-[#f4f4f5]">
                  {Math.abs(linearSpeed).toFixed(2)} m/s
                </span>
              </div>
              <div className="flex justify-between items-center text-[11px]">
                <span className="text-[#6e6e73] dark:text-[#9ca3af] flex items-center gap-1 font-medium">
                  <Compass className="w-3 h-3 text-indigo-500 dark:text-[#9ca3af]" /> {t.angularW}
                </span>
                <span className="font-mono font-bold text-indigo-600 dark:text-[#f4f4f5]">
                  {angularSpeed.toFixed(2)} rad/s
                </span>
              </div>
            </div>
          </div>

          {/* Bottom Row: Proximity Alert & Odometry Pose */}
          <div className="flex items-center justify-between gap-2 pt-2 border-t border-gray-100 dark:border-[#262626]">
            {/* Proximity / Obstacle Tag */}
            <div className={`flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-semibold border transition-all ${
              isCritical 
                ? 'bg-rose-50 dark:bg-rose-950/40 text-rose-700 dark:text-[#f87171] border-rose-200 dark:border-[#f87171]/40 animate-pulse'
                : isCaution
                ? 'bg-amber-50 dark:bg-amber-950/40 text-amber-700 dark:text-[#fbbf24] border-amber-200 dark:border-[#fbbf24]/40'
                : 'bg-emerald-50 dark:bg-emerald-950/40 text-emerald-700 dark:text-[#4ade80] border-emerald-200 dark:border-[#4ade80]/40'
            }`}>
              <ShieldAlert className={`w-3 h-3 ${isCritical ? 'text-[#ff3b30] dark:text-[#f87171]' : isCaution ? 'text-[#ff9500] dark:text-[#fbbf24]' : 'text-[#34c759] dark:text-[#4ade80]'}`} />
              <span className="truncate max-w-[140px]">
                {nearestObstacle !== null 
                  ? `${t.obstacleDistance}: ${nearestObstacle.toFixed(2)}m` 
                  : t.roadClear}
              </span>
            </div>

            {/* Coordinates Pill */}
            <div className="text-[11px] font-mono text-[#6e6e73] dark:text-[#9ca3af] flex items-center gap-2">
              <span>X: <strong className="text-[#1d1d1f] dark:text-[#f4f4f5]">{currentPose.x.toFixed(2)}</strong></span>
              <span>Y: <strong className="text-[#1d1d1f] dark:text-[#f4f4f5]">{currentPose.y.toFixed(2)}</strong></span>
              <span>θ: <strong className="text-[#1d1d1f] dark:text-[#f4f4f5]">{thetaDeg}°</strong></span>
            </div>
          </div>
        </>
      )}

      {/* TAB 2: POWER & JETSON COMPUTE */}
      {activeTab === 'POWER' && (
        <div className="flex flex-col gap-2 my-auto py-1">
          {/* INA219 Electrical Metrics Grid */}
          <div className="grid grid-cols-4 gap-1.5">
            <div className="p-1.5 rounded-lg bg-gray-50 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col">
              <span className="text-[9px] text-[#6e6e73] dark:text-[#9ca3af] font-medium flex items-center gap-1">
                <Zap className="w-2.5 h-2.5 text-[#ff9500] dark:text-[#fbbf24]" /> Volt
              </span>
              <span className="text-xs font-mono font-bold text-[#1d1d1f] dark:text-[#4ade80]">
                {busVoltage}V
              </span>
            </div>

            <div className="p-1.5 rounded-lg bg-gray-50 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col">
              <span className="text-[9px] text-[#6e6e73] dark:text-[#9ca3af] font-medium flex items-center gap-1">
                <Activity className="w-2.5 h-2.5 text-[#0071e3] dark:text-[#9ca3af]" /> Ampe
              </span>
              <span className="text-xs font-mono font-bold text-[#1d1d1f] dark:text-[#f4f4f5]">
                {currentDraw}A
              </span>
            </div>

            <div className="p-1.5 rounded-lg bg-gray-50 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col">
              <span className="text-[9px] text-[#6e6e73] dark:text-[#9ca3af] font-medium flex items-center gap-1">
                <Zap className="w-2.5 h-2.5 text-purple-500 dark:text-[#9ca3af]" /> Watt
              </span>
              <span className="text-xs font-mono font-bold text-[#1d1d1f] dark:text-[#f4f4f5]">
                {powerWatt}W
              </span>
            </div>

            <div className="p-1.5 rounded-lg bg-gray-50 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col">
              <span className="text-[9px] text-[#6e6e73] dark:text-[#9ca3af] font-medium flex items-center gap-1">
                <Clock className="w-2.5 h-2.5 text-emerald-500 dark:text-[#4ade80]" /> ETA
              </span>
              <span className="text-xs font-mono font-bold text-[#1d1d1f] dark:text-[#f4f4f5]">
                ~{runtimeHoursPart}h{runtimeMinsPart}m
              </span>
            </div>
          </div>

          {/* Jetson Nano Compute Resources */}
          <div className="grid grid-cols-2 gap-2 pt-1 border-t border-gray-100 dark:border-[#262626]">
            {/* RAM Progress */}
            <div className="flex flex-col gap-1 p-1.5 rounded-lg bg-gray-50/70 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626]">
              <div className="flex justify-between items-center text-[10px]">
                <span className="text-[#6e6e73] dark:text-[#9ca3af] flex items-center gap-1">
                  <Cpu className="w-2.5 h-2.5 text-blue-500 dark:text-[#9ca3af]" /> RAM Jetson
                </span>
                <span className="font-mono font-bold text-[#1d1d1f] dark:text-[#f4f4f5]">2.1 / 4.0 GB</span>
              </div>
              <div className="w-full h-1.5 bg-gray-200 dark:bg-[#262626] rounded-full overflow-hidden">
                <div className="h-full bg-[#0071e3] dark:bg-[#f97316] rounded-full" style={{ width: '52%' }}></div>
              </div>
            </div>

            {/* CPU & Wi-Fi */}
            <div className="flex flex-col gap-1 p-1.5 rounded-lg bg-gray-50/70 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626]">
              <div className="flex justify-between items-center text-[10px]">
                <span className="text-[#6e6e73] dark:text-[#9ca3af] flex items-center gap-1">
                  <Thermometer className="w-2.5 h-2.5 text-amber-500 dark:text-[#fbbf24]" /> CPU (4-Core)
                </span>
                <span className="font-mono font-bold text-[#1d1d1f] dark:text-[#fbbf24]">36% • {cpuTemp.toFixed(1)}°C</span>
              </div>
              <div className="flex justify-between items-center text-[10px]">
                <span className="text-[#6e6e73] dark:text-[#9ca3af] flex items-center gap-1">
                  <Wifi className="w-2.5 h-2.5 text-emerald-500 dark:text-[#4ade80]" /> Wi-Fi RSSI
                </span>
                <span className="font-mono font-bold text-emerald-600 dark:text-[#4ade80]">-56 dBm (5G)</span>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* TAB 3: ROS 2 TOPICS & IMU */}
      {activeTab === 'ROS2' && (
        <div className="flex flex-col gap-1.5 my-auto py-1">
          {/* ROS 2 Active Rates */}
          <div className="grid grid-cols-4 gap-1.5">
            <div className="p-1 rounded-lg bg-gray-50 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col items-center">
              <span className="text-[9px] text-[#6e6e73] dark:text-[#9ca3af] font-mono">/scan</span>
              <span className="text-xs font-mono font-bold text-emerald-600 dark:text-[#4ade80]">10.0 Hz</span>
            </div>
            <div className="p-1 rounded-lg bg-gray-50 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col items-center">
              <span className="text-[9px] text-[#6e6e73] dark:text-[#9ca3af] font-mono">/camera</span>
              <span className="text-xs font-mono font-bold text-[#0071e3] dark:text-[#f4f4f5]">15.0 Hz</span>
            </div>
            <div className="p-1 rounded-lg bg-gray-50 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col items-center">
              <span className="text-[9px] text-[#6e6e73] dark:text-[#9ca3af] font-mono">/cmd_vel</span>
              <span className="text-xs font-mono font-bold text-amber-600 dark:text-[#f4f4f5]">20.0 Hz</span>
            </div>
            <div className="p-1 rounded-lg bg-gray-50 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] flex flex-col items-center">
              <span className="text-[9px] text-[#6e6e73] dark:text-[#9ca3af] font-mono">/odom</span>
              <span className="text-xs font-mono font-bold text-amber-600 dark:text-[#fbbf24]">30.0 Hz</span>
            </div>
          </div>

          {/* Wheel speeds & IMU Accel */}
          <div className="flex items-center justify-between p-1.5 rounded-lg bg-gray-50/70 dark:bg-[#141414] border border-gray-200/60 dark:border-[#262626] text-[10px]">
            <div className="flex items-center gap-2">
              <span className="text-[#6e6e73] dark:text-[#9ca3af] font-medium">{t.wheelOdom}:</span>
              <span className="font-mono text-[#0071e3] dark:text-[#f4f4f5]">L: {vLeft} m/s</span>
              <span className="font-mono text-purple-600 dark:text-[#f4f4f5]">R: {vRight} m/s</span>
            </div>
            <div className="flex items-center gap-1.5 font-mono text-[#6e6e73] dark:text-[#9ca3af]">
              <span>IMU:</span>
              <span className="text-[#1d1d1f] dark:text-[#f4f4f5]">az: 9.81m/s²</span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
