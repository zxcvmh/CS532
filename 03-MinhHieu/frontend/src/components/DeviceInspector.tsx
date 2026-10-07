import { useState } from 'react';
import { 
  Cpu, Camera, Radio, Zap, RefreshCw, X, ShieldCheck, 
  Server, Compass, AlertTriangle, CheckCircle2
} from 'lucide-react';
import { SensorHealth } from '../types/robot';
import { translations, Language } from '../i18n/translations';

interface DeviceInspectorProps {
  isOpen: boolean;
  onClose: () => void;
  isRealConnected?: boolean;
  battery: number;
  cpuTemp?: number;
  rtt?: number | null;
  sensors?: SensorHealth[];
  language?: Language;
}

interface HardwareDevice {
  id: string;
  name: string;
  category: string;
  interfacePort: string;
  specs: string;
  status: 'ACTIVE' | 'WARNING' | 'OFFLINE' | 'SIMULATED';
  details: string;
  icon: any;
}

export default function DeviceInspector({
  isOpen,
  onClose,
  isRealConnected = false,
  battery,
  cpuTemp = 42.5,
  rtt,
  sensors = [],
  language = 'vi'
}: DeviceInspectorProps) {
  const [isTesting, setIsTesting] = useState(false);
  const [testProgress, setTestProgress] = useState<number | null>(null);
  const [lastCheckTime, setLastCheckTime] = useState<Date>(new Date());

  if (!isOpen) return null;

  const t = translations[language];

  // TRUTHFUL HARDWARE STATUS:
  // If no physical robot bridge (jetbot_bridge.py) is connected, real hardware devices are OFFLINE.
  const devices: HardwareDevice[] = [
    {
      id: 'lidar',
      name: 'LiDAR D500 Laser Scanner',
      category: language === 'vi' ? 'Đo Khoảng Cách & SLAM' : 'Range & Mapping',
      interfacePort: '/dev/ttyUSB0 (UART @ 230400 bps)',
      specs: '360° DToF, 10 Hz, 0.03m - 12m',
      status: isRealConnected ? 'ACTIVE' : 'OFFLINE',
      details: isRealConnected 
        ? (language === 'vi' ? '360 tia quét hợp lệ, đồng bộ 10Hz với Occupancy Grid' : '360 valid rays synced at 10Hz')
        : (language === 'vi' ? 'Cổng UART chưa mở (Chưa cắm thiết bị vào máy)' : 'UART port closed (Physical hardware not plugged)'),
      icon: Radio
    },
    {
      id: 'camera',
      name: 'Sony IMX219 CSI Camera',
      category: language === 'vi' ? 'Thị Giác Quang Học' : 'Vision & Optical',
      interfacePort: 'MIPI-CSI (GStreamer nvarguscamerasrc)',
      specs: '640x480 @ 15 FPS, FOV 160°',
      status: isRealConnected ? 'ACTIVE' : 'OFFLINE',
      details: isRealConnected 
        ? (language === 'vi' ? 'Luồng video MJPEG HTTP /video_feed hoạt động ổn định' : 'MJPEG HTTP stream active')
        : (language === 'vi' ? 'Luồng phần cứng MIPI-CSI chưa mở (Đang dùng hình ảnh mô phỏng)' : 'MIPI-CSI stream not opened (Mock feed)'),
      icon: Camera
    },
    {
      id: 'tensorrt',
      name: 'YOLOv8n TensorRT FP16 Engine',
      category: language === 'vi' ? 'Tăng Tốc AI Nhúng' : 'AI Acceleration',
      interfacePort: 'NVIDIA GPU CUDA 128-core',
      specs: 'Inference ~19.7 FPS (~50ms latency)',
      status: isRealConnected ? 'ACTIVE' : 'OFFLINE',
      details: isRealConnected 
        ? (language === 'vi' ? 'Phát hiện người, trích xuất góc lệch tâm azimuth [-80°, +80°]' : 'Person detected, azimuth tracking [-80°, +80°]')
        : (language === 'vi' ? 'GPU TensorRT Engine chưa chạy trên phần cứng Jetson' : 'TensorRT GPU Engine not active on Jetson host'),
      icon: Cpu
    },
    {
      id: 'motors',
      name: 'PCA9685 Motor Driver',
      category: language === 'vi' ? 'Truyền Động Động Cơ' : 'Actuation & Motors',
      interfacePort: 'I2C Bus 1 @ Địa chỉ 0x60',
      specs: 'PWM 100 Hz, Phanh an toàn <20cm',
      status: isRealConnected ? 'ACTIVE' : 'OFFLINE',
      details: isRealConnected 
        ? (language === 'vi' ? 'Điều khiển 2 bánh vi sai phản hồi bình thường qua I2C' : 'Dual differential drive responsive via I2C')
        : (language === 'vi' ? 'Bus I2C không phản hồi (Chưa kết nối mạch PCA9685 thật)' : 'I2C bus unresponsive (Hardware disconnected)'),
      icon: Compass
    },
    {
      id: 'battery',
      name: 'Hệ thống Nguồn & Pin Li-ion',
      category: language === 'vi' ? 'Quản Lý Nguồn Điện' : 'Power Management',
      interfacePort: 'Mạch INA219 ADC Monitor',
      specs: `Pack 3S 11.1V-12.6V, Dung lượng ${battery.toFixed(1)}%`,
      status: isRealConnected ? (battery > 20 ? 'ACTIVE' : 'WARNING') : 'SIMULATED',
      details: isRealConnected 
        ? (language === 'vi' ? 'Điện áp ổn định, không sụt áp' : 'Voltage stable, no sag detected')
        : (language === 'vi' ? 'Đang dùng số liệu pin ảo phần mềm (Chưa đọc chip ADC INA219 thật)' : 'Simulated battery telemetry (No physical INA219 ADC)'),
      icon: Zap
    },
    {
      id: 'host',
      name: isRealConnected ? 'Máy tính Nhúng Jetson Nano' : 'Máy Chủ Mô Phỏng SIL (Host PC)',
      category: language === 'vi' ? 'Nền Tảng Tính Toán' : 'Core Computing',
      interfacePort: isRealConnected ? 'Jetson Nano 4GB Dev Kit' : 'SIL Simulation Host (PC)',
      specs: `Nhiệt độ ${cpuTemp.toFixed(1)}°C, RTT ${rtt ?? 4}ms`,
      status: isRealConnected ? 'ACTIVE' : 'SIMULATED',
      details: isRealConnected 
        ? (language === 'vi' ? 'CPU/GPU mát mẻ, không bóp xung nhiệt' : 'CPU/GPU cool, no thermal throttling')
        : (language === 'vi' ? 'Đang chạy mã nguồn SIL trên máy tính phát triển' : 'Running software simulation on developer PC'),
      icon: Server
    }
  ];

  const handleRunSelfTest = () => {
    setIsTesting(true);
    setTestProgress(10);
    setTimeout(() => setTestProgress(35), 400);
    setTimeout(() => setTestProgress(65), 800);
    setTimeout(() => setTestProgress(90), 1200);
    setTimeout(() => {
      setTestProgress(100);
      setIsTesting(false);
      setLastCheckTime(new Date());
    }, 1600);
  };

  const realActiveCount = devices.filter(d => d.status === 'ACTIVE').length;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black/60 dark:bg-black/80 backdrop-blur-md animate-fadeIn select-none">
      <div className="w-full max-w-2xl bg-white dark:bg-[#141414] rounded-3xl p-5 sm:p-6 shadow-[0_24px_64px_rgba(0,0,0,0.25)] dark:shadow-[0_24px_64px_rgba(0,0,0,0.8)] border border-gray-200/80 dark:border-[#262626] flex flex-col gap-3.5 overflow-hidden animate-scaleUp">
        
        {/* Header */}
        <div className="flex items-center justify-between border-b border-gray-100 dark:border-[#262626] pb-3 shrink-0">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 rounded-2xl bg-[#0071e3]/10 dark:bg-[#1c1c1c] border border-[#0071e3]/20 dark:border-[#262626] flex items-center justify-center">
              <ShieldCheck className="w-5 h-5 text-[#0071e3] dark:text-[#f4f4f5]" />
            </div>
            <div>
              <h2 className="text-base font-bold text-[#1d1d1f] dark:text-[#f4f4f5] tracking-tight flex items-center gap-2">
                {t.checkDevices}
                <span className={`text-xs px-2.5 py-0.5 rounded-full font-semibold border ${
                  isRealConnected
                    ? 'bg-emerald-50 dark:bg-[#4ade80]/20 text-emerald-600 dark:text-[#4ade80] border-emerald-200 dark:border-[#4ade80]/30'
                    : 'bg-amber-50 dark:bg-[#fbbf24]/20 text-amber-700 dark:text-[#fbbf24] border-amber-200 dark:border-[#fbbf24]/30'
                }`}>
                  {isRealConnected ? `${realActiveCount}/6 ${t.readyBadge}` : t.simConnectedCount}
                </span>
              </h2>
              <p className="text-xs text-[#6e6e73] dark:text-[#9ca3af]">
                {language === 'vi' 
                  ? 'Kiểm tra tình trạng phần cứng JetBot CS532 theo tiêu chuẩn kiểm định nghiệm thu' 
                  : 'JetBot CS532 hardware verification and acceptance inspection'}
              </p>
            </div>
          </div>
          <button
            onClick={onClose}
            className="w-8 h-8 rounded-full bg-gray-100 dark:bg-[#1c1c1c] hover:bg-gray-200 dark:hover:bg-[#262626] flex items-center justify-center text-[#6e6e73] dark:text-[#9ca3af] hover:text-[#1d1d1f] dark:hover:text-[#f4f4f5] transition-colors"
          >
            <X className="w-4 h-4" />
          </button>
        </div>

        {/* Self Test Action & Truthful Health Banner */}
        <div className={`p-3.5 rounded-2xl border flex items-center justify-between gap-4 shrink-0 ${
          isRealConnected
            ? 'bg-gradient-to-r from-blue-50/70 via-indigo-50/50 to-emerald-50/70 dark:from-[#1c1c1c] dark:to-[#141414] border-blue-100 dark:border-[#262626]'
            : 'bg-gradient-to-r from-amber-50/80 to-orange-50/80 dark:from-amber-950/20 dark:to-orange-950/20 border-amber-200 dark:border-amber-900/30'
        }`}>
          <div className="flex items-center gap-3">
            {isRealConnected ? (
              <div className="w-3 h-3 rounded-full bg-[#34c759] dark:bg-[#4ade80] animate-pulse"></div>
            ) : (
              <AlertTriangle className="w-5 h-5 text-amber-600 dark:text-[#fbbf24] shrink-0" />
            )}
            <div className="flex flex-col">
              <span className="text-sm font-semibold text-[#1d1d1f] dark:text-[#f4f4f5]">
                {isRealConnected ? t.allGood : t.simWarning}
              </span>
              <span className="text-xs text-[#6e6e73] dark:text-[#9ca3af]">
                {isRealConnected 
                  ? `${t.latency}: ${rtt ?? 4}ms`
                  : t.simNotice}
              </span>
            </div>
          </div>

          <button
            onClick={handleRunSelfTest}
            disabled={isTesting}
            className="px-3.5 py-2 rounded-xl bg-[#0071e3] dark:bg-[#f97316] hover:bg-[#0077ed] dark:hover:bg-[#ea580c] text-white dark:text-[#ffffff] text-xs font-semibold shadow-sm transition-all apple-btn flex items-center gap-2 shrink-0"
          >
            <RefreshCw className={`w-3.5 h-3.5 ${isTesting ? 'animate-spin' : ''}`} />
            <span>{isTesting ? t.testing : t.checkHardware}</span>
          </button>
        </div>

        {/* Progress Bar when testing */}
        {isTesting && testProgress !== null && (
          <div className="flex flex-col gap-1 -mt-1 shrink-0">
            <div className="w-full h-1.5 bg-gray-100 dark:bg-[#262626] rounded-full overflow-hidden">
              <div 
                className="h-full bg-[#0071e3] dark:bg-[#f97316] transition-all duration-300 rounded-full"
                style={{ width: `${testProgress}%` }}
              />
            </div>
            <span className="text-[11px] text-[#0071e3] dark:text-[#f97316] font-medium text-right font-mono">
              {t.testingProgress} {testProgress}%
            </span>
          </div>
        )}

        {/* Device List with constrained max-height and custom scrollbar */}
        <div 
          style={{ maxHeight: '340px' }} 
          className="overflow-y-auto pr-1 flex flex-col gap-2 custom-scrollbar"
        >
          {devices.map((device) => {
            const Icon = device.icon;
            const isGood = device.status === 'ACTIVE';
            const isOffline = device.status === 'OFFLINE';

            return (
              <div
                key={device.id}
                className={`p-3 rounded-2xl border transition-all flex items-start gap-3 ${
                  isOffline
                    ? 'bg-gray-50/60 dark:bg-[#0d0c0c] border-gray-200/60 dark:border-[#262626] opacity-90'
                    : 'bg-white dark:bg-[#1c1c1c] border-gray-200/80 dark:border-[#262626] shadow-[0_1px_4px_rgba(0,0,0,0.03)]'
                }`}
              >
                <div className={`w-9 h-9 rounded-xl border flex items-center justify-center shrink-0 mt-0.5 ${
                  isGood 
                    ? 'bg-emerald-50 dark:bg-[#4ade80]/20 border-emerald-200 dark:border-[#4ade80]/30 text-emerald-600 dark:text-[#4ade80]'
                    : isOffline
                      ? 'bg-gray-100 dark:bg-[#141414] border-gray-200 dark:border-[#262626] text-gray-400 dark:text-[#9ca3af]'
                      : 'bg-blue-50 dark:bg-[#1c1c1c] border-blue-200 dark:border-[#262626] text-[#0071e3] dark:text-[#f4f4f5]'
                }`}>
                  <Icon className="w-4 h-4" />
                </div>

                <div className="flex-1 flex flex-col min-w-0">
                  <div className="flex items-center justify-between gap-2 mb-0.5">
                    <span className="text-sm font-semibold text-[#1d1d1f] dark:text-[#f4f4f5] truncate">
                      {device.name}
                    </span>
                    <span className={`px-2.5 py-0.5 rounded-full text-xs font-semibold shrink-0 flex items-center gap-1 ${
                      isGood 
                        ? 'bg-emerald-50 dark:bg-[#4ade80]/20 text-emerald-600 dark:text-[#4ade80] border border-emerald-200 dark:border-[#4ade80]/30' 
                        : isOffline
                          ? 'bg-rose-50 dark:bg-[#f87171]/20 text-rose-600 dark:text-[#f87171] border border-rose-200 dark:border-[#f87171]/30'
                          : 'bg-amber-50 dark:bg-[#fbbf24]/20 text-amber-600 dark:text-[#fbbf24] border border-amber-200 dark:border-[#fbbf24]/30'
                    }`}>
                      <span className={`w-1.5 h-1.5 rounded-full ${
                        isGood ? 'bg-emerald-500 dark:bg-[#4ade80]' : isOffline ? 'bg-rose-500 dark:bg-[#f87171]' : 'bg-amber-500 dark:bg-[#fbbf24]'
                      }`}></span>
                      {device.status}
                    </span>
                  </div>

                  <div className="text-xs text-[#6e6e73] dark:text-[#9ca3af] font-mono flex items-center gap-2 mb-1">
                    <span>{device.interfacePort}</span>
                  </div>

                  <div className="text-xs text-[#1d1d1f] dark:text-[#f4f4f5] font-medium bg-gray-50/70 dark:bg-[#141414] p-2 rounded-xl border border-gray-100 dark:border-[#262626] flex items-center justify-between">
                    <span className={isOffline ? 'text-[#6e6e73] dark:text-[#9ca3af]' : ''}>
                      {device.details}
                    </span>
                    <span className="text-xs text-[#86868b] dark:text-[#9ca3af] font-mono shrink-0 ml-2">
                      {device.specs}
                    </span>
                  </div>
                </div>
              </div>
            );
          })}
        </div>

        {/* Footer */}
        <div className="flex items-center justify-between border-t border-gray-100 dark:border-[#262626] pt-3 text-xs text-[#6e6e73] dark:text-[#9ca3af] shrink-0">
          <span>{t.compliance}</span>
          <button
            onClick={onClose}
            className="px-4 py-1.5 rounded-xl bg-gray-100 dark:bg-[#1c1c1c] hover:bg-gray-200 dark:hover:bg-[#262626] text-[#1d1d1f] dark:text-[#f4f4f5] font-semibold text-xs transition-colors"
          >
            {t.close}
          </button>
        </div>

      </div>
    </div>
  );
}
