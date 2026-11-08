import { SensorHealth } from '../types/robot';

export default function SensorStatus({ sensors }: { sensors: SensorHealth[] }) {
  const allSensors = [
    { name: 'Camera', status: 'OFFLINE' },
    { name: 'LiDAR', status: 'OFFLINE' },
    { name: 'IMU', status: 'OFFLINE' },
    { name: 'Odometry', status: 'OFFLINE' },
    { name: 'SLAM', status: 'OFFLINE' },
    { name: 'YOLO', status: 'OFFLINE' },
    { name: 'Navigation', status: 'OFFLINE' }
  ];

  const mergedSensors = allSensors.map(s => {
    const found = sensors.find(x => x.name === s.name);
    return found || s;
  });

  return (
    <div className="flex flex-col h-full">
      <h3 className="text-xs font-bold text-gray-400 uppercase tracking-wider mb-2 shrink-0">Subsystems</h3>
      <div className="grid grid-cols-2 gap-2 flex-1 overflow-y-auto pr-1">
        {mergedSensors.map(sensor => (
          <div key={sensor.name} className="flex items-center justify-between bg-gray-900 px-2 py-1.5 rounded border border-gray-700">
            <span className="text-xs text-gray-300">{sensor.name}</span>
            <div className={`w-2 h-2 rounded-full ${
              sensor.status === 'ACTIVE' ? 'bg-green-500' :
              sensor.status === 'WARNING' ? 'bg-yellow-500' : 'bg-red-500'
            }`}></div>
          </div>
        ))}
      </div>
    </div>
  );
}
