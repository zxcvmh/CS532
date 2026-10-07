import { useEffect, useState } from 'react';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Ban, Square } from 'lucide-react';

interface RobotControlProps {
  mode: 'MANUAL' | 'AUTONOMOUS';
  sendManualCommand: (cmd: string) => void;
  emergencyStop: () => void;
}

export default function RobotControl({ mode, sendManualCommand, emergencyStop }: RobotControlProps) {
  const [activeKey, setActiveKey] = useState<string | null>(null);

  useEffect(() => {
    if (mode !== 'MANUAL') return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.repeat) return;
      
      switch (e.key.toLowerCase()) {
        case 'w': sendManualCommand('FORWARD'); setActiveKey('W'); break;
        case 's': sendManualCommand('BACKWARD'); setActiveKey('S'); break;
        case 'a': sendManualCommand('ROTATE_LEFT'); setActiveKey('A'); break;
        case 'd': sendManualCommand('ROTATE_RIGHT'); setActiveKey('D'); break;
        case ' ': sendManualCommand('STOP'); setActiveKey('SPACE'); e.preventDefault(); break;
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (['w', 'a', 's', 'd', ' '].includes(k)) {
        sendManualCommand('STOP');
        setActiveKey(null);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [mode, sendManualCommand]);

  const btnClass = "bg-gray-700 text-white rounded flex items-center justify-center p-3 select-none transition-colors border border-gray-600";
  const activeBtnClass = "bg-cyan-600 text-white rounded flex items-center justify-center p-3 select-none transition-colors border border-cyan-400 shadow-[0_0_10px_rgba(8,145,178,0.5)]";

  return (
    <div className="flex flex-col h-full gap-3 relative">
      <h3 className="text-xs font-bold text-gray-400 uppercase tracking-wider shrink-0">Manual Control</h3>
      
      <div className="flex-1 flex items-center justify-center">
        <div className="grid grid-cols-3 gap-2 w-48">
          <div></div>
          <button 
            className={activeKey === 'W' ? activeBtnClass : btnClass}
            onMouseDown={() => sendManualCommand('FORWARD')}
            onMouseUp={() => sendManualCommand('STOP')}
            onMouseLeave={() => sendManualCommand('STOP')}
          ><ArrowUp className="w-5 h-5" /></button>
          <div></div>

          <button 
            className={activeKey === 'A' ? activeBtnClass : btnClass}
            onMouseDown={() => sendManualCommand('ROTATE_LEFT')}
            onMouseUp={() => sendManualCommand('STOP')}
            onMouseLeave={() => sendManualCommand('STOP')}
          ><ArrowLeft className="w-5 h-5" /></button>
          
          <button 
            className={activeKey === 'SPACE' ? activeBtnClass.replace('cyan-600', 'red-500').replace('cyan-400', 'red-400') : btnClass.replace('bg-gray-700', 'bg-red-900')}
            onMouseDown={() => sendManualCommand('STOP')}
          ><Square className="w-4 h-4 fill-current" /></button>
          
          <button 
            className={activeKey === 'D' ? activeBtnClass : btnClass}
            onMouseDown={() => sendManualCommand('ROTATE_RIGHT')}
            onMouseUp={() => sendManualCommand('STOP')}
            onMouseLeave={() => sendManualCommand('STOP')}
          ><ArrowRight className="w-5 h-5" /></button>

          <div></div>
          <button 
            className={activeKey === 'S' ? activeBtnClass : btnClass}
            onMouseDown={() => sendManualCommand('BACKWARD')}
            onMouseUp={() => sendManualCommand('STOP')}
            onMouseLeave={() => sendManualCommand('STOP')}
          ><ArrowDown className="w-5 h-5" /></button>
          <div></div>
        </div>
      </div>

      <button 
        onClick={emergencyStop}
        className="w-full bg-red-600 hover:bg-red-500 text-white text-lg font-bold py-3 rounded flex justify-center items-center gap-2 shadow-[0_0_15px_rgba(220,38,38,0.4)] animate-[pulse_2s_infinite] shrink-0"
      >
        <Ban className="w-5 h-5" /> EMERGENCY STOP
      </button>

      {mode === 'AUTONOMOUS' && (
        <div className="absolute inset-0 bg-gray-800/80 backdrop-blur-[1px] flex items-center justify-center rounded z-10">
          <div className="text-xs font-bold text-gray-400 bg-gray-900 px-4 py-2 border border-gray-700 rounded shadow-lg">
            Switch to MANUAL mode
          </div>
        </div>
      )}
    </div>
  );
}
