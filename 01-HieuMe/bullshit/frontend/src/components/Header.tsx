import { Bot } from 'lucide-react';
import { useState, useEffect } from 'react';

interface HeaderProps {
  connected: boolean;
  mode: 'MANUAL' | 'AUTONOMOUS';
  setMode: (mode: 'MANUAL' | 'AUTONOMOUS') => void;
}

export default function Header({ connected, mode, setMode }: HeaderProps) {
  const [time, setTime] = useState(new Date());

  useEffect(() => {
    const timer = setInterval(() => setTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  return (
    <div className="h-14 bg-gray-900 border-b border-gray-700 flex items-center justify-between px-6 select-none shrink-0">
      <div className="flex items-center gap-3">
        <Bot className="w-6 h-6 text-cyan-400" />
        <h1 className="text-lg font-bold tracking-wider text-gray-100">JETBOT AUTONOMOUS NAVIGATION</h1>
        <div className="flex items-center gap-2 ml-6">
          <div className={`w-3 h-3 rounded-full ${connected ? 'bg-green-500 shadow-[0_0_8px_rgba(34,197,94,0.6)]' : 'bg-red-500 shadow-[0_0_8px_rgba(239,68,68,0.6)]'}`}></div>
          <span className="text-xs font-semibold text-gray-400">{connected ? 'ONLINE' : 'OFFLINE'}</span>
        </div>
      </div>

      <div className="flex items-center gap-6">
        <div className="flex bg-gray-800 rounded-lg p-1 border border-gray-700">
          <button 
            className={`px-4 py-1 text-xs font-bold rounded-md transition-colors ${mode === 'MANUAL' ? 'bg-cyan-600 text-white' : 'text-gray-400 hover:text-white'}`}
            onClick={() => setMode('MANUAL')}
          >
            MANUAL
          </button>
          <button 
            className={`px-4 py-1 text-xs font-bold rounded-md transition-colors ${mode === 'AUTONOMOUS' ? 'bg-cyan-600 text-white' : 'text-gray-400 hover:text-white'}`}
            onClick={() => setMode('AUTONOMOUS')}
          >
            AUTONOMOUS
          </button>
        </div>
        <div className="text-gray-400 text-sm font-mono">
          {time.toLocaleTimeString()}
        </div>
      </div>
    </div>
  );
}
