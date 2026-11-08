import { EventLogEntry } from '../types/events';
import { useRef, useEffect } from 'react';

export default function EventLog({ events }: { events: EventLogEntry[] }) {
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (containerRef.current) {
      containerRef.current.scrollTop = 0; // Since newest is first
    }
  }, [events]);

  const getColor = (level: string) => {
    switch (level) {
      case 'INFO': return 'text-gray-300';
      case 'WARNING': return 'text-yellow-400';
      case 'ERROR': return 'text-red-400';
      case 'SUCCESS': return 'text-green-400';
      default: return 'text-gray-300';
    }
  };

  return (
    <div className="flex flex-col h-full">
      <h3 className="text-xs font-bold text-gray-400 uppercase tracking-wider mb-2 shrink-0">Event Log</h3>
      <div 
        ref={containerRef}
        className="flex-1 overflow-y-auto space-y-1 pr-2 custom-scrollbar"
      >
        {events.length === 0 ? (
          <div className="text-gray-600 text-xs italic">No events recorded</div>
        ) : (
          events.map((ev, i) => (
            <div key={i} className="text-xs flex gap-2 font-mono">
              <span className="text-gray-500 shrink-0">[{ev.timestamp}]</span>
              <span className={getColor(ev.level)}>{ev.message}</span>
            </div>
          ))
        )}
      </div>
    </div>
  );
}
