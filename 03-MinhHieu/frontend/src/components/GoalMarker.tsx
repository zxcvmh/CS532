interface GoalMarkerProps {
  x: number;
  y: number;
  onConfirm: () => void;
  onCancel: () => void;
}

export default function GoalMarker({ x, y, onConfirm, onCancel }: GoalMarkerProps) {
  return (
    <div className="bg-gray-900/95 border border-cyan-500/70 rounded-lg shadow-2xl shadow-cyan-500/10 p-3 w-52 backdrop-blur pointer-events-auto">
      <div className="flex items-center gap-2 mb-2 border-b border-gray-700/50 pb-2">
        <div className="w-2 h-2 bg-cyan-400 rounded-full animate-pulse"></div>
        <span className="text-[11px] font-bold text-cyan-400 tracking-wider">SET NAVIGATION GOAL</span>
      </div>
      <div className="grid grid-cols-2 gap-2 mb-3">
        <div className="bg-gray-800/80 rounded px-2 py-1.5 border border-gray-700/50">
          <div className="text-[9px] text-gray-500 uppercase">X Position</div>
          <div className="text-xs font-mono text-cyan-300">{x.toFixed(2)} m</div>
        </div>
        <div className="bg-gray-800/80 rounded px-2 py-1.5 border border-gray-700/50">
          <div className="text-[9px] text-gray-500 uppercase">Y Position</div>
          <div className="text-xs font-mono text-cyan-300">{y.toFixed(2)} m</div>
        </div>
      </div>
      <div className="flex gap-2">
        <button 
          onClick={onConfirm}
          className="flex-1 bg-cyan-600 hover:bg-cyan-500 text-white text-[11px] font-bold py-1.5 rounded transition-all hover:shadow-lg hover:shadow-cyan-500/20"
        >
          ✓ Confirm
        </button>
        <button 
          onClick={onCancel}
          className="flex-1 bg-gray-700 hover:bg-gray-600 text-gray-300 text-[11px] py-1.5 rounded transition-colors"
        >
          Cancel
        </button>
      </div>
    </div>
  );
}
