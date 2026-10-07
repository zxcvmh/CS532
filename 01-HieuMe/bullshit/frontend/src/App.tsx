import { useRobotState } from './hooks/useRobotState';
import Header from './components/Header';
import CameraView from './components/CameraView';
import MapViewer from './components/MapViewer';
import RobotStatus from './components/RobotStatus';
import NavigationPanel from './components/NavigationPanel';
import RobotControl from './components/RobotControl';
import SensorStatus from './components/SensorStatus';
import EventLog from './components/EventLog';

function App() {
  const robotState = useRobotState();

  return (
    <div className="h-screen w-screen flex flex-col bg-[#0a0a0f] text-gray-200 overflow-hidden text-sm">
      <Header connected={robotState.connected} mode={robotState.mode} setMode={robotState.setMode} />
      
      <div className="flex-1 grid grid-cols-2 grid-rows-[60fr_40fr] gap-2 p-2 min-h-0">
        
        {/* Top Row: Camera and Map */}
        <div className="bg-gray-800/50 rounded-lg border border-gray-700/50 overflow-hidden flex flex-col min-h-0">
          <CameraView 
            cameraFrame={robotState.cameraFrame} 
            detections={robotState.detections}
            detectionsVersion={robotState.detectionsVersion}
          />
        </div>
        
        <div className="bg-gray-800/50 rounded-lg border border-gray-700/50 overflow-hidden flex flex-col min-h-0 relative">
          <MapViewer 
            pose={robotState.pose}
            mapData={robotState.mapData}
            lidarPoints={robotState.lidarPoints}
            trajectory={robotState.trajectory}
            plannedPath={robotState.plannedPath}
            navigationStatus={robotState.navigationStatus}
            mode={robotState.mode}
            sendGoal={robotState.sendGoal}
          />
        </div>

        {/* Bottom Row: 4 Panels */}
        <div className="col-span-2 grid grid-cols-4 gap-2 min-h-0">
          <div className="bg-gray-800/50 rounded-lg border border-gray-700/50 overflow-auto p-3 min-h-0 flex flex-col gap-3">
            <RobotStatus 
              connected={robotState.connected}
              battery={robotState.battery}
              pose={robotState.pose}
              mode={robotState.mode}
            />
            <div className="border-t border-gray-700/50 pt-3">
              <SensorStatus sensors={robotState.sensors} />
            </div>
          </div>
          
          <div className="bg-gray-800/50 rounded-lg border border-gray-700/50 overflow-hidden p-3 min-h-0">
            <NavigationPanel 
              status={robotState.navigationStatus}
              mode={robotState.mode}
              cancelGoal={robotState.cancelGoal}
              emergencyStop={robotState.emergencyStop}
            />
          </div>
          
          <div className="bg-gray-800/50 rounded-lg border border-gray-700/50 overflow-hidden p-3 min-h-0 flex flex-col justify-between relative">
            <RobotControl 
              mode={robotState.mode}
              sendManualCommand={robotState.sendManualCommand}
              emergencyStop={robotState.emergencyStop}
            />
          </div>
          
          <div className="bg-gray-800/50 rounded-lg border border-gray-700/50 overflow-hidden p-3 min-h-0 flex flex-col">
            <EventLog events={robotState.events} />
          </div>
        </div>
        
      </div>
    </div>
  );
}

export default App;
