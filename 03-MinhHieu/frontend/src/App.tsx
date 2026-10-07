import { useState } from 'react';
import { useRobotState } from './hooks/useRobotState';
import { useUiPreferences } from './hooks/useUiPreferences';
import Header from './components/Header';
import Sidebar, { ActiveView } from './components/Sidebar';
import DeviceInspector from './components/DeviceInspector';
import CameraView from './components/CameraView';
import MapViewer from './components/MapViewer';
import TelemetryWidget from './components/TelemetryWidget';
import NavigationPanel from './components/NavigationPanel';
import RobotControl from './components/RobotControl';
import SystemConsole from './components/SystemConsole';
import { WifiOff } from 'lucide-react';
import { translations } from './i18n/translations';

export default function App() {
  const robotState = useRobotState();
  const { 
    prefs, 
    toggle, 
    setSpeedLimit, 
    setMaximizedPanel, 
    toggleTheme, 
    toggleLanguage,
    setControlInput 
  } = useUiPreferences();
  
  // Navigation & Modal states
  const [isSidebarOpen, setIsSidebarOpen] = useState(true);
  const [activeView, setActiveView] = useState<ActiveView>('OVERVIEW');
  const [isDeviceInspectorOpen, setIsDeviceInspectorOpen] = useState(false);

  const t = translations[prefs.language];

  // Sync active view with maximized panel
  const handleSelectView = (view: ActiveView) => {
    setActiveView(view);
    if (view === 'MAP') {
      setMaximizedPanel('MAP');
    } else if (view === 'CAMERA') {
      setMaximizedPanel('CAMERA');
    } else {
      setMaximizedPanel(null);
    }
  };

  // Rebalanced lower stage cards: Compact RobotControl (0.85fr), expanded Telemetry (1.15fr) and System Console (1.15fr)
  const bottomCards = [
    prefs.showTelemetry && (
      <div key="telemetry" className="h-full min-h-0">
        <TelemetryWidget
          battery={robotState.battery}
          pose={robotState.pose}
          cpuTemp={robotState.cpuTemp}
          linearSpeed={robotState.linearSpeed}
          angularSpeed={robotState.angularSpeed}
          nearestObstacle={robotState.nearestObstacle}
          sensors={robotState.sensors}
          language={prefs.language}
        />
      </div>
    ),
    prefs.showNavigation && (
      <div key="navigation" className="h-full min-h-0">
        <NavigationPanel
          status={robotState.navigationStatus}
          mode={robotState.mode}
          cancelGoal={robotState.cancelGoal}
          emergencyStop={robotState.emergencyStop}
          language={prefs.language}
        />
      </div>
    ),
    prefs.showControls && (
      <div key="controls" className="h-full min-h-0">
        <RobotControl
          mode={robotState.mode}
          sendManualCommand={robotState.sendManualCommand}
          sendVelocity={robotState.sendVelocity}
          emergencyStop={robotState.emergencyStop}
          speedLimit={prefs.speedLimit}
          language={prefs.language}
          controlInput={prefs.controlInput}
          onToggleControlInput={setControlInput}
        />
      </div>
    ),
    prefs.showLogs && (
      <div key="console" className="h-full min-h-0">
        <SystemConsole
          events={robotState.events}
          sensors={robotState.sensors}
          language={prefs.language}
          isRealConnected={robotState.isRealConnected}
        />
      </div>
    ),
  ].filter(Boolean);

  return (
    <div className="h-screen w-screen flex bg-[#f5f5f7] dark:bg-[#0d0c0c] text-[#1d1d1f] dark:text-[#f4f4f5] overflow-hidden font-sans select-none relative transition-colors">
      {/* Subtle atmospheric ambient aura in top-left corner */}
      <div className="absolute -top-24 -left-24 w-80 h-80 bg-[#f97316]/[0.025] rounded-full blur-3xl pointer-events-none"></div>
      <Sidebar
        isOpen={isSidebarOpen}
        activeView={activeView}
        setActiveView={handleSelectView}
        prefs={prefs}
        toggle={toggle}
        setSpeedLimit={setSpeedLimit}
        onOpenDeviceInspector={() => setIsDeviceInspectorOpen(true)}
        battery={robotState.battery}
        cpuTemp={robotState.cpuTemp}
        isRealConnected={robotState.isRealConnected}
        emergencyStop={robotState.emergencyStop}
        resetMap={robotState.resetMap}
        resetPose={robotState.resetPose}
        language={prefs.language}
      />

      {/* Main Content Viewport */}
      <div className="flex-1 flex flex-col min-w-0 h-full overflow-hidden">
        
        {/* Header with Theme & Language Toggle, Labels v1.0 and CS532 */}
        <Header
          connected={robotState.connected}
          rtt={robotState.rtt}
          isRealConnected={robotState.isRealConnected}
          mode={robotState.mode}
          setMode={robotState.setMode}
          onToggleSidebar={() => setIsSidebarOpen(prev => !prev)}
          onOpenDeviceInspector={() => setIsDeviceInspectorOpen(true)}
          isSidebarOpen={isSidebarOpen}
          theme={prefs.theme}
          onToggleTheme={toggleTheme}
          language={prefs.language}
          onToggleLanguage={toggleLanguage}
        />

        {/* Disconnection Warning Banner (Safety Protocol) */}
        {!robotState.connected && (
          <div className="mx-4 mt-2 px-4 py-2 rounded-2xl bg-rose-50 dark:bg-rose-950/40 border border-rose-200 dark:border-rose-900 text-rose-800 dark:text-rose-300 text-xs font-semibold flex items-center justify-between shadow-xs animate-apple-pulse">
            <div className="flex items-center gap-2.5">
              <WifiOff className="w-4 h-4 text-[#ff3b30]" />
              <span>MẤT KẾT NỐI VỚI ROBOT • Đang tự động thử lại kết nối WebSocket (ws://127.0.0.1:8000/ws)...</span>
            </div>
            <span className="text-[11px] font-mono text-rose-600 dark:text-rose-400 bg-rose-100 dark:bg-rose-900/60 px-2 py-0.5 rounded-full">
              Chế độ bảo vệ an toàn kích hoạt
            </span>
          </div>
        )}

        {/* Main Workspace Stage */}
        <main className="flex-1 p-3 flex flex-col gap-3 min-h-0 overflow-hidden">
          {/* View Mode: Maximized Map */}
          {prefs.maximizedPanel === 'MAP' || activeView === 'MAP' ? (
            <div className="flex-1 min-h-0">
              <MapViewer
                pose={robotState.pose}
                mapData={robotState.mapData}
                lidarPoints={robotState.lidarPoints}
                trajectory={robotState.trajectory}
                plannedPath={robotState.plannedPath}
                navigationStatus={robotState.navigationStatus}
                mode={robotState.mode}
                sendGoal={robotState.sendGoal}
                resetMap={robotState.resetMap}
                resetPose={robotState.resetPose}
                showLidarScan={prefs.showLidarScan}
                showTrajectory={prefs.showTrajectory}
                showPlannedPath={prefs.showPlannedPath}
                showFovCone={prefs.showFovCone}
                showGrid={prefs.showGrid}
                isMaximized={true}
                onToggleMaximize={() => handleSelectView('OVERVIEW')}
                theme={prefs.theme}
                language={prefs.language}
              />
            </div>
          ) : prefs.maximizedPanel === 'CAMERA' || activeView === 'CAMERA' ? (
            /* View Mode: Maximized Camera */
            <div className="flex-1 min-h-0">
              <CameraView
                cameraFrame={robotState.cameraFrame}
                detections={robotState.detections}
                detectionsVersion={robotState.detectionsVersion}
                showYoloBoxes={prefs.showYoloBoxes}
                showDistanceTags={prefs.showDistanceTags}
                isMaximized={true}
                isRealConnected={robotState.isRealConnected}
                onToggleMaximize={() => handleSelectView('OVERVIEW')}
              />
            </div>
          ) : activeView === 'TELEOP' ? (
            /* View Mode: Teleop Driving Focus */
            <div className="flex-1 flex flex-col gap-3 min-h-0">
              <div className="flex-[60] min-h-0">
                <MapViewer
                  pose={robotState.pose}
                  mapData={robotState.mapData}
                  lidarPoints={robotState.lidarPoints}
                  trajectory={robotState.trajectory}
                  plannedPath={robotState.plannedPath}
                  navigationStatus={robotState.navigationStatus}
                  mode={robotState.mode}
                  sendGoal={robotState.sendGoal}
                  showLidarScan={prefs.showLidarScan}
                  showTrajectory={prefs.showTrajectory}
                  showPlannedPath={prefs.showPlannedPath}
                  showFovCone={prefs.showFovCone}
                  showGrid={prefs.showGrid}
                  theme={prefs.theme}
                  language={prefs.language}
                />
              </div>
              <div className="flex-[40] grid grid-cols-2 gap-3 min-h-0">
                <RobotControl
                  mode={robotState.mode}
                  sendManualCommand={robotState.sendManualCommand}
                  sendVelocity={robotState.sendVelocity}
                  emergencyStop={robotState.emergencyStop}
                  speedLimit={prefs.speedLimit}
                  language={prefs.language}
                  controlInput={prefs.controlInput}
                  onToggleControlInput={setControlInput}
                />
                <TelemetryWidget
                  battery={robotState.battery}
                  pose={robotState.pose}
                  cpuTemp={robotState.cpuTemp}
                  linearSpeed={robotState.linearSpeed}
                  angularSpeed={robotState.angularSpeed}
                  nearestObstacle={robotState.nearestObstacle}
                  sensors={robotState.sensors}
                  language={prefs.language}
                />
              </div>
            </div>
          ) : (
            /* Default View: Cockpit Overview */
            <>
              {/* Upper Stage: Camera (Left) & 2D SLAM Map (Right) */}
              <div className={`grid gap-3 min-h-0 ${
                prefs.showCamera && prefs.showMap 
                  ? 'grid-cols-2 flex-[58]' 
                  : 'grid-cols-1 flex-[58]'
              }`}>
                {/* Camera View Card */}
                {prefs.showCamera && (
                  <div className="h-full min-h-0">
                    <CameraView
                      cameraFrame={robotState.cameraFrame}
                      detections={robotState.detections}
                      detectionsVersion={robotState.detectionsVersion}
                      showYoloBoxes={prefs.showYoloBoxes}
                      showDistanceTags={prefs.showDistanceTags}
                      isMaximized={false}
                      isRealConnected={robotState.isRealConnected}
                      onToggleMaximize={() => handleSelectView('CAMERA')}
                    />
                  </div>
                )}

                {/* 2D SLAM Map Card */}
                {prefs.showMap && (
                  <div className="h-full min-h-0">
                    <MapViewer
                      pose={robotState.pose}
                      mapData={robotState.mapData}
                      lidarPoints={robotState.lidarPoints}
                      trajectory={robotState.trajectory}
                      plannedPath={robotState.plannedPath}
                      navigationStatus={robotState.navigationStatus}
                      mode={robotState.mode}
                      sendGoal={robotState.sendGoal}
                      resetMap={robotState.resetMap}
                      resetPose={robotState.resetPose}
                      showLidarScan={prefs.showLidarScan}
                      showTrajectory={prefs.showTrajectory}
                      showPlannedPath={prefs.showPlannedPath}
                      showFovCone={prefs.showFovCone}
                      showGrid={prefs.showGrid}
                      isMaximized={false}
                      onToggleMaximize={() => handleSelectView('MAP')}
                      theme={prefs.theme}
                      language={prefs.language}
                    />
                  </div>
                )}
              </div>

              {/* Lower Stage: 4 Rebalanced Cards */}
              {bottomCards.length > 0 && (
                <div 
                  className="grid gap-3 flex-[42] min-h-0"
                  style={{ 
                    gridTemplateColumns: bottomCards.length === 4 
                      ? 'minmax(0, 1.15fr) minmax(0, 1fr) minmax(0, 0.85fr) minmax(0, 1.15fr)' 
                      : `repeat(${bottomCards.length}, minmax(0, 1fr))` 
                  }}
                >
                  {bottomCards}
                </div>
              )}
            </>
          )}
        </main>
      </div>

      {/* Hardware Device Inspector Modal (Truthful Diagnostic) */}
      <DeviceInspector
        isOpen={isDeviceInspectorOpen}
        onClose={() => setIsDeviceInspectorOpen(false)}
        isRealConnected={robotState.isRealConnected}
        battery={robotState.battery}
        cpuTemp={robotState.cpuTemp}
        rtt={robotState.rtt}
        sensors={robotState.sensors}
        language={prefs.language}
      />

    </div>
  );
}
