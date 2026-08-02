#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include <SFML/Graphics.hpp>
#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "algorithm/AStarStrategy.h"
#include "algorithm/BFSStrategy.h"
#include "algorithm/DijkstraStrategy.h"

class Graph;
class Intersection;
class Road;
class TrafficSimulator;
class VisualizationEngine;
class PathFindingStrategy;
class StatsPanel;
struct StatisticsSummary;
class PointOfInterest;

/**
 * DebugConsole
 * ------------
 * Owns the unified game HUD:
 *   - a compact, always-visible simulation status bar
 *   - a bottom dock for high-frequency controls
 *   - a responsive tabbed drawer for maps, tools and diagnostics
 *
 * This keeps all panel-specific UI state (pending picks, input buffers,
 * cached dropdown snapshots) out of main.cpp, and caches the
 * intersection/road dropdown lists instead of rebuilding + sorting them
 * every single frame.
 *
 * The HUD is edge-anchored rather than freely floating, so the central map
 * remains the visual focus and the layout stays deterministic after resize.
 * Existing simulation state and callbacks remain the single source of truth.
 */
class DebugConsole {
public:
    // Callbacks for the two operations that live outside this class's
    // concerns (file dialog, populating the demo graph, etc). requestedPath
    // empty means "load the demo map".
    using LoadMapFn = std::function<void(const std::string& requestedPath)>;
    using ResetSimulationFn =
        std::function<std::unique_ptr<TrafficSimulator>(int vehicleCount)>;
    using ResetViewFn = std::function<void()>;
    // Opens a native "pick a file" dialog. Returns true and fills
    // selectedPath if the user picked one; returns false (dialog
    // unavailable or cancelled) otherwise.
    using FileDialogFn = std::function<bool(std::string& selectedPath)>;

    DebugConsole(Graph& graph,
                 VisualizationEngine& visualization,
                 LoadMapFn loadAndRefresh,
                 ResetSimulationFn resetSimulation,
                 ResetViewFn resetView,
                 FileDialogFn openFileDialog);

    // Call once per frame, after ImGui::SFML::Update and before ImGui::SFML::Render.
    void draw(sf::RenderWindow& window,
              std::unique_ptr<TrafficSimulator>& simulator,
              bool& heatMapEnabled,
              bool& showParkedVehicles,
              std::string& mapPathInput,
              bool usingDemoMap,
              const std::string& loadError,
              StatsPanel& statsPanel,
              const StatisticsSummary* statistics,
              float frameDt);

    // True while waiting for a map click to resolve an intersection pick
    // (Add Road / Spawn Vehicle). While true, route left-clicks on the map
    // to handleMapClick instead of normal click behavior.
    bool isPicking() const;

    // Handles Escape as a UI cancel action. It never exits the application.
    // Returns true when an active pick or the Control Center was dismissed.
    bool handleEscape();

    // Called from main's event loop on a left click while isPicking() is true.
    // worldPos must already be converted to world/graph space, i.e.
    // window.mapPixelToCoords(pixel, view).
    void handleMapClick(const Graph& graph,
                         const VisualizationEngine& visualization,
                         const sf::Vector2f& worldPos);

    // Draws a small red warning marker above any vehicle whose most recent
    // route recalculation failed. Call once per frame after drawing vehicles.
    void drawFailedRecalcMarkers(sf::RenderWindow& window,
                                  TrafficSimulator* simulator,
                                  const VisualizationEngine& visualization) const;

    // Draws a short-lived pulse around vehicles created explicitly from the
    // Control Center. Uses real frame time so it also fades while paused.
    void drawManualSpawnMarkers(
        sf::RenderWindow& window,
        TrafficSimulator* simulator,
        const VisualizationEngine& visualization,
        float frameDt);

    // Clears picks/messages and invalidates cached snapshots. Call whenever
    // the map is (re)loaded.
    void onMapChanged();

    // The strategy instance matching the currently selected algorithm in
    // the dropdown (A* by default). Exposed so main.cpp can construct the
    // very first TrafficSimulator with the same instance DebugConsole
    // considers "selected", instead of a separate duplicate A* object.
    PathFindingStrategy* getSelectedStrategy();

private:
    enum class PickTarget { NONE, ADD_ROAD_START, ADD_ROAD_END, SPAWN_START, SPAWN_END, TRAFFIC_LIGHT_INTERSECTION };
    enum class DrawerTab { OVERVIEW, PERFORMANCE, MAP, ROAD_TOOLS, SIMULATION, DEBUG };
    enum class NoticeTone { INFO, SUCCESS, WARNING, ERROR };

    void rebuildSnapshotsIfNeeded(const Graph& graph);
    void drawTopHud(sf::RenderWindow& window,
                    std::unique_ptr<TrafficSimulator>& simulator,
                    const std::string& mapPathInput,
                    bool usingDemoMap,
                    const StatisticsSummary* statistics);
    void drawBottomDock(sf::RenderWindow& window,
                        std::unique_ptr<TrafficSimulator>& simulator,
                        bool& heatMapEnabled,
                        bool& showParkedVehicles,
                        std::string& mapPathInput,
                        const std::string& loadError);
    void drawDrawer(sf::RenderWindow& window,
                    std::unique_ptr<TrafficSimulator>& simulator,
                    bool& heatMapEnabled,
                    bool& showParkedVehicles,
                    std::string& mapPathInput,
                    bool usingDemoMap,
                    const std::string& loadError,
                    StatsPanel& statsPanel,
                    const StatisticsSummary* statistics);
    void drawOverviewTab(std::unique_ptr<TrafficSimulator>& simulator,
                         bool& heatMapEnabled,
                         bool& showParkedVehicles,
                         const std::string& mapPathInput,
                         bool usingDemoMap,
                         const std::string& loadError,
                         StatsPanel& statsPanel,
                         const StatisticsSummary* statistics);
    void drawMapTab(std::unique_ptr<TrafficSimulator>& simulator,
                    std::string& mapPathInput,
                    bool usingDemoMap,
                    const std::string& loadError);
    void drawToast(sf::RenderWindow& window);
    void openDrawer(DrawerTab tab);
    void setNotice(NoticeTone tone, const std::string& message);
    void performMapLoad(bool useDemo,
                        std::unique_ptr<TrafficSimulator>& simulator,
                        std::string& mapPathInput,
                        const std::string& loadError);
    void completePendingMapLoad(std::unique_ptr<TrafficSimulator>& simulator,
                                std::string& mapPathInput,
                                const std::string& loadError);
    void drawAddRoadPanel(Graph& graph, VisualizationEngine& visualization);
    void drawSpawnVehiclePanel(std::unique_ptr<TrafficSimulator>& simulator);
    void drawAccidentPanel(std::unique_ptr<TrafficSimulator>& simulator);
    void drawAlgorithmPanel(std::unique_ptr<TrafficSimulator>& simulator);
    void drawTrafficLightPanel();
    void drawSimulationSetupPanel(
        std::unique_ptr<TrafficSimulator>& simulator);
    bool createConfiguredSimulation(
        std::unique_ptr<TrafficSimulator>& simulator,
        std::string& errorMessage);
    PathFindingStrategy* currentStrategy();

    Graph& graph_;
    VisualizationEngine& visualization_;
    LoadMapFn loadAndRefresh_;
    ResetSimulationFn resetSimulation_;
    ResetViewFn resetView_;
    FileDialogFn openFileDialog_;

    PickTarget pickTarget_ = PickTarget::NONE;
    DrawerTab activeTab_ = DrawerTab::SIMULATION;
    bool drawerOpen_ = true;
    bool drawerTabSelectionPending_ = true;
    bool lastLoadFailed_ = false;
    bool loadStatusInitialized_ = false;
    float smoothedFps_ = 60.0f;
    std::string noticeMessage_;
    NoticeTone noticeTone_ = NoticeTone::INFO;
    float noticeTimeRemaining_ = 0.0f;
    std::array<char, 1024> mapPathBuffer_{};
    bool mapPathBufferInitialized_ = false;
    bool mapLoadPending_ = false;
    bool pendingDemoLoad_ = false;
    std::string pendingMapPath_;

    // Startup is deliberately two-stage: lock the demand size, then start.
    // Loading a new map clears the lock so the next run can be reconfigured.
    int simulationVehicleCountInput_ = 0;
    int lockedSimulationVehicleCount_ = 0;
    bool simulationVehicleCountLocked_ = false;
    bool simulationStarted_ = false;
    std::string simulationSetupMessage_;

    // Task 4a: Add Road panel state
    int addRoadStartId_ = -1;
    int addRoadEndId_ = -1;
    bool addRoadAutoDistance_ = true;
    float addRoadDistance_ = 50.0f;
    float addRoadSpeedLimit_ = 40.0f;
    int addRoadLanes_ = 1;
    bool addRoadTwoWay_ = false;
    std::string addRoadMessage_;

    // Task 4b: Spawn Vehicle panel state
    int spawnStartId_ = -1;
    int spawnEndId_ = -1;
    int spawnVehicleTypeIdx_ = 0; // 0=Car,1=Bus,2=Motorbike,3=Emergency
    float spawnVehicleSpeed_ = 20.0f;
    int spawnVehicleCount_ = 1;
    std::string spawnMessage_;
    struct ManualSpawnHighlight {
        int vehicleId = -1;
        float remainingSeconds = 0.0f;
    };
    static constexpr float
        MANUAL_SPAWN_HIGHLIGHT_SECONDS = 5.0f;
    std::vector<ManualSpawnHighlight>
        manualSpawnHighlights_;

    // Task 4c: Trigger Accident panel state
    int eventTypeIdx_ = 0; // 0=Accident, 1=Congestion, 2=Road Closure
    int accidentRoadIdx_ = -1; // index into roadsSnapshot_, -1 = random
    float accidentDuration_ = 15.0f;
    float accidentSeverity_ = 1.5f; // for Congestion event
    int accidentLaneIdx_ = -1; // -1 = Random

    // Task 4d: Algorithm switching panel state
    AStarStrategy aStar_;
    BFSStrategy bfsStrategy_;
    DijkstraStrategy dijkstraStrategy_;
    int selectedAlgorithmIdx_ = 2; // 0=BFS,1=Dijkstra,2=A*

    // Distance-vs-speed blend shared by Dijkstra and A* (BFS ignores it -
    // BFS is hop-count-only by design). 0.0 = shortest distance,
    // 1.0 = fastest travel time (speed-limit + congestion aware).
    // Kept in sync with aStar_ / dijkstraStrategy_ via setSpeedPreference()
    // whenever the slider in drawAlgorithmPanel() changes.
    float speedPreference_ = 1.0f;

    // Traffic light panel state
    int trafficLightIntersectionIdx_ = -1; // index into intersectionsSnapshot_

    // Cached dropdown data - only rebuilt when the graph's intersection/road
    // counts change (or onMapChanged() is called), not every frame.
    bool snapshotDirty_ = true;
    std::vector<Intersection*> intersectionsSnapshot_;
    std::vector<Road*> roadsSnapshot_;
    std::vector<PointOfInterest*> poisSnapshot_;
    size_t lastKnownIntersectionCount_ = 0;
    size_t lastKnownRoadCount_ = 0;
    size_t lastKnownPoiCount_ = 0;
};

#endif
