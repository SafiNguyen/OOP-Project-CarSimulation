#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include <SFML/Graphics.hpp>
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

/**
 * DebugConsole
 * ------------
 * Owns and draws the entire interactive debug/control HUD (Task 4):
 *   - map load / demo map / pause / view / heatmap toggles
 *   - Add Road (click-to-pick or dropdown)
 *   - Spawn Vehicle (click-to-pick or dropdown)
 *   - Trigger Accident (specific road or random)
 *   - Runtime pathfinding algorithm switch
 *
 * This keeps all panel-specific UI state (pending picks, input buffers,
 * cached dropdown snapshots) out of main.cpp, and caches the
 * intersection/road dropdown lists instead of rebuilding + sorting them
 * every single frame.
 *
 * The window behaves like a normal moveable/resizable ImGui window: it
 * is not pinned (no NoMove/NoResize) and not auto-resized. Its default
 * position/size are a fraction of the window size, applied only once
 * via ImGuiCond_FirstUseEver, so it starts in a sensible place on any
 * resolution but never fights the user's own dragging/resizing
 * afterward. Collapsing uses ImGui's own titlebar arrow (click the arrow
 * or double-click the titlebar) - the same mechanism StatsPanel uses -
 * rather than a separate custom button, so there's exactly one
 * collapse/expand control instead of two competing ones. A scrollable
 * child region holds the body so content is always reachable even if
 * the window is made short.
 *
 * Implementation is split across several .cpp files, one per panel
 * (DebugConsole.cpp holds the shared/core logic; DebugConsoleTopBar.cpp,
 * DebugConsoleAddRoad.cpp, DebugConsoleSpawnVehicle.cpp,
 * DebugConsoleAccident.cpp and DebugConsoleAlgorithm.cpp each define one
 * of the private draw*Panel methods below). They're all still ordinary
 * member functions of this one class - splitting definitions across
 * multiple translation units is standard C++ and requires no change to
 * how the class is used.
 */
class DebugConsole {
public:
    // Callbacks for the two operations that live outside this class's
    // concerns (file dialog, populating the demo graph, etc). requestedPath
    // empty means "load the demo map".
    using LoadMapFn = std::function<void(const std::string& requestedPath)>;
    using ResetSimulationFn = std::function<std::unique_ptr<TrafficSimulator>()>;
    using ClampViewFn = std::function<void()>;
    // Opens a native "pick a file" dialog. Returns true and fills
    // selectedPath if the user picked one; returns false (dialog
    // unavailable or cancelled) otherwise.
    using FileDialogFn = std::function<bool(std::string& selectedPath)>;

    DebugConsole(Graph& graph,
                 VisualizationEngine& visualization,
                 LoadMapFn loadAndRefresh,
                 ResetSimulationFn resetSimulation,
                 ClampViewFn clampViewToMap,
                 FileDialogFn openFileDialog);

    // Call once per frame, after ImGui::SFML::Update and before ImGui::SFML::Render.
    void draw(sf::RenderWindow& window,
              std::unique_ptr<TrafficSimulator>& simulator,
              sf::View& view,
              float& zoomFactor,
              bool& heatMapEnabled,
              std::string& mapPathInput,
              bool usingDemoMap,
              const std::string& loadError);

    // True while waiting for a map click to resolve an intersection pick
    // (Add Road / Spawn Vehicle). While true, route left-clicks on the map
    // to handleMapClick instead of normal click behavior.
    bool isPicking() const;

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

    // Clears picks/messages and invalidates cached snapshots. Call whenever
    // the map is (re)loaded.
    void onMapChanged();

    // The strategy instance matching the currently selected algorithm in
    // the dropdown (A* by default). Exposed so main.cpp can construct the
    // very first TrafficSimulator with the same instance DebugConsole
    // considers "selected", instead of a separate duplicate A* object.
    PathFindingStrategy* getSelectedStrategy();

private:
    enum class PickTarget { NONE, ADD_ROAD_START, ADD_ROAD_END, SPAWN_START, SPAWN_END };

    void rebuildSnapshotsIfNeeded(const Graph& graph);
    void drawTopBar(sf::RenderWindow& window,
                     std::unique_ptr<TrafficSimulator>& simulator,
                     sf::View& view,
                     float& zoomFactor,
                     bool& heatMapEnabled,
                     std::string& mapPathInput,
                     bool usingDemoMap,
                     const std::string& loadError);
    void drawAddRoadPanel(Graph& graph, VisualizationEngine& visualization);
    void drawSpawnVehiclePanel(std::unique_ptr<TrafficSimulator>& simulator);
    void drawAccidentPanel(std::unique_ptr<TrafficSimulator>& simulator);
    void drawAlgorithmPanel(std::unique_ptr<TrafficSimulator>& simulator);
    PathFindingStrategy* currentStrategy();

    Graph& graph_;
    VisualizationEngine& visualization_;
    LoadMapFn loadAndRefresh_;
    ResetSimulationFn resetSimulation_;
    ClampViewFn clampViewToMap_;
    FileDialogFn openFileDialog_;

    PickTarget pickTarget_ = PickTarget::NONE;

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
    std::string spawnMessage_;

    // Task 4c: Trigger Accident panel state
    int accidentRoadIdx_ = -1; // index into roadsSnapshot_, -1 = random
    float accidentDuration_ = 15.0f;

    // Task 4d: Algorithm switching panel state
    AStarStrategy aStar_;
    BFSStrategy bfsStrategy_;
    DijkstraStrategy dijkstraStrategy_;
    int selectedAlgorithmIdx_ = 2; // 0=BFS,1=Dijkstra,2=A*

    // Cached dropdown data - only rebuilt when the graph's intersection/road
    // counts change (or onMapChanged() is called), not every frame.
    bool snapshotDirty_ = true;
    std::vector<Intersection*> intersectionsSnapshot_;
    std::vector<Road*> roadsSnapshot_;
    size_t lastKnownIntersectionCount_ = 0;
    size_t lastKnownRoadCount_ = 0;
};

#endif
