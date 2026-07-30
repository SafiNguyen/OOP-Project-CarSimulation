#ifndef TRAFFICSIMULATOR_H
#define TRAFFICSIMULATOR_H

#include <vector>
#include <memory>
#include <set>
#include <limits>
#include <unordered_map>

class Graph;
class Road;
class Vehicle;
class EventManager;
class TrafficEvent;
class PathFindingStrategy;
class StatisticsManager;


class TrafficSimulator {
public:
    
    TrafficSimulator(Graph* graph, PathFindingStrategy* strategy);

    ~TrafficSimulator();

   
    TrafficSimulator(const TrafficSimulator&) = delete;
    TrafficSimulator& operator=(const TrafficSimulator&) = delete;

    bool addVehicle(Vehicle* vehicle);

    void triggerEvent(std::unique_ptr<TrafficEvent> event);

    void update(double dt);


    void pause();
    void resume();
    bool isPaused() const;

    void setSpeedMultiplier(double factor);
    double getSpeedMultiplier() const;


    const std::vector<Vehicle*>& getVehicles() const;
    const std::vector<Vehicle*>& getFinishedVehicles() const;
    std::size_t getPendingVehicleCount() const;
    std::vector<Vehicle*> getPendingVehicles() const;
    void setMaximumActiveVehicles(std::size_t maximum);
    std::size_t getMaximumActiveVehicles() const;
    const Graph& getGraph() const;
    StatisticsManager* getStatisticsManager() const;

    double getElapsedTime() const;

    // --- Task 4: runtime pathfinding algorithm switching (UI dropdown) ---
    // Swaps the strategy used for all FUTURE routing decisions (new vehicles,
    // event-triggered recalculations, etc) and immediately recalculates the
    // route of every vehicle currently on the road so the change is visible
    // right away. Vehicles for which no new route could be found keep their
    // old route (so they don't vanish/teleport) but their id is recorded in
    // failedRecalcIds so the UI can draw a warning marker above them.
    void setPathFindingStrategy(PathFindingStrategy* strategy);
    PathFindingStrategy* getPathFindingStrategy() const;

    // Ids of vehicles whose most recent recalculation attempt failed.
    const std::set<int>& getFailedRecalcIds() const;

private:
    std::unordered_map<Road*, std::vector<Vehicle*>> mergingFromPOIByRoad_;
    double nextPendingCheckTime_ = 0.0;
    struct PendingVehicle {
        Vehicle* vehicle = nullptr;
        std::vector<Road*> route;
    };

    static constexpr double MAX_RAW_DT = 0.1;
    static constexpr double MAX_SUBSTEP = 0.05;
    static constexpr int MAX_SUBSTEPS_PER_CALL = 200;
    static constexpr double MAX_LEFTOVER_DT = 5.0;
    static constexpr double MIN_SPAWN_HEADWAY_SECONDS = 0.9;


    void recalculateAllVehicleRoutes();
    void removeFinishedVehicles();
    bool tryActivateVehicle(
        Vehicle* vehicle,
        const std::vector<Road*>& route);
    void pruneMergingIndex(Road *road);
    void activatePendingVehicles();

    Graph* graph;                                   
    PathFindingStrategy* pathFindingStrategy;  
    std::vector<Vehicle*> vehicles;
    std::vector<PendingVehicle> pendingVehicles;
    std::vector<Vehicle*> finishedVehicles;      
    std::size_t maximumActiveVehicles_ =
        std::numeric_limits<std::size_t>::max();
    std::unordered_map<const Road*, double>
        nextSpawnTimeByRoad_;
    std::unique_ptr<EventManager> eventManager;     
    std::unique_ptr<StatisticsManager> statisticsManager;               

    bool paused;
    double speedMultiplier;
    double elapsedTime;
    long long tickCount;
    double leftoverDt = 0.0;


    std::set<int> failedRecalcIds;
};

#endif
