#ifndef TRAFFICSIMULATOR_H
#define TRAFFICSIMULATOR_H

#include <vector>
#include <memory>
#include <set>

class Graph;
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

   
    void removeFinishedVehicles();

    void triggerEvent(std::unique_ptr<TrafficEvent> event);

    void update(double dt);


    void pause();
    void resume();
    bool isPaused() const;

    void setSpeedMultiplier(double factor);
    double getSpeedMultiplier() const;


    const std::vector<Vehicle*>& getVehicles() const;
    const std::vector<Vehicle*>& getFinishedVehicles() const;
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
    void recalculateAllVehicleRoutes();

    Graph* graph;                                   
    PathFindingStrategy* pathFindingStrategy;  
    std::vector<Vehicle*> vehicles;
    std::vector<Vehicle*> finishedVehicles;      
    std::unique_ptr<EventManager> eventManager;     
    std::unique_ptr<StatisticsManager> statisticsManager;               

    bool paused;
    double speedMultiplier;
    double elapsedTime;
    long long tickCount;

    std::set<int> failedRecalcIds;
};

#endif