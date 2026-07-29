#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#include <vector>
#include <memory>
#include "TrafficEvent.h"
#include "Vehicle.h"
#include "Graph.h"
#include "../algorithm/PathFindingStrategy.h"
class StatisticsManager;
class EventManager {
private:
    std::vector<std::unique_ptr<TrafficEvent>> activeEvents;
    Graph* graph;
    std::vector<Vehicle*>* vehicles; // Pointer to the list of vehicles in the Simulation Engine
    PathFindingStrategy* routingStrategy; // Routing strategy used for recomputation (e.g., Dijkstra)
    StatisticsManager* statsManager;

public:
    EventManager(Graph* g, std::vector<Vehicle*>* v, PathFindingStrategy* strategy, StatisticsManager* stats);

    // Trigger a random or predefined event
    void triggerEvent(std::unique_ptr<TrafficEvent> event);
    
    // Update the elapsed time of events
    void update(double dt);
    
    // Observer Pattern: Notify affected vehicles
    void notifyAffectedVehicles(int roadId);
    void setRoutingStrategy(PathFindingStrategy* strategy);

};

#endif