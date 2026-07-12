#include "EventManager.h"
#include "StatisticsManager.h"  
#include <iostream>

EventManager::EventManager(Graph* g, std::vector<Vehicle*>* v, PathFindingStrategy* strategy, StatisticsManager* stats)
    : graph(g), vehicles(v), routingStrategy(strategy), statsManager(stats){}

void EventManager::triggerEvent(std::unique_ptr<TrafficEvent> event) {
    if (!graph || !event) return;

    int affectedRoadId = event->getRoadId();
    std::string eventType = event->getEventType(); // Get the type to check it
    
    std::cout << "[Event] Triggered " << eventType 
              << " on Road " << affectedRoadId << "!\n";

    // 1. Apply the event to the graph (e.g., block the road)
    event->apply(*graph);
    
    // 2. Save to the active events list
    activeEvents.push_back(std::move(event));

    // 3. Notify vehicles to dynamically update their routes
    if (eventType != "Traffic Light Delay") {
        notifyAffectedVehicles(affectedRoadId);
    }
}

void EventManager::notifyAffectedVehicles(int roadId) {
    if (!vehicles || !routingStrategy) return;

    for (Vehicle* v : *vehicles) {
        if (v->isRoadInUpcomingRoute(roadId)) {
            std::cout << "[Dynamic Routing] Vehicle " << v->getId() 
                      << " is recalculating route to avoid Road " << roadId << "...\n";
            bool success = v->recalculateRoute(*graph, routingStrategy);

            if (success && statsManager){
                statsManager->recordRecalculation(v->getId());
            }   
        } else if (v->getCurrentRoad() != nullptr && v->getCurrentRoad()->getId() == roadId) {
            std::cout << "[Dynamic Routing] Vehicle " << v->getId() 
                      << " is trapped on Road " << roadId << ". Stopping and waiting...\n";
            // Do NOT U-turn instantly to prevent visual teleportation.
            // The vehicle will automatically stop because the road is blocked (speed -> 0).
        }
    }
}

void EventManager::update(double dt) {
    // Iterate through events and update their time. Remove if expired.
    for (auto it = activeEvents.begin(); it != activeEvents.end(); ) {
        (*it)->update(dt, *graph);
        
        if (!(*it)->isActive()) {
            std::cout << "[Event] " << (*it)->getEventType() 
                      << " on Road " << (*it)->getRoadId() << " resolved.\n";
            it = activeEvents.erase(it);
        } else {
            ++it;
        }
    }
}