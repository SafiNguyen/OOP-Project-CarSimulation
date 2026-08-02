#include "EventManager.h"
#include "StatisticsManager.h"  
#include <iostream>
#include <algorithm>

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

void EventManager::setRoutingStrategy(PathFindingStrategy* strategy) {
    if (strategy != nullptr) {
        routingStrategy = strategy;
    }
}

std::vector<TrafficEventSnapshot>
EventManager::captureSnapshot() const {
    std::vector<TrafficEventSnapshot> snapshot;
    snapshot.reserve(activeEvents.size());
    for (const auto& event : activeEvents) {
        if (event == nullptr || !event->isActive()) {
            continue;
        }
        TrafficEventSnapshot entry;
        entry.roadId = event->getRoadId();
        entry.duration = event->getDuration();
        entry.timeElapsed = event->getTimeElapsed();
        if (const auto* congestion =
                dynamic_cast<const CongestionEvent*>(event.get())) {
            entry.kind = TrafficEventKind::Congestion;
            entry.severity = congestion->getSeverity();
        } else if (const auto* accident =
                       dynamic_cast<const AccidentEvent*>(event.get())) {
            entry.kind = TrafficEventKind::Accident;
            entry.laneIndex = accident->getLaneIndex();
        } else if (dynamic_cast<const RoadClosureEvent*>(event.get()) !=
                   nullptr) {
            entry.kind = TrafficEventKind::RoadClosure;
        } else {
            continue;
        }
        snapshot.push_back(std::move(entry));
    }
    return snapshot;
}

void EventManager::restoreSnapshot(
    const std::vector<TrafficEventSnapshot>& snapshot) {
    // Road blocked/congestion flags are restored independently by the
    // simulator. Rebuild only the event timers here so a future expiry can
    // remove the already-restored effect at the correct simulated time.
    activeEvents.clear();
    for (const TrafficEventSnapshot& entry : snapshot) {
        std::unique_ptr<TrafficEvent> event;
        switch (entry.kind) {
            case TrafficEventKind::Congestion:
                event = std::make_unique<CongestionEvent>(
                    entry.roadId, entry.duration, entry.severity);
                break;
            case TrafficEventKind::Accident:
                event = std::make_unique<AccidentEvent>(
                    entry.roadId, entry.duration, entry.laneIndex);
                break;
            case TrafficEventKind::RoadClosure:
                event = std::make_unique<RoadClosureEvent>(
                    entry.roadId, entry.duration);
                break;
        }
        if (event == nullptr) {
            continue;
        }
        event->restoreTimeElapsed(entry.timeElapsed);
        if (event->isActive()) {
            activeEvents.push_back(std::move(event));
        }
    }
}
