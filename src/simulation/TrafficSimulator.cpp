#include "TrafficSimulator.h"
#include "model/Graph.h"
#include "model/Vehicle.h"
#include "algorithm/PathFindingStrategy.h"
#include <algorithm> 
#include <iostream>
#include "EventManager.h"
#include "StatisticsManager.h"

TrafficSimulator::TrafficSimulator(Graph* graph, PathFindingStrategy* strategy)
    : graph(graph),
      pathFindingStrategy(strategy),
      paused(false),
      speedMultiplier(1.0),
      elapsedTime(0.0),
      tickCount(0)
{
    eventManager = std::make_unique<EventManager>(graph, &vehicles, strategy);
    statisticsManager = std::make_unique<StatisticsManager>();
}

TrafficSimulator::~TrafficSimulator() {
    eventManager.reset();
    statisticsManager.reset();
    for (Vehicle* v : vehicles) {
        delete v;
    }
    vehicles.clear();
}

bool TrafficSimulator::addVehicle(Vehicle* vehicle) {
    if (!vehicle || !graph || !pathFindingStrategy) return false;

    int startId = vehicle->getSpawnPoint()->getId();
    int destId = vehicle->getDestination()->getId();
    
    PathResult result = pathFindingStrategy->findPath(*graph, startId, destId);
    
    if (result.found) {
        vehicle->setRoute(result.roadPath);
        vehicles.push_back(vehicle);
        return true;
    } else {
        std::cout << "[Simulator] Khong the tim duong cho xe ID: " << vehicle->getId() << "\n";
        delete vehicle; 
        return false;
    }
}

void TrafficSimulator::removeFinishedVehicles() {
    auto it = std::remove_if(vehicles.begin(), vehicles.end(), [this](Vehicle* v) {
        if (v->hasReachedDestination()) {
            std::cout << "[Simulator] Xe ID " << v->getId() << " da den dich!\n";
            if (this->statisticsManager) {
                this->statisticsManager->markVehicleCompleted(v->getId());
            }
            delete v;
            return true;
        }
        return false;
    });
    vehicles.erase(it, vehicles.end());
}

void TrafficSimulator::triggerEvent(std::unique_ptr<TrafficEvent> event) {
    if (eventManager) {
         eventManager->triggerEvent(std::move(event));
    }
}

void TrafficSimulator::update(double dt) {
    if (paused) return;

    double effectiveDt = dt * speedMultiplier;
    elapsedTime += effectiveDt;

    if(statisticsManager) {
        statisticsManager->recordTick(effectiveDt);
    }

    if (eventManager) {
        eventManager->update(effectiveDt);
    }

    for (Vehicle* v : vehicles) {
        v->update(effectiveDt);

        if (statisticsManager) {
            statisticsManager->recordVehicleTravel(v->getId(), effectiveDt);
        }
    }

    removeFinishedVehicles();
    tickCount++;
    if (statisticsManager && tickCount % 10 == 0) {
        statisticsManager->printPeriodicReport(tickCount, 10);
    }
}

void TrafficSimulator::pause() { paused = true; }
void TrafficSimulator::resume() { paused = false; }
bool TrafficSimulator::isPaused() const { return paused; }

void TrafficSimulator::setSpeedMultiplier(double factor) {
    if (factor > 0.0) speedMultiplier = factor;
}
double TrafficSimulator::getSpeedMultiplier() const { return speedMultiplier; }

const std::vector<Vehicle*>& TrafficSimulator::getVehicles() const { return vehicles; }
const Graph& TrafficSimulator::getGraph() const { return *graph; }
StatisticsManager* TrafficSimulator::getStatisticsManager() const { return statisticsManager.get(); }
double TrafficSimulator::getElapsedTime() const { return elapsedTime; }