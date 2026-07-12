#include "TrafficSimulator.h"
#include "../model/Graph.h"
#include "../model/Vehicle.h"
#include "../model/Intersection.h"
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
    statisticsManager = std::make_unique<StatisticsManager>();
    eventManager = std::make_unique<EventManager>(graph, &vehicles, strategy, statisticsManager.get());
}

TrafficSimulator::~TrafficSimulator() {
    eventManager.reset();
    statisticsManager.reset();
    for (Vehicle* v : vehicles) {
        delete v;
    }
    for (Vehicle* v : finishedVehicles) {
        delete v;
    }
    vehicles.clear();
    finishedVehicles.clear();
}

bool TrafficSimulator::addVehicle(Vehicle* vehicle) {
    if (!vehicle || !graph || !pathFindingStrategy)
    {delete vehicle; return false;}

    int startId = vehicle->getSpawnPoint()->getId();
    int destId = vehicle->getDestination()->getId();
    
    PathResult result;
    if (statisticsManager) {
        result = statisticsManager->measurePathfinding(*pathFindingStrategy, *graph, startId, destId);
    } else {
        result = pathFindingStrategy->findPath(*graph, startId, destId);
    }    
    
    if (!result.found) {
         delete vehicle; 
        return false;
    } 
    try
    {
        vehicle->setRoute(result.roadPath);
        vehicles.push_back(vehicle);
        return true;
    }
    catch(...)
    {
        delete vehicle;
        throw;
    }
    
}

void TrafficSimulator::removeFinishedVehicles() {
    auto it = std::remove_if(vehicles.begin(), vehicles.end(), [this](Vehicle* v) {
        if (v->hasReachedDestination()) {
            std::cout << "[Simulator] Xe ID " << v->getId() << " da den dich!\n";
            if (this->statisticsManager) {
                this->statisticsManager->markVehicleCompleted(v->getId());
            }
            finishedVehicles.push_back(v); // Keep vehicle instead of deleting
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
    
    if (graph) {
        for (Intersection* intersection : graph->getAllIntersections()) {
            intersection->updateTrafficLights(effectiveDt);
        }
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
const std::vector<Vehicle*>& TrafficSimulator::getFinishedVehicles() const { return finishedVehicles; }
const Graph& TrafficSimulator::getGraph() const { return *graph; }
StatisticsManager* TrafficSimulator::getStatisticsManager() const { return statisticsManager.get(); }
double TrafficSimulator::getElapsedTime() const { return elapsedTime; }