#include "TrafficSimulator.h"
#include "../model/Graph.h"
#include "../model/Road.h"
#include "../model/Vehicle.h"
#include "../model/Intersection.h"
#include "algorithm/PathFindingStrategy.h"
#include <algorithm> 
#include <iostream>
#include <limits>
#include <utility>
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
    for (const PendingVehicle& pending : pendingVehicles) {
        delete pending.vehicle;
    }
    vehicles.clear();
    pendingVehicles.clear();
    finishedVehicles.clear();
}

bool TrafficSimulator::tryActivateVehicle(
    Vehicle* vehicle,
    const std::vector<Road*>& route) {
    if (vehicle == nullptr) return false;
    if (vehicles.size() >= maximumActiveVehicles_) {
        return false;
    }
    if (route.empty()) {
        vehicle->setRoute(route);
        vehicles.push_back(vehicle);
        return true;
    }

    Road* road = route.front();
    if (road == nullptr || road->isBlocked() ||
        road->getDistance() < vehicle->getLength()) {
        return false;
    }
    const auto spawnGate =
        nextSpawnTimeByRoad_.find(road);
    if (spawnGate != nextSpawnTimeByRoad_.end() &&
        elapsedTime + 1e-9 < spawnGate->second) {
        return false;
    }

    const double spawnProgress = vehicle->getLength() * 0.5;
    int selectedLane = -1;
    double bestClearance =
        -std::numeric_limits<double>::infinity();
    for (int laneIndex = 0;
         laneIndex < road->getLaneCount();
         ++laneIndex) {
        const Lane& lane = road->getLane(laneIndex);
        if (lane.isBlocked() ||
            lane.getVehicleCount() >= lane.getCapacity()) {
            continue;
        }
        Intersection* entrance = road->getStart();
        if (entrance != nullptr &&
            entrance->isOutgoingLaneReserved(
                road, laneIndex)) {
            continue;
        }

        Vehicle* first =
            road->getFirstVehicleInLane(laneIndex);
        double clearance =
            std::numeric_limits<double>::infinity();
        if (first != nullptr) {
            clearance =
                first->getProgressOnRoad() -
                spawnProgress -
                (first->getLength() +
                 vehicle->getLength()) * 0.5;
            const double requiredGap = std::max(
                vehicle->getMinGap(),
                first->getMinGap());
            if (clearance + 1e-9 < requiredGap) {
                continue;
            }
        }
        if (selectedLane < 0 ||
            clearance > bestClearance) {
            selectedLane = laneIndex;
            bestClearance = clearance;
        }
    }

    if (selectedLane < 0 ||
        !vehicle->setRouteAt(
            route, selectedLane, spawnProgress)) {
        return false;
    }
    vehicles.push_back(vehicle);
    const double deterministicStagger =
        static_cast<double>(
            static_cast<unsigned int>(vehicle->getId()) % 7u) *
        0.04;
    const double vehicleLengthHeadway =
        std::clamp(
            vehicle->getLength() * 0.05,
            0.1,
            0.6);
    nextSpawnTimeByRoad_[road] =
        elapsedTime +
        MIN_SPAWN_HEADWAY_SECONDS +
        vehicleLengthHeadway +
        deterministicStagger;
    return true;
}

void TrafficSimulator::activatePendingVehicles() {
    if (vehicles.size() >= maximumActiveVehicles_) {
        return;
    }
    for (auto it = pendingVehicles.begin();
         it != pendingVehicles.end();) {
        if (tryActivateVehicle(
                it->vehicle, it->route)) {
            it = pendingVehicles.erase(it);
        } else {
            ++it;
        }
        if (vehicles.size() >= maximumActiveVehicles_) {
            break;
        }
    }
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
        if (!tryActivateVehicle(
                vehicle, result.roadPath)) {
            pendingVehicles.push_back(
                PendingVehicle{
                    vehicle,
                    std::move(result.roadPath)
                });
        }
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
            failedRecalcIds.erase(v->getId());
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

    activatePendingVehicles();

    double safeDt = std::clamp(dt, 0.0, MAX_RAW_DT);     
    double remaining = leftoverDt + safeDt * speedMultiplier;
    leftoverDt = 0.0;
    int stepsRun = 0;
    while (remaining > 0.0 && stepsRun < MAX_SUBSTEPS_PER_CALL) {
        double step = std::min(remaining, MAX_SUBSTEP);

        elapsedTime += step;

        if (statisticsManager) {
            statisticsManager->recordTick(step);
        }

        if (graph) {
            for (Intersection* intersection : graph->getAllIntersections()) {
                intersection->updateTrafficLights(step);
            }
        }

        if (eventManager) {
            eventManager->update(step);
        }

        for (Vehicle* v : vehicles) {
            v->update(step, graph, pathFindingStrategy);

            if (statisticsManager) {
                statisticsManager->recordVehicleTravel(v->getId(), step);
            }
        }

        // Don xe da den dich ngay sau moi sub-step, khong doi den cuoi
        // update(): voi speedMultiplier cao, nhieu xe co the hoan thanh
        // route ngay giua chung cac sub-step.
        removeFinishedVehicles();

        remaining -= step;
        ++stepsRun;
    }
    leftoverDt = std::min(remaining, MAX_LEFTOVER_DT); // Save any leftover time for the next update call

    tickCount++;
    if (statisticsManager && tickCount % 600 == 0) {
        statisticsManager->printPeriodicReport(tickCount, 600);
    }
}

void TrafficSimulator::setPathFindingStrategy(PathFindingStrategy* strategy) {
    if (strategy == nullptr) return;
    pathFindingStrategy = strategy;
    std::cout << "[Simulator] Switched pathfinding algorithm to: " << strategy->name() << "\n";
    if (eventManager) {
        eventManager->setRoutingStrategy(strategy);  
    }
    recalculateAllVehicleRoutes();
}

PathFindingStrategy* TrafficSimulator::getPathFindingStrategy() const {
    return pathFindingStrategy;
}

const std::set<int>& TrafficSimulator::getFailedRecalcIds() const {
    return failedRecalcIds;
}

void TrafficSimulator::recalculateAllVehicleRoutes() {
    if (!graph || !pathFindingStrategy) return;

    for (Vehicle* v : vehicles) {
        if (v->getCurrentRoad() == nullptr) {
            continue; // vehicle not actively on a road, nothing to recompute
        }

        bool success = v->recalculateRoute(*graph, pathFindingStrategy);
        if (success) {
            failedRecalcIds.erase(v->getId());
            if (statisticsManager) {
                statisticsManager->recordRecalculation(v->getId());
            }
        } else {
            std::cout << "[Simulator] WARNING: Vehicle " << v->getId()
                      << " could not recalculate route with the new algorithm. "
                      << "Keeping its previous route.\n";
            failedRecalcIds.insert(v->getId());
        }
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
std::size_t TrafficSimulator::getPendingVehicleCount() const {
    return pendingVehicles.size();
}
std::vector<Vehicle*> TrafficSimulator::getPendingVehicles() const {
    std::vector<Vehicle*> result;
    result.reserve(pendingVehicles.size());
    for (const PendingVehicle& pending : pendingVehicles) {
        result.push_back(pending.vehicle);
    }
    return result;
}
void TrafficSimulator::setMaximumActiveVehicles(
    std::size_t maximum) {
    maximumActiveVehicles_ = std::max<std::size_t>(1, maximum);
}
std::size_t TrafficSimulator::getMaximumActiveVehicles() const {
    return maximumActiveVehicles_;
}
const Graph& TrafficSimulator::getGraph() const { return *graph; }
StatisticsManager* TrafficSimulator::getStatisticsManager() const { return statisticsManager.get(); }
double TrafficSimulator::getElapsedTime() const { return elapsedTime; }
