#include "TrafficSimulator.h"
#include "../model/Graph.h"
#include "../model/Road.h"
#include "../model/Lane.h"
#include "../model/Vehicle.h"
#include "../model/Intersection.h"
#include "algorithm/PathFindingStrategy.h"
#include <algorithm> 
#include <cmath>
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

    const bool isPOI = (vehicle->getSpawnPOI() != nullptr && vehicle->getSpawnPOI()->getConnectedRoad() == road);
    double spawnProgress = vehicle->getLength() * 0.5;
    int selectedLane = -1;

    if (isPOI) {
        spawnProgress = vehicle->getSpawnPOI()->getProgressOffset();
        int curbLane = road->getCurbLaneIndex();
        
        // Check if there's already a merging vehicle at/near this offset
        pruneMergingIndex(road);   // drop any vehicles that finished merging
        auto mergingIt = mergingFromPOIByRoad_.find(road);
        if (mergingIt != mergingFromPOIByRoad_.end()) {
            for (Vehicle* existing : mergingIt->second) {
                double dist = std::fabs(existing->getProgressOnRoad() - spawnProgress);
                if (dist < vehicle->getLength() + existing->getLength()) {
                    return false;
                }
            }
        }
        
        // Check clearance with vehicles already on the curb lane
        const Lane& curbLaneRef = road->getLane(curbLane);
        for (Vehicle* laneVeh : curbLaneRef.getVehicles()) {
            if (!laneVeh) continue;
            double dist = std::fabs(laneVeh->getProgressOnRoad() - spawnProgress);
            double halfLengths = (laneVeh->getLength() + vehicle->getLength()) * 0.5;
            double requiredGap = std::max(vehicle->getMinGap(), laneVeh->getMinGap());
            if (dist < halfLengths + requiredGap) {
                return false;
            }
        }
        
        selectedLane = curbLane;
        vehicle->setMergingFromPOI(true, spawnProgress, selectedLane);
        mergingFromPOIByRoad_[road].push_back(vehicle);
    } else {
        double bestClearance = -std::numeric_limits<double>::infinity();
        for (int laneIndex = 0; laneIndex < road->getLaneCount(); ++laneIndex) {
            const Lane& lane = road->getLane(laneIndex);
            if (lane.isBlocked() || lane.getVehicleCount() >= lane.getCapacity()) {
                continue;
            }
            Intersection* entrance = road->getStart();
            if (entrance != nullptr && entrance->isOutgoingLaneReserved(road, laneIndex)) {
                continue;
            }

            Vehicle* first = road->getFirstVehicleInLane(laneIndex);
            double clearance = std::numeric_limits<double>::infinity();
            if (first != nullptr) {
                clearance = first->getProgressOnRoad() - spawnProgress - (first->getLength() + vehicle->getLength()) * 0.5;
                const double requiredGap = std::max(vehicle->getMinGap(), first->getMinGap());
                if (clearance + 1e-9 < requiredGap) continue;
            }
            if (selectedLane < 0 || clearance > bestClearance) {
                selectedLane = laneIndex;
                bestClearance = clearance;
            }
        }
    }

    if (selectedLane < 0 || !vehicle->setRouteAt(route, selectedLane, spawnProgress)) {
        if (isPOI) {
            vehicle->setMergingFromPOI(false); // Revert state if activation failed
        }
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

void TrafficSimulator::pruneMergingIndex(Road* road) {
    auto it = mergingFromPOIByRoad_.find(road);
    if (it == mergingFromPOIByRoad_.end()) return;
    auto& list = it->second;
    list.erase(
        std::remove_if(list.begin(), list.end(),
            [](Vehicle* v) { return !v->getIsMergingFromPOI(); }),
        list.end());
    if (list.empty()) {
        mergingFromPOIByRoad_.erase(it);
    }
}

void TrafficSimulator::activatePendingVehicles() {
    if (vehicles.size() >= maximumActiveVehicles_) {
        return;
    }
    if (pendingVehicles.empty()) {
        return;
    }
    // Skip the full scan on frames where no spawn-gate could plausibly
    // have opened up since the last check.
    if (elapsedTime + 1e-9 < nextPendingCheckTime_) {
        return;
    }

    double soonestGateTime = std::numeric_limits<double>::infinity();
    for (auto it = pendingVehicles.begin();
         it != pendingVehicles.end();) {
        Road* frontRoad =
            it->route.empty() ? nullptr : it->route.front();
        if (frontRoad != nullptr) {
            const auto gateIt = nextSpawnTimeByRoad_.find(frontRoad);
            if (gateIt != nextSpawnTimeByRoad_.end() &&
                elapsedTime + 1e-9 < gateIt->second) {
                soonestGateTime =
                    std::min(soonestGateTime, gateIt->second);
                ++it;
                continue;
            }
        }
        if (tryActivateVehicle(it->vehicle, it->route)) {
            it = pendingVehicles.erase(it);
        } else {
            ++it;
        }
        if (vehicles.size() >= maximumActiveVehicles_) {
            break;
        }
    }
    nextPendingCheckTime_ =
        std::isfinite(soonestGateTime) ? soonestGateTime : elapsedTime + 0.05;
}

bool TrafficSimulator::addVehicle(Vehicle* vehicle) {
    if (!vehicle || !graph || !pathFindingStrategy)
    {delete vehicle; return false;}

    // Determine start/end intersection IDs for pathfinding
    int startId = -1;
    int destId = -1;
    
    if (vehicle->getSpawnPOI() && vehicle->getSpawnPOI()->getConnectedRoad()) {
        startId = vehicle->getSpawnPOI()->getConnectedRoad()->getEnd()->getId();
    } else if (vehicle->getSpawnPoint()) {
        startId = vehicle->getSpawnPoint()->getId();
    }
    
    if (vehicle->getTargetPOI() && vehicle->getTargetPOI()->getConnectedRoad()) {
        destId = vehicle->getTargetPOI()->getConnectedRoad()->getStart()->getId();
    } else if (vehicle->getDestination()) {
        destId = vehicle->getDestination()->getId();
    }
    
    if (startId < 0 || destId < 0) {
        delete vehicle;
        return false;
    }
    
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

    if (vehicle->getSpawnPOI() && vehicle->getSpawnPOI()->getConnectedRoad()) {
        result.roadPath.insert(result.roadPath.begin(), vehicle->getSpawnPOI()->getConnectedRoad());
    }
    if (vehicle->getTargetPOI() && vehicle->getTargetPOI()->getConnectedRoad()) {
        result.roadPath.push_back(vehicle->getTargetPOI()->getConnectedRoad());
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
            if (v->getIsMergingFromPOI()) {
                v->setMergingFromPOI(false);
            }

            v->setRouteAt(std::vector<Road*>{}, -1, 0.0);
            finishedVehicles.push_back(v);
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
