#include "TrafficSimulator.h"
#include "Graph.h"
#include "Road.h"
#include "Lane.h"
#include "Vehicle.h"
#include "Intersection.h"
#include "Bus.h"
#include "PointOfInterest.h"
#include "algorithm/PathFindingStrategy.h"
#include <algorithm> 
#include <chrono>
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

void TrafficSimulator::pruneMergingIndex(Road* road)
{
    auto it = mergingFromPOIByRoad_.find(road);
    if (it == mergingFromPOIByRoad_.end())
        return;

    auto& vehicles = it->second;

    vehicles.erase(
        std::remove_if(
            vehicles.begin(),
            vehicles.end(),
            [](Vehicle* v)
            {
                return v == nullptr ||
                       !v->getIsMergingFromPOI();
            }),
        vehicles.end());

    if (vehicles.empty())
        mergingFromPOIByRoad_.erase(it);
}

bool TrafficSimulator::tryActivateVehicle(
    Vehicle* vehicle,
    const std::vector<Road*>& route) {
    if (vehicle == nullptr) {
        return false;
    }
    if (vehicles.size() >= maximumActiveVehicles_) {
        vehicle->setSpawnLifecycleState(
            SpawnLifecycleState::WaitingForRoadGap);
        return false;
    }
    if (route.empty()) {
        vehicle->setRoute(route);
        vehicle->setSpawnLifecycleState(
            SpawnLifecycleState::Active);
        vehicles.push_back(vehicle);
        ++spawnStatistics_.activated;
        return true;
    }

    Road* road = route.front();
    Bus* transitBus =
        vehicle->getVehicleKind() == VehicleKind::Bus
            ? static_cast<Bus*>(vehicle)
            : nullptr;
    const BusService* transitService =
        transitBus != nullptr
            ? transitBus->getService()
            : nullptr;
    if (transitBus != nullptr &&
        elapsedTime + 1e-9 <
            transitBus->
                getScheduledDepartureTime()) {
        vehicle->setSpawnLifecycleState(
            SpawnLifecycleState::Scheduled);
        return false;
    }
    if (transitService != nullptr &&
        elapsedTime + 1e-9 <
            nextTransitNetworkDepartureTime_) {
        vehicle->setSpawnLifecycleState(
            SpawnLifecycleState::WaitingForRoadGap);
        return false;
    }
    if (transitService != nullptr) {
        const auto departureGate =
            nextTransitDepartureTimeByService_.find(
                transitService);
        if (departureGate !=
                nextTransitDepartureTimeByService_.end() &&
            elapsedTime + 1e-9 <
                departureGate->second) {
            vehicle->setSpawnLifecycleState(
                SpawnLifecycleState::WaitingForRoadGap);
            return false;
        }
    }

    const PointOfInterest* source =
        vehicle->getSpawnPOI();
    if (source != nullptr) {
        const auto sourceGate =
            nextSpawnTimeBySource_.find(source);
        if (sourceGate !=
                nextSpawnTimeBySource_.end() &&
            elapsedTime + 1e-9 <
                sourceGate->second) {
            vehicle->setSpawnLifecycleState(
                SpawnLifecycleState::
                    WaitingForSourceCapacity);
            return false;
        }
        if (!vehicle->tryReserveSpawnSlot()) {
            return false;
        }
    }
    if (transitBus != nullptr &&
        transitBus->hasTransitService() &&
        !transitBus->tryAcquireDepartureSlot()) {
        vehicle->setSpawnLifecycleState(
            SpawnLifecycleState::
                WaitingForSourceCapacity);
        return false;
    }

    if (road == nullptr || road->isBlocked() ||
        road->getDistance() < vehicle->getLength()) {
        vehicle->setSpawnLifecycleState(
            SpawnLifecycleState::WaitingForRoadGap);
        return false;
    }
    const auto spawnGate =
        nextSpawnTimeByRoad_.find(road);
    if (spawnGate != nextSpawnTimeByRoad_.end() &&
        elapsedTime + 1e-9 < spawnGate->second) {
        vehicle->setSpawnLifecycleState(
            SpawnLifecycleState::WaitingForRoadGap);
        return false;
    }

    const bool isPOI =
        vehicle->getSpawnPOI() != nullptr &&
        vehicle->getSpawnPOI()->getConnectedRoad() ==
            road;
    const bool isTransitStation =
        transitBus != nullptr &&
        transitBus->hasTransitService() &&
        transitBus->getOriginStation() != nullptr &&
        transitBus->getOriginStation()->
                getDepartureRoad() == road;
    const PointOfInterest* accessSource =
        isPOI
            ? vehicle->getSpawnPOI()
            : (isTransitStation
                   ? transitBus->getOriginStation()
                   : nullptr);
    const double halfVehicleLength =
        vehicle->getLength() * 0.5;
    double spawnProgress = halfVehicleLength;
    int selectedLane = -1;

    if (isPOI || isTransitStation) {
        if (accessSource != nullptr) {
            spawnProgress =
                accessSource->
                    getProgressOffset();
        }
        spawnProgress = std::clamp(
            spawnProgress,
            halfVehicleLength,
            road->getDistance() -
                halfVehicleLength);
        const int configuredLane =
            accessSource != nullptr
                ? accessSource->
                      getAccessLaneIndex()
                : -1;
        const int accessLane =
            configuredLane >= 0
                ? configuredLane
                : road->getCurbLaneIndex();
        if (accessLane < 0 ||
            accessLane >= road->getLaneCount()) {
            vehicle->setSpawnLifecycleState(
                SpawnLifecycleState::
                    WaitingForRoadGap);
            return false;
        }
        const Lane& lane =
            road->getLane(accessLane);
        if (lane.isBlocked()) {
            vehicle->setSpawnLifecycleState(
                SpawnLifecycleState::
                    WaitingForRoadGap);
            return false;
        }

        // Check if there's already a merging vehicle at/near this offset.
        // Use mergingFromPoiVehicles set for O(log N) instead of O(N) scan.
        pruneMergingIndex(road);

        auto mergeIt = mergingFromPOIByRoad_.find(road);

        if (mergeIt != mergingFromPOIByRoad_.end()) {
            for (Vehicle* existing : mergeIt->second) {

                const double dist =
                    std::fabs(existing->getProgressOnRoad() -
                            spawnProgress);

                const double halfLengths =
                    (existing->getLength() +
                    vehicle->getLength()) * 0.5;

                const double requiredGap =
                    std::max(existing->getMinGap(),
                            vehicle->getMinGap());

                const double mergeApproachBuffer =
                    road->getSpeedLimit() *
                    vehicle->getPoiAnimationDuration();

                if (dist <
                    halfLengths +
                    requiredGap +
                    mergeApproachBuffer)
                {
                    vehicle->setSpawnLifecycleState(
                        SpawnLifecycleState::WaitingForRoadGap);
                    return false;
                }
            }
        }

        // Road traffic does not prevent a POI vehicle from approaching its
        // yield line. The vehicle waits outside the carriageway and performs
        // the speed-, braking-, and TTC-aware gap check immediately before it
        // commits to the lane. This avoids both invisible queued spawns and a
        // gap becoming stale during the driveway animation.

        selectedLane = accessLane;
        if (accessSource != nullptr) {
            vehicle->setMergeSourcePOI(
                accessSource);
            vehicle->setMergingFromPOI(
                true,
                spawnProgress,
                selectedLane);

            mergingFromPOIByRoad_[road].push_back(vehicle);
        }
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
        if (accessSource != nullptr) {
           vehicle->setMergingFromPOI(false);
            pruneMergingIndex(road);
        }
        vehicle->setSpawnLifecycleState(
            SpawnLifecycleState::WaitingForRoadGap);
        return false;
    }
    vehicles.push_back(vehicle);
    ++spawnStatistics_.activated;
    if (transitBus != nullptr &&
        transitBus->hasTransitService()) {
        transitBus->notifyActivatedFromStation();
    }
    if (transitService != nullptr) {
        nextTransitNetworkDepartureTime_ =
            elapsedTime +
            TRANSIT_NETWORK_HEADWAY_SECONDS;
        nextTransitDepartureTimeByService_[
            transitService] =
                elapsedTime +
                TRANSIT_DEPARTURE_HEADWAY_SECONDS;
    }
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
    if (source != nullptr) {
        nextSpawnTimeBySource_[source] =
            elapsedTime +
            source->getSpawnCooldownSeconds();
    }
    return true;
}

bool TrafficSimulator::resolveRoute(
    Vehicle* vehicle,
    std::vector<Road*>& route) {
    route.clear();
    if (vehicle == nullptr || graph == nullptr ||
        pathFindingStrategy == nullptr) {
        return false;
    }

    PointOfInterest* origin =
        vehicle->getSpawnPOI();
    const PointOfInterest* destination =
        vehicle->getTargetPOI();
    Road* originRoad =
        origin != nullptr
            ? origin->getConnectedRoad()
            : nullptr;
    Road* destinationRoad =
        destination != nullptr
            ? destination->getConnectedRoad()
            : nullptr;

    if (origin != nullptr &&
        (!origin->isSpawnPoint() ||
         originRoad == nullptr)) {
        return false;
    }
    if (destination != nullptr &&
        (!destination->isDestination() ||
         destinationRoad == nullptr)) {
        return false;
    }
    if ((originRoad != nullptr &&
         originRoad->isBlocked()) ||
        (destinationRoad != nullptr &&
         destinationRoad->isBlocked())) {
        return false;
    }

    if (originRoad != nullptr &&
        originRoad == destinationRoad &&
        destination->getProgressOffset() + 1e-9 >=
            origin->getProgressOffset()) {
        route.push_back(originRoad);
        return true;
    }

    int startId = -1;
    int destinationId = -1;
    if (originRoad != nullptr) {
        startId = originRoad->getEnd()->getId();
    } else if (vehicle->getSpawnPoint() != nullptr) {
        startId =
            vehicle->getSpawnPoint()->getId();
    }
    if (destinationRoad != nullptr) {
        destinationId =
            destinationRoad->getStart()->getId();
    } else if (vehicle->getDestination() != nullptr) {
        destinationId =
            vehicle->getDestination()->getId();
    }
    if (startId < 0 || destinationId < 0) {
        return false;
    }

    PathResult result;
    if (statisticsManager != nullptr) {
        result =
            statisticsManager->measurePathfinding(
                *pathFindingStrategy,
                *graph,
                startId,
                destinationId);
    } else {
        result =
            pathFindingStrategy->findPath(
                *graph,
                startId,
                destinationId);
    }
    if (!result.found) {
        return false;
    }

    if (originRoad != nullptr) {
        route.push_back(originRoad);
    }
    for (Road* road : result.roadPath) {
        if (road == nullptr ||
            graph->getRoad(road->getId()) != road ||
            road->isBlocked()) {
            route.clear();
            return false;
        }
        if (!route.empty() &&
            route.back()->getEnd() !=
                road->getStart()) {
            route.clear();
            return false;
        }
        route.push_back(road);
    }
    if (destinationRoad != nullptr) {
        if (!route.empty() &&
            route.back()->getEnd() !=
                destinationRoad->getStart()) {
            route.clear();
            return false;
        }
        route.push_back(destinationRoad);
    }
    return !route.empty() ||
           startId == destinationId;
}

bool TrafficSimulator::routeNeedsRefresh(
    const PendingVehicle& pending) const {
    if (pending.fixedRoute ||
        !pending.routeResolved) {
        return false;
    }
    for (Road* road : pending.route) {
        if (road == nullptr ||
            graph == nullptr ||
            graph->getRoad(road->getId()) != road ||
            road->isBlocked()) {
            return true;
        }
    }
    return false;
}

void TrafficSimulator::discardPendingVehicle(
    PendingVehicle& pending,
    bool timedOut) {
    if (timedOut) {
        ++spawnStatistics_.timedOut;
    } else {
        ++spawnStatistics_.rejected;
    }
    delete pending.vehicle;
    pending.vehicle = nullptr;
}

void TrafficSimulator::activatePendingVehicles() {
    if (vehicles.size() >= maximumActiveVehicles_ ||
        pendingVehicles.empty()) {
        return;
    }

    const std::size_t inspections =
        std::min(
            pendingVehicles.size(),
            MAX_PENDING_INSPECTIONS_PER_UPDATE);
    int phasedActivations = 0;
    for (std::size_t inspection = 0u;
         inspection < inspections &&
         !pendingVehicles.empty();
         ++inspection) {
        PendingVehicle pending =
            std::move(pendingVehicles.front());
        pendingVehicles.pop_front();

        if (pending.vehicle == nullptr) {
            continue;
        }
        if (elapsedTime + 1e-9 <
                pending.earliestActivationTime ||
            elapsedTime + 1e-9 <
                pending.nextAttemptTime ||
            (pending.phasedAdmission &&
             phasedActivations >=
                 MAX_PHASED_ACTIVATIONS_PER_ADMISSION_PASS)) {
            pendingVehicles.push_back(
                std::move(pending));
            continue;
        }
        if (elapsedTime >
            pending.deadlineTime + 1e-9) {
            discardPendingVehicle(
                pending, true);
            continue;
        }

        if (routeNeedsRefresh(pending)) {
            pending.route.clear();
            pending.routeResolved = false;
        }
        if (!pending.routeResolved) {
            pending.vehicle->
                setSpawnLifecycleState(
                    SpawnLifecycleState::
                        WaitingForRoute);
            if (!resolveRoute(
                    pending.vehicle,
                    pending.route)) {
                ++pending.routeAttempts;
                ++spawnStatistics_.delayedAttempts;
                if (pending.routeAttempts >=
                    MAX_ROUTE_ATTEMPTS) {
                    discardPendingVehicle(
                        pending, false);
                    continue;
                }
                const double backoff =
                    std::min(
                        5.0,
                        0.25 *
                            std::pow(
                                2.0,
                                pending.routeAttempts -
                                    1));
                pending.nextAttemptTime =
                    elapsedTime + backoff;
                pendingVehicles.push_back(
                    std::move(pending));
                continue;
            }
            pending.routeResolved = true;
        }

        if (tryActivateVehicle(
                pending.vehicle,
                pending.route)) {
            if (pending.phasedAdmission) {
                ++phasedActivations;
            }
        } else {
            ++spawnStatistics_.delayedAttempts;
            pending.nextAttemptTime =
                elapsedTime + 0.1;
            pendingVehicles.push_back(
                std::move(pending));
        }
        if (vehicles.size() >=
            maximumActiveVehicles_) {
            break;
        }
    }
}

bool TrafficSimulator::addVehicle(Vehicle* vehicle) {
    return addVehicleWithDelay(vehicle, 0.0);
}

bool TrafficSimulator::scheduleVehicleSpawn(
    Vehicle* vehicle,
    double delaySeconds) {
    if (!std::isfinite(delaySeconds) ||
        delaySeconds < 0.0) {
        ++spawnStatistics_.rejected;
        delete vehicle;
        return false;
    }
    return addVehicleWithDelay(
        vehicle, delaySeconds);
}

bool TrafficSimulator::addVehicleWithDelay(
    Vehicle* vehicle,
    double delaySeconds) {
    if (!vehicle || !graph || !pathFindingStrategy)
    {
        ++spawnStatistics_.rejected;
        delete vehicle;
        return false;
    }

    const bool hasOrigin =
        (vehicle->getSpawnPOI() != nullptr &&
         vehicle->getSpawnPOI()->
             getConnectedRoad() != nullptr &&
         vehicle->getSpawnPOI()->
             isSpawnPoint()) ||
        vehicle->getSpawnPoint() != nullptr;
    const bool hasDestination =
        (vehicle->getTargetPOI() != nullptr &&
         vehicle->getTargetPOI()->
             getConnectedRoad() != nullptr &&
         vehicle->getTargetPOI()->
             isDestination()) ||
        vehicle->getDestination() != nullptr;
    if (!hasOrigin || !hasDestination) {
        ++spawnStatistics_.rejected;
        delete vehicle;
        return false;
    }

    const bool phasedAdmission =
        delaySeconds > 0.0;
    const double earliestActivationTime =
        elapsedTime +
        std::max(0.0, delaySeconds);
    PendingVehicle pending;
    pending.vehicle = vehicle;
    pending.earliestActivationTime =
        earliestActivationTime;
    pending.nextAttemptTime =
        earliestActivationTime;
    pending.deadlineTime =
        earliestActivationTime +
        pendingVehicleTimeoutSeconds_;
    pending.phasedAdmission =
        phasedAdmission;
    vehicle->setSpawnLifecycleState(
        phasedAdmission
            ? SpawnLifecycleState::Scheduled
            : SpawnLifecycleState::WaitingForRoute);

    try {
        if (!phasedAdmission) {
            if (!resolveRoute(
                    vehicle,
                    pending.route)) {
                ++spawnStatistics_.rejected;
                delete vehicle;
                return false;
            }
            pending.routeResolved = true;
            if (tryActivateVehicle(
                    vehicle,
                    pending.route)) {
                ++spawnStatistics_.accepted;
                return true;
            }
        }
        pendingVehicles.push_back(
            std::move(pending));
        ++spawnStatistics_.accepted;
        return true;
    } catch (...) {
        ++spawnStatistics_.rejected;
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
                Road* road = v->getCurrentRoad();

                v->setMergingFromPOI(false);

                if (road != nullptr)
                    pruneMergingIndex(road);
            }

            finishedVehicles.push_back(v); // Keep vehicle instead of deleting
            return true;
        }
        return false;
    });
    vehicles.erase(it, vehicles.end());
    
    // Cap the size of finishedVehicles to prevent memory bloat
    const size_t MAX_FINISHED_VEHICLES = 500;
    while (finishedVehicles.size() > MAX_FINISHED_VEHICLES) {
        delete finishedVehicles.front();
        finishedVehicles.erase(finishedVehicles.begin());
    }
}

bool TrafficSimulator::addVehicleWithFixedRoute(
    Vehicle* vehicle,
    const std::vector<Road*>& route) {
    if (vehicle == nullptr || graph == nullptr ||
        route.empty() ||
        vehicle->getCurrentRoute() != route) {
        ++spawnStatistics_.rejected;
        delete vehicle;
        return false;
    }
    for (std::size_t index = 0;
         index < route.size();
         ++index) {
        Road* road = route[index];
        if (road == nullptr ||
            graph->getRoad(road->getId()) != road ||
            (index > 0 &&
             route[index - 1]->getEnd() !=
                 road->getStart())) {
            ++spawnStatistics_.rejected;
            delete vehicle;
            return false;
        }
    }

    try {
        double earliestActivationTime =
            elapsedTime;
        if (vehicle->getVehicleKind() ==
                VehicleKind::Bus) {
            const auto* bus =
                static_cast<const Bus*>(vehicle);
            if (bus->hasTransitService()) {
                earliestActivationTime =
                    std::max(
                        earliestActivationTime,
                        bus->
                            getScheduledDepartureTime());
            }
        }

        if (earliestActivationTime <=
                elapsedTime + 1e-9 &&
            tryActivateVehicle(vehicle, route)) {
            ++spawnStatistics_.accepted;
            return true;
        }

        PendingVehicle pending;
        pending.vehicle = vehicle;
        pending.route = route;
        pending.earliestActivationTime =
            earliestActivationTime;
        pending.nextAttemptTime =
            earliestActivationTime;
        pending.deadlineTime =
            std::numeric_limits<double>::
                infinity();
        pending.routeResolved = true;
        pending.fixedRoute = true;
        vehicle->setSpawnLifecycleState(
            earliestActivationTime >
                    elapsedTime + 1e-9
                ? SpawnLifecycleState::Scheduled
                : SpawnLifecycleState::
                      WaitingForRoadGap);
        pendingVehicles.push_back(
            std::move(pending));
        ++spawnStatistics_.accepted;
        return true;
    } catch (...) {
        ++spawnStatistics_.rejected;
        delete vehicle;
        throw;
    }
}

void TrafficSimulator::triggerEvent(std::unique_ptr<TrafficEvent> event) {
    if (eventManager) {
         eventManager->triggerEvent(std::move(event));
    }
}

void TrafficSimulator::update(double dt) {
    if (paused) return;

    // High-resolution profiling using std::chrono
    static int profileFrames = 0;
    static double activateTime = 0.0;
    static double trafficLightTime = 0.0;
    static double eventTime = 0.0;
    static double vehicleTime = 0.0;
    static double removeTime = 0.0;
    static double totalTime = 0.0;

    auto frameStart = std::chrono::high_resolution_clock::now();

    double safeDt = std::clamp(dt, 0.0, MAX_RAW_DT);
    double remaining = leftoverDt + safeDt * speedMultiplier;
    leftoverDt = 0.0;

    int stepsRun = 0;

    // Admission runs once per frame, not per substep, to keep scheduled
    // traffic density independent of the frame rate / speed multiplier.
    {
        auto start = std::chrono::high_resolution_clock::now();
        activatePendingVehicles();
        auto end = std::chrono::high_resolution_clock::now();
        activateTime += std::chrono::duration<double, std::micro>(end - start).count();
    }

    while (remaining > 0.0 && stepsRun < MAX_SUBSTEPS_PER_CALL) {

        double step = std::min(remaining, MAX_SUBSTEP);

        elapsedTime += step;

        if (statisticsManager) {
            statisticsManager->recordTick(step);
        }


        if (graph) {
            {
                auto start = std::chrono::high_resolution_clock::now();
                for (Intersection* intersection : graph->getAllIntersections()) {
                    intersection->updateTrafficLights(step);
                }
                auto end = std::chrono::high_resolution_clock::now();
                trafficLightTime += std::chrono::duration<double, std::micro>(end - start).count();
            }

            // Clear the per-frame cache used by
            // Intersection::hasPriorityVehicleApproaching so the O(N^2)
            // guard is recomputed once per substep rather than once per
            // vehicle at each intersection.
            for (Intersection* intersection :
                 graph->getAllIntersections()) {
                if (intersection != nullptr) {
                    intersection->clearFrameCache();
                }
            }
        }


        {
            auto start = std::chrono::high_resolution_clock::now();
            if (eventManager) {
                eventManager->update(step);
            }
            auto end = std::chrono::high_resolution_clock::now();
            eventTime += std::chrono::duration<double, std::micro>(end - start).count();
        }


        {
            auto start = std::chrono::high_resolution_clock::now();
            const std::size_t vehicleCount = vehicles.size();
            const std::size_t rerouteStart =
                vehicleCount == 0u ? 0u : dynamicRerouteCursor_ % vehicleCount;
            dynamicRerouteCursor_ +=
                MAX_DYNAMIC_REROUTES_PER_SUBSTEP;

            std::size_t vehicleIndex = 0u;
            for (Vehicle* v : vehicles) {
                const std::size_t offset =
                    (vehicleIndex + vehicleCount - rerouteStart) %
                    (vehicleCount == 0u ? 1u : vehicleCount);
                const bool allowDynamicReroute =
                    offset < MAX_DYNAMIC_REROUTES_PER_SUBSTEP;
                ++vehicleIndex;

                v->update(
                    step, graph, pathFindingStrategy, allowDynamicReroute);

                if (statisticsManager) {
                    statisticsManager->recordVehicleTravel(v->getId(), step);
                }
            }
            auto end = std::chrono::high_resolution_clock::now();
            vehicleTime += std::chrono::duration<double, std::micro>(end - start).count();
        }


        {
            auto start = std::chrono::high_resolution_clock::now();
            removeFinishedVehicles();
            auto end = std::chrono::high_resolution_clock::now();
            removeTime += std::chrono::duration<double, std::micro>(end - start).count();
        }


        remaining -= step;
        ++stepsRun;
    }

    leftoverDt = std::min(remaining, MAX_LEFTOVER_DT);

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
    for (PendingVehicle& pending :
         pendingVehicles) {
        if (!pending.fixedRoute) {
            pending.route.clear();
            pending.routeResolved = false;
            pending.routeAttempts = 0;
            pending.nextAttemptTime =
                std::max(
                    elapsedTime,
                    pending.earliestActivationTime);
        }
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
        if (!v->allowsDynamicRerouting()) {
            failedRecalcIds.erase(v->getId());
            continue;
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
void TrafficSimulator::setPendingVehicleTimeout(
    double seconds) {
    if (std::isfinite(seconds) &&
        seconds > 0.0) {
        pendingVehicleTimeoutSeconds_ =
            seconds;
    }
}
const Graph& TrafficSimulator::getGraph() const { return *graph; }
StatisticsManager* TrafficSimulator::getStatisticsManager() const { return statisticsManager.get(); }
double TrafficSimulator::getElapsedTime() const { return elapsedTime; }


