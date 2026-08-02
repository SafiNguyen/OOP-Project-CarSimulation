#include "TrafficSimulator.h"
#include "Graph.h"
#include "Road.h"
#include "Lane.h"
#include "Vehicle.h"
#include "Intersection.h"
#include "Bus.h"
#include "Car.h"
#include "Motorbike.h"
#include "EmergencyVehicle.h"
#include "PointOfInterest.h"
#include "SpawnPoint.h"
#include "algorithm/PathFindingStrategy.h"
#include <algorithm> 
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <utility>
#include "EventManager.h"
#include "StatisticsManager.h"
#include "SnapshotManager.h"
#include "TimePlaybackController.h"

namespace {

bool snapshotError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
    return false;
}

std::unique_ptr<Vehicle> createSnapshotVehicle(
    const VehicleSnapshot& snapshot) {
    switch (snapshot.kind) {
        case VehicleKind::Car:
            return std::make_unique<Car>(
                snapshot.id, snapshot.baseSpeed, nullptr, nullptr);
        case VehicleKind::Bus:
            return std::make_unique<Bus>(
                snapshot.id, snapshot.baseSpeed, nullptr, nullptr);
        case VehicleKind::Motorbike:
            return std::make_unique<Motorbike>(
                snapshot.id, snapshot.baseSpeed, nullptr, nullptr);
        case VehicleKind::Emergency:
            return std::make_unique<EmergencyVehicle>(
                snapshot.id, snapshot.baseSpeed, nullptr, nullptr);
    }
    return nullptr;
}

bool validateVehicleSnapshot(const Graph& graph,
                             const VehicleSnapshot& snapshot,
                             std::string* error) {
    auto requireRoad =
        [&](const std::optional<int>& roadId, const char* field) -> bool {
        if (roadId.has_value() && graph.getRoad(*roadId) == nullptr) {
            return snapshotError(
                error,
                "Vehicle " + std::to_string(snapshot.id) +
                    " references missing " + field + " " +
                    std::to_string(*roadId) + ".");
        }
        return true;
    };
    auto requireIntersection =
        [&](int intersectionId, const char* field) -> bool {
        if (intersectionId >= 0 &&
            graph.getIntersection(intersectionId) == nullptr) {
            return snapshotError(
                error,
                "Vehicle " + std::to_string(snapshot.id) +
                    " references missing " + field + " " +
                    std::to_string(intersectionId) + ".");
        }
        return true;
    };
    auto requirePoi = [&](int poiId, const char* field) -> bool {
        if (poiId >= 0 &&
            graph.getPointOfInterest(poiId) == nullptr) {
            return snapshotError(
                error,
                "Vehicle " + std::to_string(snapshot.id) +
                    " references missing " + field + " " +
                    std::to_string(poiId) + ".");
        }
        return true;
    };

    if (snapshot.id < 0 ||
        !requireRoad(snapshot.currentRoadId, "current road") ||
        !requireRoad(snapshot.junctionIncomingRoadId, "incoming road") ||
        !requireRoad(snapshot.junctionOutgoingRoadId, "outgoing road") ||
        !requireIntersection(snapshot.spawnPointId, "spawn intersection") ||
        !requireIntersection(snapshot.destinationId, "destination intersection") ||
        !requireIntersection(
            snapshot.reservedIntersectionId, "reserved intersection") ||
        !requirePoi(snapshot.spawnPOIId, "spawn POI") ||
        !requirePoi(snapshot.targetPOIId, "target POI") ||
        !requirePoi(snapshot.mergeSourcePOIId, "merge POI") ||
        !requirePoi(snapshot.reservedSpawnPointId, "reserved spawn POI")) {
        return false;
    }
    if (!std::isfinite(snapshot.baseSpeed) || snapshot.baseSpeed < 0.0 ||
        !std::isfinite(snapshot.progressOnCurrentRoad) ||
        !std::isfinite(snapshot.currentSpeed) ||
        !std::isfinite(snapshot.recalculateTimer)) {
        return snapshotError(
            error,
            "Vehicle " + std::to_string(snapshot.id) +
                " contains invalid numeric state.");
    }
    for (std::size_t routeIndex = 0;
         routeIndex < snapshot.currentRoute.size();
         ++routeIndex) {
        const std::optional<int>& roadId =
            snapshot.currentRoute[routeIndex];
        if (!roadId.has_value() || graph.getRoad(*roadId) == nullptr) {
            return snapshotError(
                error,
                "Vehicle " + std::to_string(snapshot.id) +
                    " has invalid route road " +
                    (roadId.has_value()
                         ? std::to_string(*roadId)
                         : std::string("<null>")) +
                    " at index " +
                    std::to_string(routeIndex) + ".");
        }
    }
    for (const std::optional<int>& roadId : snapshot.travelHistory) {
        if (!roadId.has_value() || graph.getRoad(*roadId) == nullptr) {
            return snapshotError(
                error,
                "Vehicle " + std::to_string(snapshot.id) +
                    " has an invalid travel-history road.");
        }
    }
    Road* currentRoad = snapshot.currentRoadId.has_value()
        ? graph.getRoad(*snapshot.currentRoadId)
        : nullptr;
    if (currentRoad != nullptr &&
        (snapshot.currentLaneIndex < 0 ||
         snapshot.currentLaneIndex >= currentRoad->getLaneCount())) {
        return snapshotError(
            error,
            "Vehicle " + std::to_string(snapshot.id) +
                " has an invalid lane index.");
    }
    if (!snapshot.currentRoute.empty() &&
        (snapshot.currentRouteIndex < 0 ||
         static_cast<std::size_t>(snapshot.currentRouteIndex) >=
             snapshot.currentRoute.size())) {
        return snapshotError(
            error,
            "Vehicle " + std::to_string(snapshot.id) +
                " has an invalid route cursor.");
    }
    if (snapshot.movementState == MovementState::TraversingJunction) {
        Road* outgoing = snapshot.junctionOutgoingRoadId.has_value()
            ? graph.getRoad(*snapshot.junctionOutgoingRoadId)
            : nullptr;
        if (currentRoad == nullptr || outgoing == nullptr ||
            currentRoad->getEnd() == nullptr ||
            currentRoad->getEnd()->getConnector(
                currentRoad,
                snapshot.junctionIncomingLane,
                outgoing,
                snapshot.junctionOutgoingLane) == nullptr) {
            return snapshotError(
                error,
                "Vehicle " + std::to_string(snapshot.id) +
                    " has an invalid junction connector.");
        }
    }
    if (snapshot.kind == VehicleKind::Bus) {
        if ((snapshot.serviceId >= 0 &&
             graph.getBusService(snapshot.serviceId) == nullptr) ||
            (snapshot.originStationId >= 0 &&
             graph.getBusStation(snapshot.originStationId) == nullptr) ||
            (snapshot.destinationStationId >= 0 &&
             graph.getBusStation(snapshot.destinationStationId) == nullptr)) {
            return snapshotError(
                error,
                "Bus " + std::to_string(snapshot.id) +
                    " references missing transit metadata.");
        }
        for (const int stopId : snapshot.assignedStopIds) {
            if (stopId >= 0 && graph.getBusStop(stopId) == nullptr) {
                return snapshotError(
                    error,
                    "Bus " + std::to_string(snapshot.id) +
                        " references missing stop " +
                        std::to_string(stopId) + ".");
            }
        }
    }
    return true;
}

bool validateSimulationSnapshot(const Graph* graph,
                                const SimulationSnapshot& snapshot,
                                std::string* error) {
    if (graph == nullptr) {
        return snapshotError(error, "Simulator graph is unavailable.");
    }
    std::unordered_set<int> activeVehicleIds;
    std::unordered_set<int> vehicleIds;
    for (const VehicleSnapshot& vehicle : snapshot.vehicles) {
        if (!vehicleIds.insert(vehicle.id).second) {
            return snapshotError(error, "Snapshot contains duplicate vehicle ids.");
        }
        activeVehicleIds.insert(vehicle.id);
        if (!validateVehicleSnapshot(*graph, vehicle, error)) {
            return false;
        }
    }
    for (const PendingVehicleSnapshot& pending : snapshot.pendingVehicles) {
        if (!vehicleIds.insert(pending.vehicle.id).second) {
            return snapshotError(error, "Snapshot contains duplicate vehicle ids.");
        }
        if (!validateVehicleSnapshot(*graph, pending.vehicle, error)) {
            return false;
        }
        for (const std::optional<int>& roadId : pending.route) {
            if (!roadId.has_value() ||
                graph->getRoad(*roadId) == nullptr) {
                return snapshotError(error, "Pending vehicle has an invalid route.");
            }
        }
    }
    const auto allRoads = graph->getAllRoads();
    if (snapshot.roads.size() != allRoads.size()) {
        return snapshotError(
            error,
            "Snapshot road topology no longer matches the loaded map.");
    }
    std::unordered_set<int> restoredRoadIds;
    for (const RoadRuntimeSnapshot& roadState : snapshot.roads) {
        Road* road = graph->getRoad(roadState.roadId);
        if (road == nullptr ||
            !restoredRoadIds.insert(roadState.roadId).second ||
            roadState.blockedLanes.size() !=
                static_cast<std::size_t>(road->getLaneCount()) ||
            !std::isfinite(roadState.congestionLevel) ||
            roadState.congestionLevel <= 0.0) {
            return snapshotError(
                error,
                "Snapshot road topology no longer matches the loaded map.");
        }
    }
    const auto allIntersections = graph->getAllIntersections();
    if (snapshot.intersections.size() != allIntersections.size()) {
        return snapshotError(
            error,
            "Snapshot intersection topology no longer matches the loaded map.");
    }
    std::unordered_set<int> restoredIntersectionIds;
    for (const IntersectionSnapshot& intersection : snapshot.intersections) {
        Intersection* liveIntersection =
            graph->getIntersection(intersection.id);
        if (liveIntersection == nullptr ||
            !restoredIntersectionIds.insert(intersection.id).second) {
            return snapshotError(
                error,
                "Snapshot references a missing intersection.");
        }
        for (const auto& occupant : intersection.occupants) {
            if (activeVehicleIds.count(occupant.first) == 0u) {
                return snapshotError(
                    error,
                    "Intersection reservation references a missing vehicle.");
            }
            const auto& reservation = occupant.second;
            Road* incoming = reservation.fromRoadId.has_value()
                ? graph->getRoad(*reservation.fromRoadId)
                : nullptr;
            if (incoming == nullptr || incoming->getEnd() != liveIntersection ||
                !std::isfinite(reservation.progressMetres) ||
                !std::isfinite(reservation.vehicleLengthMetres) ||
                !std::isfinite(reservation.vehicleWidthMetres) ||
                reservation.vehicleLengthMetres <= 0.0 ||
                reservation.vehicleWidthMetres <= 0.0) {
                return snapshotError(
                    error,
                    "Intersection reservation contains invalid state.");
            }
            if (reservation.outgoingRoadId.has_value()) {
                Road* outgoing =
                    graph->getRoad(*reservation.outgoingRoadId);
                if (outgoing == nullptr ||
                    liveIntersection->getConnector(
                        incoming,
                        reservation.incomingLane,
                        outgoing,
                        reservation.outgoingLane) == nullptr) {
                    return snapshotError(
                        error,
                        "Intersection reservation references an invalid connector.");
                }
            }
        }
        if (intersection.emergencyVehicleId >= 0) {
            Road* incoming =
                intersection.emergencyIncomingRoadId.has_value()
                    ? graph->getRoad(*intersection.emergencyIncomingRoadId)
                    : nullptr;
            Road* outgoing =
                intersection.emergencyOutgoingRoadId.has_value()
                ? graph->getRoad(*intersection.emergencyOutgoingRoadId)
                : nullptr;
            if (activeVehicleIds.count(intersection.emergencyVehicleId) == 0u ||
                incoming == nullptr || incoming->getEnd() != liveIntersection ||
                intersection.emergencyIncomingLane < 0 ||
                intersection.emergencyIncomingLane >= incoming->getLaneCount() ||
                (intersection.emergencyOutgoingRoadId.has_value() &&
                 outgoing == nullptr) ||
                !std::isfinite(intersection.emergencyVehicleWidthMetres) ||
                intersection.emergencyVehicleWidthMetres <= 0.0 ||
                !std::isfinite(
                    intersection.emergencyPriorityRemainingSeconds) ||
                intersection.emergencyPriorityRemainingSeconds < 0.0) {
                return snapshotError(
                    error,
                    "Intersection emergency priority contains invalid state.");
            }
        }
    }
    for (const TrafficEventSnapshot& event : snapshot.activeEvents) {
        Road* road = graph->getRoad(event.roadId);
        if (road == nullptr ||
            !std::isfinite(event.duration) || event.duration <= 0.0 ||
            !std::isfinite(event.timeElapsed) || event.timeElapsed < 0.0 ||
            event.timeElapsed >= event.duration ||
            !std::isfinite(event.severity)) {
            return snapshotError(error, "Snapshot event contains invalid state.");
        }
        if ((event.kind == TrafficEventKind::Congestion &&
             event.severity < 1.0) ||
            (event.kind == TrafficEventKind::Accident &&
             (event.laneIndex < 0 ||
              event.laneIndex >= road->getLaneCount()))) {
            return snapshotError(error, "Snapshot event contains invalid state.");
        }
    }
    for (const auto& entry : snapshot.nextSpawnTimeByRoad) {
        if (graph->getRoad(entry.first) == nullptr ||
            !std::isfinite(entry.second)) {
            return snapshotError(error, "Snapshot spawn schedule is invalid.");
        }
    }
    for (const auto& entry : snapshot.nextSpawnTimeBySource) {
        if (graph->getPointOfInterest(entry.first) == nullptr ||
            !std::isfinite(entry.second)) {
            return snapshotError(error, "Snapshot spawn schedule is invalid.");
        }
    }
    for (const auto& entry : snapshot.nextTransitDepartureTimeByService) {
        if (graph->getBusService(entry.first) == nullptr ||
            !std::isfinite(entry.second)) {
            return snapshotError(error, "Snapshot transit schedule is invalid.");
        }
    }
    if (snapshot.maximumActiveVehicles == 0u ||
        !std::isfinite(snapshot.elapsedTime) || snapshot.elapsedTime < 0.0 ||
        !std::isfinite(snapshot.speedMultiplier) ||
        snapshot.speedMultiplier <= 0.0 ||
        !std::isfinite(snapshot.leftoverDt) || snapshot.leftoverDt < 0.0 ||
        !std::isfinite(snapshot.lastSnapshotTime) ||
        !std::isfinite(snapshot.nextTransitNetworkDepartureTime) ||
        !std::isfinite(snapshot.pendingVehicleTimeoutSeconds) ||
        snapshot.pendingVehicleTimeoutSeconds <= 0.0) {
        return snapshotError(error, "Snapshot contains invalid simulator limits.");
    }
    return true;
}

} // namespace

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
    // Snapshot ring buffer: capacity sized so auto-capture at the default
    // 1s interval keeps ~10 minutes of sim time in history.
    snapshotManager_ = std::make_unique<SnapshotManager>(600);
    playbackController_ = std::make_unique<TimePlaybackController>(this, snapshotManager_.get());
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
    PointOfInterest* destination =
        vehicle->getTargetPOI();
    Road* originRoad =
        origin != nullptr
            ? origin->getConnectedRoad()
            : nullptr;
    Road* destinationRoad =
        destination != nullptr
            ? destination->getConnectedRoad()
            : nullptr;

    if (origin != nullptr && originRoad == nullptr) {
        return false;
    }
    if (destination != nullptr && destinationRoad == nullptr) {
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
         vehicle->getSpawnPOI()->getConnectedRoad() != nullptr) ||
        vehicle->getSpawnPoint() != nullptr;
    const bool hasDestination =
        (vehicle->getTargetPOI() != nullptr &&
         vehicle->getTargetPOI()->getConnectedRoad() != nullptr) ||
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

            delete v; // Delete vehicle instead of keeping it in parking slot
            return true;
        }
        return false;
    });
    vehicles.erase(it, vehicles.end());
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

    maybeAutoCaptureSnapshot();

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
void TrafficSimulator::resume() {
    if (playbackController_ != nullptr) {
        playbackController_->resumeLive();
    }
    lastSnapshotTime_ = elapsedTime;
    paused = false;
}
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

// --- Snapshot support (Memento pattern) ---

SimulationSnapshot TrafficSimulator::captureSnapshot() const {
    SimulationSnapshot snap;
    snap.elapsedTime = elapsedTime;
    snap.tickCount = tickCount;
    snap.paused = paused;
    snap.speedMultiplier = speedMultiplier;
    snap.leftoverDt = leftoverDt;
    snap.lastSnapshotTime = lastSnapshotTime_;
    snap.dynamicRerouteCursor = dynamicRerouteCursor_;
    snap.maximumActiveVehicles = maximumActiveVehicles_;
    snap.pendingVehicleTimeoutSeconds =
        pendingVehicleTimeoutSeconds_;

    snap.accepted = spawnStatistics_.accepted;
    snap.activated = spawnStatistics_.activated;
    snap.delayedAttempts = spawnStatistics_.delayedAttempts;
    snap.rejected = spawnStatistics_.rejected;
    snap.timedOut = spawnStatistics_.timedOut;

    snap.nextSpawnTimeByRoad.clear();
    for (const auto& entry : nextSpawnTimeByRoad_) {
        if (entry.first != nullptr) {
            snap.nextSpawnTimeByRoad[entry.first->getId()] = entry.second;
        }
    }
    snap.nextSpawnTimeBySource.clear();
    for (const auto& entry : nextSpawnTimeBySource_) {
        if (entry.first != nullptr) {
            snap.nextSpawnTimeBySource[entry.first->getId()] = entry.second;
        }
    }
    snap.nextTransitDepartureTimeByService.clear();
    for (const auto& entry : nextTransitDepartureTimeByService_) {
        if (entry.first != nullptr) {
            snap.nextTransitDepartureTimeByService[entry.first->getId()] = entry.second;
        }
    }
    snap.nextTransitNetworkDepartureTime = nextTransitNetworkDepartureTime_;

    // Vehicles
    snap.vehicles.clear();
    snap.vehicles.reserve(vehicles.size());
    for (const Vehicle* v : vehicles) {
        VehicleSnapshot vs;
        v->captureSnapshot(vs, *graph);
        snap.vehicles.push_back(std::move(vs));
    }

    // Intersections
    snap.intersections.clear();
    if (graph != nullptr) {
        const auto allIntersections = graph->getAllIntersections();
        snap.intersections.reserve(allIntersections.size());
        for (const Intersection* intersection : allIntersections) {
            if (intersection == nullptr) continue;
            IntersectionSnapshot is;
            intersection->captureSnapshot(is);
            snap.intersections.push_back(std::move(is));
        }
    }

    // Pending vehicles
    snap.pendingVehicles.clear();
    snap.pendingVehicles.reserve(pendingVehicles.size());
    for (const PendingVehicle& pending : pendingVehicles) {
        if (pending.vehicle == nullptr) continue;
        PendingVehicleSnapshot ps;
        pending.vehicle->captureSnapshot(ps.vehicle, *graph);
        ps.route.clear();
        for (const Road* road : pending.route) {
            ps.route.push_back(
                road != nullptr
                    ? std::optional<int>(road->getId())
                    : std::nullopt);
        }
        ps.earliestActivationTime = pending.earliestActivationTime;
        ps.nextAttemptTime = pending.nextAttemptTime;
        ps.deadlineTime = pending.deadlineTime;
        ps.phasedAdmission = pending.phasedAdmission;
        ps.routeResolved = pending.routeResolved;
        ps.fixedRoute = pending.fixedRoute;
        ps.routeAttempts = pending.routeAttempts;
        snap.pendingVehicles.push_back(std::move(ps));
    }

    const auto allRoads = graph->getAllRoads();
    snap.roads.reserve(allRoads.size());
    for (const Road* road : allRoads) {
        if (road == nullptr) continue;
        RoadRuntimeSnapshot roadState;
        roadState.roadId = road->getId();
        // Persist the configured/event congestion multiplier. Dynamic
        // occupancy is derived again from the restored lane membership.
        roadState.congestionLevel = road->getCongestionLevel();
        roadState.blockedLanes.reserve(
            static_cast<std::size_t>(road->getLaneCount()));
        for (const Lane& lane : road->getLanes()) {
            roadState.blockedLanes.push_back(lane.isBlocked());
        }
        snap.roads.push_back(std::move(roadState));
    }

    if (eventManager != nullptr) {
        snap.activeEvents = eventManager->captureSnapshot();
    }
    if (statisticsManager != nullptr) {
        snap.statistics = statisticsManager->captureSnapshot();
    }

    // Merging-from-POI index
    snap.mergingFromPOIByRoad.clear();
    for (const auto& entry : mergingFromPOIByRoad_) {
        if (entry.first == nullptr) continue;
        std::vector<int> ids;
        ids.reserve(entry.second.size());
        for (const Vehicle* v : entry.second) {
            if (v != nullptr) ids.push_back(v->getId());
        }
        snap.mergingFromPOIByRoad[entry.first->getId()] = std::move(ids);
    }

    // Failed recalc ids
    snap.failedRecalcIds.clear();
    snap.failedRecalcIds.reserve(failedRecalcIds.size());
    for (const int id : failedRecalcIds) {
        snap.failedRecalcIds.push_back(id);
    }

    return snap;
}

bool TrafficSimulator::restoreSnapshot(const SimulationSnapshot& snapshot,
                                       std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (!validateSimulationSnapshot(graph, snapshot, error)) {
        return false;
    }

    for (Road* road : graph->getAllRoads()) {
        if (road != nullptr) {
            road->clearRuntimeVehicleReferences();
        }
    }

    // Destroy all live vehicles (they will be reconstructed from snapshot).
    for (Vehicle* v : vehicles) {
        delete v;
    }
    vehicles.clear();
    for (Vehicle* v : finishedVehicles) {
        delete v;
    }
    finishedVehicles.clear();
    for (const PendingVehicle& pending : pendingVehicles) {
        delete pending.vehicle;
    }
    pendingVehicles.clear();
    mergingFromPOIByRoad_.clear();

    for (PointOfInterest* poi : graph->getAllPOIs()) {
        if (auto* spawn = dynamic_cast<SpawnPoint*>(poi)) {
            spawn->resetSpawnSlotsForRestore();
        }
    }
    for (BusStation* station : graph->getAllBusStations()) {
        if (station != nullptr) {
            station->resetSpawnSlotsForRestore();
        }
    }

    // Restore core state
    elapsedTime = snapshot.elapsedTime;
    tickCount = snapshot.tickCount;
    paused = snapshot.paused;
    speedMultiplier = snapshot.speedMultiplier;
    leftoverDt = snapshot.leftoverDt;
    lastSnapshotTime_ = snapshot.lastSnapshotTime;
    dynamicRerouteCursor_ = snapshot.dynamicRerouteCursor;
    maximumActiveVehicles_ = snapshot.maximumActiveVehicles;
    pendingVehicleTimeoutSeconds_ =
        snapshot.pendingVehicleTimeoutSeconds;

    spawnStatistics_.accepted = snapshot.accepted;
    spawnStatistics_.activated = snapshot.activated;
    spawnStatistics_.delayedAttempts = snapshot.delayedAttempts;
    spawnStatistics_.rejected = snapshot.rejected;
    spawnStatistics_.timedOut = snapshot.timedOut;

    nextSpawnTimeByRoad_.clear();
    for (const auto& entry : snapshot.nextSpawnTimeByRoad) {
        Road* road = graph->getRoad(entry.first);
        if (road != nullptr) {
            nextSpawnTimeByRoad_[road] = entry.second;
        }
    }
    nextSpawnTimeBySource_.clear();
    for (const auto& entry : snapshot.nextSpawnTimeBySource) {
        PointOfInterest* poi =
            graph->getPointOfInterest(entry.first);
        if (poi != nullptr) {
            nextSpawnTimeBySource_[poi] = entry.second;
        }
    }
    nextTransitDepartureTimeByService_.clear();
    for (const auto& entry : snapshot.nextTransitDepartureTimeByService) {
        BusService* service = graph->getBusService(entry.first);
        if (service != nullptr) {
            nextTransitDepartureTimeByService_[service] = entry.second;
        }
    }
    nextTransitNetworkDepartureTime_ = snapshot.nextTransitNetworkDepartureTime;

    for (const RoadRuntimeSnapshot& roadState : snapshot.roads) {
        Road* road = graph->getRoad(roadState.roadId);
        road->updateCongestionLevel(roadState.congestionLevel);
        for (std::size_t laneIndex = 0;
             laneIndex < roadState.blockedLanes.size();
             ++laneIndex) {
            if (roadState.blockedLanes[laneIndex]) {
                road->blockLane(static_cast<int>(laneIndex));
            } else {
                road->unblockLane(static_cast<int>(laneIndex));
            }
        }
    }

    // Restore active vehicles
    for (const VehicleSnapshot& vs : snapshot.vehicles) {
        std::unique_ptr<Vehicle> restored =
            createSnapshotVehicle(vs);
        if (restored == nullptr) continue;
        restored->restoreSnapshot(vs, *graph);
        vehicles.push_back(restored.release());
    }

    // Rebuild the lane/POI membership indexes from vehicle state instead of
    // trusting raw pointers from the previous timeline.
    mergingFromPOIByRoad_.clear();
    for (Vehicle* vehicle : vehicles) {
        if (vehicle == nullptr ||
            vehicle->getCurrentRoad() == nullptr) {
            continue;
        }
        Road* road = vehicle->getCurrentRoad();
        if (vehicle->getIsMergingFromPOI()) {
            road->addMergingVehicle(vehicle);
            mergingFromPOIByRoad_[road].push_back(vehicle);
        } else if (vehicle->getMovementState() !=
                       MovementState::TraversingJunction &&
                   !vehicle->getIsEnteringPOI()) {
            road->getLane(vehicle->getCurrentLaneIndex()).
                addVehicle(vehicle);
        }
    }

    // Restore pending vehicles
    for (const PendingVehicleSnapshot& ps : snapshot.pendingVehicles) {
        std::unique_ptr<Vehicle> restored =
            createSnapshotVehicle(ps.vehicle);
        if (restored == nullptr) continue;
        restored->restoreSnapshot(ps.vehicle, *graph);

        PendingVehicle pending;
        pending.vehicle = restored.release();
        pending.route.clear();
        for (const std::optional<int>& roadId : ps.route) {
            pending.route.push_back(
                roadId.has_value() ? graph->getRoad(*roadId) : nullptr);
        }
        pending.earliestActivationTime = ps.earliestActivationTime;
        pending.nextAttemptTime = ps.nextAttemptTime;
        pending.deadlineTime = ps.deadlineTime;
        pending.phasedAdmission = ps.phasedAdmission;
        pending.routeResolved = ps.routeResolved;
        pending.fixedRoute = ps.fixedRoute;
        pending.routeAttempts = ps.routeAttempts;
        pendingVehicles.push_back(std::move(pending));
    }

    for (const IntersectionSnapshot& is : snapshot.intersections) {
        Intersection* intersection = graph->getIntersection(is.id);
        intersection->restoreSnapshot(is);
    }

    if (statisticsManager != nullptr) {
        statisticsManager->restoreSnapshot(snapshot.statistics);
    }
    if (eventManager != nullptr) {
        eventManager->restoreSnapshot(snapshot.activeEvents);
    }

    // Restore failed recalc ids
    failedRecalcIds.clear();
    for (const int id : snapshot.failedRecalcIds) {
        failedRecalcIds.insert(id);
    }
    return true;
}

void TrafficSimulator::setSnapshotInterval(double intervalSeconds) {
    snapshotIntervalSeconds_ = std::max(0.0, intervalSeconds);
}

void TrafficSimulator::maybeAutoCaptureSnapshot() {
    if (snapshotManager_ == nullptr ||
        snapshotIntervalSeconds_ <= 0.0) {
        return;
    }
    if (elapsedTime - lastSnapshotTime_ >=
            snapshotIntervalSeconds_) {
        lastSnapshotTime_ = elapsedTime;
        snapshotManager_->capture(*this);
    }
}

std::size_t TrafficSimulator::captureSnapshotNow() {
    if (snapshotManager_ == nullptr) {
        return SnapshotManager::npos;
    }
    lastSnapshotTime_ = elapsedTime;
    return snapshotManager_->capture(*this);
}


