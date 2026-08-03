#include "RouteFollower.h"
#include "Vehicle.h"
#include "Road.h"
#include "RoadGeometry.h"
#include "Graph.h"
#include "algorithm/PathFindingStrategy.h"
#include <algorithm>
#include <vector>

namespace {
constexpr double UTURN_POSE_TRANSITION_SECONDS = 0.7;
}

bool RouteFollower::recalculateRoute(Vehicle& vehicle, const Graph& graph, PathFindingStrategy* strategy) const {
    if (!vehicle.allowsDynamicRerouting() || vehicle.currentRoad == nullptr || (vehicle.destination == nullptr && vehicle.targetPOI == nullptr) || vehicle.movementState_ == MovementState::TraversingJunction) {
        return false;
    }

    if (vehicle.currentRoad->isBlocked()) {
        return false;
    }

    int startNodeId = vehicle.currentRoad->getEnd()->getId();
    int destNodeId = (vehicle.destination != nullptr) ? vehicle.destination->getId() : -1;
    Road* poiRoad = (vehicle.targetPOI != nullptr) ? vehicle.targetPOI->getConnectedRoad() : nullptr;

    if (poiRoad != nullptr) {
        if (vehicle.currentRoad == poiRoad) {
            return false;
        }
        destNodeId = poiRoad->getStart()->getId();
    }

    PathResult result = strategy->findPath(graph, startNodeId, destNodeId);
    if (!result.found) {
        return false;
    }

    std::vector<Road*> newRoute;
    newRoute.reserve(static_cast<size_t>(vehicle.currentRouteIndex) + 1 + result.roadPath.size());
    for (int i = 0; i <= vehicle.currentRouteIndex; ++i) {
        newRoute.push_back(vehicle.currentRoute[i]);
    }
    for (Road* r : result.roadPath) {
        newRoute.push_back(r);
    }
    
    if (poiRoad != nullptr) {
        newRoute.push_back(poiRoad);
    }

    vehicle.currentRoute = newRoute;
    return true;
}

bool RouteFollower::performUTurn(Vehicle& vehicle, const Graph& graph, PathFindingStrategy* strategy) const {
    if (!vehicle.allowsUTurn() || vehicle.currentRoad == nullptr || (vehicle.destination == nullptr && vehicle.targetPOI == nullptr) || vehicle.movementState_ == MovementState::TraversingJunction) {
        return false;
    }

    int startId = vehicle.currentRoad->getStart()->getId();
    Road* reverseRoad = vehicle.currentRoad->getReverseRoad();
    if (reverseRoad == nullptr) return false;

    const Pose2D fromPose = vehicle.getPose();
    int destNodeId = (vehicle.destination != nullptr) ? vehicle.destination->getId() : -1;
    Road* poiRoad = (vehicle.targetPOI != nullptr) ? vehicle.targetPOI->getConnectedRoad() : nullptr;
    if (poiRoad != nullptr) {
        destNodeId = poiRoad->getStart()->getId();
    }

    PathResult result = strategy->findPath(graph, startId, destNodeId);
    if (!result.found) {
        return false;
    }

    vehicle.currentRoad->getLane(vehicle.currentLaneIndex).removeVehicle(&vehicle);
    vehicle.currentRoad = reverseRoad;
    vehicle.currentLaneIndex = vehicle.currentRoad->getLane(0).isBlocked() ? vehicle.currentRoad->getFreestLaneIndex() : 0;
    vehicle.currentRoad->getLane(vehicle.currentLaneIndex).addVehicle(&vehicle);

    vehicle.progressOnCurrentRoad = std::clamp(vehicle.currentRoad->getDistance() - vehicle.progressOnCurrentRoad, 0.0, vehicle.currentRoad->getDistance());
    vehicle.currentSpeed = 0.0;
    vehicle.clearPause();

    const Pose2D toPose = RoadGeometry::sampleLane(*vehicle.currentRoad, vehicle.currentLaneIndex, vehicle.progressOnCurrentRoad);
    vehicle.startPoseTransition(fromPose, toPose, UTURN_POSE_TRANSITION_SECONDS);

    std::vector<Road*> newRoute;
    for (int i = 0; i < vehicle.currentRouteIndex; ++i) {
        newRoute.push_back(vehicle.currentRoute[i]);
    }
    newRoute.push_back(vehicle.currentRoad);
    for (Road* r : result.roadPath) {
        newRoute.push_back(r);
    }

    if (poiRoad != nullptr) {
        newRoute.push_back(poiRoad);
    }

    vehicle.currentRoute = newRoute;
    vehicle.onRoadChanged();
    return true;
}

bool RouteFollower::advanceToNextRoad(Vehicle& vehicle) const {
    if (vehicle.currentRoad != nullptr) {
        vehicle.addTravelHistory(vehicle.currentRoad);
    }
    ++vehicle.currentRouteIndex;

    if (vehicle.currentRouteIndex < static_cast<int>(vehicle.currentRoute.size())) {
        if (vehicle.currentRoad) {
            vehicle.currentRoad->getLane(vehicle.currentLaneIndex).removeVehicle(&vehicle);
        }
        vehicle.currentRoad = vehicle.currentRoute[vehicle.currentRouteIndex];
        if (vehicle.currentRoad) {
            const int maxLaneIndex = vehicle.currentRoad->getLaneCount() - 1;
            if (vehicle.currentLaneIndex > maxLaneIndex) {
                vehicle.currentLaneIndex = maxLaneIndex;
            }
            if (vehicle.currentRoad->getLane(vehicle.currentLaneIndex).isBlocked()) {
                vehicle.currentLaneIndex = vehicle.currentRoad->getFreestLaneIndex();
            }
            vehicle.currentRoad->getLane(vehicle.currentLaneIndex).addVehicle(&vehicle);
        }
        vehicle.onRoadChanged();
        return true;
    }

    if (vehicle.currentRoad) {
        vehicle.currentRoad->getLane(vehicle.currentLaneIndex).removeVehicle(&vehicle);
    }
    vehicle.currentRoad = nullptr;
    vehicle.progressOnCurrentRoad = 0.0;
    vehicle.clearLaneChangeIntent();
    vehicle.junctionTurnSignal_ = TurnSignal::Off;
    vehicle.onRoadChanged();
    vehicle.refreshTurnSignal();
    return false;
}
