#include "Vehicle.h"
#include "Road.h"
#include "Intersection.h"
#include "Graph.h"
#include "algorithm/PathFindingStrategy.h"
#include <algorithm>

Vehicle::Vehicle(int id, double speed, Intersection* start, Intersection* dest)
    : id(id),
      baseSpeed(speed),
      spawnPoint(start),
      destination(dest),
      currentRoad(nullptr),
      progressOnCurrentRoad(0.0),
      currentSpeed(0.0),
      currentRouteIndex(0), 
      paused(false)
{
}

Vehicle::~Vehicle() {
    if (currentRoad != nullptr) {
        currentRoad->getLane(0).removeVehicle(this);
        currentRoad = nullptr;
    }
}

void Vehicle::setRoute(const std::vector<Road*>& route) {
    currentRoute = route;
    currentRouteIndex = 0;
    progressOnCurrentRoad = 0.0;
    currentSpeed = 0.0; // vehicle starts from a standstill on a fresh route
    paused = false;
    routeAssigned = true;
    
    if (!currentRoute.empty()) {
        currentRoad = currentRoute[0];
        if (currentRoad) currentRoad->getLane(0).addVehicle(this);
    } else {
        currentRoad = nullptr;
    }

    onRoadChanged();
}

bool Vehicle::advanceToNextRoad() {
    addTravelHistory(currentRoad);
    ++currentRouteIndex;
 
    if (currentRouteIndex < static_cast<int>(currentRoute.size())) {
        if (currentRoad) {
            currentRoad->getLane(0).removeVehicle(this);
        }
        currentRoad = currentRoute[currentRouteIndex];
        if (currentRoad) {
            currentRoad->getLane(0).addVehicle(this);
        }
        onRoadChanged(); 
        return true;
    }
 
    // Route finished
    if (currentRoad) {
        currentRoad->getLane(0).removeVehicle(this);
    }
    currentRoad = nullptr;
    progressOnCurrentRoad = 0.0;
    return false;
}
 

double Vehicle::getProgressRatio() const {
    if (currentRoad == nullptr || currentRoad->getDistance() <= 0.0) {
        return 0.0;
    }
    return progressOnCurrentRoad / currentRoad->getDistance();
}


void Vehicle::update(double dt) {
    if (hasReachedDestination() || currentRoad == nullptr) return;
 
    if (paused) {
        if (updatePause(dt)) {
            paused = false; // subclass says: done waiting, resume
        }
        return;
    }
 
    double remainingTime = dt; // we may need to split dt across multiple roads
 
    while (remainingTime > 0.0 && currentRoad != nullptr && !paused) {
 
        // calculateCurrentSpeed() gives the TARGET speed allowed right now
        // (speed limit / congestion / blocked status for this road & vehicle
        // type). The vehicle doesn't teleport to that speed - it ramps
        // towards it using its acceleration/deceleration, so starts/stops
        // and speed-limit changes feel gradual instead of instantaneous.
        const double targetSpeed = calculateCurrentSpeed();

        if (currentSpeed < targetSpeed) {
            currentSpeed = std::min(targetSpeed, currentSpeed + getAcceleration() * remainingTime);
        } else if (currentSpeed > targetSpeed) {
            currentSpeed = std::max(targetSpeed, currentSpeed - getDeceleration() * remainingTime);
        }

        double speed = currentSpeed;
 
        // If speed is 0 (e.g. congestion/blocked returned 0 and we've already
        // decelerated all the way down), nothing to do this tick
        if (speed <= 0.0) break;
 
        double distanceThisTick    = speed * remainingTime;
        double currentPos          = progressOnCurrentRoad;
        double projectedPos        = currentPos + distanceThisTick;
 
        double pausePos = -1.0;
        if (shouldPauseAt(currentPos, projectedPos, pausePos)
            && pausePos > currentPos
            && pausePos <= currentRoad->getDistance()) {
 
            progressOnCurrentRoad = pausePos;
            paused = true;
            onPauseStarted();
            break; // consume the rest of dt while dwelling (next ticks)
        }
 
        if (projectedPos < currentRoad->getDistance()) {
            // Stay on tis road
            progressOnCurrentRoad = projectedPos;
            remainingTime = 0.0; // all time consumed
        } else {
            // Reached (or passed) the end of this road segment
            double distToEnd  = currentRoad->getDistance() - currentPos;
            double timeToEnd  = (speed > 0.0) ? distToEnd / speed : 0.0;
            remainingTime    -= timeToEnd;
            progressOnCurrentRoad = 0.0; // reset for next road
 
            if (!advanceToNextRoad()) {
                break; // route finished
            }
 
            // After entering the new road, check if the subclass wants to
            // pause immediately at position 0 (e.g. first stop at road start)
            // — handled naturally by shouldPauseAt on the next loop iteration
        }
    }
}

bool Vehicle::isRoadInUpcomingRoute(int roadId) const {
    // Only check upcoming roads (from currentRouteIndex + 1 onwards)
    for (size_t i = currentRouteIndex + 1; i < currentRoute.size(); ++i) {
        if (currentRoute[i]->getId() == roadId) {
            return true;
        }
    }
    return false;
}

bool Vehicle::recalculateRoute(const Graph& graph, PathFindingStrategy* strategy) {
    if (currentRoad == nullptr || destination == nullptr) return false;

    // Start routing from the NEXT intersection (since the vehicle is already on the current road and cannot turn around instantly)
    int startNodeId = currentRoad->getEnd()->getId();
    int destNodeId = destination->getId();

    PathResult result = strategy->findPath(graph, startNodeId, destNodeId);

    if (!result.found) {
        return false; // No alternative route found
    }

    std::vector<Road*> newRoute;
    newRoute.reserve(static_cast<size_t>(currentRouteIndex) + 1 + result.roadPath.size());
    for (int i = 0; i <= currentRouteIndex; ++i) {
        newRoute.push_back(currentRoute[i]);
    }

    for (Road* r : result.roadPath) {
        newRoute.push_back(r);
    }

    currentRoute = newRoute;
    paused = false; // Reset pause state in case it was paused
    return true;
}

bool Vehicle::performUTurn(const Graph& graph, PathFindingStrategy* strategy) {
    if (currentRoad == nullptr || destination == nullptr) return false;

    // Find the reverse road
    int startId = currentRoad->getStart()->getId();
    int endId = currentRoad->getEnd()->getId();
    Road* reverseRoad = nullptr;
    
    for (Road* r : graph.getAllRoads()) {
        if (r->getStart()->getId() == endId && r->getEnd()->getId() == startId) {
            reverseRoad = r;
            break;
        }
    }

    if (reverseRoad == nullptr) return false; // Cannot U-turn, no reverse road

    // Recalculate route from the start of the reverse road (which is the current endId)
    PathResult result = strategy->findPath(graph, startId, destination->getId());
    if (!result.found) return false;

    // Swap to reverse road
    currentRoad = reverseRoad;
    
    // Invert progress
    progressOnCurrentRoad = currentRoad->getDistance() - progressOnCurrentRoad;
    if (progressOnCurrentRoad < 0) progressOnCurrentRoad = 0;
    
    // Reset speed as we stopped to turn around
    currentSpeed = 0.0;
    paused = false;

    // Build new route: keep history, then add reverse road, then result path
    std::vector<Road*> newRoute;
    for (int i = 0; i < currentRouteIndex; ++i) {
        newRoute.push_back(currentRoute[i]);
    }
    
    // Now we are at currentRouteIndex
    newRoute.push_back(currentRoad);
    for (Road* r : result.roadPath) {
        newRoute.push_back(r);
    }

    currentRoute = newRoute;
    return true;
}