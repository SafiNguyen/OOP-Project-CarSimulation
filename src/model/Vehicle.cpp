#include <iostream>
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
      paused(false),
      awaitingIntersectionTransition(false),
      intersectionTransitionTimer(0.0) {
}

Vehicle::~Vehicle() {
    if (currentRoad != nullptr) {
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        currentRoad = nullptr;
    }
}

void Vehicle::setRoute(const std::vector<Road*>& route) {
    currentRoute = route;
    currentRouteIndex = 0;
    progressOnCurrentRoad = 0.0;
    currentSpeed = 0.0;
    paused = false;
    routeAssigned = true;

    if (!currentRoute.empty()) {
        currentRoad = currentRoute[0];
        onRoadChanged();
        if (currentRoad) {
            currentRoad->getLane(currentLaneIndex).addVehicle(this);
        }
    } else {
        currentRoad = nullptr;
    }

    onRoadChanged();
}

bool Vehicle::advanceToNextRoad() {
    if (currentRoad != nullptr) {
        addTravelHistory(currentRoad);
    }
    ++currentRouteIndex;

    if (currentRouteIndex < static_cast<int>(currentRoute.size())) {
        if (currentRoad) {
            currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        }
        currentRoad = currentRoute[currentRouteIndex];
        if (currentRoad) {
            currentRoad->getLane(currentLaneIndex).addVehicle(this);
        }
        onRoadChanged();
        return true;
    }

    if (currentRoad) {
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
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

bool Vehicle::mustStopForTrafficLight(Intersection* nextIntersection) const {
    if (nextIntersection == nullptr || currentRoad == nullptr) {
        return false;
    }
    return nextIntersection->mustStopForRoad(currentRoad);
}

Road* Vehicle::getNextRoad() const {
    if (currentRouteIndex + 1 < static_cast<int>(currentRoute.size())) {
        return currentRoute[currentRouteIndex + 1];
    }
    return nullptr;
}

void Vehicle::update(double dt) {
    if (hasReachedDestination() || currentRoad == nullptr) {
        return;
    }

    if (awaitingIntersectionTransition) {
        intersectionTransitionTimer -= dt;
        if (intersectionTransitionTimer <= 0.0) {
            intersectionTransitionTimer = 0.0;
            awaitingIntersectionTransition = false;
        }
    }

    if (paused) {
        if (updatePause(dt)) {
            paused = false;
        }
        return;
    }

    double remainingTime = dt;

    while (remainingTime > 0.0 && currentRoad != nullptr && !paused) {
        const double targetSpeed = calculateCurrentSpeed();

        if (currentSpeed < targetSpeed) {
            currentSpeed = std::min(targetSpeed, currentSpeed + getAcceleration() * remainingTime);
        } else if (currentSpeed > targetSpeed) {
            currentSpeed = std::max(targetSpeed, currentSpeed - getDeceleration() * remainingTime);
        }

        double speed = currentSpeed;
        if (speed <= 0.0) {
            break;
        }

        double distanceThisTick = speed * remainingTime;
        double currentPos = progressOnCurrentRoad;
        double projectedPos = currentPos + distanceThisTick;

        double pausePos = -1.0;
        if (shouldPauseAt(currentPos, projectedPos, pausePos)
            && pausePos > currentPos
            && pausePos <= currentRoad->getDistance()) {
            progressOnCurrentRoad = pausePos;
            paused = true;
            onPauseStarted();
            break;
        }

        if (projectedPos < currentRoad->getDistance()) {
            progressOnCurrentRoad = projectedPos;
            remainingTime = 0.0;
        } else {
            Intersection* nextIntersection = currentRoad->getEnd();

            if (mustStopForTrafficLight(nextIntersection)) {
                progressOnCurrentRoad = currentRoad->getDistance();
                currentSpeed = 0.0;
                remainingTime = 0.0;
                break;
            }   

            if (currentRouteIndex + 1 < static_cast<int>(currentRoute.size())) {
                double distToEnd = currentRoad->getDistance() - currentPos;
                double timeToEnd = (speed > 0.0) ? distToEnd / speed : 0.0;
                remainingTime -= timeToEnd;
                if (remainingTime < 0.0) {
                    remainingTime = 0.0;
                }
                progressOnCurrentRoad = 0.0;
                awaitingIntersectionTransition = true;
                intersectionTransitionTimer = INTERSECTION_TRANSITION_DURATION;
                if (!advanceToNextRoad()) break;
                continue;   
            }

            double distToEnd = currentRoad->getDistance() - currentPos;
            double timeToEnd = (speed > 0.0) ? distToEnd / speed : 0.0;
            remainingTime -= timeToEnd;
            progressOnCurrentRoad = 0.0;

            if (!advanceToNextRoad()) {
                break;
            }
        }
    }
}

bool Vehicle::isRoadInUpcomingRoute(int roadId) const {
    for (size_t i = currentRouteIndex + 1; i < currentRoute.size(); ++i) {
        if (currentRoute[i]->getId() == roadId) {
            return true;
        }
    }
    return false;
}

bool Vehicle::recalculateRoute(const Graph& graph, PathFindingStrategy* strategy) {
    if (currentRoad == nullptr || destination == nullptr) {
        return false;
    }

    int startNodeId = currentRoad->getEnd()->getId();
    int destNodeId = destination->getId();

    PathResult result = strategy->findPath(graph, startNodeId, destNodeId);

    if (!result.found) {
        return false;
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
    paused = false;
    return true;
}

bool Vehicle::performUTurn(const Graph& graph, PathFindingStrategy* strategy) {
    if (currentRoad == nullptr || destination == nullptr) {
        return false;
    }

    int startId = currentRoad->getStart()->getId();
    int endId = currentRoad->getEnd()->getId();
    Road* reverseRoad = nullptr;

    for (Road* r : graph.getAllRoads()) {
        if (r->getStart()->getId() == endId && r->getEnd()->getId() == startId) {
            reverseRoad = r;
            break;
        }
    }

    if (reverseRoad == nullptr) {
        return false;
    }

    PathResult result = strategy->findPath(graph, startId, destination->getId());
    if (!result.found) {
        return false;
    }

    currentRoad = reverseRoad;
    progressOnCurrentRoad = currentRoad->getDistance() - progressOnCurrentRoad;
    if (progressOnCurrentRoad < 0) {
        progressOnCurrentRoad = 0;
    }

    currentSpeed = 0.0;
    paused = false;

    std::vector<Road*> newRoute;
    for (int i = 0; i < currentRouteIndex; ++i) {
        newRoute.push_back(currentRoute[i]);
    }

    newRoute.push_back(currentRoad);
    for (Road* r : result.roadPath) {
        newRoute.push_back(r);
    }

    currentRoute = newRoute;
    return true;
}