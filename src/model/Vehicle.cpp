#include "Vehicle.h"
#include "Road.h"
#include "Intersection.h"

Vehicle::Vehicle(int id, double speed, Intersection* start, Intersection* dest)
    : id(id),
      baseSpeed(speed),
      spawnPoint(start),
      destination(dest),
      currentRoad(nullptr),
      progressOnCurrentRoad(0.0),
      currentRouteIndex(0), 
      paused(false)
{
}

void Vehicle::setRoute(const std::vector<Road*>& route) {
    currentRoute = route;
    currentRouteIndex = 0;
    progressOnCurrentRoad = 0.0;
    paused = false;
    
    if (!currentRoute.empty()) {
        currentRoad = currentRoute[0]; 
    } else {
        currentRoad = nullptr;
    }
}

bool Vehicle::advanceToNextRoad() {
    addTravelHistory(currentRoad);
    ++currentRouteIndex;
 
    if (currentRouteIndex < static_cast<int>(currentRoute.size())) {
        currentRoad = currentRoute[currentRouteIndex];
        onRoadChanged(); 
        return true;
    }
 
    // Route finished
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
 
        double speed = calculateCurrentSpeed();
 
        // If speed is 0 (e.g. congestion returned 0), nothing to do this tick
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
