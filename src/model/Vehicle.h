#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

class Intersection;
class Road;
class Graph;
class PathFindingStrategy;
class PointOfInterest;

class Vehicle {
public:
    static constexpr double INTERSECTION_TRANSITION_DURATION = 0.12;

protected:
    int id;
    double baseSpeed; // m/s
    Intersection* spawnPoint;
    PointOfInterest* targetPOI = nullptr; // optional POI destination
    Intersection* destination;
    Road* currentRoad;
    double progressOnCurrentRoad;
    double currentSpeed;
    std::vector<Road*> currentRoute;
    int currentRouteIndex;
    std::vector<Road*> travelHistory;
    bool paused;
    bool routeAssigned = false;
    int currentLaneIndex = 0;
    bool awaitingIntersectionTransition = false;
    double intersectionTransitionTimer = 0.0;

public:
    Vehicle(int id, double speed, Intersection* start, Intersection* dest);
    Vehicle(const Vehicle&) = delete;
    Vehicle& operator=(const Vehicle&) = delete;
    Vehicle(Vehicle&&) = delete;
    Vehicle& operator=(Vehicle&&) = delete;

    virtual ~Vehicle();

    virtual double calculateCurrentSpeed() const = 0;
    virtual void onRoadChanged() {}
    virtual double getAcceleration() const { return 3.0; }
    virtual double getDeceleration() const { return 5.0; }

    double getCurrentSpeed() const { return currentSpeed; }

    virtual bool shouldPauseAt(double currentPos,
                                double projectedPos,
                                double& pausePos) { return false; }
    virtual bool mustStopForTrafficLight(Intersection* nextIntersection) const;
    virtual void onPauseStarted() {}
    virtual bool updatePause(double dt) { return true; }
    virtual void update(double dt);
    void setRoute(const std::vector<Road*>& route);

    int getId() const { return id; }
    Intersection* getSpawnPoint() const { return spawnPoint; }
    Intersection* getDestination() const { return destination; }
    double getBaseSpeed() const { return baseSpeed; }
    virtual double getLength() const { return 4.5; }    // metres
    virtual double getHeight() const { return 1.5; }    // metres
    virtual double getWeight() const { return 1.5; }    // tonnes
    virtual double getMiniGap() const { return 0.0; }
    virtual double getMinGap() const { return getMiniGap(); }
    virtual double getTimeHeadway() const { return 1.5; }
    double getProgressOnRoad() const { return progressOnCurrentRoad; }

    // --- POI support ---
    PointOfInterest* getTargetPOI() const { return targetPOI; }
    void setTargetPOI(PointOfInterest* poi) { targetPOI = poi; }
    Road* getCurrentRoad() const { return currentRoad; }
    bool isPaused() const { return paused; }
    bool hasReachedDestination() const {
        return routeAssigned && currentRoad == nullptr && currentRouteIndex >= static_cast<int>(currentRoute.size());
    }
    double getProgressRatio() const;
    int getCurrentLaneIndex() const { return currentLaneIndex; }
    const std::vector<Road*>& getCurrentRoute() const { return currentRoute; }
    int getCurrentRouteIndex() const { return currentRouteIndex; }
    Road* getNextRoad() const;

    void addTravelHistory(Road* road) { travelHistory.push_back(road); }
    const std::vector<Road*>& getTravelHistory() const { return travelHistory; }

    bool isRoadInUpcomingRoute(int roadId) const;
    bool recalculateRoute(const Graph& graph, PathFindingStrategy* strategy);
    bool performUTurn(const Graph& graph, PathFindingStrategy* strategy);
    bool isAwaitingIntersectionTransition() const { return awaitingIntersectionTransition; }
    double getIntersectionTransitionProgress() const {
        if (!awaitingIntersectionTransition) {
            return 0.0;
        }
        if (intersectionTransitionTimer <= 0.0) {
            return 1.0;
        }
        return 1.0 - (intersectionTransitionTimer / INTERSECTION_TRANSITION_DURATION);
    }

private:
    bool advanceToNextRoad();
};

#endif