#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

class Intersection;
class Road;
class Graph;
class PathFindingStrategy;
class PointOfInterest;

enum class PauseReason {
    None,
    TrafficLight,
    Intersection,
    BusStop
};

struct PauseUpdateResult {
    bool resumed;
    double remainingTime;
};

class Vehicle {
public:
    static constexpr double INTERSECTION_TRANSITION_DURATION = 0.12;
    static constexpr double LANE_CHANGE_COOLDOWN = 3.0;
    static constexpr double LANE_CHANGE_GAP_IMPROVEMENT_FACTOR = 1.3;
    static constexpr double LANE_CHANGE_REAR_SAFETY_TIME = 2.0;
    static constexpr double LANE_CHANGE_FRONT_SAFETY_TIME = 1.5;
    static constexpr double LANE_CHANGE_REACTION_TIME = 0.5;
    static constexpr double LANE_CHANGE_MIN_TTC = 3.0;
    static constexpr double YIELD_LANE_CHANGE_MIN_TTC = 1.5;
    static constexpr double NO_LANE_CHANGE_DISTANCE = 12.0;
    static constexpr double NO_LANE_CHANGE_ROAD_FRACTION = 0.2;
    static constexpr double MAX_PHYSICS_SUBSTEP = 0.05;
    static constexpr double YIELD_COOLDOWN_DURATION = 2.0;  
    static constexpr double YIELD_SPEED_FACTOR = 0.4; 
    static constexpr double YIELD_ESCAPE_SPEED_FACTOR = 1.25;

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
    PauseReason pauseReason;
    bool routeAssigned = false;
    int currentLaneIndex = 0;
    bool awaitingIntersectionTransition = false;
    double intersectionTransitionTimer = 0.0;

    // Intersection whose box-reservation slot this vehicle currently holds
    // (nullptr if it isn't holding one). Released automatically once the
    // intersection-transition animation finishes, or on destruction.
    Intersection* reservedIntersection_ = nullptr;
    double laneChangeCooldownTimer = 0.0;
    bool yielding = false;              // yielding state for ambulance priority
    double yieldCooldownTimer = 0.0;    
    int emergencyLaneToAvoid = -1;          // lane index to avoid when yielding to an ambulance

    double stuckTimer = 0.0;
    double patienceThreshold = 5.0;
    double uTurnCooldownTimer = 0.0;
    
    double recalculateTimer = 5.0;

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
    virtual bool canChangeLanes() const { return true; }

    double getCurrentSpeed() const { return currentSpeed; }

    virtual bool shouldPauseAt(double currentPos,
                                double projectedPos,
                                double& pausePos);
    virtual bool mustStopForTrafficLight(Intersection* nextIntersection) const;
    virtual void onPauseStarted() {}
    virtual PauseUpdateResult updatePause(double availableTime) {
        return {true, availableTime};
    }
    virtual void update(double dt, Graph* graph = nullptr, PathFindingStrategy* strategy = nullptr);
    virtual double getYieldSpeedFactor() const { return YIELD_SPEED_FACTOR; }
    virtual double getYieldEscapeSpeedFactor() const { return YIELD_ESCAPE_SPEED_FACTOR; }
    virtual void notifyEmergencyApproaching(int emergencyLaneIndex = -1) {
        yielding = true;
        yieldCooldownTimer = YIELD_COOLDOWN_DURATION;
        emergencyLaneToAvoid = emergencyLaneIndex;
    }
    bool isYielding() const { return yielding; }

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
    PauseReason getPauseReason() const { return pauseReason; }
    bool hasReachedDestination() const {
        return routeAssigned && currentRoad == nullptr && currentRouteIndex >= static_cast<int>(currentRoute.size());
    }
    double getProgressRatio() const;
    int getCurrentLaneIndex() const { return currentLaneIndex; }
    const std::vector<Road*>& getCurrentRoute() const { return currentRoute; }
    int getCurrentRouteIndex() const { return currentRouteIndex; }
    Road* getNextRoad() const;
    Road* getPreviousRoad() const { return travelHistory.empty() ? nullptr : travelHistory.back(); }
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

protected:
    PauseReason getIntersectionControlReason() const;
    double getIntersectionStopPosition() const;
    void beginPause(PauseReason reason);
    void clearPause();
    virtual int getRequiredLaneIndex() const { return -1; }
    virtual double getLanePreparationSpeedLimit(double freeFlowSpeed) const {
        return freeFlowSpeed;
    }

private:
    bool advanceToNextRoad();
    bool tryRequiredLaneChange(int requiredLaneIndex);
    void tryLaneChange(double freeFlowSpeedHint);
    void tryYieldLaneChange(); 
};

#endif
