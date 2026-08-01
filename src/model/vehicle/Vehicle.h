#ifndef VEHICLE_H
#define VEHICLE_H

#include <cstdlib>
#include <memory>
#include <vector>
#include "Geometry.h"
#include "JunctionConnector.h"
#include "VehicleTypes.h"
#include "simulation/SnapshotTypes.h"

class Intersection;
class Road;
class Graph;
class PathFindingStrategy;
class PointOfInterest;
class SpawnPoint;
enum class JunctionDecision;

class Vehicle {
public:
    static constexpr double LANE_CHANGE_COOLDOWN = 8.0;
    static constexpr double LANE_CHANGE_GAP_IMPROVEMENT_FACTOR = 1.3;
    static constexpr double LANE_CHANGE_REAR_SAFETY_TIME = 2.0;
    static constexpr double LANE_CHANGE_FRONT_SAFETY_TIME = 1.5;
    static constexpr double LANE_CHANGE_REACTION_TIME = 0.5;
    static constexpr double LANE_CHANGE_MIN_TTC = 3.0;
    static constexpr double YIELD_LANE_CHANGE_MIN_TTC = 1.5;
    static constexpr double NO_LANE_CHANGE_DISTANCE = 12.0;
    static constexpr double NO_LANE_CHANGE_ROAD_FRACTION = 0.2;
    static constexpr double MAX_PHYSICS_SUBSTEP = 1.0 / 60.0;
    static constexpr double YIELD_COOLDOWN_DURATION = 2.0;  
    static constexpr double YIELD_SPEED_FACTOR = 0.4; 
    static constexpr double YIELD_ESCAPE_SPEED_FACTOR = 1.25;
    static constexpr double EMERGENCY_JUNCTION_CAUTION_DISTANCE = 45.0;
    static constexpr double EMERGENCY_JUNCTION_YIELD_SPEED_FACTOR = 0.25;
    static constexpr double MIN_SIGNAL_LEAD_TIME_SECONDS = 1.0;
    static constexpr double PRIORITY_SIGNAL_LEAD_TIME_SECONDS = 0.25;
    static constexpr double PRIORITY_LANE_CHANGE_COOLDOWN = 0.5;
    static constexpr double TURN_SIGNAL_BLINK_PERIOD_SECONDS = 1.0;

protected:
    int id;
    double baseSpeed; // m/s
    Intersection* spawnPoint;
    PointOfInterest* spawnPOI = nullptr;
    PointOfInterest* targetPOI = nullptr; // optional POI destination
    const SpawnPoint* reservedSpawnPoint_ = nullptr;
    SpawnLifecycleState spawnLifecycleState_ =
        SpawnLifecycleState::Scheduled;

    // POI Mid-road merging state
    bool isMergingFromPOI = false;
    PoiMergePhase poiMergePhase_ = PoiMergePhase::None;
    bool isEnteringPOI = false;
    const PointOfInterest* mergeSourcePOI_ = nullptr;
    double mergeProgressOffset = -1.0;
    int mergeLaneIndex = -1;
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
    MovementState movementState_ = MovementState::OnRoad;
    std::shared_ptr<const JunctionConnector> activeConnector_;
    double junctionProgressMetres_ = 0.0;
    int incomingLaneIndex_ = 0;
    int outgoingLaneIndex_ = 0;
    Road* junctionOutgoingRoad_ = nullptr;

    // Intersection whose box-reservation slot this vehicle currently holds
    // (nullptr if it isn't holding one). Released automatically once the
    // intersection-transition animation finishes, or on destruction.
    Intersection* reservedIntersection_ = nullptr;
    double laneChangeCooldownTimer = 0.0;
    bool yielding = false;              // yielding state for ambulance priority
    double yieldCooldownTimer = 0.0;    
    int emergencyLaneToAvoid = -1;          // lane index to avoid when yielding to an ambulance

    Pose2D poseTransitionFrom_{};
    Pose2D poseTransitionTo_{};
    double poseTransitionTimer_ = 0.0;
    double poseTransitionDuration_ = 0.0;

    double stuckTimer = 0.0;
    double patienceThreshold = 5.0;
    double uTurnCooldownTimer = 0.0;
    TurnSignal turnSignal_ = TurnSignal::Off;
    TurnSignalReason turnSignalReason_ = TurnSignalReason::None;
    TurnSignal junctionTurnSignal_ = TurnSignal::Off;
    LaneChangeState laneChangeState_ = LaneChangeState::Idle;
    int laneChangeTargetLane_ = -1;
    TurnSignalReason laneChangeReason_ = TurnSignalReason::None;
    double laneChangeSignalElapsedSeconds_ = 0.0;
    double simulationTimeSeconds_ = 0.0;
    
    double recalculateTimer = 10.0 + static_cast<double>(std::rand() % 50) / 10.0; // 10-15s initial spread
    
    bool isWaitingForLight_ = false;
    Vehicle* currentLeader_ = nullptr;
    
    double poiAnimationTimer = 2.0;
    double poiAnimationDuration = 2.0;

public:
    Vehicle(int id, double speed, Intersection* start, Intersection* dest);
    Vehicle(const Vehicle&) = delete;
    Vehicle& operator=(const Vehicle&) = delete;
    Vehicle(Vehicle&&) = delete;
    Vehicle& operator=(Vehicle&&) = delete;

    virtual ~Vehicle();

    // --- Snapshot support (Memento pattern) ---
    // Captures this vehicle's full state into `snap`.
    // `graph` is needed to resolve Road*/Intersection* pointers to ids.
    virtual void captureSnapshot(VehicleSnapshot& snap,
                                 const class Graph& graph) const;
    // Restores this vehicle's state from `snap`.
    // `graph` is needed to resolve ids back to Road*/Intersection* pointers.
    virtual void restoreSnapshot(const VehicleSnapshot& snap,
                                 class Graph& graph);

    virtual double calculateCurrentSpeed() const = 0;
    virtual void onRoadChanged() {}
    virtual double getAcceleration() const;
    virtual double getDeceleration() const;
    virtual double getMaxLateralAcceleration() const;
    virtual VehicleKind getVehicleKind() const;
    virtual bool hasTrafficPriority() const { return false; }
    virtual bool canTurnRightOnRed() const { return false; }
    virtual bool canChangeLanes() const;
    virtual bool allowsDynamicRerouting() const { return true; }
    virtual bool allowsUTurn() const { return true; }

    double getCurrentSpeed() const { return currentSpeed; }
    Pose2D getPose() const;
    MovementState getMovementState() const { return movementState_; }
    double getJunctionProgress() const { return junctionProgressMetres_; }
    int getIncomingLaneIndex() const { return incomingLaneIndex_; }
    int getOutgoingLaneIndex() const { return outgoingLaneIndex_; }
    TurnSignal getTurnSignal() const { return turnSignal_; }
    TurnSignalReason getTurnSignalReason() const {
        return turnSignalReason_;
    }
    LaneChangeState getLaneChangeState() const {
        return laneChangeState_;
    }
    bool isTurnSignalBlinkOn() const;
    double getSimulationTimeSeconds() const {
        return simulationTimeSeconds_;
    }

    virtual bool shouldPauseAt(double currentPos,
                                double projectedPos,
                                double& pausePos);
    virtual void onPauseStarted() {}
    virtual PauseUpdateResult updatePause(double availableTime);
    bool isStuckInJam(int depth = 0) const;
    // TrafficSimulator can defer expensive route searches without pausing
    // the vehicle's regular movement update.
    virtual void update(double dt,
                        Graph* graph = nullptr,
                        PathFindingStrategy* strategy = nullptr,
                        bool allowDynamicReroute = true);
    virtual double getYieldSpeedFactor() const;
    virtual double getYieldEscapeSpeedFactor() const;
    virtual void notifyEmergencyApproaching(int emergencyLaneIndex = -1);
    bool isYielding() const { return yielding; }

    void setRoute(const std::vector<Road*>& route);
    bool setRouteAt(const std::vector<Road*>& route,
                    int initialLaneIndex,
                    double initialProgressMetres);

    int getId() const { return id; }
    Intersection* getSpawnPoint() const { return spawnPoint; }
    Intersection* getDestination() const { return destination; }
    void setSpawnPoint(Intersection* start) { spawnPoint = start; }
    void setDestination(Intersection* dest) { destination = dest; }
    double getBaseSpeed() const { return baseSpeed; }
    virtual double getLength() const;    // metres
    virtual double getWidth() const;     // metres
    virtual double getHeight() const;    // metres
    virtual double getWeight() const;    // tonnes
    virtual double getMiniGap() const;
    virtual double getMinGap() const;
    virtual double getTimeHeadway() const;
    double getProgressOnRoad() const { return progressOnCurrentRoad; }

    // --- POI support ---
    PointOfInterest* getSpawnPOI() const { return spawnPOI; }
    void setSpawnPOI(PointOfInterest* poi) { spawnPOI = poi; }
    PointOfInterest* getTargetPOI() const { return targetPOI; }
    void setTargetPOI(PointOfInterest* poi) { targetPOI = poi; }
    bool tryReserveSpawnSlot();
    void releaseSpawnSlot();
    bool hasSpawnSlotReservation() const {
        return reservedSpawnPoint_ != nullptr;
    }
    SpawnLifecycleState getSpawnLifecycleState() const {
        return spawnLifecycleState_;
    }
    void setSpawnLifecycleState(
        SpawnLifecycleState state) {
        spawnLifecycleState_ = state;
    }

    bool getIsMergingFromPOI() const { return isMergingFromPOI; }
    void setIsMergingFromPOI(bool merging) {
        setMergingFromPOI(
            merging,
            mergeProgressOffset,
            mergeLaneIndex);
    }
    void setMergeSourcePOI(const PointOfInterest* poi) {
        mergeSourcePOI_ = poi;
    }
    
    double getPoiAnimationTimer() const { return poiAnimationTimer; }
    double getPoiAnimationDuration() const { return poiAnimationDuration; }
    PoiMergePhase getPoiMergePhase() const {
        return poiMergePhase_;
    }
    bool hasActiveMergeReservation() const {
        return isMergingFromPOI &&
               poiMergePhase_ == PoiMergePhase::Committed;
    }
    void updatePoiAnimation(double dt);

    void setMergingFromPOI(bool merging, double offset = -1.0, int laneIdx = -1);

    bool getIsEnteringPOI() const { return isEnteringPOI; }
    void setEnteringPOI(bool entering);

    double getMergeProgressOffset() const { return mergeProgressOffset; }
    int getMergeLaneIndex() const { return mergeLaneIndex; }
    Road* getCurrentRoad() const { return currentRoad; }
    bool isPaused() const { return paused; }
    PauseReason getPauseReason() const { return pauseReason; }
    bool hasReachedDestination() const;
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
    bool isAwaitingIntersectionTransition() const {
        return movementState_ == MovementState::TraversingJunction;
    }
    double getIntersectionTransitionProgress() const;

protected:
    PauseReason getIntersectionControlReason() const;
    double getIntersectionStopPosition() const;
    void beginPause(PauseReason reason);
    void clearPause();
    virtual int getRequiredLaneIndex() const;
    virtual double getLanePreparationSpeedLimit(double freeFlowSpeed) const {
        return freeFlowSpeed;
    }
    virtual double getJunctionLanePreparationDistance() const {
        return 30.0;
    }
    virtual double getEmergencyJunctionSpeedLimit(
        double freeFlowSpeed) const {
        return freeFlowSpeed *
               EMERGENCY_JUNCTION_YIELD_SPEED_FACTOR;
    }
    virtual bool shouldBypassQueueBeforeJunction(
        const LaneMapping& preferredMapping) const {
        (void)preferredMapping;
        return false;
    }
    LaneMapping getJunctionEntryLaneMapping() const;

private:
    double getPoiMergeYieldPathRatio() const;
    double getPoiMergePhaseDuration(
        PoiMergePhase phase) const;
    bool canCommitPoiMerge() const;
    JunctionDecision getJunctionDecision(
        Intersection* intersection,
        const LaneMapping& mapping,
        Road* outgoingRoad) const;
    int getRedLightCurbYieldLane() const;
    bool advanceToNextRoad();
    LaneMapping getUpcomingLaneMapping() const;
    bool beginJunctionTraversal(
        const LaneMapping& mapping,
        Intersection* intersection);
    void completeJunctionTraversal(double overshootMetres);
    double advanceJunction(double availableTime);
    bool tryRequiredLaneChange(
        int requiredLaneIndex,
        TurnSignalReason reason);
    bool requestLaneChange(
        int targetLaneIndex,
        TurnSignalReason reason);
    void clearLaneChangeIntent();
    void refreshTurnSignal();
    TurnSignal deriveUpcomingJunctionSignal() const;
    void startPoseTransition(const Pose2D& fromPose,
                             const Pose2D& toPose,
                             double durationSeconds);
    void tryLaneChange(double freeFlowSpeedHint);
    void tryYieldLaneChange(); 
};

#endif
