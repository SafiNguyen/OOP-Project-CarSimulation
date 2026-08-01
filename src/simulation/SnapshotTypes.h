#ifndef SNAPSHOT_TYPES_H
#define SNAPSHOT_TYPES_H

#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>

#include "VehicleTypes.h"

// Forward declarations
class TrafficSimulator;
class Vehicle;
class Intersection;
class Road;
class Bus;
class PointOfInterest;

// --- Per-vehicle snapshot (compact POD) ---
struct VehicleSnapshot {
    int id = -1;
    VehicleKind kind = VehicleKind::Car;

    int currentRoadId = -1;
    int currentLaneIndex = 0;
    double progressOnCurrentRoad = 0.0;
    double currentSpeed = 0.0;

    std::vector<int> currentRoute;
    int currentRouteIndex = 0;
    std::vector<int> travelHistory;

    bool paused = false;
    PauseReason pauseReason = PauseReason::None;
    MovementState movementState = MovementState::OnRoad;

    int junctionIncomingRoadId = -1;
    int junctionIncomingLane = -1;
    int junctionOutgoingRoadId = -1;
    int junctionOutgoingLane = -1;
    double junctionProgressMetres = 0.0;
    int reservedIntersectionId = -1;

    double laneChangeCooldownTimer = 0.0;
    bool yielding = false;
    double yieldCooldownTimer = 0.0;
    int emergencyLaneToAvoid = -1;
    LaneChangeState laneChangeState = LaneChangeState::Idle;
    int laneChangeTargetLane = -1;
    TurnSignalReason laneChangeReason = TurnSignalReason::None;
    double laneChangeSignalElapsedSeconds = 0.0;
    TurnSignal turnSignal = TurnSignal::Off;
    TurnSignalReason turnSignalReason = TurnSignalReason::None;
    TurnSignal junctionTurnSignal = TurnSignal::Off;

    double poseTransitionFromX = 0.0;
    double poseTransitionFromY = 0.0;
    double poseTransitionFromHeading = 0.0;
    double poseTransitionToX = 0.0;
    double poseTransitionToY = 0.0;
    double poseTransitionToHeading = 0.0;
    double poseTransitionTimer = 0.0;
    double poseTransitionDuration = 0.0;

    double stuckTimer = 0.0;
    double patienceThreshold = 5.0;
    double uTurnCooldownTimer = 0.0;
    double simulationTimeSeconds = 0.0;
    double recalculateTimer = 10.0;
    bool isWaitingForLight = false;

    bool isMergingFromPOI = false;
    PoiMergePhase poiMergePhase = PoiMergePhase::None;
    bool isEnteringPOI = false;
    int mergeSourcePOIId = -1;
    double mergeProgressOffset = -1.0;
    int mergeLaneIndex = -1;
    double poiAnimationTimer = 0.0;
    double poiAnimationDuration = 2.0;

    int spawnPointId = -1;
    int destinationId = -1;
    double baseSpeed = 0.0;
    int spawnPOIId = -1;
    int targetPOIId = -1;
    SpawnLifecycleState spawnLifecycleState = SpawnLifecycleState::Scheduled;
    int reservedSpawnPointId = -1;

    // Bus-specific
    double dwellTime = 0.0;
    double dwellTimer = 0.0;
    int nextStopId = -1;
    double nextStopPos = -1.0;
    std::string fleetCode;
    int serviceId = -1;
    int originStationId = -1;
    int destinationStationId = -1;
    std::vector<int> assignedStopIds;
    std::vector<std::size_t> assignedStopRouteIndices;
    double scheduledDepartureTime = 0.0;
    std::size_t scheduledStopIndex = 0;
    std::vector<int> servedStopIds;
    std::vector<int> missedStopIds;
    int tripState = 0;
    bool departureSlotHeld = false;
};

// --- Per-intersection snapshot (traffic light + reservations) ---
struct IntersectionSnapshot {
    int id = -1;
    std::size_t activePhaseGroup = 0;
    int signalStage = 0;  // SignalStage enum as int
    double stageRemainingSeconds = 0.0;

    struct ReservationData {
        int fromRoadId = -1;
        double progressMetres = 0.0;
        double vehicleLengthMetres = 4.5;
        double vehicleWidthMetres = 1.8;
    };
    std::unordered_map<int, ReservationData> occupants;

    int emergencyVehicleId = -1;
    int emergencyIncomingRoadId = -1;
    int emergencyIncomingLane = -1;
    int emergencyOutgoingRoadId = -1;
    int emergencyOutgoingLane = -1;
    double emergencyPriorityRemainingSeconds = 0.0;
};

// --- Per-pending-vehicle snapshot ---
struct PendingVehicleSnapshot {
    int vehicleId = -1;
    VehicleKind kind = VehicleKind::Car;
    std::vector<int> route;
    double earliestActivationTime = 0.0;
    double nextAttemptTime = 0.0;
    double deadlineTime = 0.0;
    bool phasedAdmission = false;
    bool routeResolved = false;
    bool fixedRoute = false;
    int routeAttempts = 0;
    int spawnPOIId = -1;
    int targetPOIId = -1;
    int spawnPointId = -1;
    int destinationId = -1;
    double baseSpeed = 0.0;
    std::string fleetCode;
    int serviceId = -1;
    double scheduledDepartureTime = 0.0;
    std::vector<int> assignedStopIds;
    std::vector<std::size_t> assignedStopRouteIndices;
};

// --- Top-level simulation snapshot (Memento) ---
struct SimulationSnapshot {
    double elapsedTime = 0.0;
    long long tickCount = 0;
    bool paused = false;
    double speedMultiplier = 1.0;
    double leftoverDt = 0.0;

    std::size_t accepted = 0;
    std::size_t activated = 0;
    std::size_t delayedAttempts = 0;
    std::size_t rejected = 0;
    std::size_t timedOut = 0;

    std::unordered_map<int, double> nextSpawnTimeByRoad;
    std::unordered_map<int, double> nextSpawnTimeBySource;
    std::unordered_map<int, double> nextTransitDepartureTimeByService;
    double nextTransitNetworkDepartureTime = 0.0;

    std::vector<VehicleSnapshot> vehicles;
    std::vector<IntersectionSnapshot> intersections;
    std::vector<PendingVehicleSnapshot> pendingVehicles;

    std::unordered_map<int, std::vector<int>> mergingFromPOIByRoad;
    std::vector<int> failedRecalcIds;
};

#endif // SNAPSHOT_TYPES_H
