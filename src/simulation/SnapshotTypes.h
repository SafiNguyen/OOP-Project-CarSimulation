#ifndef SNAPSHOT_TYPES_H
#define SNAPSHOT_TYPES_H

#include <cstdint>
#include <optional>
#include <vector>
#include <string>
#include <unordered_map>

#include "VehicleTypes.h"
#include "StatisticsManager.h"

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

    std::optional<int> currentRoadId;
    int currentLaneIndex = 0;
    double progressOnCurrentRoad = 0.0;
    double currentSpeed = 0.0;

    std::vector<std::optional<int>> currentRoute;
    int currentRouteIndex = 0;
    std::vector<std::optional<int>> travelHistory;

    bool paused = false;
    PauseReason pauseReason = PauseReason::None;
    MovementState movementState = MovementState::OnRoad;

    std::optional<int> junctionIncomingRoadId;
    int junctionIncomingLane = -1;
    std::optional<int> junctionOutgoingRoadId;
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

    // Per-vehicle deterministic state used for future reroute scheduling.
    std::uint32_t rerouteRandomState = 0;
};

// --- Per-intersection snapshot (traffic light + reservations) ---
struct IntersectionSnapshot {
    int id = -1;
    std::size_t activePhaseGroup = 0;
    int signalStage = 0;  // SignalStage enum as int
    double stageRemainingSeconds = 0.0;

    struct ReservationData {
        std::optional<int> fromRoadId;
        int incomingLane = -1;
        std::optional<int> outgoingRoadId;
        int outgoingLane = -1;
        double progressMetres = 0.0;
        double vehicleLengthMetres = 4.5;
        double vehicleWidthMetres = 1.8;
        std::uint64_t entryId = 0;
    };
    std::unordered_map<int, ReservationData> occupants;
    std::uint64_t nextEntryId = 0;

    int emergencyVehicleId = -1;
    std::optional<int> emergencyIncomingRoadId;
    int emergencyIncomingLane = -1;
    std::optional<int> emergencyOutgoingRoadId;
    int emergencyOutgoingLane = -1;
    double emergencyVehicleWidthMetres = 0.0;
    double emergencyPriorityRemainingSeconds = 0.0;
};

// --- Per-pending-vehicle snapshot ---
struct PendingVehicleSnapshot {
    // Keep the complete polymorphic vehicle payload. The previous flattened
    // representation silently dropped Bus service/stop/dwell state.
    VehicleSnapshot vehicle;
    std::vector<std::optional<int>> route;
    double earliestActivationTime = 0.0;
    double nextAttemptTime = 0.0;
    double deadlineTime = 0.0;
    bool phasedAdmission = false;
    bool routeResolved = false;
    bool fixedRoute = false;
    int routeAttempts = 0;
};

struct RoadRuntimeSnapshot {
    int roadId = -1;
    double congestionLevel = 1.0;
    std::vector<bool> blockedLanes;
};

enum class TrafficEventKind {
    Congestion,
    Accident,
    RoadClosure
};

struct TrafficEventSnapshot {
    TrafficEventKind kind = TrafficEventKind::Congestion;
    int roadId = -1;
    double duration = 0.0;
    double timeElapsed = 0.0;
    double severity = 1.0;
    int laneIndex = -1;
};

struct StatisticsSnapshot {
    std::unordered_map<std::string, AlgorithmMetric> algorithmMetrics;
    std::unordered_map<int, TravelMetric> travelMetrics;
    NetworkMetric networkMetric;
    double totalSimulatedTime = 0.0;
    long long totalRecalculations = 0;
    long long tickCounter = 0;
    int completedTrips = 0;
};

// --- Top-level simulation snapshot (Memento) ---
struct SimulationSnapshot {
    double elapsedTime = 0.0;
    long long tickCount = 0;
    bool paused = false;
    double speedMultiplier = 1.0;
    double leftoverDt = 0.0;
    double lastSnapshotTime = 0.0;
    std::size_t dynamicRerouteCursor = 0;
    std::size_t maximumActiveVehicles = 1;
    double pendingVehicleTimeoutSeconds = 600.0;

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
    std::vector<RoadRuntimeSnapshot> roads;
    std::vector<TrafficEventSnapshot> activeEvents;
    StatisticsSnapshot statistics;

    std::unordered_map<int, std::vector<int>> mergingFromPOIByRoad;
    std::vector<int> failedRecalcIds;
};

#endif // SNAPSHOT_TYPES_H
