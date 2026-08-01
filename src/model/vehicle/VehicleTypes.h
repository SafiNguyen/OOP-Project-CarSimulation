#ifndef VEHICLE_TYPES_H
#define VEHICLE_TYPES_H

enum class PauseReason {
    None,
    TrafficLight,
    Intersection,
    BusStop
};

enum class MovementState {
    OnRoad,
    WaitingAtIntersection,
    TraversingJunction,
    DwellingAtBusStop
};

enum class VehicleKind {
    Car,
    Bus,
    Motorbike,
    Emergency
};

enum class TurnSignal {
    Off,
    Left,
    Right
};

enum class TurnSignalReason {
    None,
    Junction,
    LaneChange,
    BusStop
};

enum class LaneChangeState {
    Idle,
    Signaling,
    WaitingForGap
};

enum class SpawnLifecycleState {
    Scheduled,
    WaitingForSourceCapacity,
    WaitingForRoute,
    WaitingForRoadGap,
    Merging,
    Active
};

// A POI departure is deliberately split around the road-edge yield line.
// Vehicles do not reserve traffic space while they are still on the private
// driveway. Once committed, the reservation is visible to normal car-following
// and lane-change queries until the vehicle becomes a regular lane occupant.
enum class PoiMergePhase {
    None,
    ApproachingYieldLine,
    WaitingForGap,
    Committed
};

struct PauseUpdateResult {
    bool resumed;
    double remainingTime;
};

#endif // VEHICLE_TYPES_H
