#ifndef VEHICLE_TYPES_H
#define VEHICLE_TYPES_H

enum class PauseReason {
    None,
    TrafficLight,
    PedestrianCrossing,
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

struct PauseUpdateResult {
    bool resumed;
    double remainingTime;
};

#endif // VEHICLE_TYPES_H
