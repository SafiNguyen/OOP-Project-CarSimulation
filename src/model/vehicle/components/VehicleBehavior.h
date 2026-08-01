#ifndef VEHICLE_BEHAVIOR_H
#define VEHICLE_BEHAVIOR_H

class Vehicle;
class Road;
class Graph;
class PathFindingStrategy;

class VehicleBehavior {
public:
    void handleTrafficLightYield(Vehicle& vehicle, double remainingTime);
    void handleEmergencyYield(Vehicle& vehicle, double freeFlowSpeed);
    void handlePoiBehavior(Vehicle& vehicle, double dt);
    void updateSignals(Vehicle& vehicle, double dt);
};

#endif
