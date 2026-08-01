#ifndef VEHICLE_DYNAMICS_H
#define VEHICLE_DYNAMICS_H

#include "Geometry.h"

class Vehicle;

class VehicleDynamics {
public:
    void updatePoseTransition(Vehicle& vehicle, double dt);
    void applyLongitudinalUpdate(Vehicle& vehicle, double targetSpeed, double subDt);
};

#endif
