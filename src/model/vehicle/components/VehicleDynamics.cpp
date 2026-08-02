#include "VehicleDynamics.h"
#include "Vehicle.h"
#include <algorithm>

void VehicleDynamics::updatePoseTransition(Vehicle& vehicle, double dt) {
    if (vehicle.poseTransitionTimer_ > 0.0) {
        vehicle.poseTransitionTimer_ = std::max(0.0, vehicle.poseTransitionTimer_ - dt);
    }
}

void VehicleDynamics::applyLongitudinalUpdate(Vehicle& vehicle, double targetSpeed, double subDt) {
    if (vehicle.currentSpeed < targetSpeed) {
        vehicle.currentSpeed = std::min(targetSpeed, vehicle.currentSpeed + vehicle.getAcceleration() * subDt);
    } else if (vehicle.currentSpeed > targetSpeed) {
        vehicle.currentSpeed = std::max(targetSpeed, vehicle.currentSpeed - vehicle.getDeceleration() * subDt);
    }
}
