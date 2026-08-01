#include "VehicleBehavior.h"
#include "Vehicle.h"
#include "Road.h"
#include <algorithm>

void VehicleBehavior::handleTrafficLightYield(Vehicle& vehicle, double remainingTime) {
    (void)vehicle;
    (void)remainingTime;
}

void VehicleBehavior::handleEmergencyYield(Vehicle& vehicle, double freeFlowSpeed) {
    (void)vehicle;
    (void)freeFlowSpeed;
}

void VehicleBehavior::handlePoiBehavior(Vehicle& vehicle, double dt) {
    (void)vehicle;
    (void)dt;
}

void VehicleBehavior::updateSignals(Vehicle& vehicle, double dt) {
    if (vehicle.laneChangeState_ != LaneChangeState::Idle) {
        vehicle.laneChangeSignalElapsedSeconds_ += dt;
    }
    vehicle.refreshTurnSignal();
}
