#ifndef EMERGENCY_VEHICLE_H
#define EMERGENCY_VEHICLE_H

#include "Vehicle.h"
#include "Road.h"


class EmergencyVehicle : public Vehicle {
public:
    static constexpr double YIELD_LOOKAHEAD_DISTANCE = 60.0;
    EmergencyVehicle(int id, double speed, Intersection* start, Intersection* dest)
        : Vehicle(id, speed, start, dest) {}

    double calculateCurrentSpeed() const override {
        if (currentRoad == nullptr)   return 0.0;
        if (currentRoad->isBlocked()) return 0.0;
        return baseSpeed;
    }

    // Emergency vehicles are built/driven to accelerate and brake harder
    // than everyday traffic.
    double getAcceleration() const override { return 30.0; }
    double getDeceleration() const override { return 40.0; }
    bool mustStopForTrafficLight(Intersection* nextIntersection) const override {
    return false; // xe cứu thương được ưu tiên vượt đèn đỏ
    }

    // Sturdier brakes let emergency vehicles safely run a tighter gap.
    double getLength() const override { return 6.0; }  // metres — ambulance/fire truck
    double getHeight() const override { return 2.5; }  // metres — tall with equipment
    double getWeight() const override { return 3.5; }  // tonnes
    double getMinGap() const override { return 1.5; }
    //ambulance yielding: ambulance will not yield to any vehicle
    double getYieldSpeedFactor() const override { return 1.0; }
    void notifyEmergencyApproaching() override { /* no-op: ambulance không nhường ai */ }

    void update(double dt) override {
        Vehicle::update(dt);
        notifyVehiclesAhead();
    }

private:
    void notifyVehiclesAhead() const {
        Road* road = getCurrentRoad();
        if (road == nullptr) {
            return;
        }

        const double selfProgress = getProgressOnRoad();
        const double remainingOnRoad = road->getDistance() - selfProgress;

        for (Vehicle* v : road->getVehiclesInProgressRange(
                 selfProgress, selfProgress + YIELD_LOOKAHEAD_DISTANCE)) {
            if (v != this) {
                v->notifyEmergencyApproaching();
            }
        }

        if (remainingOnRoad < YIELD_LOOKAHEAD_DISTANCE) {
            Road* next = getNextRoad();
            if (next != nullptr) {
                const double spillover = YIELD_LOOKAHEAD_DISTANCE - remainingOnRoad;
                for (Vehicle* v : next->getVehiclesInProgressRange(0.0, spillover)) {
                    if (v != this) {
                        v->notifyEmergencyApproaching();
                    }
                }
            }
        }
    }
};

#endif