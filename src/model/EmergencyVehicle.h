#ifndef EMERGENCY_VEHICLE_H
#define EMERGENCY_VEHICLE_H

#include "Vehicle.h"
#include "Road.h"


class EmergencyVehicle : public Vehicle {
public:
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
    double getMinGap() const override { return 1.5; }
};

#endif