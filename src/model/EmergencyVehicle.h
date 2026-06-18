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
};

#endif