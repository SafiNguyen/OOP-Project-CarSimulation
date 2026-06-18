#ifndef MOTORBIKE_H
#define MOTORBIKE_H

#include "Vehicle.h"
#include "Road.h"
#include <algorithm>

class Motorbike : public Vehicle {
public:

    static constexpr double WEAVING_FACTOR = 0.5;

    Motorbike(int id, double speed, Intersection* start, Intersection* dest)
        : Vehicle(id, speed, start, dest) {}

    double calculateCurrentSpeed() const override {
        if (currentRoad == nullptr)   return 0.0;
        
        if (currentRoad->isBlocked()) return 0.0;

        double maxAllowedSpeed = std::min(baseSpeed, currentRoad->getSpeedLimit());

        double rawCongestion = currentRoad->getCongestionLevel();

        double effectiveCongestion = 1.0 + (rawCongestion - 1.0) * WEAVING_FACTOR;

        return maxAllowedSpeed / effectiveCongestion;
    }
};

#endif