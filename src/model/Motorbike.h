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
        
        if (currentRoad->getLane(currentLaneIndex).isBlocked()) return 0.0;

        double maxAllowedSpeed = std::min(baseSpeed, currentRoad->getSpeedLimit());

        double rawCongestion = currentRoad->getCongestionLevel();

        double effectiveCongestion = 1.0 + (rawCongestion - 1.0) * WEAVING_FACTOR;

        return maxAllowedSpeed / effectiveCongestion;
    }

    // Motorbikes are light and nimble: quick to speed up and quick to
    // brake, letting them weave through traffic more responsively.
    double getAcceleration() const override { return 25.0; }
    double getDeceleration() const override { return 35.0; }

    // Motorbikes are small and can tuck in much closer to the vehicle ahead
    // than a car or bus.
    double getLength() const override { return 2.0; }
    double getHeight() const override { return 1.1; }  // metres — low profile
    double getWeight() const override { return 0.2; }  // tonnes — very light
    double getMinGap() const override { return 1.0; }
};

#endif