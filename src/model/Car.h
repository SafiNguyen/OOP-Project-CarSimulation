#ifndef CAR_H
#define CAR_H

#include "Vehicle.h"
#include "Road.h"
#include <algorithm>


class Car : public Vehicle {
public:
    Car(int id, double speed, Intersection* start, Intersection* dest)
        : Vehicle(id, speed, start, dest) {}

    double calculateCurrentSpeed() const override {
        if (currentRoad == nullptr)   return 0.0;
        if (currentRoad->isBlocked()) return 0.0;

        double maxAllowed = std::min(baseSpeed, currentRoad->getSpeedLimit());
        return maxAllowed / currentRoad->getCongestionLevel();
    }

    // Ordinary passenger car: moderate, everyday acceleration/braking.
    double getAcceleration() const override { return 18.0; }
    double getDeceleration() const override { return 28.0; }

    // Physical dimensions
    double getLength() const override { return 4.5; }  // metres
    double getHeight() const override { return 1.5; }  // metres
    double getWeight() const override { return 1.5; }  // tonnes
    double getMinGap()  const override { return 2.0; } // metres
};

#endif 