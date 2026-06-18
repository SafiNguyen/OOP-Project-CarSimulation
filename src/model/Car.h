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
};

#endif 