#ifndef VEHICLE_FACTORY_H
#define VEHICLE_FACTORY_H

#include "VehicleTypes.h"

class Vehicle;
class Intersection;

class VehicleFactory {
public:
    static Vehicle* createVehicle(VehicleKind kind, int id, double speed, Intersection* start, Intersection* dest);
};

#endif // VEHICLE_FACTORY_H
