#include "VehicleFactory.h"
#include "Vehicle.h"
#include "Car.h"
#include "Bus.h"
#include "Motorbike.h"
#include "EmergencyVehicle.h"

Vehicle* VehicleFactory::createVehicle(VehicleKind kind, int id, double speed, Intersection* start, Intersection* dest) {
    switch (kind) {
        case VehicleKind::Car:
            return new Car(id, speed, start, dest);
        case VehicleKind::Bus:
            return new Bus(id, speed, start, dest);
        case VehicleKind::Motorbike:
            return new Motorbike(id, speed, start, dest);
        case VehicleKind::Emergency:
            return new EmergencyVehicle(id, speed, start, dest);
    }
    return nullptr;
}
