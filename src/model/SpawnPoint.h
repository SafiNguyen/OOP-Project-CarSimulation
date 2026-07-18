#ifndef SPAWN_POINT_H
#define SPAWN_POINT_H

#include "PointOfInterest.h"

/**
 * SpawnPoint — A POI subclass representing locations where vehicles can
 * appear (spawn) in the simulation.
 *
 * Examples: ParkingLot, BusStation, Hospital.
 *
 * OOP: Inheritance (SpawnPoint IS-A PointOfInterest),
 *      Polymorphism (overrides isSpawnPoint).
 */
class SpawnPoint : public PointOfInterest {
private:
    int capacity; // max vehicles that can spawn here simultaneously

public:
    SpawnPoint(int id, const std::string& name, POIType type,
               double x, double y, Intersection* nearest = nullptr,
               int capacity = 10)
        : PointOfInterest(id, name, type, x, y, nearest),
          capacity(capacity) {}

    bool isSpawnPoint() const override { return true; }
    bool isDestination() const override { return false; }

    int getCapacity() const { return capacity; }
    void setCapacity(int c) { capacity = c; }
};

// --- Convenience subclasses for specific spawn types ---

class ParkingLot : public SpawnPoint {
public:
    ParkingLot(int id, const std::string& name, double x, double y,
               Intersection* nearest = nullptr, int capacity = 20)
        : SpawnPoint(id, name, POIType::PARKING_LOT, x, y, nearest, capacity) {}
};

class BusStation : public SpawnPoint {
public:
    BusStation(int id, const std::string& name, double x, double y,
               Intersection* nearest = nullptr, int capacity = 5)
        : SpawnPoint(id, name, POIType::BUS_STATION, x, y, nearest, capacity) {}
};

class HospitalSpawn : public SpawnPoint {
public:
    HospitalSpawn(int id, const std::string& name, double x, double y,
                  Intersection* nearest = nullptr, int capacity = 3)
        : SpawnPoint(id, name, POIType::HOSPITAL, x, y, nearest, capacity) {}
};

#endif
