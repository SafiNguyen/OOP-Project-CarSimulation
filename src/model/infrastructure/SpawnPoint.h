#ifndef SPAWN_POINT_H
#define SPAWN_POINT_H

#include <utility>

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
    int capacity; // max vehicles staged here while waiting to enter traffic
    mutable int occupiedSpawnSlots;

public:
    SpawnPoint(int id, const std::string& name, POIType type,
               double x, double y, Intersection* nearest = nullptr,
               int capacity = 10)
        : PointOfInterest(id, name, type, x, y, nearest),
          capacity(capacity),
          occupiedSpawnSlots(0) {}

    bool isSpawnPoint() const override { return true; }
    bool isDestination() const override { return false; }

    int getCapacity() const { return capacity; }
    void setCapacity(int c) { capacity = c; }
    bool tryReserveSpawnSlot() const {
        if (occupiedSpawnSlots >= capacity) {
            return false;
        }
        ++occupiedSpawnSlots;
        return true;
    }
    void releaseSpawnSlot() const {
        if (occupiedSpawnSlots > 0) {
            --occupiedSpawnSlots;
        }
    }
    int getOccupiedSpawnSlots() const {
        return occupiedSpawnSlots;
    }
    void resetSpawnSlotsForRestore() const {
        occupiedSpawnSlots = 0;
    }
};

// --- Convenience subclasses for specific spawn types ---

class ParkingLot : public SpawnPoint {
public:
    ParkingLot(int id, const std::string& name, double x, double y,
               Intersection* nearest = nullptr, int capacity = 20)
        : SpawnPoint(id, name, POIType::PARKING_LOT, x, y, nearest, capacity) {}

    bool isDestination() const override { return true; }
};

class BusStation : public SpawnPoint {
private:
    using SpawnPoint::setCapacity;

    std::string code_;
    Road* departureRoad_;
    Road* arrivalRoad_;

public:
    BusStation(int id, const std::string& name, double x, double y,
               Intersection* nearest = nullptr, int capacity = 5)
        : SpawnPoint(id, name, POIType::BUS_STATION, x, y, nearest, capacity),
          departureRoad_(nullptr),
          arrivalRoad_(nullptr) {}

    BusStation(int id,
               std::string code,
               const std::string& name,
               double x,
               double y,
               Intersection* accessIntersection,
               Road* departureRoad,
               Road* arrivalRoad,
               int capacity,
               double accessProgressMetres = 0.0,
               int accessLaneIndex = -1)
        : SpawnPoint(
              id,
              name,
              POIType::BUS_STATION,
              x,
              y,
              accessIntersection,
              capacity),
          code_(std::move(code)),
          departureRoad_(departureRoad),
          arrivalRoad_(arrivalRoad) {
        configureRoadAccess(
            departureRoad,
            accessProgressMetres,
            accessLaneIndex);
    }

    bool isDestination() const override { return true; }

    const std::string& getCode() const { return code_; }
    Intersection* getAccessIntersection() const {
        return getNearestIntersection();
    }
    Road* getDepartureRoad() const { return departureRoad_; }
    Road* getArrivalRoad() const { return arrivalRoad_; }
    bool usesSharedAccessRoad() const {
        return departureRoad_ != nullptr &&
               arrivalRoad_ == departureRoad_;
    }
    bool isConfiguredTransitStation() const {
        return !code_.empty() &&
               getAccessIntersection() != nullptr &&
               departureRoad_ != nullptr &&
               arrivalRoad_ != nullptr;
    }

    bool tryAcquireDepartureSlot() const {
        return tryReserveSpawnSlot();
    }
    void releaseDepartureSlot() const {
        releaseSpawnSlot();
    }
    int getOccupiedDepartureSlots() const {
        return getOccupiedSpawnSlots();
    }
};

class HospitalSpawn : public SpawnPoint {
public:
    HospitalSpawn(int id, const std::string& name, double x, double y,
                  Intersection* nearest = nullptr, int capacity = 3)
        : SpawnPoint(id, name, POIType::HOSPITAL, x, y, nearest, capacity) {}
};

class ResidentialArea : public SpawnPoint {
public:
    ResidentialArea(int id, const std::string& name, double x, double y,
                    Intersection* nearest = nullptr, int capacity = 12)
        : SpawnPoint(
              id,
              name,
              POIType::RESIDENTIAL_AREA,
              x,
              y,
              nearest,
              capacity) {}

    bool isDestination() const override { return true; }
};

#endif
