#ifndef DESTINATION_H
#define DESTINATION_H

#include "PointOfInterest.h"

/**
 * Destination — A POI subclass representing locations where vehicles
 * want to travel to.
 *
 * Examples: Restaurant, Cinema, Supermarket, Tourist Spot.
 *
 * OOP: Inheritance (Destination IS-A PointOfInterest),
 *      Polymorphism (overrides isDestination).
 */
class Destination : public PointOfInterest {
private:
    int parkingSpaces; // how many vehicles can park at this destination

public:
    Destination(int id, const std::string& name, POIType type,
                double x, double y, Intersection* nearest = nullptr,
                int parkingSpaces = 5)
        : PointOfInterest(id, name, type, x, y, nearest),
          parkingSpaces(parkingSpaces) {}

    bool isSpawnPoint() const override { return false; }
    bool isDestination() const override { return true; }

    int getParkingSpaces() const { return parkingSpaces; }
};

// --- Convenience subclasses for specific destination types ---

class Restaurant : public Destination {
public:
    Restaurant(int id, const std::string& name, double x, double y,
               Intersection* nearest = nullptr)
        : Destination(id, name, POIType::RESTAURANT, x, y, nearest, 10) {}
};

class Cinema : public Destination {
public:
    Cinema(int id, const std::string& name, double x, double y,
           Intersection* nearest = nullptr)
        : Destination(id, name, POIType::CINEMA, x, y, nearest, 15) {}
};

class Supermarket : public Destination {
public:
    Supermarket(int id, const std::string& name, double x, double y,
                Intersection* nearest = nullptr)
        : Destination(id, name, POIType::SUPERMARKET, x, y, nearest, 20) {}
};

class TouristSpot : public Destination {
public:
    TouristSpot(int id, const std::string& name, double x, double y,
                Intersection* nearest = nullptr)
        : Destination(id, name, POIType::TOURIST_SPOT, x, y, nearest, 30) {}
};

#endif
