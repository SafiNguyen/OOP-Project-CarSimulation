#ifndef DESTINATION_H
#define DESTINATION_H

#include "SpawnPoint.h"

/**
 * Destination — A POI subclass representing locations where vehicles
 * want to travel to.
 *
 * Examples: Restaurant, Cinema, Supermarket, Tourist Spot.
 *
 * OOP: Destination reuses SpawnPoint capacity because map data can configure
 *      the same physical place as both an origin and a destination.
 */
class Destination : public SpawnPoint {
public:
    Destination(int id, const std::string& name, POIType type,
                double x, double y, Intersection* nearest = nullptr,
                int parkingSpaces = 5)
        : SpawnPoint(id, name, type, x, y, nearest, parkingSpaces) {
        setSpawnWeight(0.0);
    }

    int getParkingSpaces() const { return getCapacity(); }
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
