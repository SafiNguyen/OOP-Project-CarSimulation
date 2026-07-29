#ifndef POINT_OF_INTEREST_H
#define POINT_OF_INTEREST_H

#include <string>

class Intersection;
class Road;

/// Types of Point of Interest in the city.
enum class POIType {
    GENERIC,
    // --- Spawn points (where vehicles appear) ---
    PARKING_LOT,
    BUS_STATION,
    HOSPITAL,
    // --- Destinations (where vehicles travel to) ---
    RESTAURANT,
    CINEMA,
    SUPERMARKET,
    TOURIST_SPOT
};

/**
 * PointOfInterest — Base class for named locations in the city.
 *
 * Each POI has real-world coordinates and is linked to the nearest
 * Intersection so the pathfinding system can route vehicles to/from it.
 *
 * OOP: Inheritance base for SpawnPoint and Destination.
 */
class PointOfInterest {
protected:
    int id;
    std::string name;
    POIType type;
    double x;  // world coordinate (km)
    double y;  // world coordinate (km)
    Intersection* nearestIntersection; // for pathfinding
    Road* connectedRoad;               // Road the POI is physically attached to
    double progressOffset;             // Distance from the start of the connectedRoad

public:
    PointOfInterest(int id, const std::string& name, POIType type,
                    double x, double y, Intersection* nearest = nullptr)
        : id(id), name(name), type(type), x(x), y(y),
          nearestIntersection(nearest), connectedRoad(nullptr), progressOffset(0.0) {}

    virtual ~PointOfInterest() = default;

    // --- Getters ---
    int getId() const { return id; }
    const std::string& getName() const { return name; }
    POIType getType() const { return type; }
    double getX() const { return x; }
    double getY() const { return y; }
    Intersection* getNearestIntersection() const { return nearestIntersection; }
    Road* getConnectedRoad() const { return connectedRoad; }
    double getProgressOffset() const { return progressOffset; }

    void setNearestIntersection(Intersection* i) { nearestIntersection = i; }
    void setConnectedRoad(Road* r) { connectedRoad = r; }
    void setProgressOffset(double p) { progressOffset = p; }

    /// Returns true if this POI can be used as a vehicle spawn point.
    virtual bool isSpawnPoint() const { return false; }

    /// Returns true if this POI can be used as a vehicle destination.
    virtual bool isDestination() const { return false; }

    /// Human-readable type label for UI/logging.
    virtual std::string getTypeLabel() const {
        switch (type) {
            case POIType::PARKING_LOT:  return "Parking Lot";
            case POIType::BUS_STATION:  return "Bus Station";
            case POIType::HOSPITAL:     return "Hospital";
            case POIType::RESTAURANT:   return "Restaurant";
            case POIType::CINEMA:       return "Cinema";
            case POIType::SUPERMARKET:  return "Supermarket";
            case POIType::TOURIST_SPOT: return "Tourist Spot";
            default:                    return "Generic";
        }
    }

    /// Converts a string (from JSON) to a POIType enum.
    static POIType typeFromString(const std::string& s) {
        if (s == "parking_lot")  return POIType::PARKING_LOT;
        if (s == "bus_station")  return POIType::BUS_STATION;
        if (s == "hospital")     return POIType::HOSPITAL;
        if (s == "restaurant")   return POIType::RESTAURANT;
        if (s == "cinema")       return POIType::CINEMA;
        if (s == "supermarket")  return POIType::SUPERMARKET;
        if (s == "tourist_spot") return POIType::TOURIST_SPOT;
        return POIType::GENERIC;
    }
};

#endif
