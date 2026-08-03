#ifndef POINT_OF_INTEREST_H
#define POINT_OF_INTEREST_H

#include <cstdint>
#include <string>

#include "VehicleTypes.h"

class Intersection;
class Road;

/// Types of Point of Interest in the city.
enum class POIType {
    GENERIC,
    // --- Spawn points (where vehicles appear) ---
    PARKING_LOT,
    BUS_STATION,
    HOSPITAL,
    RESIDENTIAL_AREA,
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
public:
    using VehicleKindMask = std::uint8_t;

    static constexpr VehicleKindMask NO_VEHICLES = 0u;
    static constexpr VehicleKindMask CAR_VEHICLES = 1u << 0;
    static constexpr VehicleKindMask BUS_VEHICLES = 1u << 1;
    static constexpr VehicleKindMask MOTORBIKE_VEHICLES = 1u << 2;
    static constexpr VehicleKindMask EMERGENCY_VEHICLES = 1u << 3;
    static constexpr VehicleKindMask CIVILIAN_VEHICLES =
        CAR_VEHICLES | MOTORBIKE_VEHICLES;

protected:
    int id;
    std::string name;
    POIType type;
    double x;  // world coordinate (km)
    double y;  // world coordinate (km)
    Intersection* nearestIntersection; // for pathfinding
    Road* connectedRoad;               // Road the POI is physically attached to
    double progressOffset;             // Distance from the start of the connectedRoad
    int accessLaneIndex;               // -1 selects the curb lane automatically
    double spawnWeight;
    double destinationWeight;
    double spawnCooldownSeconds;
    bool explicitRoadAccess;
    bool labelOnLeft;
    std::string sourceType;
    bool spawnEnabled;
    bool destinationEnabled;
    VehicleKindMask spawnVehicleKinds;
    VehicleKindMask destinationVehicleKinds;

    void configureLegacyRoles() {
        spawnEnabled = false;
        destinationEnabled = false;
        spawnVehicleKinds = NO_VEHICLES;
        destinationVehicleKinds = NO_VEHICLES;

        switch (type) {
            case POIType::PARKING_LOT:
            case POIType::RESIDENTIAL_AREA:
                spawnEnabled = true;
                destinationEnabled = true;
                spawnVehicleKinds = CIVILIAN_VEHICLES;
                destinationVehicleKinds =
                    CIVILIAN_VEHICLES | EMERGENCY_VEHICLES;
                break;
            case POIType::BUS_STATION:
                spawnEnabled = true;
                destinationEnabled = true;
                spawnVehicleKinds = BUS_VEHICLES;
                destinationVehicleKinds = BUS_VEHICLES;
                break;
            case POIType::HOSPITAL:
                spawnEnabled = true;
                spawnVehicleKinds = EMERGENCY_VEHICLES;
                break;
            case POIType::CINEMA:
            case POIType::SUPERMARKET:
            case POIType::TOURIST_SPOT:
                spawnEnabled = true;
                destinationEnabled = true;
                spawnVehicleKinds = CIVILIAN_VEHICLES;
                destinationVehicleKinds =
                    CIVILIAN_VEHICLES | EMERGENCY_VEHICLES;
                break;
            case POIType::RESTAURANT:
                destinationEnabled = true;
                destinationVehicleKinds =
                    CIVILIAN_VEHICLES | EMERGENCY_VEHICLES;
                break;
            case POIType::GENERIC:
                break;
        }
    }

public:
    PointOfInterest(int id, const std::string& name, POIType type,
                    double x, double y, Intersection* nearest = nullptr)
        : id(id), name(name), type(type), x(x), y(y),
          nearestIntersection(nearest), connectedRoad(nullptr),
          progressOffset(0.0), accessLaneIndex(-1),
          spawnWeight(1.0), destinationWeight(1.0),
          spawnCooldownSeconds(1.0), explicitRoadAccess(false),
          labelOnLeft(false), spawnEnabled(false),
          destinationEnabled(false), spawnVehicleKinds(NO_VEHICLES),
          destinationVehicleKinds(NO_VEHICLES) {}

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
    int getAccessLaneIndex() const { return accessLaneIndex; }
    double getSpawnWeight() const { return spawnWeight; }
    double getDestinationWeight() const { return destinationWeight; }
    double getSpawnCooldownSeconds() const {
        return spawnCooldownSeconds;
    }
    bool hasExplicitRoadAccess() const { return explicitRoadAccess; }
    bool isLabelOnLeft() const { return labelOnLeft; }
    const std::string& getSourceType() const { return sourceType; }
    VehicleKindMask getSpawnVehicleKinds() const {
        return spawnVehicleKinds;
    }
    VehicleKindMask getDestinationVehicleKinds() const {
        return destinationVehicleKinds;
    }

    void setNearestIntersection(Intersection* i) { nearestIntersection = i; }
    void setConnectedRoad(Road* r) { connectedRoad = r; }
    void setProgressOffset(double p) { progressOffset = p; }
    void setAccessLaneIndex(int laneIndex) {
        accessLaneIndex = laneIndex;
    }
    void setSpawnWeight(double weight) { spawnWeight = weight; }
    void setDestinationWeight(double weight) {
        destinationWeight = weight;
    }
    void setSpawnCooldownSeconds(double seconds) {
        spawnCooldownSeconds = seconds;
    }
    void setLabelOnLeft(bool enabled) { labelOnLeft = enabled; }
    void setSourceType(const std::string& value) { sourceType = value; }
    void configureMobilityRoles(
            bool canSpawn,
            bool canBeDestination,
            VehicleKindMask spawnKinds,
            VehicleKindMask destinationKinds) {
        spawnEnabled = canSpawn;
        destinationEnabled = canBeDestination;
        spawnVehicleKinds = canSpawn ? spawnKinds : NO_VEHICLES;
        destinationVehicleKinds =
            canBeDestination ? destinationKinds : NO_VEHICLES;
    }
    void configureRoadAccess(Road* road,
                             double progressMetres,
                             int laneIndex = -1) {
        connectedRoad = road;
        progressOffset = progressMetres;
        accessLaneIndex = laneIndex;
        explicitRoadAccess = road != nullptr;
    }

    /// Returns true if this POI can be used as a vehicle spawn point.
    virtual bool isSpawnPoint() const { return spawnEnabled; }

    /// Returns true if this POI can be used as a vehicle destination.
    virtual bool isDestination() const { return destinationEnabled; }

    bool allowsSpawnVehicle(VehicleKind kind) const {
        return isSpawnPoint() &&
               (spawnVehicleKinds & vehicleKindMask(kind)) != 0u;
    }

    bool allowsDestinationVehicle(VehicleKind kind) const {
        return isDestination() &&
               (destinationVehicleKinds & vehicleKindMask(kind)) != 0u;
    }

    static constexpr VehicleKindMask vehicleKindMask(VehicleKind kind) {
        switch (kind) {
            case VehicleKind::Car: return CAR_VEHICLES;
            case VehicleKind::Bus: return BUS_VEHICLES;
            case VehicleKind::Motorbike: return MOTORBIKE_VEHICLES;
            case VehicleKind::Emergency: return EMERGENCY_VEHICLES;
        }
        return NO_VEHICLES;
    }

    /// Human-readable type label for UI/logging.
    virtual std::string getTypeLabel() const {
        switch (type) {
            case POIType::PARKING_LOT:  return "Parking Lot";
            case POIType::BUS_STATION:  return "Bus Station";
            case POIType::HOSPITAL:     return "Hospital";
            case POIType::RESIDENTIAL_AREA:
                return "Residential Area";
            case POIType::RESTAURANT:   return "Restaurant";
            case POIType::CINEMA:       return "Cinema";
            case POIType::SUPERMARKET:  return "Supermarket";
            case POIType::TOURIST_SPOT: return "Tourist Spot";
            default:
                return sourceType.empty() ? "Generic" : sourceType;
        }
    }

    /// Converts a string (from JSON) to a POIType enum.
    static POIType typeFromString(const std::string& s) {
        if (s == "parking_lot" || s == "parking")
            return POIType::PARKING_LOT;
        if (s == "bus_station")  return POIType::BUS_STATION;
        if (s == "hospital")     return POIType::HOSPITAL;
        if (s == "residential_area" || s == "residential")
            return POIType::RESIDENTIAL_AREA;
        if (s == "restaurant")   return POIType::RESTAURANT;
        if (s == "cinema")       return POIType::CINEMA;
        if (s == "supermarket")  return POIType::SUPERMARKET;
        if (s == "tourist_spot") return POIType::TOURIST_SPOT;
        return POIType::GENERIC;
    }
};

#endif
