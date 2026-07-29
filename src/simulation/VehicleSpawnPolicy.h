#ifndef VEHICLE_SPAWN_POLICY_H
#define VEHICLE_SPAWN_POLICY_H

#include <array>
#include <cstdint>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Vehicle.h"

class Graph;
class PointOfInterest;

/**
 * Central policy for automatic traffic demand.
 *
 * This class owns only demand decisions (vehicle mix and POI eligibility).
 * TrafficSimulator remains responsible for routing and safe road admission.
 */
class VehicleSpawnPolicy {
public:
    struct Trip {
        PointOfInterest* origin = nullptr;
        PointOfInterest* destination = nullptr;

        bool isValid() const {
            return origin != nullptr &&
                   destination != nullptr &&
                   origin != destination;
        }
    };

    struct DemandWeights {
        double car = 45.5;
        double motorbike = 45.5;
        double bus = 5.0;
        double emergency = 4.0;
    };

    explicit VehicleSpawnPolicy(
        Graph& graph,
        std::uint32_t seed = 42u,
        DemandWeights weights = {});

    VehicleKind selectVehicleKind(bool includeGenericBus);
    Trip selectTrip(VehicleKind kind);

    std::vector<PointOfInterest*> getEligibleOrigins(
        VehicleKind kind) const;
    std::vector<PointOfInterest*> getEligibleDestinations(
        VehicleKind kind) const;

    static bool canSpawnFrom(
        VehicleKind kind,
        const PointOfInterest& poi);
    static bool canTravelTo(
        VehicleKind kind,
        const PointOfInterest& poi);

    int transitBusCount(int totalVehicleCount) const;
    const DemandWeights& getDemandWeights() const {
        return weights_;
    }

private:
    using Credits =
        std::unordered_map<const PointOfInterest*, double>;

    static std::size_t kindIndex(VehicleKind kind);
    PointOfInterest* selectWeighted(
        const std::vector<PointOfInterest*>& candidates,
        Credits& credits,
        bool originSelection);

    Graph& graph_;
    std::mt19937 random_;
    DemandWeights weights_;
    std::array<double, 4> vehicleCredits_{};
    std::array<Credits, 4> originCredits_;
    std::array<Credits, 4> destinationCredits_;
};

#endif
