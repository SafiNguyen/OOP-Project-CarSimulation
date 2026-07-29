#include "VehicleSpawnPolicy.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Graph.h"
#include "PointOfInterest.h"

VehicleSpawnPolicy::VehicleSpawnPolicy(
    Graph& graph,
    std::uint32_t seed,
    DemandWeights weights)
    : graph_(graph),
      random_(seed),
      weights_(weights) {}

std::size_t VehicleSpawnPolicy::kindIndex(
    VehicleKind kind) {
    switch (kind) {
        case VehicleKind::Car:
            return 0u;
        case VehicleKind::Bus:
            return 1u;
        case VehicleKind::Motorbike:
            return 2u;
        case VehicleKind::Emergency:
            return 3u;
    }
    return 0u;
}

bool VehicleSpawnPolicy::canSpawnFrom(
    VehicleKind kind,
    const PointOfInterest& poi) {
    if (!poi.isSpawnPoint() ||
        poi.getConnectedRoad() == nullptr ||
        poi.getSpawnWeight() <= 0.0) {
        return false;
    }

    switch (kind) {
        case VehicleKind::Car:
        case VehicleKind::Motorbike:
            return poi.getType() ==
                       POIType::RESIDENTIAL_AREA ||
                   poi.getType() ==
                       POIType::PARKING_LOT;
        case VehicleKind::Bus:
            return poi.getType() ==
                   POIType::BUS_STATION;
        case VehicleKind::Emergency:
            return poi.getType() ==
                   POIType::HOSPITAL;
    }
    return false;
}

bool VehicleSpawnPolicy::canTravelTo(
    VehicleKind kind,
    const PointOfInterest& poi) {
    if (!poi.isDestination() ||
        poi.getConnectedRoad() == nullptr ||
        poi.getDestinationWeight() <= 0.0) {
        return false;
    }

    switch (kind) {
        case VehicleKind::Car:
        case VehicleKind::Motorbike:
            return poi.getType() !=
                   POIType::BUS_STATION;
        case VehicleKind::Bus:
            return poi.getType() ==
                   POIType::BUS_STATION;
        case VehicleKind::Emergency:
            // A destination POI represents the location of an
            // emergency call until EventManager exposes incident
            // routing as a public API.
            return poi.getType() !=
                   POIType::BUS_STATION;
    }
    return false;
}

std::vector<PointOfInterest*>
VehicleSpawnPolicy::getEligibleOrigins(
    VehicleKind kind) const {
    std::vector<PointOfInterest*> result;
    for (PointOfInterest* poi :
         graph_.getSpawnPoints()) {
        if (poi != nullptr &&
            canSpawnFrom(kind, *poi)) {
            result.push_back(poi);
        }
    }
    std::sort(
        result.begin(),
        result.end(),
        [](const PointOfInterest* lhs,
           const PointOfInterest* rhs) {
            return lhs->getId() < rhs->getId();
        });
    return result;
}

std::vector<PointOfInterest*>
VehicleSpawnPolicy::getEligibleDestinations(
    VehicleKind kind) const {
    std::vector<PointOfInterest*> result;
    for (PointOfInterest* poi :
         graph_.getDestinations()) {
        if (poi != nullptr &&
            canTravelTo(kind, *poi)) {
            result.push_back(poi);
        }
    }
    std::sort(
        result.begin(),
        result.end(),
        [](const PointOfInterest* lhs,
           const PointOfInterest* rhs) {
            return lhs->getId() < rhs->getId();
        });
    return result;
}

PointOfInterest* VehicleSpawnPolicy::selectWeighted(
    const std::vector<PointOfInterest*>& candidates,
    Credits& credits,
    bool originSelection) {
    if (candidates.empty()) {
        return nullptr;
    }

    double totalWeight = 0.0;
    PointOfInterest* selected = nullptr;
    double bestScore =
        -std::numeric_limits<double>::infinity();
    std::uniform_real_distribution<double> jitter(
        -0.025, 0.025);

    for (PointOfInterest* poi : candidates) {
        const double weight =
            originSelection
                ? poi->getSpawnWeight()
                : poi->getDestinationWeight();
        if (weight <= 0.0) {
            continue;
        }
        totalWeight += weight;
        double& credit = credits[poi];
        credit += weight;
        const double score =
            credit + weight * jitter(random_);
        if (selected == nullptr ||
            score > bestScore) {
            selected = poi;
            bestScore = score;
        }
    }

    if (selected != nullptr) {
        credits[selected] -= totalWeight;
    }
    return selected;
}

VehicleKind VehicleSpawnPolicy::selectVehicleKind(
    bool includeGenericBus) {
    const std::array<VehicleKind, 4> kinds = {
        VehicleKind::Car,
        VehicleKind::Bus,
        VehicleKind::Motorbike,
        VehicleKind::Emergency
    };
    const std::array<double, 4> configured = {
        weights_.car,
        includeGenericBus ? weights_.bus : 0.0,
        weights_.motorbike,
        weights_.emergency
    };

    double total = 0.0;
    std::size_t selected = 0u;
    double best =
        -std::numeric_limits<double>::infinity();
    std::uniform_real_distribution<double> jitter(
        -0.025, 0.025);
    for (std::size_t index = 0u;
         index < kinds.size();
         ++index) {
        const double weight = configured[index];
        if (weight <= 0.0) {
            continue;
        }
        total += weight;
        vehicleCredits_[index] += weight;
        const double score =
            vehicleCredits_[index] +
            weight * jitter(random_);
        if (score > best) {
            selected = index;
            best = score;
        }
    }
    vehicleCredits_[selected] -= total;
    return kinds[selected];
}

VehicleSpawnPolicy::Trip
VehicleSpawnPolicy::selectTrip(
    VehicleKind kind) {
    const auto origins =
        getEligibleOrigins(kind);
    const auto allDestinations =
        getEligibleDestinations(kind);
    PointOfInterest* origin =
        selectWeighted(
            origins,
            originCredits_[kindIndex(kind)],
            true);
    if (origin == nullptr) {
        return {};
    }

    std::vector<PointOfInterest*> destinations;
    destinations.reserve(allDestinations.size());
    for (PointOfInterest* candidate :
         allDestinations) {
        if (candidate != origin) {
            destinations.push_back(candidate);
        }
    }
    PointOfInterest* destination =
        selectWeighted(
            destinations,
            destinationCredits_[kindIndex(kind)],
            false);
    return {origin, destination};
}

int VehicleSpawnPolicy::transitBusCount(
    int totalVehicleCount) const {
    const double total =
        weights_.car +
        weights_.motorbike +
        weights_.bus +
        weights_.emergency;
    if (totalVehicleCount <= 0 ||
        total <= 0.0) {
        return 0;
    }
    return static_cast<int>(
        std::lround(
            static_cast<double>(totalVehicleCount) *
            weights_.bus / total));
}
