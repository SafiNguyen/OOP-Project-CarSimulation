#include "SimulatorFactory.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "algorithm/DijkstraStrategy.h"
#include "Graph.h"
#include "Intersection.h"
#include "Vehicle.h"
#include "model/vehicle/VehicleFactory.h"
#include "Bus.h"
#include "BusService.h"
#include "BusStop.h"
#include "Crosswalk.h"
#include "Pedestrian.h"
#include "PedestrianRoute.h"
#include "simulation/BusTripPlanner.h"
#include "simulation/TrafficSimulator.h"
#include "simulation/VehicleSpawnPolicy.h"

namespace {

constexpr int DEMO_VEHICLE_COUNT = 1000;

constexpr double FIRST_BUS_DEPARTURE_SECONDS = 5.0;
constexpr double NETWORK_BUS_DEPARTURE_INTERVAL_SECONDS = 4.0;
constexpr double FIRST_GENERAL_VEHICLE_DEPARTURE_SECONDS = 1.0;
constexpr double GENERAL_VEHICLE_DEPARTURE_INTERVAL_SECONDS = 0.18;
constexpr double GENERAL_DEPARTURE_JITTER_SECONDS = 0.06;
constexpr int INITIAL_CIVILIAN_WARM_START_COUNT = 144;
constexpr double FIRST_CIVILIAN_WARM_START_SECONDS = 0.05;
constexpr double CIVILIAN_WARM_START_INTERVAL_SECONDS = 0.035;
constexpr std::size_t MINIMUM_STOPS_PER_BUS = 2u;
constexpr std::size_t MAXIMUM_STOPS_PER_BUS = 4u;

std::size_t recommendedActiveVehicleLimit(
    const Graph& graph) {
    std::size_t directionalLaneCount = 0;
    for (const Road* road : graph.getAllRoads()) {
        if (road != nullptr) {
            directionalLaneCount +=
                static_cast<std::size_t>(
                    std::max(1, road->getLaneCount()));
        }
    }

    // The demo still owns all 1000 requested trips, but releases only a
    // readable amount of simultaneous traffic for the map's lane count.
    // Permit a denser steady state while keeping the cap proportional to the
    // amount of road space available on each map.
    return std::clamp<std::size_t>(
        directionalLaneCount * 3u,
        36u,
        400u);
}

} // namespace

std::unique_ptr<TrafficSimulator> createDemoSimulator(Graph& graph, PathFindingStrategy* strategy) {
    auto simulator = std::make_unique<TrafficSimulator>(&graph, strategy);
    simulator->setMaximumActiveVehicles(
        recommendedActiveVehicleLimit(graph));

    auto intersections = graph.getAllIntersections();
    if (intersections.size() < 2) {
        return simulator;
    }

    std::mt19937 rng(42);
    VehicleSpawnPolicy spawnPolicy(graph, 42u);
    std::uniform_int_distribution<std::size_t> intersectionDistribution(
        0,
        intersections.size() - 1);

    auto busServices = graph.getAllBusServices();
    std::sort(
        busServices.begin(),
        busServices.end(),
        [](const BusService* lhs, const BusService* rhs) {
            return lhs->getId() < rhs->getId();
        });
    std::vector<const BusStop*> busStops;
    for (const Road* road : graph.getAllRoads()) {
        if (road == nullptr) {
            continue;
        }
        for (const auto& ownedStop :
             road->getBusStops()) {
            if (ownedStop != nullptr) {
                busStops.push_back(
                    ownedStop.get());
            }
        }
    }
    std::sort(
        busStops.begin(),
        busStops.end(),
        [](const BusStop* lhs, const BusStop* rhs) {
            return lhs->getId() < rhs->getId();
        });

    int firstGenericVehicleId = 0;
    int genericVehicleCount = DEMO_VEHICLE_COUNT;
    if (!busServices.empty()) {
        if (busStops.empty()) {
            throw std::runtime_error(
                "Configured transit services require "
                "at least one Bus Stop.");
        }
        const int transitBusCount =
            spawnPolicy.transitBusCount(
                DEMO_VEHICLE_COUNT);
        std::vector<int> fleetOrdinals(
            busServices.size(), 0);
        std::set<std::vector<int>>
            assignedStopPlans;
        std::mt19937 transitRandomEngine(20250729u);
        DijkstraStrategy transitTripStrategy;
        for (int busIndex = 0;
             busIndex < transitBusCount;
             ++busIndex) {
            const std::size_t serviceIndex =
                static_cast<std::size_t>(busIndex) %
                busServices.size();
            const BusService* service =
                busServices[serviceIndex];
            const int ordinal =
                ++fleetOrdinals[serviceIndex];
            std::ostringstream fleetCode;
            fleetCode
                << service->getCode()
                << '-'
                << std::setw(2)
                << std::setfill('0')
                << ordinal;
            const double scheduledDepartureTime =
                FIRST_BUS_DEPARTURE_SECONDS +
                static_cast<double>(busIndex) *
                    NETWORK_BUS_DEPARTURE_INTERVAL_SECONDS;
            std::optional<BusTripPlan> tripPlan;
            constexpr int MAX_PLAN_ATTEMPTS = 16;
            for (int attempt = 0;
                 attempt < MAX_PLAN_ATTEMPTS;
                 ++attempt) {
                auto candidate =
                    BusTripPlanner::buildRandomPlan(
                        graph,
                        transitTripStrategy,
                        *service,
                        busStops,
                        transitRandomEngine,
                        MINIMUM_STOPS_PER_BUS,
                        MAXIMUM_STOPS_PER_BUS,
                        scheduledDepartureTime);
                if (!candidate.has_value()) {
                    continue;
                }
                std::vector<int> stopIds;
                for (const BusStop* stop :
                     candidate->orderedStops) {
                    if (stop != nullptr) {
                        stopIds.push_back(
                            stop->getId());
                    }
                }
                const bool uniquePlan =
                    assignedStopPlans.insert(
                        stopIds).second;
                tripPlan =
                    std::move(candidate);
                if (uniquePlan) {
                    break;
                }
            }
            if (!tripPlan.has_value()) {
                throw std::runtime_error(
                    "Failed to build a connected Bus trip "
                    "plan for " +
                    fleetCode.str() + ".");
            }

            auto* bus = new Bus(
                busIndex,
                fleetCode.str(),
                *service,
                std::move(*tripPlan),
                15.0);
            if (!simulator->addVehicleWithFixedRoute(
                    bus,
                    bus->getCurrentRoute())) {
                throw std::runtime_error(
                    "Failed to queue configured transit Bus " +
                    fleetCode.str() + ".");
            }
        }
        firstGenericVehicleId = transitBusCount;
        genericVehicleCount =
            DEMO_VEHICLE_COUNT - transitBusCount;
    }

    std::unordered_map<const PointOfInterest*, double>
        nextDepartureByOrigin;
    std::uniform_real_distribution<double>
        departureJitter(
            -GENERAL_DEPARTURE_JITTER_SECONDS,
            GENERAL_DEPARTURE_JITTER_SECONDS);
    int civilianScheduleIndex = 0;

    for (int genericIndex = 0;
         genericIndex < genericVehicleCount;
         ++genericIndex) {
        const int i =
            firstGenericVehicleId + genericIndex;
        PointOfInterest* startPOI = nullptr;
        PointOfInterest* endPOI = nullptr;
        Intersection* startIntersection = nullptr;
        Intersection* endIntersection = nullptr;
        const VehicleKind kind =
            spawnPolicy.selectVehicleKind(
                busServices.empty());
        const VehicleSpawnPolicy::Trip trip =
            spawnPolicy.selectTrip(kind);
        startPOI = trip.origin;
        endPOI = trip.destination;

        if (startPOI == nullptr) {
            startIntersection =
                intersections[intersectionDistribution(rng)];
        }
        if (endPOI == nullptr) {
            endIntersection =
                intersections[intersectionDistribution(rng)];
        }
        if (startIntersection != nullptr &&
            endIntersection != nullptr) {
            while (startIntersection ==
                   endIntersection) {
                endIntersection =
                    intersections[intersectionDistribution(rng)];
            }
        }

        Vehicle* v = nullptr;
        double speed = 20.0;
        if (kind == VehicleKind::Motorbike) {
            speed = 30.0;
        } else if (kind == VehicleKind::Bus) {
            speed = 15.0;
        } else if (kind == VehicleKind::Emergency) {
            speed = 35.0;
        }
        v = VehicleFactory::createVehicle(
            kind, i, speed, startIntersection, endIntersection);
        if (startPOI != nullptr) {
            v->setSpawnPOI(startPOI);
        }
        if (endPOI != nullptr) {
            v->setTargetPOI(endPOI);
        }

        const bool isCivilian =
            kind == VehicleKind::Car ||
            kind == VehicleKind::Motorbike;
        const double jitter = departureJitter(rng);
        if (isCivilian) {
            // Keep the post-warm-start schedule continuous. Basing it on the
            // generic vehicle index would otherwise create a long empty gap
            // after the initial cohort.
            const int civilianIndex =
                civilianScheduleIndex;
            double spawnDelay =
                FIRST_CIVILIAN_WARM_START_SECONDS;
            if (civilianIndex <
                    INITIAL_CIVILIAN_WARM_START_COUNT) {
                spawnDelay +=
                    static_cast<double>(civilianIndex) *
                    CIVILIAN_WARM_START_INTERVAL_SECONDS;
            } else {
                spawnDelay +=
                    static_cast<double>(
                        INITIAL_CIVILIAN_WARM_START_COUNT) *
                    CIVILIAN_WARM_START_INTERVAL_SECONDS +
                    static_cast<double>(
                        civilianIndex -
                        INITIAL_CIVILIAN_WARM_START_COUNT) *
                    GENERAL_VEHICLE_DEPARTURE_INTERVAL_SECONDS;
            }
            spawnDelay += jitter;
            ++civilianScheduleIndex;
            spawnDelay =
                std::max(
                    FIRST_CIVILIAN_WARM_START_SECONDS,
                    spawnDelay);
            if (startPOI != nullptr) {
                double& nextDeparture =
                    nextDepartureByOrigin[startPOI];
                spawnDelay =
                    std::max(
                        spawnDelay,
                        nextDeparture);
                nextDeparture =
                    spawnDelay +
                    startPOI->getSpawnCooldownSeconds();
            }
            if (!simulator->scheduleVehicleSpawn(
                    v, spawnDelay)) {
                throw std::runtime_error(
                    "Failed to schedule demo vehicle " +
                    std::to_string(i) + ".");
            }
            continue;
        }

        double spawnDelay =
            std::max(
                FIRST_GENERAL_VEHICLE_DEPARTURE_SECONDS,
                FIRST_GENERAL_VEHICLE_DEPARTURE_SECONDS +
                    static_cast<double>(genericIndex) *
                        GENERAL_VEHICLE_DEPARTURE_INTERVAL_SECONDS +
                    jitter);
        if (startPOI != nullptr) {
            double& nextDeparture =
                nextDepartureByOrigin[startPOI];
            spawnDelay =
                std::max(
                    spawnDelay,
                    nextDeparture);
            nextDeparture =
                spawnDelay +
                startPOI->getSpawnCooldownSeconds();
        }
        if (!simulator->scheduleVehicleSpawn(
                v, spawnDelay)) {
            throw std::runtime_error(
                "Failed to schedule demo vehicle " +
                std::to_string(i) + ".");
        }
    }

    int pedestrianId = 100000;
    for (Crosswalk* crosswalk :
         graph.getAllCrosswalks()) {
        if (crosswalk == nullptr) continue;
        constexpr int PEDESTRIANS_PER_CROSSWALK = 2;
        for (int index = 0;
             index < PEDESTRIANS_PER_CROSSWALK;
             ++index) {
            const CrossingDirection direction =
                index % 2 == 0
                    ? CrossingDirection::SideAToB
                    : CrossingDirection::SideBToA;
            const double approachDistance =
                10.0 +
                static_cast<double>(index) * 8.0;
            const double departureDistance =
                30.0 +
                static_cast<double>(index % 4) * 7.0;
            auto route = buildCrosswalkJourney(
                *crosswalk,
                direction,
                approachDistance,
                departureDistance);
            if (route.empty()) continue;
            const double speed =
                1.25 +
                static_cast<double>(index % 5) * 0.05;
            simulator->addPedestrian(
                std::make_unique<Pedestrian>(
                    pedestrianId++,
                    speed,
                    std::move(route)));
        }
    }
    return simulator;
}
