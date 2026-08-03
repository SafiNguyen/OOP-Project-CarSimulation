#include "SimulatorFactory.h"

#include <algorithm>
#include <iomanip>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "algorithm/DijkstraStrategy.h"
#include "Bus.h"
#include "BusService.h"
#include "BusStop.h"
#include "Graph.h"
#include "Intersection.h"
#include "model/vehicle/VehicleFactory.h"
#include "PointOfInterest.h"
#include "simulation/BusTripPlanner.h"
#include "simulation/TrafficSimulator.h"
#include "simulation/VehicleSpawnPolicy.h"
#include "Vehicle.h"

namespace {
constexpr double FIRST_BUS_DEPARTURE_SECONDS = 5.0;
constexpr double NETWORK_BUS_DEPARTURE_INTERVAL_SECONDS = 4.0;
constexpr double FIRST_GENERAL_VEHICLE_DEPARTURE_SECONDS = 1.0;
constexpr double GENERAL_VEHICLE_DEPARTURE_INTERVAL_SECONDS = 0.30;
constexpr double GENERAL_DEPARTURE_JITTER_SECONDS = 0.06;
constexpr int INITIAL_CIVILIAN_WARM_START_COUNT = 72;
constexpr double FIRST_CIVILIAN_WARM_START_SECONDS = 0.15;
constexpr double CIVILIAN_WARM_START_INTERVAL_SECONDS = 0.08;
constexpr std::size_t MINIMUM_STOPS_PER_BUS = 2u;
constexpr std::size_t MAXIMUM_STOPS_PER_BUS = 4u;

// Small runs retain eager construction for deterministic test fixtures and
// instant startup. Larger demand is streamed through TrafficSimulator's
// per-frame budget so the window continues processing OS events.
constexpr int DEFERRED_DEMAND_THRESHOLD = 2000;
constexpr int MEDIUM_PLAYBACK_DEMAND = 10000;

class DemoDemandState {
public:
    DemoDemandState(Graph& graph, int requestedVehicleCount)
        : graph_(graph),
          spawnPolicy_(
              graph,
              42u,
              VehicleSpawnPolicy::DemandWeights{}),
          random_(42u),
          transitRandomEngine_(20250729u),
          departureJitter_(
              -GENERAL_DEPARTURE_JITTER_SECONDS,
              GENERAL_DEPARTURE_JITTER_SECONDS),
          requestedVehicleCount_(
              std::max(0, requestedVehicleCount)) {
        intersections_ = graph_.getAllIntersections();
        busServices_ = graph_.getAllBusServices();
        std::sort(
            busServices_.begin(),
            busServices_.end(),
            [](const BusService* lhs, const BusService* rhs) {
                return lhs->getId() < rhs->getId();
            });

        for (const Road* road : graph_.getAllRoads()) {
            if (road == nullptr) {
                continue;
            }
            for (const auto& ownedStop : road->getBusStops()) {
                if (ownedStop != nullptr) {
                    busStops_.push_back(ownedStop.get());
                }
            }
        }
        std::sort(
            busStops_.begin(),
            busStops_.end(),
            [](const BusStop* lhs, const BusStop* rhs) {
                return lhs->getId() < rhs->getId();
            });

        if (!busServices_.empty() && busStops_.empty()) {
            throw std::runtime_error(
                "Configured transit services require at least one Bus Stop.");
        }

        if (!busServices_.empty()) {
            transitBusCount_ =
                spawnPolicy_.transitBusCount(
                    requestedVehicleCount_);
            fleetOrdinals_.assign(
                busServices_.size(), 0);
        }
        firstGenericVehicleId_ = transitBusCount_;
        genericVehicleCount_ =
            requestedVehicleCount_ - transitBusCount_;
    }

    std::size_t remaining() const {
        return static_cast<std::size_t>(
            transitBusCount_ - nextBusIndex_ +
            genericVehicleCount_ - nextGenericIndex_);
    }

    void produceNext(TrafficSimulator& simulator) {
        if (nextBusIndex_ < transitBusCount_) {
            produceTransitBus(simulator);
            return;
        }
        if (nextGenericIndex_ < genericVehicleCount_) {
            produceGenericVehicle(simulator);
            return;
        }
        throw std::runtime_error(
            "Deferred demand producer was called after completion.");
    }

private:
    void produceTransitBus(TrafficSimulator& simulator) {
        const int busIndex = nextBusIndex_;
        const std::size_t serviceIndex =
            static_cast<std::size_t>(busIndex) %
            busServices_.size();
        const BusService* service =
            busServices_[serviceIndex];
        const int ordinal =
            ++fleetOrdinals_[serviceIndex];
        std::ostringstream fleetCode;
        fleetCode << service->getCode()
                  << '-' << std::setw(2)
                  << std::setfill('0') << ordinal;
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
                    graph_,
                    transitTripStrategy_,
                    *service,
                    busStops_,
                    transitRandomEngine_,
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
                    stopIds.push_back(stop->getId());
                }
            }
            const bool uniquePlan =
                assignedStopPlans_.insert(stopIds).second;
            tripPlan = std::move(candidate);
            if (uniquePlan) {
                break;
            }
        }
        if (!tripPlan.has_value()) {
            throw std::runtime_error(
                "Failed to build a connected Bus trip plan for " +
                fleetCode.str() + ".");
        }

        auto* bus = new Bus(
            busIndex,
            fleetCode.str(),
            *service,
            std::move(*tripPlan),
            15.0);
        if (!simulator.addVehicleWithFixedRoute(
                bus,
                bus->getCurrentRoute())) {
            throw std::runtime_error(
                "Failed to queue configured transit Bus " +
                fleetCode.str() + ".");
        }
        ++nextBusIndex_;
    }

    void produceGenericVehicle(
        TrafficSimulator& simulator) {
        const int genericIndex = nextGenericIndex_;
        const int vehicleId =
            firstGenericVehicleId_ + genericIndex;
        PointOfInterest* startPOI = nullptr;
        PointOfInterest* endPOI = nullptr;
        Intersection* startIntersection = nullptr;
        Intersection* endIntersection = nullptr;
        const VehicleKind kind =
            spawnPolicy_.selectVehicleKind(
                busServices_.empty());
        const VehicleSpawnPolicy::Trip trip =
            spawnPolicy_.selectTrip(kind);
        startPOI = trip.origin;
        endPOI = trip.destination;

        if (startPOI == nullptr) {
            startIntersection = randomIntersection();
        }
        if (endPOI == nullptr) {
            endIntersection = randomIntersection();
        }
        while (startIntersection != nullptr &&
               startIntersection == endIntersection) {
            endIntersection = randomIntersection();
        }

        double speed = 20.0;
        if (kind == VehicleKind::Motorbike) {
            speed = 30.0;
        } else if (kind == VehicleKind::Bus) {
            speed = 15.0;
        } else if (kind == VehicleKind::Emergency) {
            speed = 35.0;
        }
        Vehicle* vehicle =
            VehicleFactory::createVehicle(
                kind,
                vehicleId,
                speed,
                startIntersection,
                endIntersection);
        if (startPOI != nullptr) {
            vehicle->setSpawnPOI(startPOI);
        }
        if (endPOI != nullptr) {
            vehicle->setTargetPOI(endPOI);
        }

        const bool isCivilian =
            kind == VehicleKind::Car ||
            kind == VehicleKind::Motorbike;
        const double jitter = departureJitter_(random_);
        double spawnTime =
            FIRST_GENERAL_VEHICLE_DEPARTURE_SECONDS;
        if (isCivilian) {
            const int civilianIndex =
                civilianScheduleIndex_++;
            spawnTime =
                FIRST_CIVILIAN_WARM_START_SECONDS;
            if (civilianIndex <
                INITIAL_CIVILIAN_WARM_START_COUNT) {
                spawnTime +=
                    static_cast<double>(civilianIndex) *
                    CIVILIAN_WARM_START_INTERVAL_SECONDS;
            } else {
                spawnTime +=
                    static_cast<double>(
                        INITIAL_CIVILIAN_WARM_START_COUNT) *
                        CIVILIAN_WARM_START_INTERVAL_SECONDS +
                    static_cast<double>(
                        civilianIndex -
                        INITIAL_CIVILIAN_WARM_START_COUNT) *
                        GENERAL_VEHICLE_DEPARTURE_INTERVAL_SECONDS;
            }
            spawnTime = std::max(
                FIRST_CIVILIAN_WARM_START_SECONDS,
                spawnTime + jitter);
        } else {
            spawnTime = std::max(
                FIRST_GENERAL_VEHICLE_DEPARTURE_SECONDS,
                FIRST_GENERAL_VEHICLE_DEPARTURE_SECONDS +
                    static_cast<double>(genericIndex) *
                        GENERAL_VEHICLE_DEPARTURE_INTERVAL_SECONDS +
                    jitter);
        }

        if (startPOI != nullptr) {
            double& nextDeparture =
                nextDepartureByOrigin_[startPOI];
            spawnTime =
                std::max(spawnTime, nextDeparture);
            nextDeparture =
                spawnTime +
                startPOI->getSpawnCooldownSeconds();
        }
        const double delayFromNow =
            std::max(
                0.0,
                spawnTime - simulator.getElapsedTime());
        if (!simulator.scheduleVehicleSpawn(
                vehicle,
                delayFromNow)) {
            throw std::runtime_error(
                "Failed to schedule demo vehicle " +
                std::to_string(vehicleId) + ".");
        }
        ++nextGenericIndex_;
    }

    Intersection* randomIntersection() {
        std::uniform_int_distribution<std::size_t> distribution(
            0u,
            intersections_.size() - 1u);
        return intersections_[distribution(random_)];
    }

    Graph& graph_;
    VehicleSpawnPolicy spawnPolicy_;
    std::mt19937 random_;
    std::mt19937 transitRandomEngine_;
    std::uniform_real_distribution<double>
        departureJitter_;
    DijkstraStrategy transitTripStrategy_;
    int requestedVehicleCount_ = 0;
    int transitBusCount_ = 0;
    int genericVehicleCount_ = 0;
    int firstGenericVehicleId_ = 0;
    int nextBusIndex_ = 0;
    int nextGenericIndex_ = 0;
    int civilianScheduleIndex_ = 0;
    std::vector<Intersection*> intersections_;
    std::vector<BusService*> busServices_;
    std::vector<const BusStop*> busStops_;
    std::vector<int> fleetOrdinals_;
    std::set<std::vector<int>> assignedStopPlans_;
    std::unordered_map<const PointOfInterest*, double>
        nextDepartureByOrigin_;
};

} // namespace

std::unique_ptr<TrafficSimulator> createDemoSimulator(
    Graph& graph,
    PathFindingStrategy* strategy,
    int vehicleCount) {
    auto simulator =
        std::make_unique<TrafficSimulator>(
            &graph, strategy);
    const int requestedVehicleCount =
        std::max(0, vehicleCount);
    if (requestedVehicleCount > 0) {
        simulator->reserveVehicleIdsThrough(
            requestedVehicleCount - 1);
    }
    if (requestedVehicleCount == 0 ||
        graph.getAllIntersections().size() < 2u) {
        return simulator;
    }

    auto demand = std::make_shared<DemoDemandState>(
        graph,
        requestedVehicleCount);
    if (requestedVehicleCount <=
        DEFERRED_DEMAND_THRESHOLD) {
        while (demand->remaining() > 0u) {
            demand->produceNext(*simulator);
        }
        return simulator;
    }

    // Do not duplicate the large pending queue while demand is still being
    // materialized. Playback is enabled automatically afterwards with a
    // load-aware interval: ten seconds for 10k trips and thirty seconds for
    // larger stress runs.
    simulator->setSnapshotInterval(0.0);
    simulator->setSnapshotIntervalAfterDeferredDemand(
        requestedVehicleCount <= MEDIUM_PLAYBACK_DEMAND
            ? 10.0
            : 30.0);
    const std::size_t remaining = demand->remaining();
    simulator->setDeferredDemand(
        remaining,
        [demand](TrafficSimulator& target) {
            demand->produceNext(target);
        });
    return simulator;
}
