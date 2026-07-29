#include "SimulatorFactory.h"

#include <algorithm>
#include <iostream>
#include <random>

#include "model/Bus.h"
#include "model/Car.h"
#include "model/EmergencyVehicle.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Motorbike.h"
#include "model/Vehicle.h"
#include "model/Crosswalk.h"
#include "model/Pedestrian.h"
#include "model/PedestrianRoute.h"
#include "simulation/TrafficSimulator.h"

namespace {

constexpr int DEMO_VEHICLE_COUNT = 1000;

// Car, Motorbike, Bus, EmergencyVehicle. Buses are deliberately a small
// share of vehicle units: they carry many people, but are far less numerous
// on the road than private cars and motorbikes.
// Keep the weights at a total of 100 so each value is also the exact
// percentage used when selecting a demo vehicle type.
constexpr double CAR_WEIGHT = 39.0;
constexpr double MOTORBIKE_WEIGHT = 41.0;
constexpr double BUS_WEIGHT = 12.0;
constexpr double EMERGENCY_WEIGHT = 8.0;

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
    // Roughly two active vehicles per directional lane keeps traffic visible
    // without turning every road into a permanent jam at startup.
    return std::clamp<std::size_t>(
        directionalLaneCount * 2u,
        24u,
        320u);
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
    const auto& pois = graph.getAllPOIs();
    const bool usePois = pois.size() >= 2;
    std::uniform_int_distribution<std::size_t> poiDistribution(
        0,
        usePois ? pois.size() - 1 : 0);
    std::uniform_int_distribution<std::size_t> intersectionDistribution(
        0,
        intersections.size() - 1);
    std::discrete_distribution<int> vehicleTypeDist({
        CAR_WEIGHT, MOTORBIKE_WEIGHT, BUS_WEIGHT, EMERGENCY_WEIGHT
    });

    for (int i = 0; i < DEMO_VEHICLE_COUNT; ++i) {
        PointOfInterest* startPOI = nullptr;
        PointOfInterest* endPOI = nullptr;
        Intersection* startIntersection = nullptr;
        Intersection* endIntersection = nullptr;
        if (usePois) {
            startPOI = pois[poiDistribution(rng)];
            endPOI = pois[poiDistribution(rng)];
            while (startPOI == endPOI) {
                endPOI = pois[poiDistribution(rng)];
            }
        } else {
            startIntersection =
                intersections[intersectionDistribution(rng)];
            endIntersection =
                intersections[intersectionDistribution(rng)];
            while (startIntersection == endIntersection) {
                endIntersection =
                    intersections[intersectionDistribution(rng)];
            }
        }

        Vehicle* v = nullptr;
        const int type = vehicleTypeDist(rng);
        if (type == 0) {
            v = new Car(
                i, 20.0, startIntersection, endIntersection);
        } else if (type == 1) {
            v = new Motorbike(
                i, 30.0, startIntersection, endIntersection);
        } else if (type == 2) {
            v = new Bus(
                i, 15.0, startIntersection, endIntersection);
        } else {
            v = new EmergencyVehicle(
                i, 35.0, startIntersection, endIntersection);
        }
        if (usePois) {
            v->setSpawnPOI(startPOI);
            v->setTargetPOI(endPOI);
        }
        simulator->addVehicle(v);
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
