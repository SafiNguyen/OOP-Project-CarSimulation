#include "SimulatorFactory.h"

#include <algorithm>
#include <random>

#include "model/Bus.h"
#include "model/Car.h"
#include "model/EmergencyVehicle.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Motorbike.h"
#include "model/Vehicle.h"
#include "simulation/TrafficSimulator.h"

namespace {

constexpr int DEMO_VEHICLE_COUNT = 1000;

// Car, Motorbike, Bus, EmergencyVehicle. Buses are deliberately a small
// share of vehicle units: they carry many people, but are far less numerous
// on the road than private cars and motorbikes.
constexpr double CAR_WEIGHT = 45.0;
constexpr double MOTORBIKE_WEIGHT = 48.0;
constexpr double BUS_WEIGHT = 5.0;
constexpr double EMERGENCY_WEIGHT = 2.0;

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
    std::uniform_int_distribution<size_t> dist(0, intersections.size() - 1);
    std::discrete_distribution<int> vehicleTypeDist({
        CAR_WEIGHT, MOTORBIKE_WEIGHT, BUS_WEIGHT, EMERGENCY_WEIGHT
    });

    for (int i = 0; i < DEMO_VEHICLE_COUNT; ++i) {
        Intersection* start = intersections[dist(rng)];
        Intersection* end = intersections[dist(rng)];
        while (start == end) {
            end = intersections[dist(rng)];
        }
        Vehicle* v = nullptr;
        const int type = vehicleTypeDist(rng);
        if (type == 0) {
            v = new Car(i, 20.0, start, end);
        } else if (type == 1) {
            v = new Motorbike(i, 30.0, start, end);
        } else if (type == 2) {
            v = new Bus(i, 15.0, start, end);
        } else {
            v = new EmergencyVehicle(i, 35.0, start, end);
        }
        simulator->addVehicle(v);
    }
    return simulator;
}
