#include "SimulatorFactory.h"

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

// Car, Motorbike, Bus, EmergencyVehicle. Keep ordinary traffic close to the
// previous even split while making emergency vehicles uncommon.
constexpr double CAR_WEIGHT = 32.7;
constexpr double MOTORBIKE_WEIGHT = 32.7;
constexpr double BUS_WEIGHT = 32.6;
constexpr double EMERGENCY_WEIGHT = 2.0;

} // namespace

std::unique_ptr<TrafficSimulator> createDemoSimulator(Graph& graph, PathFindingStrategy* strategy) {
    auto simulator = std::make_unique<TrafficSimulator>(&graph, strategy);

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
