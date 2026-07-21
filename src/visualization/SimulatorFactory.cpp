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

std::unique_ptr<TrafficSimulator> createDemoSimulator(Graph& graph, PathFindingStrategy* strategy) {
    auto simulator = std::make_unique<TrafficSimulator>(&graph, strategy);

    auto intersections = graph.getAllIntersections();
    if (intersections.size() < 2) {
        return simulator;
    }

    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> dist(0, intersections.size() - 1);
    for (int i = 0; i < 1000; ++i) {
        Intersection* start = intersections[dist(rng)];
        Intersection* end = intersections[dist(rng)];
        while (start == end) {
            end = intersections[dist(rng)];
        }
        Vehicle* v = nullptr;
        int type = rng() % 4;
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
