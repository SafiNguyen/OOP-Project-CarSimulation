#ifndef SIMULATOR_FACTORY_H
#define SIMULATOR_FACTORY_H

#include <memory>

class Graph;
class PathFindingStrategy;
class TrafficSimulator;

inline constexpr int DEFAULT_DEMO_VEHICLE_COUNT = 1000;

// Builds a fresh TrafficSimulator over `graph` using `strategy`, then attempts
// to seed it with the requested number of demo vehicles. The same RNG seed is
// used every call, so resets with the same count remain reproducible.
std::unique_ptr<TrafficSimulator> createDemoSimulator(
    Graph& graph,
    PathFindingStrategy* strategy,
    int vehicleCount = DEFAULT_DEMO_VEHICLE_COUNT);

#endif
