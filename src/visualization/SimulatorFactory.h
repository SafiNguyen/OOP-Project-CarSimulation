#ifndef SIMULATOR_FACTORY_H
#define SIMULATOR_FACTORY_H

#include <memory>

class Graph;
class PathFindingStrategy;
class TrafficSimulator;

// Builds a fresh TrafficSimulator over `graph` using `strategy`, then attempts
// to seed it with 1000 demo vehicles (same RNG seed every call, so resets are
// reproducible) on random start/end intersection pairs.
std::unique_ptr<TrafficSimulator> createDemoSimulator(Graph& graph, PathFindingStrategy* strategy);

#endif
