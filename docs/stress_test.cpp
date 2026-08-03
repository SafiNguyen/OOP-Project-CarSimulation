// Stress test for TV2 (Data Analyst) - Sunday deliverable
// Spawns 100-1000 vehicles, compares BFS / Dijkstra / A* under normal
// traffic and heavy congestion, and measures simulation throughput (FPS proxy).
// Builds independently of SFML - pure logic benchmark.

#include <iostream>
#include <fstream>
#include <random>
#include <chrono>
#include <vector>
#include <memory>

#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "model/Car.h"
#include "algorithm/BFSStrategy.h"
#include "algorithm/DijkstraStrategy.h"
#include "algorithm/AStarStrategy.h"
#include "simulation/StatisticsManager.h"
#include "simulation/TrafficSimulator.h"

using Clock = std::chrono::steady_clock;

// ---------------------------------------------------------------------
// Graph builders
// ---------------------------------------------------------------------

// Legacy project map (5 intersections, 6 roads), embedded here so this
// test has no JSON dependency.
static void buildRealMapGraph(Graph& graph) {
    graph.clearGraph();
    Intersection* n1 = new Intersection(1, 0.0, 0.0);
    Intersection* n2 = new Intersection(2, 100.0, 0.0);
    Intersection* n3 = new Intersection(3, 100.0, 100.0);
    Intersection* n4 = new Intersection(4, 0.0, 100.0);
    Intersection* n5 = new Intersection(5, 50.0, 150.0);
    graph.addIntersection(n1);
    graph.addIntersection(n2);
    graph.addIntersection(n3);
    graph.addIntersection(n4);
    graph.addIntersection(n5);

    graph.addRoad(new Road(101, n1, n2, 100.0, 50.0, 1.0));
    graph.addRoad(new Road(102, n2, n3, 100.0, 50.0, 1.2));
    graph.addRoad(new Road(103, n3, n4, 100.0, 50.0, 1.0));
    graph.addRoad(new Road(104, n4, n1, 100.0, 50.0, 1.0));
    graph.addRoad(new Road(105, n2, n5, 111.8, 40.0, 1.5));
    Road* r106 = new Road(106, n5, n3, 111.8, 40.0, 1.0);
    graph.addRoad(r106);
    r106->blockRoad();
}

// A synthetic grid map used purely for stress testing at scale
// (the legacy project map only has 5 nodes, too small to meaningfully
// stress 100-1000 vehicles). Two directed roads are added per edge
// (one each direction) so vehicles can route both ways, matching the
// directed-graph model used by Graph/Road.
static void buildGridGraph(Graph& graph, int rows, int cols, double cellSize, double speedLimit) {
    graph.clearGraph();
    auto idOf = [cols](int r, int c) { return r * cols + c + 1; };

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            graph.addIntersection(new Intersection(idOf(r, c), c * cellSize, r * cellSize));
        }
    }

    int roadId = 1;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            Intersection* here = graph.getIntersection(idOf(r, c));
            if (c + 1 < cols) {
                Intersection* right = graph.getIntersection(idOf(r, c + 1));
                graph.addRoad(new Road(roadId++, here, right, cellSize, speedLimit, 1.0));
                graph.addRoad(new Road(roadId++, right, here, cellSize, speedLimit, 1.0));
            }
            if (r + 1 < rows) {
                Intersection* down = graph.getIntersection(idOf(r + 1, c));
                graph.addRoad(new Road(roadId++, here, down, cellSize, speedLimit, 1.0));
                graph.addRoad(new Road(roadId++, down, here, cellSize, speedLimit, 1.0));
            }
        }
    }
}

// Reset every road to free-flow (congestionLevel = 1.0, not blocked).
static void applyNormalTraffic(Graph& graph, std::mt19937& /*rng*/) {
    for (Road* road : graph.getAllRoads()) {
        graph.updateRoadCondition(road->getId(), 1.0, false);
    }
}

// Simulate heavy traffic: ~35% of roads get congestion 3x-6x, and ~5%
// of roads are blocked outright (accidents / closures).
static void applyHeavyTraffic(Graph& graph, std::mt19937& rng) {
    std::uniform_real_distribution<double> pickRoll(0.0, 1.0);
    std::uniform_real_distribution<double> congestionRoll(3.0, 6.0);
    for (Road* road : graph.getAllRoads()) {
        double roll = pickRoll(rng);
        if (roll < 0.05) {
            graph.updateRoadCondition(road->getId(), 1.0, true);
        } else if (roll < 0.40) {
            graph.updateRoadCondition(road->getId(), congestionRoll(rng), false);
        } else {
            graph.updateRoadCondition(road->getId(), 1.0, false);
        }
    }
}

struct ODPair { int start; int goal; };

static std::vector<ODPair> generateODPairs(const Graph& graph, int count, std::mt19937& rng) {
    auto nodes = graph.getAllIntersections();
    std::uniform_int_distribution<size_t> dist(0, nodes.size() - 1);
    std::vector<ODPair> pairs;
    pairs.reserve(count);
    for (int i = 0; i < count; ++i) {
        int startId = nodes[dist(rng)]->getId();
        int goalId = nodes[dist(rng)]->getId();
        int guard = 0;
        while (goalId == startId && guard++ < 10) {
            goalId = nodes[dist(rng)]->getId();
        }
        pairs.push_back({startId, goalId});
    }
    return pairs;
}

struct BenchRow {
    std::string scenario;      // "Grid-Normal" / "Grid-Heavy" / "RealMap-Normal" / "RealMap-Heavy"
    std::string algorithm;
    int vehicleCount;
    int pathsFound;
    int pathsNotFound;
    double avgTimeMs;
    double avgNodesExplored;
    double avgPathCost;
    double totalTimeMs;
};

static void runBenchmark(const std::string& scenarioLabel,
                          Graph& graph,
                          const std::vector<ODPair>& pairs,
                          std::vector<BenchRow>& rows) {
    BFSStrategy bfs;
    DijkstraStrategy dijkstra;
    AStarStrategy astar;
    std::vector<PathFindingStrategy*> strategies = {&bfs, &dijkstra, &astar};

    for (PathFindingStrategy* strategy : strategies) {
        StatisticsManager stats;
        const auto t0 = Clock::now();
        for (const auto& od : pairs) {
            stats.measurePathfinding(*strategy, graph, od.start, od.goal);
        }
        const auto t1 = Clock::now();
        const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

        const AlgorithmMetric* metric = stats.getAlgorithmMetric(strategy->name());
        BenchRow row;
        row.scenario = scenarioLabel;
        row.algorithm = strategy->name();
        row.vehicleCount = static_cast<int>(pairs.size());
        row.pathsFound = metric ? metric->pathsFound : 0;
        row.pathsNotFound = metric ? metric->pathsNotFound : 0;
        row.avgTimeMs = metric ? metric->averageComputeTimeMs() : 0.0;
        row.avgNodesExplored = metric ? metric->averageNodesExplored() : 0.0;
        row.avgPathCost = metric ? metric->averagePathCost() : 0.0;
        row.totalTimeMs = totalMs;
        rows.push_back(row);
    }
}

// ---------------------------------------------------------------------
// Simulation throughput ("logic FPS") test: how many simulation ticks
// per second the engine can process (vehicle update + stats + events),
// WITHOUT any rendering. This is a proxy for how heavy the simulation
// core is, independent from SFML draw calls.
// ---------------------------------------------------------------------
struct ThroughputRow {
    int vehicleCount;
    double avgTickTimeMs;
    double logicFps;
};

static ThroughputRow measureThroughput(Graph& graph, int vehicleCount, int ticks, std::mt19937& rng) {
    AStarStrategy astar;
    TrafficSimulator simulator(&graph, &astar);

    auto nodes = graph.getAllIntersections();
    std::uniform_int_distribution<size_t> dist(0, nodes.size() - 1);

    int spawned = 0;
    int attempts = 0;
    while (spawned < vehicleCount && attempts < vehicleCount * 5) {
        ++attempts;
        Intersection* start = nodes[dist(rng)];
        Intersection* goal = nodes[dist(rng)];
        if (start == goal) continue;
        Car* car = new Car(spawned, 40.0, start, goal);
        if (simulator.addVehicle(car)) {
            ++spawned;
        }
    }

    const double dt = 1.0 / 60.0;
    const auto t0 = Clock::now();
    for (int i = 0; i < ticks; ++i) {
        simulator.update(dt);
    }
    const auto t1 = Clock::now();
    const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    ThroughputRow row;
    row.vehicleCount = spawned;
    row.avgTickTimeMs = totalMs / ticks;
    row.logicFps = row.avgTickTimeMs > 0.0 ? 1000.0 / row.avgTickTimeMs : 0.0;
    return row;
}

int main() {
    std::mt19937 rng(2026);

    std::vector<BenchRow> allRows;
    std::vector<int> vehicleCounts = {100, 500, 1000};

    // --- Legacy project map (5 intersections / 6 roads) ---
    {
        Graph realGraph;
        buildRealMapGraph(realGraph);
        for (int n : vehicleCounts) {
            auto pairs = generateODPairs(realGraph, n, rng);

            applyNormalTraffic(realGraph, rng);
            runBenchmark("RealMap-Normal", realGraph, pairs, allRows);

            applyHeavyTraffic(realGraph, rng);
            runBenchmark("RealMap-Heavy", realGraph, pairs, allRows);
        }
    }

    // --- Synthetic 20x20 grid (400 intersections) for meaningful scale ---
    {
        Graph gridGraph;
        buildGridGraph(gridGraph, 20, 20, 80.0, 50.0);
        for (int n : vehicleCounts) {
            auto pairs = generateODPairs(gridGraph, n, rng);

            applyNormalTraffic(gridGraph, rng);
            runBenchmark("Grid20x20-Normal", gridGraph, pairs, allRows);

            applyHeavyTraffic(gridGraph, rng);
            runBenchmark("Grid20x20-Heavy", gridGraph, pairs, allRows);
        }
    }

    // --- Write pathfinding benchmark CSV ---
    {
        std::ofstream csv("bench_results.csv");
        csv << "scenario,algorithm,vehicleCount,pathsFound,pathsNotFound,avgTimeMs,avgNodesExplored,avgPathCost,totalTimeMs\n";
        for (const auto& row : allRows) {
            csv << row.scenario << "," << row.algorithm << "," << row.vehicleCount << ","
                << row.pathsFound << "," << row.pathsNotFound << "," << row.avgTimeMs << ","
                << row.avgNodesExplored << "," << row.avgPathCost << "," << row.totalTimeMs << "\n";
        }
    }

    // --- Simulation throughput ("logic FPS") on the grid map ---
    std::vector<ThroughputRow> throughputRows;
    {
        Graph gridGraph;
        buildGridGraph(gridGraph, 20, 20, 80.0, 50.0);
        applyNormalTraffic(gridGraph, rng);
        for (int n : vehicleCounts) {
            throughputRows.push_back(measureThroughput(gridGraph, n, 300, rng));
        }

        std::ofstream csv("throughput_results.csv");
        csv << "vehicleCount,avgTickTimeMs,logicFps\n";
        for (const auto& row : throughputRows) {
            csv << row.vehicleCount << "," << row.avgTickTimeMs << "," << row.logicFps << "\n";
        }
    }

    // --- Console summary ---
    std::cout << "=== Pathfinding Benchmark (BFS vs Dijkstra vs A*) ===\n";
    for (const auto& row : allRows) {
        std::cout << row.scenario << " | " << row.algorithm << " | n=" << row.vehicleCount
                   << " | avgTime=" << row.avgTimeMs << "ms"
                   << " | avgNodes=" << row.avgNodesExplored
                   << " | found=" << row.pathsFound << "/" << (row.pathsFound + row.pathsNotFound)
                   << " | avgCost=" << row.avgPathCost << "\n";
    }

    std::cout << "\n=== Simulation Throughput (logic-only, no rendering) ===\n";
    for (const auto& row : throughputRows) {
        std::cout << "vehicles=" << row.vehicleCount
                   << " | avgTick=" << row.avgTickTimeMs << "ms"
                   << " | logicFPS=" << row.logicFps << "\n";
    }

    std::cout << "\nWrote bench_results.csv and throughput_results.csv\n";
    return 0;
}
