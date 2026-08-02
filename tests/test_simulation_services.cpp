#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Graph.h"
#include "Intersection.h"
#include "Road.h"
#include "algorithm/DijkstraStrategy.h"
#include "simulation/EventManager.h"
#include "simulation/SnapshotManager.h"
#include "simulation/StatisticsManager.h"
#include "simulation/TimePlaybackController.h"
#include "simulation/TrafficSimulator.h"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "[FAIL] " << message << '\n';
    }
}

bool near(double actual, double expected, double tolerance = 1e-8) {
    return std::fabs(actual - expected) <= tolerance;
}

Graph buildLineGraph(int laneCount = 2) {
    Graph graph;
    graph.addIntersection(new Intersection(1, 0.0, 0.0));
    graph.addIntersection(new Intersection(2, 100.0, 0.0));
    graph.addRoad(new Road(
        10,
        "Simulation service test road",
        graph.getIntersection(1),
        graph.getIntersection(2),
        100.0,
        20.0,
        1.0,
        laneCount));
    return graph;
}

void testTrafficEventLifecycle() {
    Graph graph = buildLineGraph();
    Road* road = graph.getRoad(10);
    std::vector<Vehicle*> vehicles;
    DijkstraStrategy strategy;
    StatisticsManager statistics;
    EventManager events(&graph, &vehicles, &strategy, &statistics);

    events.triggerEvent(
        std::make_unique<CongestionEvent>(10, 1.0, 2.5));
    check(near(road->getCongestionLevel(), 2.5),
          "Congestion event applies its severity immediately");
    events.update(0.4);
    const auto congestionSnapshot = events.captureSnapshot();
    check(congestionSnapshot.size() == 1 &&
              near(congestionSnapshot.front().timeElapsed, 0.4),
          "Active event snapshot retains elapsed time");
    events.update(0.6);
    check(near(road->getCongestionLevel(), 1.0) &&
              events.captureSnapshot().empty(),
          "Congestion event expires and restores the normal road state");

    events.triggerEvent(
        std::make_unique<AccidentEvent>(10, 1.0, 1));
    check(!road->getLane(0).isBlocked() &&
              road->getLane(1).isBlocked(),
          "Lane accident blocks only the selected lane");
    events.update(1.0);
    check(!road->getLane(1).isBlocked(),
          "Expired lane accident reopens its lane");

    events.triggerEvent(
        std::make_unique<RoadClosureEvent>(10, 1.0));
    check(road->isBlocked(),
          "Road closure blocks every lane");
    events.update(1.0);
    check(!road->isBlocked(),
          "Expired road closure reopens the road");
}

void testStatisticsAggregationAndRestore() {
    Graph graph = buildLineGraph(1);
    DijkstraStrategy strategy;
    StatisticsManager statistics;

    const PathResult found = statistics.measurePathfinding(
        strategy, graph, 1, 2);
    const PathResult missing = statistics.measurePathfinding(
        strategy, graph, 1, 99);
    check(found.found && near(found.totalCost, 5.0),
          "Measured Dijkstra result preserves the computed path cost");
    check(!missing.found,
          "Statistics records a pathfinding miss without changing its result");

    const AlgorithmMetric* algorithm =
        statistics.getAlgorithmMetric(strategy.name());
    check(algorithm != nullptr &&
              algorithm->callCount == 2 &&
              algorithm->pathsFound == 1 &&
              algorithm->pathsNotFound == 1 &&
              near(algorithm->averagePathCost(), 5.0),
          "Pathfinding metrics aggregate successful and failed calls");

    statistics.recordTick(0.25);
    statistics.recordTick(0.75);
    statistics.recordVehicleTravel(7, 1.0);
    statistics.recordRecalculation(7);
    statistics.recordRecalculation(7);
    statistics.markVehicleCompleted(7);
    statistics.markVehicleCompleted(7);
    statistics.recordVehicleTravel(8, 0.5);

    const StatisticsSummary summary = statistics.getSummary();
    check(near(summary.totalSimulatedTime, 1.0) &&
              summary.totalVehiclesTracked == 2 &&
              summary.totalCompletedTrips == 1 &&
              summary.totalRecalculations == 2,
          "Runtime statistics aggregate time, vehicles and idempotent completion");
    const TravelMetric* vehicle = statistics.getTravelMetric(7);
    check(vehicle != nullptr &&
              near(vehicle->totalTravelTime, 1.0) &&
              vehicle->recalculationCount == 2 &&
              vehicle->completed,
          "Per-vehicle travel metrics retain their lifecycle data");

    const StatisticsSnapshot snapshot = statistics.captureSnapshot();
    statistics.recordTick(5.0);
    statistics.markVehicleCompleted(8);
    statistics.resetAlgorithmMetrics();
    statistics.restoreSnapshot(snapshot);

    const StatisticsSummary restored = statistics.getSummary();
    check(near(restored.totalSimulatedTime, 1.0) &&
              restored.totalCompletedTrips == 1 &&
              restored.perAlgorithm.size() == 1,
          "Statistics snapshot restores state after later mutations");
}

void testSnapshotPlaybackBranching() {
    Graph graph = buildLineGraph(1);
    DijkstraStrategy strategy;
    TrafficSimulator simulator(&graph, &strategy);
    simulator.setSnapshotInterval(0.0);

    SnapshotManager* snapshots = simulator.getSnapshotManager();
    TimePlaybackController* playback =
        simulator.getPlaybackController();
    check(snapshots != nullptr && playback != nullptr,
          "Simulator owns snapshot and playback services");
    if (snapshots == nullptr || playback == nullptr) {
        return;
    }

    const std::size_t initial = simulator.captureSnapshotNow();
    simulator.triggerEvent(
        std::make_unique<CongestionEvent>(10, 1.0, 2.0));
    simulator.update(0.1);
    const std::size_t congested = simulator.captureSnapshotNow();
    simulator.update(0.1);
    simulator.captureSnapshotNow();

    check(initial == 0 && congested == 1 && snapshots->size() == 3,
          "Manual snapshots are appended in timeline order");
    check(near(playback->timeAt(initial), 0.0) &&
              near(playback->timeAt(congested), 0.1),
          "Playback exposes each snapshot simulation timestamp");

    check(playback->seek(initial) == initial &&
              playback->hasPendingSeek(),
          "Seeking queues a snapshot for the next frame boundary");
    std::string error;
    check(playback->applyPendingSeek(&error) && error.empty(),
          "Queued seek restores a valid snapshot");
    check(simulator.isPaused() &&
              near(simulator.getElapsedTime(), 0.0) &&
              near(graph.getRoad(10)->getCongestionLevel(), 1.0),
          "Rewind restores simulator time and road state, then pauses");

    check(playback->forward() == congested &&
              playback->applyPendingSeek(&error),
          "Forward playback restores the following snapshot");
    check(near(simulator.getElapsedTime(), 0.1) &&
              near(graph.getRoad(10)->getCongestionLevel(), 2.0),
          "Forward playback restores active traffic-event effects");

    playback->resumeLive();
    check(snapshots->size() == 2 && !playback->isSeeking(),
          "Resuming from the past truncates the abandoned future branch");
    check(playback->seek(99) == SnapshotManager::npos,
          "Out-of-range playback seek is rejected");
}

} // namespace

int main() {
    testTrafficEventLifecycle();
    testStatisticsAggregationAndRestore();
    testSnapshotPlaybackBranching();

    if (failures == 0) {
        std::cout << "All simulation service tests passed.\n";
        return 0;
    }
    std::cerr << failures << " simulation service assertion(s) failed.\n";
    return 1;
}
