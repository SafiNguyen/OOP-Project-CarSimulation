#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include "Graph.h"
#include "Intersection.h"
#include "Mapload.h"
#include "PointOfInterest.h"
#include "Road.h"
#include "model/vehicle/VehicleFactory.h"
#include "algorithm/DijkstraStrategy.h"
#include "simulation/EventManager.h"
#include "simulation/SnapshotManager.h"
#include "simulation/StatisticsManager.h"
#include "simulation/TimePlaybackController.h"
#include "simulation/TrafficSimulator.h"
#include "simulation/VehicleSpawnPolicy.h"
#include "visualization/SimulatorFactory.h"

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
    check(playback->canRewind() && !playback->canForward(),
          "Live playback can rewind with a single snapshot");
    std::string singleSnapshotError;
    check(playback->rewind() == initial &&
              playback->applyPendingSeek(&singleSnapshotError),
          "Single-snapshot rewind restores the first captured state");
    check(!playback->canRewind() && !playback->canForward(),
          "Playback buttons stop at the only snapshot boundary");
    playback->resumeLive();
    simulator.resume();

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

void testLargeDemandStreamsWithoutActiveLimit() {
    Graph graph;
    graph.addIntersection(new Intersection(1, 0.0, 0.0));
    graph.addIntersection(new Intersection(2, 100.0, 0.0));
    graph.addIntersection(new Intersection(3, 200.0, 0.0));
    graph.addRoad(new Road(
        10, "East 1", graph.getIntersection(1),
        graph.getIntersection(2), 100.0, 20.0));
    graph.addRoad(new Road(
        11, "East 2", graph.getIntersection(2),
        graph.getIntersection(3), 100.0, 20.0));
    graph.addRoad(new Road(
        12, "West 1", graph.getIntersection(3),
        graph.getIntersection(2), 100.0, 20.0));
    graph.addRoad(new Road(
        13, "West 2", graph.getIntersection(2),
        graph.getIntersection(1), 100.0, 20.0));

    DijkstraStrategy strategy;
    constexpr int demandCount = 10000;
    auto simulator = createDemoSimulator(
        graph, &strategy, demandCount);

    check(simulator != nullptr &&
              simulator->getDeferredDemandCount() ==
                  static_cast<std::size_t>(demandCount) &&
              simulator->getHighestReservedVehicleId() ==
                  demandCount - 1,
          "Large demand returns immediately as deferred work");
    check(simulator->getMaximumActiveVehicles() ==
              std::numeric_limits<std::size_t>::max(),
          "Automatic vehicle activation has no fleet-size cap");
    check(near(simulator->getSnapshotInterval(), 0.0),
          "Large-demand mode disables duplicating the full queue into snapshots");

    for (int frame = 0; frame < 5; ++frame) {
        simulator->update(0.016);
    }
    const std::size_t materialized =
        simulator->getVehicles().size() +
        simulator->getPendingVehicleCount();
    check(materialized > 0u &&
              simulator->getDeferredDemandCount() <
                  static_cast<std::size_t>(demandCount) &&
              simulator->getDeferredDemandError().empty(),
          "Deferred demand is materialized in bounded per-frame batches");
}

void testMap4TenThousandDemandKeepsRunning() {
    Graph graph;
    std::string error;
    std::string path = "map4.json";
    if (!std::filesystem::exists(path)) {
        path = "../map4.json";
    }
    check(MapLoad::loadGraphFromJsonFile(
              path, graph, &error),
          "map4 loads for the 10,000-demand stress regression");
    if (!error.empty() ||
        graph.getAllIntersections().size() < 2u) {
        return;
    }

    VehicleSpawnPolicy policy(graph, 42u);
    const auto containsId = [](
        const std::vector<PointOfInterest*>& endpoints,
        int id) {
        return std::any_of(
            endpoints.begin(), endpoints.end(),
            [id](const PointOfInterest* poi) {
                return poi != nullptr &&
                       poi->getId() == id;
            });
    };

    const auto carOrigins =
        policy.getEligibleOrigins(VehicleKind::Car);
    const auto carDestinations =
        policy.getEligibleDestinations(VehicleKind::Car);
    check(containsId(carOrigins, 301) &&
              containsId(carOrigins, 302) &&
              containsId(carOrigins, 304) &&
              containsId(carOrigins, 305) &&
              containsId(carOrigins, 306) &&
              !containsId(carOrigins, 300) &&
              !containsId(carOrigins, 601) &&
              containsId(carDestinations, 302) &&
              containsId(carDestinations, 305) &&
              !containsId(carDestinations, 300) &&
              !containsId(carDestinations, 601),
          "Car endpoints include Mall/Cinema/Park/Residence/Parking origins and non-Hospital, non-Bus destinations");

    const auto motorbikeOrigins =
        policy.getEligibleOrigins(VehicleKind::Motorbike);
    const auto motorbikeDestinations =
        policy.getEligibleDestinations(VehicleKind::Motorbike);
    check(containsId(motorbikeOrigins, 301) &&
              containsId(motorbikeOrigins, 302) &&
              containsId(motorbikeOrigins, 304) &&
              containsId(motorbikeOrigins, 305) &&
              containsId(motorbikeOrigins, 306) &&
              !containsId(motorbikeOrigins, 300) &&
              containsId(motorbikeDestinations, 302) &&
              containsId(motorbikeDestinations, 305) &&
              !containsId(motorbikeDestinations, 601),
          "Motorbike endpoints follow the same civilian policy as Car");

    const auto busOrigins =
        policy.getEligibleOrigins(VehicleKind::Bus);
    const auto busDestinations =
        policy.getEligibleDestinations(VehicleKind::Bus);
    check(containsId(busOrigins, 601) &&
              containsId(busDestinations, 602) &&
              !containsId(busOrigins, 304) &&
              !containsId(busDestinations, 302),
          "Bus endpoints are configured Bus Stations only");

    const auto emergencyOrigins =
        policy.getEligibleOrigins(VehicleKind::Emergency);
    const auto emergencyDestinations =
        policy.getEligibleDestinations(VehicleKind::Emergency);
    check(containsId(emergencyOrigins, 300) &&
              !containsId(emergencyOrigins, 304) &&
              containsId(emergencyDestinations, 302) &&
              containsId(emergencyDestinations, 305) &&
              !containsId(emergencyDestinations, 601),
          "Emergency endpoints start at City Hospital and exclude Bus Stations as destinations");

    DijkstraStrategy strategy;
    constexpr int demandCount = 10000;
    auto simulator = createDemoSimulator(
        graph, &strategy, demandCount);

    // Ten seconds of UI-like frames exercises incremental transit planning,
    // pending admission, vehicle motion, and cleanup without blocking the
    // event loop or requiring any Control Center action after Start.
    for (int frame = 0; frame < 600; ++frame) {
        simulator->update(1.0 / 60.0);
    }

    const std::size_t accounted =
        simulator->getDeferredDemandCount() +
        simulator->getVehicles().size() +
        simulator->getPendingVehicleCount() +
        simulator->getFinishedVehicles().size();
    check(simulator->getDeferredDemandError().empty() &&
              accounted == static_cast<std::size_t>(demandCount),
          "map4 keeps all 10,000 trips accounted for while running");
    check(simulator->getSnapshotManager() != nullptr &&
              simulator->getSnapshotManager()->capacity() == 12u,
          "10,000-trip playback history uses bounded memory capacity");

    // Regression for the interactive map4 failure: manual spawning used to
    // rescan and copy the complete pending fleet once per new vehicle. That
    // made a batch effectively quadratic and could block the window long
    // enough for Windows to terminate it as AppHangB1. Exercise the same
    // sequence together with snapshot capture/restore, which reconstructs
    // every vehicle and invalidates all old addresses.
    const std::size_t stableSnapshot =
        simulator->captureSnapshotNow();
    check(stableSnapshot != SnapshotManager::npos,
          "map4 captures a 10,000-trip stability snapshot");

    PointOfInterest* manualOrigin = graph.getPointOfInterest(301);
    PointOfInterest* manualDestination = graph.getPointOfInterest(302);
    int lastManualId = simulator->getHighestReservedVehicleId();
    int manualAccepted = 0;
    for (int index = 0; index < 32; ++index) {
        const int vehicleId = simulator->reserveNextVehicleId();
        lastManualId = vehicleId;
        Vehicle* vehicle = VehicleFactory::createVehicle(
            VehicleKind::Car,
            vehicleId,
            20.0,
            manualOrigin->getNearestIntersection(),
            manualDestination->getNearestIntersection());
        vehicle->setSpawnPOI(manualOrigin);
        vehicle->setTargetPOI(manualDestination);
        if (simulator->addVehicleImmediately(vehicle)) {
            ++manualAccepted;
        }
    }
    check(manualAccepted == 32 &&
              lastManualId >= demandCount + 31,
          "map4 reserves unique manual-spawn IDs without fleet rescans");

    std::string restoreError;
    TimePlaybackController* playback =
        simulator->getPlaybackController();
    check(playback != nullptr &&
              playback->seek(stableSnapshot) == stableSnapshot &&
              playback->applyPendingSeek(&restoreError),
          "map4 restores the stability snapshot after manual spawning");
    const std::size_t restoredAccounted =
        simulator->getDeferredDemandCount() +
        simulator->getVehicles().size() +
        simulator->getPendingVehicleCount() +
        simulator->getFinishedVehicles().size();
    check(restoreError.empty() &&
              restoredAccounted == static_cast<std::size_t>(demandCount) &&
              simulator->reserveNextVehicleId() > lastManualId,
          "snapshot restore keeps fleet ownership and future IDs valid");
}

} // namespace

int main() {
    testTrafficEventLifecycle();
    testStatisticsAggregationAndRestore();
    testSnapshotPlaybackBranching();
    testLargeDemandStreamsWithoutActiveLimit();
    testMap4TenThousandDemandKeepsRunning();

    if (failures == 0) {
        std::cout << "All simulation service tests passed.\n";
        return 0;
    }
    std::cerr << failures << " simulation service assertion(s) failed.\n";
    return 1;
}
