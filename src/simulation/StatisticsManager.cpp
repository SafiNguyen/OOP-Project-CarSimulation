#include "StatisticsManager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#include "Graph.h"
#include "Road.h"
#include "SnapshotTypes.h"

namespace {

bool isPositiveFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

} // namespace

double AlgorithmMetric::averageComputeTimeMs() const {
    return callCount > 0 ? totalComputeTimeMs / callCount : 0.0;
}

double AlgorithmMetric::averageNodesExplored() const {
    return callCount > 0
        ? static_cast<double>(totalNodesExplored) / callCount
        : 0.0;
}

double AlgorithmMetric::averagePathCost() const {
    return pathsFound > 0 ? totalPathCost / pathsFound : 0.0;
}

double NetworkMetric::blockedRoadPercentage() const {
    return totalRoads > 0
        ? 100.0 * static_cast<double>(blockedRoads) / totalRoads
        : 0.0;
}

PathResult StatisticsManager::measurePathfinding(
    const PathFindingStrategy& strategy,
    const Graph& graph,
    int startId,
    int goalId) {
    using Clock = std::chrono::steady_clock;

    const auto start = Clock::now();
    PathResult result = strategy.findPath(graph, startId, goalId);
    const auto end = Clock::now();
    const double elapsedMs =
        std::chrono::duration<double, std::milli>(end - start).count();

    AlgorithmMetric& metric = algorithmMetrics[strategy.name()];
    metric.algorithmName = strategy.name();
    ++metric.callCount;
    metric.totalNodesExplored += result.nodesExplored;
    metric.totalComputeTimeMs += elapsedMs;
    if (result.found) {
        ++metric.pathsFound;
        metric.totalPathCost += result.totalCost;
    } else {
        ++metric.pathsNotFound;
    }
    return result;
}

const std::unordered_map<std::string, AlgorithmMetric>&
StatisticsManager::getAlgorithmMetrics() const {
    return algorithmMetrics;
}

const AlgorithmMetric* StatisticsManager::getAlgorithmMetric(
    const std::string& algorithmName) const {
    const auto it = algorithmMetrics.find(algorithmName);
    return it != algorithmMetrics.end() ? &it->second : nullptr;
}

void StatisticsManager::resetAlgorithmMetrics() {
    algorithmMetrics.clear();
}

TravelMetric& StatisticsManager::getOrCreateTravelMetric(int vehicleId) {
    auto [it, inserted] = travelMetrics.try_emplace(vehicleId);
    if (inserted) {
        it->second.vehicleId = vehicleId;
    }
    return it->second;
}

void StatisticsManager::recordTick(double dt) {
    if (!isPositiveFinite(dt)) {
        return;
    }
    totalSimulatedTime += dt;
    ++tickCounter;
}

void StatisticsManager::recordVehicleTravel(
    int vehicleId,
    double dt,
    double distanceMetres) {
    if (vehicleId < 0 ||
        (!isPositiveFinite(dt) && !isPositiveFinite(distanceMetres))) {
        return;
    }

    TravelMetric& metric = getOrCreateTravelMetric(vehicleId);
    if (isPositiveFinite(dt)) {
        metric.totalTravelTime += dt;
    }
    if (isPositiveFinite(distanceMetres)) {
        metric.totalDistanceMetres += distanceMetres;
    }
}

void StatisticsManager::recordRecalculation(int vehicleId) {
    if (vehicleId < 0) {
        return;
    }
    ++getOrCreateTravelMetric(vehicleId).recalculationCount;
    ++totalRecalculations;
}

void StatisticsManager::markVehicleCompleted(int vehicleId) {
    if (vehicleId < 0) {
        return;
    }
    TravelMetric& metric = getOrCreateTravelMetric(vehicleId);
    if (!metric.completed) {
        metric.completed = true;
        ++completedTrips;
    }
}

void StatisticsManager::recordNetworkState(
    const Graph& graph,
    std::size_t activeVehicleCount) {
    double laneLengthMetres = 0.0;
    double congestionLaneLengthMetres = 0.0;
    double weightedCongestion = 0.0;
    int roadCount = 0;
    int blockedRoadCount = 0;

    for (const Road* road : graph.getAllRoads()) {
        if (road == nullptr) {
            continue;
        }
        ++roadCount;
        if (road->isBlocked()) {
            ++blockedRoadCount;
        }

        const double roadDistance = road->getDistance();
        const int laneCount = road->getLaneCount();
        const double roadLaneLength =
            isPositiveFinite(roadDistance) && laneCount > 0
                ? roadDistance * static_cast<double>(laneCount)
                : 0.0;
        const double congestion = road->getDynamicCongestionLevel();
        laneLengthMetres += roadLaneLength;
        if (isPositiveFinite(congestion)) {
            congestionLaneLengthMetres += roadLaneLength;
            weightedCongestion += congestion * roadLaneLength;
        }
    }

    networkMetric.totalRoads = roadCount;
    networkMetric.blockedRoads = blockedRoadCount;
    networkMetric.densityVehiclesPerLaneKilometre =
        laneLengthMetres > 0.0
            ? static_cast<double>(activeVehicleCount) * 1000.0 /
                  laneLengthMetres
            : 0.0;
    networkMetric.averageCongestionLevel =
        congestionLaneLengthMetres > 0.0
            ? weightedCongestion / congestionLaneLengthMetres
            : 0.0;
    networkMetric.peakDensityVehiclesPerLaneKilometre = std::max(
        networkMetric.peakDensityVehiclesPerLaneKilometre,
        networkMetric.densityVehiclesPerLaneKilometre);
    networkMetric.peakAverageCongestionLevel = std::max(
        networkMetric.peakAverageCongestionLevel,
        networkMetric.averageCongestionLevel);
}

void StatisticsManager::printPeriodicReport(
    long long tickCount,
    int everyNTicks) const {
    if (everyNTicks <= 0 || tickCount % everyNTicks != 0) {
        return;
    }

    const StatisticsSummary summary = getSummary();
    std::cout << "---- [StatisticsManager] Report @ tick "
              << tickCount << " ----\n"
              << "  Simulated time      : "
              << summary.totalSimulatedTime << "s\n"
              << "  Vehicles tracked    : "
              << summary.totalVehiclesTracked << '\n'
              << "  Completed trips     : "
              << summary.totalCompletedTrips << '\n'
              << "  Distance travelled  : "
              << summary.totalDistanceTravelledMetres << "m\n"
              << "  Average trip time   : "
              << summary.averageCompletedTripTimeSeconds << "s\n"
              << "  Density             : "
              << summary.network.densityVehiclesPerLaneKilometre
              << " vehicles/lane-km\n"
              << "  Average congestion  : "
              << summary.network.averageCongestionLevel << "x\n"
              << "  Blocked roads       : "
              << summary.network.blockedRoads << '/'
              << summary.network.totalRoads << '\n'
              << "  Route recalculations: "
              << summary.totalRecalculations << '\n';

    for (const AlgorithmMetric& metric : summary.perAlgorithm) {
        std::cout << "  [" << metric.algorithmName
                  << "] calls=" << metric.callCount
                  << " avgTime=" << metric.averageComputeTimeMs() << "ms"
                  << " avgNodes=" << metric.averageNodesExplored()
                  << " found=" << metric.pathsFound << '/'
                  << metric.callCount << '\n';
    }
    std::cout << "-----------------------------------------------\n";
}

const std::unordered_map<int, TravelMetric>&
StatisticsManager::getTravelMetrics() const {
    return travelMetrics;
}

const TravelMetric* StatisticsManager::getTravelMetric(int vehicleId) const {
    const auto it = travelMetrics.find(vehicleId);
    return it != travelMetrics.end() ? &it->second : nullptr;
}

StatisticsSummary StatisticsManager::getSummary() const {
    StatisticsSummary summary;
    summary.totalSimulatedTime = totalSimulatedTime;
    summary.totalVehiclesTracked = static_cast<int>(travelMetrics.size());
    summary.totalCompletedTrips = completedTrips;
    summary.totalRecalculations = totalRecalculations;
    summary.network = networkMetric;

    double completedTravelTime = 0.0;
    for (const auto& [vehicleId, metric] : travelMetrics) {
        static_cast<void>(vehicleId);
        summary.totalDistanceTravelledMetres +=
            metric.totalDistanceMetres;
        if (metric.completed) {
            completedTravelTime += metric.totalTravelTime;
        }
    }
    if (completedTrips > 0) {
        summary.averageCompletedTripTimeSeconds =
            completedTravelTime / completedTrips;
    }

    summary.perAlgorithm.reserve(algorithmMetrics.size());
    for (const auto& [name, metric] : algorithmMetrics) {
        static_cast<void>(name);
        summary.perAlgorithm.push_back(metric);
    }
    std::sort(
        summary.perAlgorithm.begin(),
        summary.perAlgorithm.end(),
        [](const AlgorithmMetric& lhs, const AlgorithmMetric& rhs) {
            return lhs.algorithmName < rhs.algorithmName;
        });
    return summary;
}

StatisticsSnapshot StatisticsManager::captureSnapshot() const {
    StatisticsSnapshot snapshot;
    snapshot.algorithmMetrics = algorithmMetrics;
    snapshot.travelMetrics = travelMetrics;
    snapshot.networkMetric = networkMetric;
    snapshot.totalSimulatedTime = totalSimulatedTime;
    snapshot.totalRecalculations = totalRecalculations;
    snapshot.tickCounter = tickCounter;
    snapshot.completedTrips = completedTrips;
    return snapshot;
}

void StatisticsManager::restoreSnapshot(
    const StatisticsSnapshot& snapshot) {
    algorithmMetrics = snapshot.algorithmMetrics;
    travelMetrics = snapshot.travelMetrics;
    networkMetric = snapshot.networkMetric;
    totalSimulatedTime = snapshot.totalSimulatedTime;
    totalRecalculations = snapshot.totalRecalculations;
    tickCounter = snapshot.tickCounter;
    completedTrips = snapshot.completedTrips;
}
