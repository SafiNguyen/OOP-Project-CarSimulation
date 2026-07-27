#include "StatisticsManager.h"

#include <chrono>
#include <iostream>

#include "../algorithm/PathFindingStrategy.h"
#include "../model/Graph.h"

// ------------------------------------------------------------------
// Thứ 5: Benchmark thuật toán tìm đường (độc lập với TrafficSimulator)
// ------------------------------------------------------------------

PathResult StatisticsManager::measurePathfinding(const PathFindingStrategy& strategy,
                                                  const Graph& graph,
                                                  int startId,
                                                  int goalId) {
    using clock = std::chrono::steady_clock;

    const auto t0 = clock::now();
    PathResult result = strategy.findPath(graph, startId, goalId);
    const auto t1 = clock::now();

    const double elapsedMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    AlgorithmMetric& metric = algorithmMetrics[strategy.name()];
    metric.algorithmName = strategy.name();
    metric.callCount += 1;
    metric.totalNodesExplored += result.nodesExplored;
    metric.totalComputeTimeMs += elapsedMs;

    if (result.found) {
        metric.pathsFound += 1;
        metric.totalPathCost += result.totalCost;
    } else {
        metric.pathsNotFound += 1;
    }

    return result;
}

const std::unordered_map<std::string, AlgorithmMetric>&
StatisticsManager::getAlgorithmMetrics() const {
    return algorithmMetrics;
}

const AlgorithmMetric* StatisticsManager::getAlgorithmMetric(const std::string& algorithmName) const {
    auto it = algorithmMetrics.find(algorithmName);
    if (it == algorithmMetrics.end()) return nullptr;
    return &it->second;
}

void StatisticsManager::resetAlgorithmMetrics() {
    algorithmMetrics.clear();
}

// ------------------------------------------------------------------
// Thứ 6: Tích hợp runtime với TrafficSimulator::update()
// ------------------------------------------------------------------

TravelMetric& StatisticsManager::getOrCreateTravelMetric(int vehicleId) {
    auto it = travelMetrics.find(vehicleId);
    if (it != travelMetrics.end()) return it->second;

    TravelMetric metric;
    metric.vehicleId = vehicleId;
    auto insertResult = travelMetrics.emplace(vehicleId, metric);
    return insertResult.first->second;
}

void StatisticsManager::recordTick(double dt) {
    totalSimulatedTime += dt;
    ++tickCounter;
}

void StatisticsManager::recordVehicleTravel(int vehicleId, double dt) {
    TravelMetric& metric = getOrCreateTravelMetric(vehicleId);
    metric.totalTravelTime += dt;
}

void StatisticsManager::recordRecalculation(int vehicleId) {
    TravelMetric& metric = getOrCreateTravelMetric(vehicleId);
    metric.recalculationCount += 1;
    ++totalRecalculations;
}

void StatisticsManager::markVehicleCompleted(int vehicleId) {
    TravelMetric& metric = getOrCreateTravelMetric(vehicleId);
    if (!metric.completed) {
        ++completedTrips;
    }
    metric.completed = true;
}

void StatisticsManager::printPeriodicReport(long long tickCount, int everyNTicks) const {
    if (everyNTicks <= 0 || tickCount % everyNTicks != 0) return;

    std::cout << "---- [StatisticsManager] Report @ tick " << tickCount << " ----\n";
    std::cout << "  Simulated time     : " << totalSimulatedTime << "s\n";
    std::cout << "  Vehicles tracked   : " << travelMetrics.size() << "\n";
    std::cout << "  Completed trips    : " << completedTrips << "\n";
    std::cout << "  Total recalculations: " << totalRecalculations << "\n";

    for (const auto& pair : algorithmMetrics) {
        const AlgorithmMetric& m = pair.second;
        std::cout << "  [" << m.algorithmName << "] calls=" << m.callCount
                  << " avgTime=" << m.averageComputeTimeMs() << "ms"
                  << " avgNodes=" << m.averageNodesExplored()
                  << " found=" << m.pathsFound << "/" << m.callCount << "\n";
    }
    std::cout << "-----------------------------------------------\n";
}

const std::unordered_map<int, TravelMetric>& StatisticsManager::getTravelMetrics() const {
    return travelMetrics;
}

const TravelMetric* StatisticsManager::getTravelMetric(int vehicleId) const {
    auto it = travelMetrics.find(vehicleId);
    if (it == travelMetrics.end()) return nullptr;
    return &it->second;
}

// ------------------------------------------------------------------
// Thứ 7 (chuẩn bị trước): tổng hợp cho Statistics Panel
// ------------------------------------------------------------------

StatisticsSummary StatisticsManager::getSummary() const {
    StatisticsSummary summary;
    summary.totalSimulatedTime = totalSimulatedTime;
    summary.totalVehiclesTracked = static_cast<int>(travelMetrics.size());
    summary.totalRecalculations = totalRecalculations;
    summary.totalCompletedTrips = completedTrips;

    summary.perAlgorithm.reserve(algorithmMetrics.size());
    for (const auto& pair : algorithmMetrics) {
        summary.perAlgorithm.push_back(pair.second);
    }

    return summary;
}
