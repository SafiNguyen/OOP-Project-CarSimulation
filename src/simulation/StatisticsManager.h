#ifndef STATISTICS_MANAGER_H
#define STATISTICS_MANAGER_H

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "../algorithm/PathFindingStrategy.h"

class Graph;
class PathFindingStrategy;
struct StatisticsSnapshot;

struct TravelMetric {
    int vehicleId = -1;
    double totalTravelTime = 0.0;
    double totalDistanceMetres = 0.0;
    int recalculationCount = 0;
    bool completed = false;
};

struct AlgorithmMetric {
    std::string algorithmName;
    int callCount = 0;
    long long totalNodesExplored = 0;
    double totalComputeTimeMs = 0.0;
    double totalPathCost = 0.0;
    int pathsFound = 0;
    int pathsNotFound = 0;

    double averageComputeTimeMs() const;
    double averageNodesExplored() const;
    double averagePathCost() const;
};

struct NetworkMetric {
    double densityVehiclesPerLaneKilometre = 0.0;
    double peakDensityVehiclesPerLaneKilometre = 0.0;
    double averageCongestionLevel = 0.0;
    double peakAverageCongestionLevel = 0.0;
    int totalRoads = 0;
    int blockedRoads = 0;

    double blockedRoadPercentage() const;
};

struct StatisticsSummary {
    double totalSimulatedTime = 0.0;
    double totalDistanceTravelledMetres = 0.0;
    double averageCompletedTripTimeSeconds = 0.0;
    int totalVehiclesTracked = 0;
    int totalCompletedTrips = 0;
    long long totalRecalculations = 0;
    NetworkMetric network;
    std::vector<AlgorithmMetric> perAlgorithm;
};

class StatisticsManager {
public:
    PathResult measurePathfinding(const PathFindingStrategy& strategy,
                                  const Graph& graph,
                                  int startId,
                                  int goalId);

    const std::unordered_map<std::string, AlgorithmMetric>&
    getAlgorithmMetrics() const;
    const AlgorithmMetric* getAlgorithmMetric(
        const std::string& algorithmName) const;
    void resetAlgorithmMetrics();

    void recordTick(double dt);
    void recordVehicleTravel(int vehicleId,
                             double dt,
                             double distanceMetres = 0.0);
    void recordRecalculation(int vehicleId);
    void markVehicleCompleted(int vehicleId);
    void recordNetworkState(const Graph& graph,
                            std::size_t activeVehicleCount);
    void printPeriodicReport(long long tickCount,
                             int everyNTicks = 10) const;

    const std::unordered_map<int, TravelMetric>& getTravelMetrics() const;
    const TravelMetric* getTravelMetric(int vehicleId) const;
    StatisticsSummary getSummary() const;

    StatisticsSnapshot captureSnapshot() const;
    void restoreSnapshot(const StatisticsSnapshot& snapshot);

private:
    std::unordered_map<std::string, AlgorithmMetric> algorithmMetrics;
    std::unordered_map<int, TravelMetric> travelMetrics;
    NetworkMetric networkMetric;
    double totalSimulatedTime = 0.0;
    long long totalRecalculations = 0;
    long long tickCounter = 0;
    int completedTrips = 0;

    TravelMetric& getOrCreateTravelMetric(int vehicleId);
};

#endif
