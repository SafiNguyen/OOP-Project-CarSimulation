#ifndef STATISTICS_MANAGER_H
#define STATISTICS_MANAGER_H

#include <string>
#include <unordered_map>
#include <vector>

#include "../algorithm/PathFindingStrategy.h" // PathResult

class Graph;
class PathFindingStrategy;

/**
 * TravelMetric
 * ------------
 * Số liệu tích lũy cho MỘT xe cụ thể trong suốt vòng đời của nó
 * trong simulation. Được cập nhật mỗi tick bởi
 * StatisticsManager::recordVehicleTravel() và mỗi lần xe phải đổi
 * lộ trình (recalculateRoute) bởi recordRecalculation().
 */
struct TravelMetric {
    int vehicleId = -1;
    double totalTravelTime = 0.0;   // tổng thời gian mô phỏng xe đã di chuyển (giây sim)
    int recalculationCount = 0;     // số lần recalculateRoute() được gọi cho xe này
    bool completed = false;         // xe đã tới đích hay chưa
};

/**
 * AlgorithmMetric
 * ---------------
 * Số liệu tổng hợp cho MỘT thuật toán tìm đường (BFS / Dijkstra / A*).
 * Dùng để so sánh hiệu năng giữa các strategy: số lần gọi, tổng số
 * node đã duyệt (nodesExplored), tổng thời gian tính toán thực tế
 * (wall-clock), và tỉ lệ tìm được đường.
 */
struct AlgorithmMetric {
    std::string algorithmName;
    int callCount = 0;              // số lần findPath() được gọi
    long long totalNodesExplored = 0;
    double totalComputeTimeMs = 0.0; // tổng thời gian wall-clock (ms) để chạy findPath()
    double totalPathCost = 0.0;      // tổng totalCost của các đường tìm được (để tính trung bình)
    int pathsFound = 0;
    int pathsNotFound = 0;

    double averageComputeTimeMs() const {
        return callCount > 0 ? totalComputeTimeMs / callCount : 0.0;
    }

    double averageNodesExplored() const {
        return callCount > 0 ? static_cast<double>(totalNodesExplored) / callCount : 0.0;
    }

    double averagePathCost() const {
        return pathsFound > 0 ? totalPathCost / pathsFound : 0.0;
    }
};

/**
 * StatisticsSummary
 * -----------------
 * Dữ liệu tổng hợp cuối cùng, dùng cho Thứ 7 khi TV4 vẽ Statistics
 * Panel. Gom lại những con số quan trọng nhất từ toàn bộ
 * StatisticsManager thành 1 struct phẳng, dễ hiển thị.
 */
struct StatisticsSummary {
    double totalSimulatedTime = 0.0;
    int totalVehiclesTracked = 0;
    int totalCompletedTrips = 0;
    long long totalRecalculations = 0;
    std::vector<AlgorithmMetric> perAlgorithm; // 1 dòng cho mỗi thuật toán đã đo
};

/**
 * StatisticsManager
 * -----------------
 * Trách nhiệm (Single Responsibility): thu thập & tổng hợp số liệu,
 * KHÔNG tự chạy simulation và KHÔNG tự chạy pathfinding thật sự cho
 * mục đích routing (đó là việc của Vehicle / TrafficSimulator).
 *
 * Hai chế độ sử dụng:
 *  1) Benchmark độc lập (Thứ 5): measurePathfinding() gọi trực tiếp
 *     một PathFindingStrategy để đo thời gian/số node duyệt, không
 *     cần TrafficSimulator.
 *  2) Tích hợp runtime (Thứ 6): TrafficSimulator::update() gọi
 *     recordTick()/recordVehicleTravel()/recordRecalculation() mỗi
 *     tick để tích lũy số liệu thật của simulation đang chạy.
 */
class StatisticsManager {
public:
    StatisticsManager() = default;

    // --- Thứ 5: Benchmark thuật toán (độc lập với TrafficSimulator) ---

    // Chạy strategy.findPath(graph, startId, goalId), đo thời gian
    // wall-clock, cập nhật AlgorithmMetric tương ứng (key = strategy.name())
    // và trả lại PathResult y hệt như gọi thẳng strategy (để không làm
    // thay đổi hành vi routing thật).
    PathResult measurePathfinding(const PathFindingStrategy& strategy,
                                   const Graph& graph,
                                   int startId,
                                   int goalId);

    const std::unordered_map<std::string, AlgorithmMetric>& getAlgorithmMetrics() const;
    const AlgorithmMetric* getAlgorithmMetric(const std::string& algorithmName) const;
    void resetAlgorithmMetrics();

    // --- Thứ 6: Tích hợp với TrafficSimulator::update() ---

    // Gọi 1 lần mỗi tick, trước hoặc sau khi cập nhật các xe.
    void recordTick(double dt);

    // Gọi cho MỖI xe đang active trong tick này (dt = thời gian tick,
    // dùng chung cho toàn bộ xe đang chạy trong tick đó).
    void recordVehicleTravel(int vehicleId, double dt);

    // Gọi khi Vehicle::recalculateRoute() được gọi thành công cho vehicleId.
    void recordRecalculation(int vehicleId);

    // Đánh dấu 1 xe đã hoàn thành hành trình (tới đích).
    void markVehicleCompleted(int vehicleId);

    // In báo cáo tạm ra terminal mỗi N tick (mặc định 10) - dùng bởi
    // TrafficSimulator::update() ở Thứ 6.
    void printPeriodicReport(long long tickCount, int everyNTicks = 10) const;

    const std::unordered_map<int, TravelMetric>& getTravelMetrics() const;
    const TravelMetric* getTravelMetric(int vehicleId) const;

    // --- Thứ 7: tổng hợp cho Statistics Panel ---
    StatisticsSummary getSummary() const;

private:
    std::unordered_map<std::string, AlgorithmMetric> algorithmMetrics;
    std::unordered_map<int, TravelMetric> travelMetrics;

    double totalSimulatedTime = 0.0;
    long long totalRecalculations = 0;
    long long tickCounter = 0;
    int completedTrips = 0;

    TravelMetric& getOrCreateTravelMetric(int vehicleId);
};

#endif
