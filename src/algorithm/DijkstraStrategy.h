#ifndef DIJKSTRASTRATEGY_H
#define DIJKSTRASTRATEGY_H

#include <algorithm>

#include "PathFindingStrategy.h"

/**
 * DijkstraStrategy (Congestion-aware, speed-tunable)
 * -------------------------------------
 * Classic Dijkstra shortest-path using a min-priority-queue.
 *
 * The "cost" of each road is Road::getWeightedCost(speedPreference_),
 * which blends two objectives:
 *
 *   speedPreference = 1.0  -> pure travel-time optimization. Cost already
 *                             factors in distance, speed limit AND
 *                             congestion level (cost = distance /
 *                             (speedLimit / congestion)), and returns
 *                             +infinity for blocked roads. This is the
 *                             original behaviour (equivalent to using
 *                             Road::getTravelCost()) and makes Dijkstra a
 *                             TRAVEL-TIME-OPTIMAL, congestion-aware
 *                             router: it actively avoids congested/blocked
 *                             roads AND prefers roads with a higher speed
 *                             limit even when they are physically longer.
 *   speedPreference = 0.0  -> pure distance optimization; speed limit and
 *                             congestion are ignored, only physical
 *                             distance matters.
 *   0 < speedPreference < 1 -> a blend of both, e.g. 0.5 means distance
 *                             and travel-time both matter roughly equally.
 *
 * speedPreference defaults to 1.0, so any existing code that constructs
 * DijkstraStrategy with no arguments behaves exactly as before.
 *
 * Time complexity: O((V + E) log V) with a binary heap.
 */
class DijkstraStrategy : public PathFindingStrategy {
public:
    explicit DijkstraStrategy(double speedPreference = 1.0)
        : speedPreference_(std::clamp(speedPreference, 0.0, 1.0)) {}

    PathResult findPath(const Graph& graph, int startId, int goalId) const override;
    std::string name() const override { return "Dijkstra (Congestion-aware)"; }

    // Runtime-adjustable blend between distance-optimal (0.0) and
    // time-optimal/speed-aware (1.0) routing. Lets the UI (e.g. a
    // DebugConsole slider) tune this without recreating the strategy.
    void setSpeedPreference(double speedPreference) {
        speedPreference_ = std::clamp(speedPreference, 0.0, 1.0);
    }
    double getSpeedPreference() const { return speedPreference_; }

private:
    double speedPreference_;
};

#endif
