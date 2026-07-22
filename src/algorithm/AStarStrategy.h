#ifndef ASTARSTRATEGY_H
#define ASTARSTRATEGY_H

#include <algorithm>

#include "PathFindingStrategy.h"

/**
 * AStarStrategy (Speed-tunable)
 * ----------------------------------
 * A* = Dijkstra + a heuristic h(n) that estimates the remaining cost
 * from node n to the goal, allowing the search to expand far fewer
 * nodes than plain Dijkstra while still being provably optimal,
 * PROVIDED the heuristic never overestimates the true cost
 * (admissible) and is consistent.
 *
 * Like DijkstraStrategy, the edge cost used here is
 * Road::getWeightedCost(speedPreference_), which blends:
 *
 *   speedPreference = 1.0  -> pure travel-time optimization (the original
 *                             behaviour): a road with a higher speed
 *                             limit is cheaper even if physically longer,
 *                             and congestion/blocked roads are penalized.
 *   speedPreference = 0.0  -> pure distance optimization; speed limit is
 *                             ignored entirely.
 *   0 < speedPreference < 1 -> a blend of both.
 *
 * Heuristic used here (kept admissible for ANY speedPreference in [0,1]):
 *   h(n) = EuclideanDistance(n, goal) * (speedPreference / maxSpeedLimit
 *                                         + (1 - speedPreference))
 * This is the best-case (optimistic) cost assuming a vehicle could travel
 * the straight-line distance with congestionLevel == 1 at the fastest
 * speed limit that exists anywhere in the network. Real edge cost is
 * always >= this estimate for both the time term and the distance term,
 * so the heuristic stays admissible and A* remains correct while
 * exploring substantially fewer intersections than Dijkstra on larger
 * maps - hence "speed-optimized".
 *
 * speedPreference defaults to 1.0, so any existing code that constructs
 * AStarStrategy with no arguments behaves exactly as before.
 *
 * Time complexity: O((V + E) log V), with typically far fewer node
 * expansions than Dijkstra in practice.
 */
class AStarStrategy : public PathFindingStrategy {
public:
    explicit AStarStrategy(double speedPreference = 1.0)
        : speedPreference_(std::clamp(speedPreference, 0.0, 1.0)) {}

    PathResult findPath(const Graph& graph, int startId, int goalId) const override;
    std::string name() const override { return "A* (Speed-optimized)"; }

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
