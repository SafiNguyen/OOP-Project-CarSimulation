#ifndef ASTARSTRATEGY_H
#define ASTARSTRATEGY_H

#include "PathFindingStrategy.h"

/**
 * AStarStrategy (Speed-optimized)
 * ----------------------------------
 * A* = Dijkstra + a heuristic h(n) that estimates the remaining cost
 * from node n to the goal, allowing the search to expand far fewer
 * nodes than plain Dijkstra while still being provably optimal,
 * PROVIDED the heuristic never overestimates the true cost
 * (admissible) and is consistent.
 *
 * Heuristic used here:
 *   h(n) = EuclideanDistance(n, goal) / maxSpeedLimitInGraph
 * This is the best-case (optimistic) travel time assuming a vehicle
 * could travel the straight-line distance at the fastest speed limit
 * that exists anywhere in the network, with congestionLevel == 1
 * (i.e. no traffic at all). Real travel cost is always
 * >= this estimate, so the heuristic is admissible and A* remains
 * correct while exploring substantially fewer intersections than
 * Dijkstra on larger maps - hence "speed-optimized".
 *
 * Time complexity: O((V + E) log V), with typically far fewer node
 * expansions than Dijkstra in practice.
 */
class AStarStrategy : public PathFindingStrategy {
public:
    PathResult findPath(const Graph& graph, int startId, int goalId) const override;
    std::string name() const override { return "A* (Speed-optimized)"; }
};

#endif
