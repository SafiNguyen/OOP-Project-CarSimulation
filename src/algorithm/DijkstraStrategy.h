#ifndef DIJKSTRASTRATEGY_H
#define DIJKSTRASTRATEGY_H

#include "PathFindingStrategy.h"

/**
 * DijkstraStrategy (Congestion-aware)
 * -------------------------------------
 * Classic Dijkstra shortest-path using a min-priority-queue.
 * The "cost" of each road is Road::getTravelCost(), which already
 * factors in distance, speed limit AND congestion level
 * (cost = distance / (speedLimit / congestionLevel)), and returns
 * +infinity for blocked roads. This means Dijkstra here naturally
 * becomes a TRAVEL-TIME-OPTIMAL, congestion-aware router: it will
 * actively avoid congested or blocked roads in favor of a longer but
 * faster/clearer route.
 *
 * Time complexity: O((V + E) log V) with a binary heap.
 */
class DijkstraStrategy : public PathFindingStrategy {
public:
    PathResult findPath(const Graph& graph, int startId, int goalId) const override;
    std::string name() const override { return "Dijkstra (Congestion-aware)"; }
};

#endif
