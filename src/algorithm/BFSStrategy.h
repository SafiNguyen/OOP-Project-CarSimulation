#ifndef BFSSTRATEGY_H
#define BFSSTRATEGY_H

#include "PathFindingStrategy.h"

/**
 * BFSStrategy (Basic)
 * --------------------
 * Breadth-First Search explores the graph level by level using a FIFO
 * queue. It guarantees the path with the FEWEST NUMBER OF ROADS
 * (hops), ignoring distance, speed limit and congestion.
 *
 * Blocked roads are treated as impassable (never expanded), but among
 * the roads that ARE passable, BFS does not care which one is
 * "better" - it just wants to reach the goal in the fewest hops.
 *
 * Time complexity: O(V + E)
 */
class BFSStrategy : public PathFindingStrategy {
public:
    PathResult findPath(const Graph& graph, int startId, int goalId) const override;
    std::string name() const override { return "BFS (Basic - fewest roads)"; }
};

#endif
