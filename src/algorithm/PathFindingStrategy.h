#ifndef PATHFINDINGSTRATEGY_H
#define PATHFINDINGSTRATEGY_H

#include <vector>
#include <string>
#include <limits>

class Graph;
class Intersection;
class Road;

/**
 * PathResult
 * ----------
 * Standardized output of any path-finding strategy.
 * Keeping this separate from the strategies themselves makes it easy
 * to compare algorithms (distance routing vs time routing vs
 * congestion-aware routing) using the same data shape.
 */
struct PathResult {
    bool found = false;                 // whether a path was found
    std::vector<int> intersectionPath;  // ordered list of intersection IDs, start..end
    std::vector<Road*> roadPath;        // ordered list of roads taken
    double totalCost = std::numeric_limits<double>::infinity(); // cost in the unit the
                                                                  // strategy optimizes for
                                                                  // (edge count / time / distance)
    int nodesExplored = 0;              // how many intersections were popped from the frontier
                                         // (used to compare algorithm efficiency)
};

/**
 * PathFindingStrategy
 * --------------------
 * Abstract base class (OOP: Abstraction + the basis for Polymorphism).
 * Concrete routing algorithms (BFS, Dijkstra, A*) derive from this class
 * (OOP: Inheritance) and can be swapped at runtime by code that only
 * depends on this interface (Strategy Pattern).
 */
class PathFindingStrategy {
public:
    virtual ~PathFindingStrategy() = default;

    // Computes the path from startId to goalId inside the given graph.
    virtual PathResult findPath(const Graph& graph, int startId, int goalId) const = 0;

    // Human readable name of the strategy, useful for logging/statistics.
    virtual std::string name() const = 0;
};

#endif
