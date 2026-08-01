#ifndef ROUTE_FOLLOWER_H
#define ROUTE_FOLLOWER_H

class Vehicle;
class Graph;
class PathFindingStrategy;

class RouteFollower {
public:
    bool recalculateRoute(Vehicle& vehicle, const Graph& graph, PathFindingStrategy* strategy) const;
    bool performUTurn(Vehicle& vehicle, const Graph& graph, PathFindingStrategy* strategy) const;
    bool advanceToNextRoad(Vehicle& vehicle) const;
};

#endif
