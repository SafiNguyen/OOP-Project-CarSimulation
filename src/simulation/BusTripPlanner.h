#ifndef BUS_TRIP_PLANNER_H
#define BUS_TRIP_PLANNER_H

#include <cstddef>
#include <optional>
#include <random>
#include <vector>

#include "BusTripPlan.h"

class BusService;
class BusStop;
class Graph;
class PathFindingStrategy;

class BusTripPlanner {
public:
    static std::optional<BusTripPlan> buildRandomPlan(
        const Graph& graph,
        const PathFindingStrategy& strategy,
        const BusService& service,
        const std::vector<const BusStop*>& availableStops,
        std::mt19937& randomEngine,
        std::size_t minimumStops,
        std::size_t maximumStops,
        double scheduledDepartureTime);
};

#endif
