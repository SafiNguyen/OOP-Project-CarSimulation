#ifndef BUS_TRIP_PLAN_H
#define BUS_TRIP_PLAN_H

#include <cstddef>
#include <vector>

class BusStop;
class Road;

struct BusTripPlan {
    std::vector<Road*> roadRoute;
    std::vector<const BusStop*> orderedStops;
    std::vector<std::size_t> stopRouteIndices;
    double scheduledDepartureTime = 0.0;
};

#endif
