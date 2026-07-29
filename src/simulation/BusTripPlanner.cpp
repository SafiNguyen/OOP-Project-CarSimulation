#include "BusTripPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

#include "algorithm/PathFindingStrategy.h"
#include "BusService.h"
#include "BusStop.h"
#include "Graph.h"
#include "Intersection.h"
#include "Road.h"
#include "SpawnPoint.h"

namespace {

bool appendRoadAndCollectStops(
    BusTripPlan& plan,
    Road* road,
    std::vector<const BusStop*>& remainingStops) {
    if (road == nullptr) {
        return false;
    }
    if (!plan.roadRoute.empty() &&
        plan.roadRoute.back()->getEnd() !=
            road->getStart()) {
        return false;
    }

    plan.roadRoute.push_back(road);
    const std::size_t routeIndex =
        plan.roadRoute.size() - 1u;

    std::vector<const BusStop*> stopsOnRoad;
    for (const BusStop* stop : remainingStops) {
        if (stop != nullptr &&
            stop->getRoad() == road) {
            stopsOnRoad.push_back(stop);
        }
    }
    std::sort(
        stopsOnRoad.begin(),
        stopsOnRoad.end(),
        [](const BusStop* lhs, const BusStop* rhs) {
            if (lhs->getPositionOnRoad() !=
                rhs->getPositionOnRoad()) {
                return lhs->getPositionOnRoad() <
                       rhs->getPositionOnRoad();
            }
            return lhs->getId() < rhs->getId();
        });

    for (const BusStop* stop : stopsOnRoad) {
        plan.orderedStops.push_back(stop);
        plan.stopRouteIndices.push_back(routeIndex);
        remainingStops.erase(
            std::remove(
                remainingStops.begin(),
                remainingStops.end(),
                stop),
            remainingStops.end());
    }
    return true;
}

bool appendPath(
    BusTripPlan& plan,
    const std::vector<Road*>& roads,
    std::vector<const BusStop*>& remainingStops) {
    for (Road* road : roads) {
        if (!appendRoadAndCollectStops(
                plan, road, remainingStops)) {
            return false;
        }
    }
    return true;
}

bool isValidStopCandidate(
    const Graph& graph,
    const BusStop* stop) {
    if (stop == nullptr ||
        stop->getRoad() == nullptr) {
        return false;
    }
    Road* road =
        graph.getRoad(stop->getRoad()->getId());
    return road == stop->getRoad() &&
           road->getStart() != nullptr &&
           road->getEnd() != nullptr;
}

} // namespace

std::optional<BusTripPlan>
BusTripPlanner::buildRandomPlan(
    const Graph& graph,
    const PathFindingStrategy& strategy,
    const BusService& service,
    const std::vector<const BusStop*>& availableStops,
    std::mt19937& randomEngine,
    std::size_t minimumStops,
    std::size_t maximumStops,
    double scheduledDepartureTime) {
    Road* departureRoad =
        service.getOriginStation().getDepartureRoad();
    Road* arrivalRoad =
        service.getDestinationStation().getArrivalRoad();
    if (departureRoad == nullptr ||
        arrivalRoad == nullptr ||
        !std::isfinite(scheduledDepartureTime) ||
        scheduledDepartureTime < 0.0) {
        return std::nullopt;
    }

    std::vector<const BusStop*> candidates;
    std::unordered_set<int> seenStopIds;
    for (const BusStop* stop : availableStops) {
        if (isValidStopCandidate(graph, stop) &&
            seenStopIds.insert(stop->getId()).second) {
            candidates.push_back(stop);
        }
    }
    if (candidates.empty()) {
        return std::nullopt;
    }

    maximumStops =
        std::min(maximumStops, candidates.size());
    minimumStops =
        std::min(minimumStops, maximumStops);
    if (minimumStops == 0u) {
        minimumStops = 1u;
    }

    std::shuffle(
        candidates.begin(),
        candidates.end(),
        randomEngine);
    std::uniform_int_distribution<std::size_t>
        stopCountDistribution(
            minimumStops,
            maximumStops);
    candidates.resize(
        stopCountDistribution(randomEngine));

    BusTripPlan plan;
    plan.scheduledDepartureTime =
        scheduledDepartureTime;
    std::vector<const BusStop*> remainingStops =
        std::move(candidates);

    if (!appendRoadAndCollectStops(
            plan,
            departureRoad,
            remainingStops)) {
        return std::nullopt;
    }

    Intersection* cursor = departureRoad->getEnd();
    while (std::any_of(
        remainingStops.begin(),
        remainingStops.end(),
        [arrivalRoad](const BusStop* stop) {
            return stop != nullptr &&
                   stop->getRoad() != arrivalRoad;
        })) {
        const BusStop* bestStop = nullptr;
        PathResult bestPath;
        double bestCost =
            std::numeric_limits<double>::infinity();

        for (const BusStop* stop : remainingStops) {
            if (stop == nullptr ||
                stop->getRoad() == arrivalRoad) {
                continue;
            }
            Road* stopRoad = stop->getRoad();
            const PathResult path =
                strategy.findPath(
                    graph,
                    cursor->getId(),
                    stopRoad->getStart()->getId());
            if (!path.found) {
                continue;
            }
            const double candidateCost =
                path.totalCost +
                stopRoad->getTravelCost();
            if (!std::isfinite(candidateCost)) {
                continue;
            }
            if (bestStop == nullptr ||
                candidateCost < bestCost ||
                (candidateCost == bestCost &&
                 stop->getId() < bestStop->getId())) {
                bestStop = stop;
                bestPath = path;
                bestCost = candidateCost;
            }
        }

        if (bestStop == nullptr ||
            !appendPath(
                plan,
                bestPath.roadPath,
                remainingStops)) {
            return std::nullopt;
        }

        if (std::find(
                remainingStops.begin(),
                remainingStops.end(),
                bestStop) !=
            remainingStops.end()) {
            if (!appendRoadAndCollectStops(
                    plan,
                    bestStop->getRoad(),
                    remainingStops)) {
                return std::nullopt;
            }
        }
        cursor = plan.roadRoute.back()->getEnd();
    }

    const PathResult destinationPath =
        strategy.findPath(
            graph,
            cursor->getId(),
            arrivalRoad->getStart()->getId());
    if (!destinationPath.found ||
        !appendPath(
            plan,
            destinationPath.roadPath,
            remainingStops) ||
        !appendRoadAndCollectStops(
            plan,
            arrivalRoad,
            remainingStops) ||
        !remainingStops.empty()) {
        return std::nullopt;
    }

    return plan;
}
