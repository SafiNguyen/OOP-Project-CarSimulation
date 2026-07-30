#include "RoadGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Intersection.h"
#include "Road.h"

namespace RoadGeometry {

namespace {

constexpr double MIN_GEOMETRY_LENGTH = 1e-6;

Vec2 pointOf(const Intersection* intersection) {
    return intersection != nullptr
        ? Vec2{intersection->getX(), intersection->getY()}
        : Vec2{};
}

double lateralOffsetWorld(const Road& road, int laneIndex) {
    laneIndex = std::clamp(laneIndex, 0, road.getLaneCount() - 1);
    const double scale = metresPerWorldUnit(road);
    if (hasReverseDirection(road)) {
        return (MEDIAN_GAP_METRES * 0.5 +
                (static_cast<double>(laneIndex) + 0.5) *
                    LANE_WIDTH_METRES) / scale;
    }
    return ((static_cast<double>(laneIndex) + 0.5) -
            static_cast<double>(road.getLaneCount()) * 0.5) *
           LANE_WIDTH_METRES / scale;
}

double surfaceOffsetWorld(const Road& road) {
    if (!hasReverseDirection(road)) return 0.0;
    return (MEDIAN_GAP_METRES * 0.5 +
            static_cast<double>(road.getLaneCount()) *
                LANE_WIDTH_METRES * 0.5) /
           metresPerWorldUnit(road);
}

Vec2 boundaryPoint(const Road& road, bool atStart) {
    const Intersection* intersection =
        atStart ? road.getStart() : road.getEnd();
    const Vec2 centre = pointOf(intersection);
    const Vec2 direction = roadDirection(road);
    const double radius = intersection != nullptr
        ? junctionBoundaryRadiusWorld(*intersection)
        : 0.0;
    return atStart
        ? centre + direction * radius
        : centre - direction * radius;
}

} // namespace

double metresPerWorldUnit(const Road& road) {
    const double geometryLength =
        distance(pointOf(road.getStart()), pointOf(road.getEnd()));
    if (geometryLength <= MIN_GEOMETRY_LENGTH ||
        !std::isfinite(road.getDistance()) ||
        road.getDistance() <= 0.0) {
        return 1.0;
    }
    return std::clamp(
        road.getDistance() / geometryLength, 1e-3, 1e3);
}

double metresPerWorldUnit(const Intersection& intersection) {
    double total = 0.0;
    int count = 0;
    const auto accumulate = [&](const std::vector<Road*>& roads) {
        for (const Road* road : roads) {
            if (road == nullptr) continue;
            total += metresPerWorldUnit(*road);
            ++count;
        }
    };
    accumulate(intersection.getIncomingRoads());
    accumulate(intersection.getOutgoingRoads());
    return count > 0 ? total / static_cast<double>(count) : 1.0;
}

double junctionBoundaryRadiusMetres(const Intersection& intersection) {
    if (intersection.isRoundabout()) {
        return std::max(
            LANE_WIDTH_METRES,
            intersection.getTraversalRadiusMetres() +
                LANE_WIDTH_METRES * 0.5);
    }

    int maxLaneCount = 0;
    double shortestRoad = std::numeric_limits<double>::infinity();
    const auto inspect = [&](const std::vector<Road*>& roads) {
        for (const Road* road : roads) {
            if (road == nullptr) continue;
            maxLaneCount = std::max(maxLaneCount, road->getLaneCount());
            if (road->getDistance() > 0.0) {
                shortestRoad =
                    std::min(shortestRoad, road->getDistance());
            }
        }
    };
    inspect(intersection.getIncomingRoads());
    inspect(intersection.getOutgoingRoads());

    // Unregistered stack-allocated test roads deliberately retain a zero-size
    // pass-through node, preserving their historical centre-to-centre length.
    if (maxLaneCount == 0) return 0.0;

    double radius = std::max(
        8.0,
        static_cast<double>(maxLaneCount) *
                LANE_WIDTH_METRES * 0.5 +
            4.0);
    if (std::isfinite(shortestRoad)) {
        radius = std::min(radius, shortestRoad * 0.35);
    }
    return std::max(0.0, radius);
}

double junctionBoundaryRadiusWorld(const Intersection& intersection) {
    return junctionBoundaryRadiusMetres(intersection) /
           metresPerWorldUnit(intersection);
}

bool hasReverseDirection(const Road& road) {
    if (road.getStart() == nullptr || road.getEnd() == nullptr) {
        return false;
    }
    for (const Road* candidate : road.getEnd()->getOutgoingRoads()) {
        if (candidate != nullptr &&
            candidate->getEnd() == road.getStart()) {
            return true;
        }
    }
    return false;
}

Vec2 roadDirection(const Road& road) {
    return normalized(
        pointOf(road.getEnd()) - pointOf(road.getStart()),
        {1.0, 0.0});
}

Vec2 roadReferenceEndpoint(const Road& road, bool atStart) {
    return boundaryPoint(road, atStart);
}

Vec2 roadSurfaceEndpoint(const Road& road, bool atStart) {
    const Vec2 base = boundaryPoint(road, atStart);
    return base +
           rightNormal(roadDirection(road)) * surfaceOffsetWorld(road);
}

Vec2 laneEndpoint(const Road& road, int laneIndex, bool atStart) {
    const Vec2 base = boundaryPoint(road, atStart);
    return base +
           rightNormal(roadDirection(road)) *
               lateralOffsetWorld(road, laneIndex);
}

Pose2D sampleLane(const Road& road,
                  int laneIndex,
                  double progressMetres) {
    const Vec2 start = laneEndpoint(road, laneIndex, true);
    const Vec2 end = laneEndpoint(road, laneIndex, false);
    const double ratio = road.getDistance() > 1e-12
        ? std::clamp(progressMetres / road.getDistance(), 0.0, 1.0)
        : 0.0;
    const Vec2 tangent = normalized(end - start, roadDirection(road));
    return {
        lerp(start, end, ratio),
        std::atan2(tangent.y, tangent.x),
        0.0
    };
}

} // namespace RoadGeometry
