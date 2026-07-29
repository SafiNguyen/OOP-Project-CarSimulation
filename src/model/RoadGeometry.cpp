#include "RoadGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Intersection.h"
#include "Crosswalk.h"
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
    const double laneWidth = road.getLaneWidthMetres();
    if (hasReverseDirection(road)) {
        return (MEDIAN_GAP_METRES * 0.5 +
                (static_cast<double>(laneIndex) + 0.5) *
                    laneWidth) / scale;
    }
    return ((static_cast<double>(laneIndex) + 0.5) -
            static_cast<double>(road.getLaneCount()) * 0.5) *
           laneWidth / scale;
}

double surfaceOffsetWorld(const Road& road) {
    if (!hasReverseDirection(road)) return 0.0;
    return (MEDIAN_GAP_METRES * 0.5 +
            static_cast<double>(road.getLaneCount()) *
                road.getLaneWidthMetres() * 0.5) /
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

    double maximumPhysicalHalfWidth = 0.0;
    double shortestRoad = std::numeric_limits<double>::infinity();
    const auto inspect = [&](const std::vector<Road*>& roads) {
        for (const Road* road : roads) {
            if (road == nullptr) continue;
            const double carriagewayWidth =
                carriagewayWidthMetres(*road);
            maximumPhysicalHalfWidth = std::max(
                maximumPhysicalHalfWidth,
                hasReverseDirection(*road)
                    ? carriagewayWidth +
                          MEDIAN_GAP_METRES * 0.5
                    : carriagewayWidth * 0.5);
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
    if (maximumPhysicalHalfWidth <= 0.0) return 0.0;

    double radius = std::max(
        8.0,
        maximumPhysicalHalfWidth + 3.0);
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
    return road.getReverseRoad() != nullptr;
}

double carriagewayWidthMetres(const Road& road) {
    return static_cast<double>(std::max(1, road.getLaneCount())) *
           road.getLaneWidthMetres();
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

Vec2 laneBoundaryEndpoint(const Road& road,
                          int boundaryIndex,
                          bool atStart) {
    boundaryIndex =
        std::clamp(boundaryIndex, 0, road.getLaneCount());
    const Vec2 surface = roadSurfaceEndpoint(road, atStart);
    const double acrossMetres =
        (static_cast<double>(boundaryIndex) -
         static_cast<double>(road.getLaneCount()) * 0.5) *
        road.getLaneWidthMetres();
    return surface +
           rightNormal(roadDirection(road)) *
               (acrossMetres / metresPerWorldUnit(road));
}

Vec2 roadEdgeEndpoint(const Road& road,
                      bool rightEdge,
                      bool atStart) {
    return laneBoundaryEndpoint(
        road,
        rightEdge ? road.getLaneCount() : 0,
        atStart);
}

Vec2 sampleRoadEdge(const Road& road,
                    bool rightEdge,
                    double progressMetres) {
    const double ratio = road.getDistance() > 1e-12
        ? std::clamp(
              progressMetres / road.getDistance(),
              0.0,
              1.0)
        : 0.0;
    return lerp(
        roadEdgeEndpoint(road, rightEdge, true),
        roadEdgeEndpoint(road, rightEdge, false),
        ratio);
}

Vec2 sampleSidewalk(const Road& road,
                    bool rightSide,
                    double progressMetres) {
    const Vec2 edge = sampleRoadEdge(
        road,
        rightSide,
        progressMetres);
    const Vec2 outward = rightNormal(
        roadDirection(road)) *
        (rightSide ? 1.0 : -1.0);
    // The sidewalk surface starts outside the curb. Including the curb in
    // the centre offset keeps the complete sidewalk strip beyond the
    // carriageway instead of centring its inner border on the road edge.
    const double centreOffsetMetres =
        SIDEWALK_CURB_WIDTH_METRES +
        SIDEWALK_WIDTH_METRES * 0.5;
    return edge +
           outward *
               (centreOffsetMetres /
                metresPerWorldUnit(road));
}

Vec2 sampleRoadSurface(const Road& road, double progressMetres) {
    const double ratio = road.getDistance() > 1e-12
        ? std::clamp(progressMetres / road.getDistance(), 0.0, 1.0)
        : 0.0;
    return lerp(
        roadSurfaceEndpoint(road, true),
        roadSurfaceEndpoint(road, false),
        ratio);
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

double stopLineProgressMetres(const Road& incomingRoad) {
    const Intersection* intersection =
        incomingRoad.getEnd();
    if (intersection != nullptr) {
        const Crosswalk* crosswalk =
            intersection->getCrosswalkForIncomingRoad(
                &incomingRoad);
        if (crosswalk != nullptr) {
            return crosswalk->
                getStopLineProgressMetres();
        }
    }
    return std::max(
        0.0,
        incomingRoad.getDistance() - STOP_LINE_SETBACK_METRES);
}

Vec2 stopLineCentre(const Road& incomingRoad) {
    return sampleRoadSurface(
        incomingRoad,
        stopLineProgressMetres(incomingRoad));
}

} // namespace RoadGeometry
