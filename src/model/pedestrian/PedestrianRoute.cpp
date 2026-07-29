#include "PedestrianRoute.h"

#include <algorithm>
#include <cmath>

#include "Crosswalk.h"
#include "Road.h"
#include "RoadGeometry.h"

bool PedestrianRouteSegment::isValid() const {
    return std::isfinite(start.x) &&
           std::isfinite(start.y) &&
           std::isfinite(end.x) &&
           std::isfinite(end.y) &&
           std::isfinite(lengthMetres) &&
           lengthMetres > 1e-6 &&
           (kind != PedestrianSegmentKind::Crosswalk ||
            crosswalk != nullptr);
}

Pose2D PedestrianRouteSegment::sample(
    double progressMetres) const {
    const double ratio = lengthMetres > 1e-9
        ? std::clamp(
              progressMetres / lengthMetres,
              0.0,
              1.0)
        : 0.0;
    const Vec2 tangent = normalized(
        end - start,
        {1.0, 0.0});
    return {
        lerp(start, end, ratio),
        std::atan2(tangent.y, tangent.x),
        0.0
    };
}

namespace {

PedestrianRouteSegment makeSidewalkSegment(
    Vec2 start,
    Vec2 end,
    double lengthMetres,
    const Road* road) {
    return {
        PedestrianSegmentKind::Sidewalk,
        start,
        end,
        lengthMetres,
        nullptr,
        road
    };
}

PedestrianRouteSegment makeCrosswalkSegment(
    Vec2 start,
    Vec2 end,
    Crosswalk& crosswalk) {
    return {
        PedestrianSegmentKind::Crosswalk,
        start,
        end,
        crosswalk.getCrossingLengthMetres(),
        &crosswalk,
        crosswalk.getIncomingRoad()
    };
}

} // namespace

std::vector<PedestrianRouteSegment>
buildCrosswalkJourney(
    Crosswalk& crosswalk,
    CrossingDirection direction,
    double sidewalkApproachMetres,
    double sidewalkDepartureMetres) {
    std::vector<PedestrianRouteSegment> route;
    Road* incoming = crosswalk.getIncomingRoad();
    if (incoming == nullptr ||
        !std::isfinite(sidewalkApproachMetres) ||
        !std::isfinite(sidewalkDepartureMetres) ||
        sidewalkApproachMetres <= 0.0 ||
        sidewalkDepartureMetres <= 0.0) {
        return route;
    }

    const double incomingCrossProgress =
        crosswalk.getCentreProgressMetres();
    const double incomingAwayProgress = std::max(
        0.0,
        incomingCrossProgress -
            sidewalkApproachMetres);
    const double incomingDepartureProgress = std::max(
        0.0,
        incomingCrossProgress -
            sidewalkDepartureMetres);
    const double incomingApproachLength =
        incomingCrossProgress -
        incomingAwayProgress;
    const double incomingDepartureLength =
        incomingCrossProgress -
        incomingDepartureProgress;

    const Vec2 sideA = crosswalk.getSideAPosition();
    const Vec2 sideB = crosswalk.getSideBPosition();
    Road* reverse = crosswalk.getReverseRoad();

    Vec2 sideBAway;
    Vec2 sideBDeparture;
    double sideBApproachLength = 0.0;
    double sideBDepartureLength = 0.0;
    const Road* sideBRoad = incoming;
    if (reverse != nullptr) {
        const double reverseCrossProgress =
            crosswalk.getReverseCentreProgressMetres();
        const double reverseApproachProgress = std::min(
            reverse->getDistance(),
            reverseCrossProgress +
                sidewalkApproachMetres);
        const double reverseDepartureProgress = std::min(
            reverse->getDistance(),
            reverseCrossProgress +
                sidewalkDepartureMetres);
        sideBAway = RoadGeometry::sampleSidewalk(
            *reverse,
            true,
            reverseApproachProgress);
        sideBDeparture = RoadGeometry::sampleSidewalk(
            *reverse,
            true,
            reverseDepartureProgress);
        sideBApproachLength =
            reverseApproachProgress -
            reverseCrossProgress;
        sideBDepartureLength =
            reverseDepartureProgress -
            reverseCrossProgress;
        sideBRoad = reverse;
    } else {
        sideBAway = RoadGeometry::sampleSidewalk(
            *incoming,
            false,
            incomingAwayProgress);
        sideBDeparture = RoadGeometry::sampleSidewalk(
            *incoming,
            false,
            incomingDepartureProgress);
        sideBApproachLength = incomingApproachLength;
        sideBDepartureLength = incomingDepartureLength;
    }

    if (direction == CrossingDirection::SideAToB) {
        const Vec2 start = RoadGeometry::sampleSidewalk(
            *incoming,
            true,
            incomingAwayProgress);
        route.push_back(makeSidewalkSegment(
            start,
            sideA,
            incomingApproachLength,
            incoming));
        route.push_back(makeCrosswalkSegment(
            sideA,
            sideB,
            crosswalk));
        route.push_back(makeSidewalkSegment(
            sideB,
            sideBDeparture,
            sideBDepartureLength,
            sideBRoad));
    } else {
        route.push_back(makeSidewalkSegment(
            sideBAway,
            sideB,
            sideBApproachLength,
            sideBRoad));
        route.push_back(makeCrosswalkSegment(
            sideB,
            sideA,
            crosswalk));
        const Vec2 finish = RoadGeometry::sampleSidewalk(
            *incoming,
            true,
            incomingDepartureProgress);
        route.push_back(makeSidewalkSegment(
            sideA,
            finish,
            incomingDepartureLength,
            incoming));
    }

    if (std::any_of(
            route.begin(),
            route.end(),
            [](const PedestrianRouteSegment& segment) {
                return !segment.isValid();
            })) {
        route.clear();
    }
    return route;
}
