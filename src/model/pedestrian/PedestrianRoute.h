#ifndef PEDESTRIAN_ROUTE_H
#define PEDESTRIAN_ROUTE_H

#include <vector>

#include "Geometry.h"

class Crosswalk;
class Road;

enum class PedestrianSegmentKind {
    Sidewalk,
    Crosswalk
};

enum class CrossingDirection {
    SideAToB,
    SideBToA
};

// Lightweight route value. Crosswalk and Road references are non-owning.
struct PedestrianRouteSegment {
    PedestrianSegmentKind kind =
        PedestrianSegmentKind::Sidewalk;
    Vec2 start;
    Vec2 end;
    double lengthMetres = 0.0;
    Crosswalk* crosswalk = nullptr;
    const Road* referenceRoad = nullptr;

    bool isValid() const;
    Pose2D sample(double progressMetres) const;
};

std::vector<PedestrianRouteSegment>
buildCrosswalkJourney(
    Crosswalk& crosswalk,
    CrossingDirection direction,
    double sidewalkApproachMetres = 15.0,
    double sidewalkDepartureMetres = 15.0);

#endif
