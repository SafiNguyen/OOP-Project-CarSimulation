#ifndef ROAD_GEOMETRY_H
#define ROAD_GEOMETRY_H

#include "Geometry.h"

class Intersection;
class Road;

namespace RoadGeometry {

struct RoadAccessPath {
    Vec2 source;
    Vec2 corner;
    Vec2 curb;
    Pose2D lanePose;
};

constexpr double LANE_WIDTH_METRES = 3.5;
constexpr double MEDIAN_GAP_METRES = 0.5;
constexpr double STOP_LINE_SETBACK_METRES = 1.5;
constexpr double SIDEWALK_WIDTH_METRES = 5.0;
constexpr double SIDEWALK_CURB_WIDTH_METRES = 0.35;

double metresPerWorldUnit(const Road& road);
double metresPerWorldUnit(const Intersection& intersection);
double junctionBoundaryRadiusMetres(const Intersection& intersection);
double junctionBoundaryRadiusWorld(const Intersection& intersection);
bool hasReverseDirection(const Road& road);
double carriagewayWidthMetres(const Road& road);

Vec2 roadDirection(const Road& road);
Vec2 roadReferenceEndpoint(const Road& road, bool atStart);
Vec2 roadSurfaceEndpoint(const Road& road, bool atStart);
Vec2 laneEndpoint(const Road& road, int laneIndex, bool atStart);
Vec2 laneBoundaryEndpoint(const Road& road,
                          int boundaryIndex,
                          bool atStart);
Vec2 roadEdgeEndpoint(const Road& road,
                      bool rightEdge,
                      bool atStart);
Vec2 sampleRoadEdge(const Road& road,
                    bool rightEdge,
                    double progressMetres);
Vec2 sampleSidewalk(const Road& road,
                    bool rightSide,
                    double progressMetres);
Vec2 sampleRoadSurface(const Road& road, double progressMetres);
Pose2D sampleLane(const Road& road,
                  int laneIndex,
                  double progressMetres);
RoadAccessPath makeRoadAccessPath(
    const Road& road,
    int laneIndex,
    double progressMetres,
    Vec2 source);
Pose2D sampleRoadAccessPath(
    const RoadAccessPath& path,
    double normalizedProgress);
double roadAccessPathProgressAt(
    const RoadAccessPath& path,
    Vec2 point);
double stopLineProgressMetres(const Road& incomingRoad);
Vec2 stopLineCentre(const Road& incomingRoad);

} // namespace RoadGeometry

#endif
