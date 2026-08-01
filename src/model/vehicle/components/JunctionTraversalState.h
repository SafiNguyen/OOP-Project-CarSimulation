#ifndef JUNCTION_TRAVERSAL_STATE_H
#define JUNCTION_TRAVERSAL_STATE_H

#include <memory>
#include "Geometry.h"

class Vehicle;
class JunctionConnector;
class Intersection;
class Road;
class LaneMapping;

class JunctionTraversalState {
public:
    bool beginTraversal(Vehicle& vehicle, const LaneMapping& mapping, Intersection* intersection);
    void completeTraversal(Vehicle& vehicle, double outgoingProgressMetres);
    double advance(Vehicle& vehicle, double availableTime);
};

#endif
