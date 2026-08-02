#ifndef LANE_CHANGE_POLICY_H
#define LANE_CHANGE_POLICY_H

#include <limits>

class Vehicle;
class Road;

struct LaneChangeCandidate {
    int laneIndex = -1;
    double gapAhead = -std::numeric_limits<double>::infinity();
    double gapBehind = -std::numeric_limits<double>::infinity();
    bool safe = false;
};

class LaneChangePolicy {
public:
    LaneChangeCandidate assess(Vehicle& vehicle, Road& road, int laneIndex, double rearSafetyTime, double minTimeToCollision) const;
    bool isBetter(const LaneChangeCandidate& candidate, const LaneChangeCandidate& best) const;
    bool usesDedicatedEdgeLane(int movement) const;
};

#endif
