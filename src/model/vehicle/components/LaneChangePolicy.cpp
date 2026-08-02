#include "LaneChangePolicy.h"
#include "Vehicle.h"
#include "Road.h"
#include "LaneMapping.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
double combinedHalfLength(const Vehicle& first, const Vehicle& second) {
    return (first.getLength() + second.getLength()) * 0.5;
}
double bumperGap(const Vehicle& follower, const Vehicle& leader) {
    return leader.getProgressOnRoad() - follower.getProgressOnRoad() - combinedHalfLength(follower, leader);
}
double gapAheadOf(const Vehicle& vehicle, const Vehicle* leader) {
    if (leader == nullptr) {
        return std::numeric_limits<double>::infinity();
    }
    return bumperGap(vehicle, *leader);
}
double gapBehindOf(const Vehicle& vehicle, const Vehicle* follower) {
    if (follower == nullptr) {
        return std::numeric_limits<double>::infinity();
    }
    return bumperGap(*follower, vehicle);
}
}

LaneChangeCandidate LaneChangePolicy::assess(Vehicle& vehicle, Road& road, int laneIndex, double rearSafetyTime, double minTimeToCollision) const {
    LaneChangeCandidate result;
    result.laneIndex = laneIndex;

    if (laneIndex < 0 || laneIndex >= road.getLaneCount() || road.getLane(laneIndex).isBlocked()) {
        return result;
    }

    Vehicle* leader = road.findLeader(laneIndex, &vehicle);
    result.gapAhead = gapAheadOf(vehicle, leader);
    if (leader != nullptr) {
        const double closingSpeed = std::max(0.0, vehicle.getCurrentSpeed() - leader->getCurrentSpeed());
        const double baseGap = std::max(vehicle.getMinGap(), leader->getMinGap());
        const double requiredGap = baseGap + vehicle.getCurrentSpeed() * Vehicle::LANE_CHANGE_REACTION_TIME + closingSpeed * Vehicle::LANE_CHANGE_FRONT_SAFETY_TIME;
        if (result.gapAhead <= requiredGap) {
            return result;
        }
        if (closingSpeed > 1e-6 && result.gapAhead / closingSpeed < minTimeToCollision) {
            return result;
        }
    }

    Vehicle* follower = road.findFollower(laneIndex, &vehicle);
    result.gapBehind = gapBehindOf(vehicle, follower);
    if (follower != nullptr) {
        const double closingSpeed = std::max(0.0, follower->getCurrentSpeed() - vehicle.getCurrentSpeed());
        const double baseGap = std::max(vehicle.getMinGap(), follower->getMinGap());
        const double requiredGap = baseGap + follower->getCurrentSpeed() * Vehicle::LANE_CHANGE_REACTION_TIME + closingSpeed * rearSafetyTime;
        if (result.gapBehind < requiredGap) {
            return result;
        }
        if (closingSpeed > 1e-6 && result.gapBehind / closingSpeed < minTimeToCollision) {
            return result;
        }
    }

    result.safe = true;
    return result;
}

bool LaneChangePolicy::isBetter(const LaneChangeCandidate& candidate, const LaneChangeCandidate& best) const {
    if (!candidate.safe) return false;
    if (!best.safe) return true;
    if (candidate.gapAhead != best.gapAhead) {
        return candidate.gapAhead > best.gapAhead;
    }
    return candidate.gapBehind > best.gapBehind;
}

bool LaneChangePolicy::usesDedicatedEdgeLane(int movement) const {
    return movement == static_cast<int>(MovementType::Right) ||
           movement == static_cast<int>(MovementType::Left) ||
           movement == static_cast<int>(MovementType::UTurn);
}
