#pragma once
#include "Vehicle.h"
#include "Road.h"
#include "components/LaneChangePolicy.h"
#include <limits>
#include <cmath>

namespace VehicleMath {

inline double combinedHalfLength(const Vehicle& first,
                          const Vehicle& second) {
    return (first.getLength() + second.getLength()) * 0.5;
}

inline double bumperGap(const Vehicle& follower,
                 const Vehicle& leader) {
    return leader.getProgressOnRoad()
         - follower.getProgressOnRoad()
         - combinedHalfLength(follower, leader);
}

inline double outgoingReleaseGap(const Vehicle& entering,
                          const Vehicle& leader) {
    // A connector remains reserved until the entering vehicle's rear bumper
    // clears the junction boundary. At release, its centre is half its own
    // length down the outgoing road, which must be included in the headroom.
    return leader.getProgressOnRoad()
         - entering.getLength() * 0.5
         - combinedHalfLength(entering, leader);
}

inline double gapAheadOf(const Vehicle& vehicle, const Vehicle* leader) {
    if (leader == nullptr) {
        return std::numeric_limits<double>::infinity();
    }
    return bumperGap(vehicle, *leader);
}

inline double gapBehindOf(const Vehicle& vehicle, const Vehicle* follower) {
    if (follower == nullptr) {
        return std::numeric_limits<double>::infinity();
    }
    return bumperGap(*follower, vehicle);
}

inline LaneChangeCandidate assessLaneChange(const Vehicle& vehicle,
                                     Road& road,
                                     int laneIndex,
                                     double rearSafetyTime,
                                     double minTimeToCollision) {
    LaneChangeCandidate result;
    result.laneIndex = laneIndex;

    if (laneIndex < 0 || laneIndex >= road.getLaneCount()
        || road.getLane(laneIndex).isBlocked()) {
        return result;
    }

    Vehicle* leader = road.findLeader(laneIndex, &vehicle);
    result.gapAhead = gapAheadOf(vehicle, leader);
    if (leader != nullptr) {
        const double closingSpeed = std::max(
            0.0, vehicle.getCurrentSpeed() - leader->getCurrentSpeed());
        const double baseGap = std::max(vehicle.getMinGap(), leader->getMinGap());
        const double requiredGap = baseGap
                                 + vehicle.getCurrentSpeed() * Vehicle::LANE_CHANGE_REACTION_TIME
                                 + closingSpeed * Vehicle::LANE_CHANGE_FRONT_SAFETY_TIME;
        if (result.gapAhead <= requiredGap) {
            return result;
        }
        if (closingSpeed > 1e-6
            && result.gapAhead / closingSpeed < minTimeToCollision) {
            return result;
        }
    }

    Vehicle* follower = road.findFollower(laneIndex, &vehicle);
    result.gapBehind = gapBehindOf(vehicle, follower);
    if (follower != nullptr) {
        const double closingSpeed = std::max(
            0.0, follower->getCurrentSpeed() - vehicle.getCurrentSpeed());
        const double baseGap = std::max(vehicle.getMinGap(), follower->getMinGap());
        const double requiredGap = baseGap
                                 + follower->getCurrentSpeed() * Vehicle::LANE_CHANGE_REACTION_TIME
                                 + closingSpeed * rearSafetyTime;
        if (result.gapBehind < requiredGap) {
            return result;
        }
        if (closingSpeed > 1e-6
            && result.gapBehind / closingSpeed < minTimeToCollision) {
            return result;
        }
    }

    result.safe = true;
    return result;
}

inline bool isBetterCandidate(const LaneChangeCandidate& candidate,
                       const LaneChangeCandidate& best) {
    if (!candidate.safe) return false;
    if (!best.safe) return true;
    if (candidate.gapAhead != best.gapAhead) {
        return candidate.gapAhead > best.gapAhead;
    }
    return candidate.gapBehind > best.gapBehind;
}

inline double smoothStep(double value) {
    const double ratio = std::clamp(value, 0.0, 1.0);
    return ratio * ratio * (3.0 - 2.0 * ratio);
}

inline double laneChangeLeanDirection(int targetLaneIndex, int currentLaneIndex) {
    if (targetLaneIndex == currentLaneIndex) {
        return 0.0;
    }
    return targetLaneIndex > currentLaneIndex ? 1.0 : -1.0;
}

inline double laneChangeLeanMagnitude(double progressRatio) {
    constexpr double PI = 3.14159265358979323846;
    constexpr double MAX_LANE_CHANGE_TILT_RADIANS = 0.12;
    return MAX_LANE_CHANGE_TILT_RADIANS * std::sin(PI * std::clamp(progressRatio, 0.0, 1.0));
}

constexpr double POI_MERGE_YIELD_BUFFER_METRES = 0.5;
constexpr double MIN_POI_MERGE_PHASE_SECONDS = 0.25;
constexpr double RED_LIGHT_CURB_YIELD_DISTANCE_METRES = 45.0;
constexpr double LANE_CHANGE_POSE_TRANSITION_SECONDS = 0.35;
constexpr double UTURN_POSE_TRANSITION_SECONDS = 0.7;

}
