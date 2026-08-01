#include "CarFollowingModel.h"
#include "Vehicle.h"
#include "Road.h"
#include <algorithm>
#include <limits>

namespace {
double combinedHalfLength(const Vehicle& first, const Vehicle& second) {
    return (first.getLength() + second.getLength()) * 0.5;
}

double bumperGap(const Vehicle& follower, const Vehicle& leader) {
    return leader.getProgressOnRoad() - follower.getProgressOnRoad() - combinedHalfLength(follower, leader);
}
}

CarFollowingModel::FollowDecision CarFollowingModel::evaluate(Vehicle& vehicle, Road* road, double freeFlowSpeed, double minGap, double desiredGap) {
    FollowDecision decision;
    decision.targetSpeed = freeFlowSpeed;
    decision.hasLeader = false;
    decision.gapToLeader = std::numeric_limits<double>::infinity();

    if (road == nullptr) {
        return decision;
    }

    Vehicle* leader = road->findLeader(vehicle.getCurrentLaneIndex(), &vehicle);
    if (leader != nullptr) {
        decision.hasLeader = true;
        decision.gapToLeader = bumperGap(vehicle, *leader);
        decision.targetSpeed = std::min(decision.targetSpeed, freeFlowSpeed);

        if (decision.gapToLeader <= minGap) {
            decision.targetSpeed = 0.0;
        } else {
            const double leaderTargetSpeed = freeFlowSpeed * (decision.gapToLeader - minGap) / std::max(desiredGap - minGap, 1e-6);
            decision.targetSpeed = std::min(decision.targetSpeed, leaderTargetSpeed);
        }

        const double stoppingDistance = (vehicle.getCurrentSpeed() * vehicle.getCurrentSpeed()) / (2.0 * std::max(vehicle.getDeceleration(), 1e-6));
        if (decision.gapToLeader - minGap < stoppingDistance) {
            decision.targetSpeed = 0.0;
        }
    }

    return decision;
}
