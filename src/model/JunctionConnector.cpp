#include "JunctionConnector.h"

#include <algorithm>
#include <cmath>
#include <functional>

#include "MotionPath.h"

std::size_t ConnectorKeyHash::operator()(
    const ConnectorKey& key) const noexcept {
    std::size_t seed = 0;
    const auto combine = [&seed](int value) {
        seed ^= std::hash<int>{}(value) +
                0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    combine(key.incomingRoadId);
    combine(key.incomingLane);
    combine(key.outgoingRoadId);
    combine(key.outgoingLane);
    return seed;
}

JunctionConnector::JunctionConnector(
    ConnectorKey key,
    const Road* incomingRoad,
    const Road* outgoingRoad,
    MovementType movement,
    Vec2 startTangent,
    Vec2 endTangent,
    double outgoingMetresPerWorldUnit,
    std::shared_ptr<const MotionPath> path)
    : key_(key),
      incomingRoad_(incomingRoad),
      outgoingRoad_(outgoingRoad),
      movement_(movement),
      startTangent_(normalized(startTangent)),
      endTangent_(normalized(endTangent)),
      outgoingMetresPerWorldUnit_(
          std::max(outgoingMetresPerWorldUnit, 1e-6)),
      path_(std::move(path)) {
}

double JunctionConnector::getLength() const {
    return path_ != nullptr ? path_->getLength() : 0.0;
}

Pose2D JunctionConnector::sampleByDistance(
    double distanceMetres) const {
    if (path_ == nullptr) return {};
    const double lengthMetres = getLength();
    if (distanceMetres <= lengthMetres) {
        return path_->sampleByDistance(distanceMetres);
    }

    Pose2D pose = path_->sampleByDistance(lengthMetres);
    const double extraMetres = distanceMetres - lengthMetres;
    pose.position = pose.position +
        endTangent_ *
            (extraMetres / outgoingMetresPerWorldUnit_);
    pose.headingRadians =
        std::atan2(endTangent_.y, endTangent_.x);
    pose.curvature = 0.0;
    return pose;
}
