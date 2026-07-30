#include "Roundabout.h"

#include <algorithm>
#include <cmath>

#include "JunctionConnector.h"
#include "LaneMapping.h"
#include "MotionPath.h"
#include "Road.h"
#include "RoadGeometry.h"

namespace {
constexpr double PI = 3.14159265358979323846;
}

Roundabout::Roundabout(int id,
                       double x,
                       double y,
                       double radiusMetres,
                       double roadWidthMetres)
    : Intersection(id, x, y),
      radiusMetres_(
          std::isfinite(radiusMetres) && radiusMetres > 0.0
              ? radiusMetres
              : 20.0),
      roadWidthMetres_(
          std::isfinite(roadWidthMetres) && roadWidthMetres > 0.0
              ? roadWidthMetres
              : 7.0) {
    const int practicalCapacity = static_cast<int>(std::floor(
        2.0 * PI * radiusMetres_ / 10.0));
    setCapacity(std::clamp(practicalCapacity, 2, 12));
}

std::shared_ptr<const JunctionConnector> Roundabout::createConnector(
    const Road& incoming,
    int incomingLane,
    const Road& outgoing,
    int outgoingLane) const {
    const Vec2 centre{getX(), getY()};
    const Vec2 entry =
        RoadGeometry::laneEndpoint(incoming, incomingLane, false);
    const Vec2 exit =
        RoadGeometry::laneEndpoint(outgoing, outgoingLane, true);
    const Vec2 incomingTangent =
        RoadGeometry::roadDirection(incoming);
    const Vec2 outgoingTangent =
        RoadGeometry::roadDirection(outgoing);
    const double scale =
        RoadGeometry::metresPerWorldUnit(*this);
    const double circulatingRadiusWorld =
        radiusMetres_ / std::max(scale, 1e-6);
    auto path = std::make_shared<RoundaboutTraversalPath>(
        centre,
        circulatingRadiusWorld,
        entry,
        incomingTangent,
        exit,
        outgoingTangent,
        true,
        scale,
        TurnLanePolicy::classify(
            incoming, outgoing) == MovementType::UTurn);
    const ConnectorKey key{
        incoming.getId(), incomingLane,
        outgoing.getId(), outgoingLane
    };
    return std::make_shared<JunctionConnector>(
        key,
        &incoming,
        &outgoing,
        TurnLanePolicy::classify(incoming, outgoing),
        incomingTangent,
        outgoingTangent,
        RoadGeometry::metresPerWorldUnit(outgoing),
        std::move(path));
}

bool Roundabout::canEnterMovement(
    int vehicleId,
    const std::shared_ptr<const JunctionConnector>& connector,
    double requiredGapMetres,
    double vehicleLengthMetres,
    double vehicleWidthMetres) const {
    if (occupants_.count(vehicleId) > 0) return true;
    if (connector == nullptr ||
        static_cast<int>(occupants_.size()) >= getCapacity()) {
        return false;
    }

    const Pose2D mergePose = connector->sampleByDistance(0.0);
    const double scale =
        RoadGeometry::metresPerWorldUnit(*this);
    const double enteringLength =
        std::max(0.1, vehicleLengthMetres);
    const double enteringWidth =
        std::max(0.1, vehicleWidthMetres);
    for (const auto& entry : occupants_) {
        const Reservation& reservation = entry.second;
        if (reservation.connector == nullptr) {
            return false;
        }
        const Pose2D circulatingPose =
            reservation.connector->sampleByDistance(
                reservation.progressMetres);
        const double separationMetres =
            distance(mergePose.position,
                     circulatingPose.position) * scale;
        const double longitudinalSpacing =
            (enteringLength +
             reservation.vehicleLengthMetres) * 0.5 +
            std::max(2.0, requiredGapMetres);
        const double enteringRadius = std::hypot(
            enteringLength * 0.5, enteringWidth * 0.5);
        const double circulatingRadius = std::hypot(
            reservation.vehicleLengthMetres * 0.5,
            reservation.vehicleWidthMetres * 0.5);
        const double safeGap = std::max(
            longitudinalSpacing,
            enteringRadius + circulatingRadius + 0.5);
        if (separationMetres < safeGap) {
            return false;
        }
    }
    return true;
}
