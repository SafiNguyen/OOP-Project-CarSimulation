#include "LaneMapping.h"

#include <algorithm>
#include <cmath>

#include "Intersection.h"
#include "Road.h"
#include "Geometry.h"

namespace {

Vec2 roadDirection(const Road& road) {
    if (road.getStart() == nullptr || road.getEnd() == nullptr) {
        return {1.0, 0.0};
    }
    return normalized({
        road.getEnd()->getX() - road.getStart()->getX(),
        road.getEnd()->getY() - road.getStart()->getY()
    });
}

int nearestOpenLane(const Road& road, int preferred) {
    preferred = std::clamp(preferred, 0, road.getLaneCount() - 1);
    if (!road.getLane(preferred).isBlocked()) {
        return preferred;
    }
    for (int offset = 1; offset < road.getLaneCount(); ++offset) {
        const int lower = preferred - offset;
        const int upper = preferred + offset;
        if (lower >= 0 && !road.getLane(lower).isBlocked()) return lower;
        if (upper < road.getLaneCount() &&
            !road.getLane(upper).isBlocked()) return upper;
    }
    return -1;
}

} // namespace

MovementType TurnLanePolicy::classify(const Road& incoming,
                                      const Road& outgoing) {
    const Vec2 from = roadDirection(incoming);
    const Vec2 to = roadDirection(outgoing);
    const double cosine = std::clamp(dot(from, to), -1.0, 1.0);
    const double angle = std::acos(cosine);
    if (angle <= STRAIGHT_THRESHOLD_RADIANS) {
        return MovementType::Straight;
    }
    if (angle >= UTURN_THRESHOLD_RADIANS) {
        return MovementType::UTurn;
    }
    // Model coordinates use +Y upward. A positive cross product is a
    // counter-clockwise (left) turn; SFML's inverted screen Y is irrelevant.
    return cross(from, to) > 0.0
        ? MovementType::Left
        : MovementType::Right;
}

LaneMapping TurnLanePolicy::map(const Road& incoming,
                                int currentIncomingLane,
                                const Road& outgoing,
                                bool allowUTurn) {
    LaneMapping result;
    if (incoming.getLaneCount() <= 0 || outgoing.getLaneCount() <= 0 ||
        incoming.getEnd() == nullptr || outgoing.getStart() == nullptr ||
        incoming.getEnd() != outgoing.getStart()) {
        return result;
    }

    result.movement = classify(incoming, outgoing);
    if (result.movement == MovementType::UTurn && !allowUTurn) {
        return result;
    }

    const int incomingCount = incoming.getLaneCount();
    const int outgoingCount = outgoing.getLaneCount();
    currentIncomingLane =
        std::clamp(currentIncomingLane, 0, incomingCount - 1);

    switch (result.movement) {
        case MovementType::Right:
            result.incomingLane = incoming.getCurbLaneIndex();
            result.outgoingLane = outgoing.getCurbLaneIndex();
            break;
        case MovementType::Left:
        case MovementType::UTurn:
            result.incomingLane = 0;
            result.outgoingLane = 0;
            break;
        case MovementType::Straight: {
            result.incomingLane = currentIncomingLane;
            const double normalizedLane = incomingCount > 1
                ? static_cast<double>(currentIncomingLane) /
                      static_cast<double>(incomingCount - 1)
                : 0.0;
            result.outgoingLane = static_cast<int>(std::lround(
                normalizedLane * static_cast<double>(outgoingCount - 1)));
            break;
        }
    }

    result.incomingLane =
        nearestOpenLane(incoming, result.incomingLane);
    result.outgoingLane =
        nearestOpenLane(outgoing, result.outgoingLane);
    result.valid =
        result.incomingLane >= 0 && result.outgoingLane >= 0;
    return result;
}

LaneMapping TurnLanePolicy::mapFromCurrentLane(
    const Road& incoming,
    int currentIncomingLane,
    const Road& outgoing,
    bool allowUTurn) {
    LaneMapping result;
    if (incoming.getLaneCount() <= 0 ||
        outgoing.getLaneCount() <= 0 ||
        incoming.getEnd() == nullptr ||
        outgoing.getStart() == nullptr ||
        incoming.getEnd() != outgoing.getStart()) {
        return result;
    }

    result.movement = classify(incoming, outgoing);
    if (result.movement == MovementType::UTurn && !allowUTurn) {
        return result;
    }

    result.incomingLane =
        nearestOpenLane(incoming, currentIncomingLane);
    if (result.incomingLane < 0) {
        return result;
    }

    const int incomingCount = incoming.getLaneCount();
    const int outgoingCount = outgoing.getLaneCount();
    int preferredOutgoingLane = 0;
    if (incomingCount == 1) {
        preferredOutgoingLane =
            result.movement == MovementType::Right
                ? outgoing.getCurbLaneIndex()
                : 0;
    } else {
        const double normalizedLane =
            static_cast<double>(result.incomingLane) /
            static_cast<double>(incomingCount - 1);
        preferredOutgoingLane =
            static_cast<int>(std::lround(
                normalizedLane *
                static_cast<double>(outgoingCount - 1)));
    }

    result.outgoingLane =
        nearestOpenLane(outgoing, preferredOutgoingLane);
    result.valid = result.outgoingLane >= 0;
    return result;
}
