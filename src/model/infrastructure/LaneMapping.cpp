#include "LaneMapping.h"

#include <algorithm>
#include <cmath>

#include "Intersection.h"
#include "Road.h"
#include "Geometry.h"
#include "Vehicle.h"

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

void applyDynamicLaneSelection(
    const Road& incoming,
    const Road& outgoing,
    bool isDestinationRoad,
    LaneMapping& result) {
    
    int outgoingCount = outgoing.getLaneCount();
    if (outgoingCount <= 1 || result.outgoingLane < 0) return;
    
    // Feature: Priority lanes for Destination Road
    int minAllowedLane = 0;
    if (isDestinationRoad && outgoingCount > 2) {
        minAllowedLane = 1; // Forbid lane 0 when heading to a POI on a road with > 2 lanes
        // Force the preferred lane to be curb lane if it's currently 0
        if (result.outgoingLane < minAllowedLane) {
            result.outgoingLane = std::max(minAllowedLane, outgoing.getCurbLaneIndex());
            result.outgoingLane = nearestOpenLane(outgoing, result.outgoingLane);
        }
    }

    Intersection* junction = incoming.getEnd();
    
    auto getLaneCongestion = [&](int lane) -> double {
        if (junction != nullptr && junction->isOutgoingLaneReserved(&outgoing, lane)) {
            return 0.0; // fully congested if someone in the intersection reserved it
        }
        const Vehicle* leader = outgoing.getFirstVehicleInLane(lane);
        return leader ? leader->getProgressOnRoad() : 9999.0;
    };

    double gapCurrent = getLaneCongestion(result.outgoingLane);
    
    // Dynamic selection for ALL movements (including Straight)
    
    // Only change lane dynamically if the current mapped lane is congested (gap < 40.0)
    if (gapCurrent < 40.0) {
        int bestLane = result.outgoingLane;
        double bestGap = gapCurrent;
        
        const int candidateLanes[2] = { result.outgoingLane - 1, result.outgoingLane + 1 };
        for (int altLane : candidateLanes) {
            if (altLane >= minAllowedLane && altLane < outgoingCount && !outgoing.getLane(altLane).isBlocked()) {
                double gapAlt = getLaneCongestion(altLane);
                
                // If the alternative lane is significantly more empty
                if (gapAlt > bestGap + 15.0) {
                    bestGap = gapAlt;
                    bestLane = altLane;
                }
            }
        }
        result.outgoingLane = bestLane;
    }
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
                                bool allowUTurn,
                                bool isDestinationRoad) {
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
        case MovementType::Right: {
            int rightmost = incomingCount - 1;
            int outRightmost = outgoingCount - 1;
            if (currentIncomingLane >= rightmost - 1) {
                result.incomingLane = currentIncomingLane;
            } else {
                result.incomingLane = std::max(0, rightmost - 1);
            }
            if (result.incomingLane == rightmost) {
                result.outgoingLane = outRightmost;
            } else {
                result.outgoingLane = std::max(0, outRightmost - 1);
            }
            break;
        }
        case MovementType::Left:
        case MovementType::UTurn: {
            if (currentIncomingLane <= 1) {
                result.incomingLane = currentIncomingLane;
            } else {
                result.incomingLane = std::min(1, incomingCount - 1);
            }
            if (result.incomingLane == 0) {
                result.outgoingLane = 0;
            } else {
                result.outgoingLane = std::min(1, outgoingCount - 1);
            }
            break;
        }
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

    applyDynamicLaneSelection(incoming, outgoing, isDestinationRoad, result);

    result.valid =
        result.incomingLane >= 0 && result.outgoingLane >= 0;
    return result;
}

LaneMapping TurnLanePolicy::mapFromCurrentLane(
    const Road& incoming,
    int currentIncomingLane,
    const Road& outgoing,
    bool allowUTurn,
    bool isDestinationRoad) {
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
    
    switch (result.movement) {
        case MovementType::Right: {
            int rightmost = incomingCount - 1;
            int outRightmost = outgoingCount - 1;
            int offsetFromRight = rightmost - result.incomingLane;
            preferredOutgoingLane = std::max(0, outRightmost - offsetFromRight);
            break;
        }
        case MovementType::Left:
        case MovementType::UTurn: {
            preferredOutgoingLane = std::min(result.incomingLane, outgoingCount - 1);
            break;
        }
        case MovementType::Straight: {
            if (incomingCount == 1) {
                preferredOutgoingLane = 0;
            } else {
                const double normalizedLane =
                    static_cast<double>(result.incomingLane) /
                    static_cast<double>(incomingCount - 1);
                preferredOutgoingLane =
                    static_cast<int>(std::lround(
                        normalizedLane *
                        static_cast<double>(outgoingCount - 1)));
            }
            break;
        }
    }

    result.outgoingLane =
        nearestOpenLane(outgoing, preferredOutgoingLane);

    applyDynamicLaneSelection(incoming, outgoing, isDestinationRoad, result);

    result.valid = result.outgoingLane >= 0;
    return result;
}

