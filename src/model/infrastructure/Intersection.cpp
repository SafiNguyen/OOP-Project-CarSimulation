#include "Intersection.h"
#include "Road.h" 
#include "TrafficLight.h"
#include "LaneMapping.h"
#include "MotionPath.h"
#include "RoadGeometry.h"
#include "Vehicle.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace {
constexpr double PI = 3.14159265358979323846;
// How far from exactly opposite (180 degrees) two approaches may be and
// still be considered "the same through movement" for phase grouping.
// Kept tight on purpose: two approaches must be genuinely a straight-through
// pair (e.g. North vs South) to safely share a green phase. A wider
// tolerance (the old value was 45 degrees) can merge two approaches that
// are only ~135 degrees apart into a single "safe" group - i.e. two roads
// that actually cross paths inside the intersection end up green at the
// same time. Worse, if an intersection only has two incoming roads and
// they get merged this way, rebuildPhaseGroups() produces a single phase
// group, so that intersection's light never has anything to hand off to
// and stays GREEN/YELLOW forever - it can never turn RED, so vehicles on
// either approach never get stopped at all.
constexpr double OPPOSITE_TOLERANCE_RAD = PI / 9.0; // 20 degrees
constexpr double RIGHT_ON_RED_PRIORITY_LOOKAHEAD_METRES = 45.0;
constexpr double RIGHT_ON_RED_PRIORITY_TIME_SECONDS = 4.0;

struct OrientedVehicleBounds {
    Vec2 centreMetres;
    Vec2 forward;
    Vec2 side;
    double halfLengthMetres = 0.0;
    double halfWidthMetres = 0.0;
};

OrientedVehicleBounds makeVehicleBounds(
    const Pose2D& pose,
    double metresPerWorldUnit,
    double lengthMetres,
    double widthMetres,
    double clearanceMetres) {
    const Vec2 forward{
        std::cos(pose.headingRadians),
        std::sin(pose.headingRadians)
    };
    const double padding =
        std::max(0.0, clearanceMetres) * 0.5;
    return {
        pose.position * metresPerWorldUnit,
        forward,
        rightNormal(forward),
        std::max(0.1, lengthMetres) * 0.5 + padding,
        std::max(0.1, widthMetres) * 0.5 + padding
    };
}

bool boundsOverlap(const OrientedVehicleBounds& first,
                   const OrientedVehicleBounds& second) {
    const Vec2 delta =
        second.centreMetres - first.centreMetres;
    const double firstHorizontalExtent =
        first.halfLengthMetres * std::fabs(first.forward.x) +
        first.halfWidthMetres * std::fabs(first.side.x);
    const double firstVerticalExtent =
        first.halfLengthMetres * std::fabs(first.forward.y) +
        first.halfWidthMetres * std::fabs(first.side.y);
    const double secondHorizontalExtent =
        second.halfLengthMetres * std::fabs(second.forward.x) +
        second.halfWidthMetres * std::fabs(second.side.x);
    const double secondVerticalExtent =
        second.halfLengthMetres * std::fabs(second.forward.y) +
        second.halfWidthMetres * std::fabs(second.side.y);
    if (std::fabs(delta.x) >=
            firstHorizontalExtent +
                secondHorizontalExtent + 1e-9 ||
        std::fabs(delta.y) >=
            firstVerticalExtent +
                secondVerticalExtent + 1e-9) {
        return false;
    }

    const Vec2 axes[] = {
        first.forward,
        first.side,
        second.forward,
        second.side
    };
    for (const Vec2& axis : axes) {
        const double firstRadius =
            first.halfLengthMetres *
                std::fabs(dot(first.forward, axis)) +
            first.halfWidthMetres *
                std::fabs(dot(first.side, axis));
        const double secondRadius =
            second.halfLengthMetres *
                std::fabs(dot(second.forward, axis)) +
            second.halfWidthMetres *
                std::fabs(dot(second.side, axis));
        if (std::fabs(dot(delta, axis)) + 1e-9 >=
            firstRadius + secondRadius) {
            return false;
        }
    }
    return true;
}

bool connectorsPreserveLaneOrder(
    const JunctionConnector& first,
    const JunctionConnector& second) {
    if (first.getIncomingRoad() != second.getIncomingRoad() ||
        first.getOutgoingRoad() != second.getOutgoingRoad() ||
        first.getMovementType() != second.getMovementType() ||
        first.getIncomingLane() == second.getIncomingLane() ||
        first.getOutgoingLane() == second.getOutgoingLane()) {
        return false;
    }

    const int incomingOrder =
        first.getIncomingLane() - second.getIncomingLane();
    const int outgoingOrder =
        first.getOutgoingLane() - second.getOutgoingLane();
    return (incomingOrder < 0 && outgoingOrder < 0) ||
           (incomingOrder > 0 && outgoingOrder > 0);
}

bool oppositeApproachMovementsAreNonCrossing(
    const JunctionConnector& first,
    const JunctionConnector& second) {
    if (first.getIncomingRoad() == second.getIncomingRoad()) {
        return false;
    }
    const auto mayCrossCentre = [](MovementType movement) {
        return movement == MovementType::Left ||
               movement == MovementType::UTurn;
    };
    return !mayCrossCentre(first.getMovementType()) &&
           !mayCrossCentre(second.getMovementType());
}
}

Intersection::Intersection(int id, double x, double y)
    : id(id), x(x), y(y) {}
//getter 
int Intersection::getId() const {
    return id;
}

double Intersection::getX() const {
    return x;
}

double Intersection::getY() const {
    return y;
}

const std::vector<Road*>& Intersection::getIncomingRoads() const {
    return incomingRoads;
}

const std::vector<Road*>& Intersection::getOutgoingRoads() const {
    return outgoingRoads;
}

IntersectionType Intersection::getIntersectionType() const {
    const std::size_t approaches = getApproachCount();
    if (approaches <= 2) return IntersectionType::PASS_THROUGH;
    if (approaches == 3) return IntersectionType::THREE_WAY;
    if (approaches == 4) return IntersectionType::FOUR_WAY;
    return IntersectionType::COMPLEX;
}

std::size_t Intersection::getApproachCount() const {
    std::unordered_set<int> adjacentIntersectionIds;
    const auto collect = [this, &adjacentIntersectionIds](
                             const std::vector<Road*>& roads) {
        for (const Road* road : roads) {
            if (road == nullptr) continue;
            const Intersection* other =
                road->getStart() == this
                    ? road->getEnd()
                    : road->getStart();
            if (other != nullptr && other != this) {
                adjacentIntersectionIds.insert(other->getId());
            }
        }
    };
    collect(incomingRoads);
    collect(outgoingRoads);
    return adjacentIntersectionIds.size();
}

std::string Intersection::getIntersectionTypeLabel() const {
    switch (getIntersectionType()) {
        case IntersectionType::PASS_THROUGH: return "Pass-through";
        case IntersectionType::THREE_WAY:    return "3-way (nga ba)";
        case IntersectionType::FOUR_WAY:     return "4-way (nga tu)";
        case IntersectionType::COMPLEX:      return "Complex junction";
    }
    return "Unknown";
}

//method: add road 
void Intersection::addIncomingRoad(Road* road) {
    if (road!= nullptr) {
        incomingRoads.push_back(road);
        // One non-conflicting movement per lane may enter concurrently.
        // Connector overlap checks remain the final safety gate.
        capacity_ = std::max(capacity_, road->getLaneCount());
        invalidateConnectorCache();
        // Traffic lights are NOT auto-registered. Use registerIncomingLight()
        // explicitly or through the debug console to add lights.
    }
}

void Intersection::addOutgoingRoad(Road* road) {
    if (road != nullptr) {
        outgoingRoads.push_back(road);
        invalidateConnectorCache();
    }
}

//method: remove road
void Intersection::removeIncomingRoad(Road* road) {
    if (road == nullptr) return;
    incomingRoads.erase(std::remove(incomingRoads.begin(),
                        incomingRoads.end(), road), incomingRoads.end());
    trafficLights.erase(road->getId());
    rebuildPhaseGroups();
    invalidateConnectorCache();
}

void Intersection::removeOutgoingRoad(Road* road) {
    outgoingRoads.erase(std::remove(outgoingRoads.begin(),
                        outgoingRoads.end(), road), outgoingRoads.end());
    invalidateConnectorCache();
}

//Traffic light management 
 
void Intersection::registerIncomingLight(Road* road) {
    if (road == nullptr || road->getEnd() != this) return;

    const int roadId = road->getId();
    if (trafficLights.find(roadId) != trafficLights.end()) {
        return;
    }
    trafficLights[roadId] = std::make_unique<TrafficLight>(
        roadId,
        greenDurationSeconds_,
        yellowDurationSeconds_,
        greenDurationSeconds_ + yellowDurationSeconds_ +
            allRedDurationSeconds_,
        LightState::RED);
    rebuildPhaseGroups();
}

bool Intersection::configureTrafficSignals(
    const std::vector<std::vector<Road*>>& phases,
    double greenDuration,
    double yellowDuration,
    double allRedDuration,
    std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) *error = message;
        return false;
    };
    if (phases.empty()) {
        return fail("signal plan must contain at least one phase.");
    }
    if (!std::isfinite(greenDuration) || greenDuration <= 0.0 ||
        !std::isfinite(yellowDuration) || yellowDuration <= 0.0 ||
        !std::isfinite(allRedDuration) || allRedDuration <= 0.0) {
        return fail(
            "green, yellow and all-red durations must be positive.");
    }

    std::unordered_set<int> configuredRoadIds;
    for (std::size_t phaseIndex = 0;
         phaseIndex < phases.size();
         ++phaseIndex) {
        if (phases[phaseIndex].empty()) {
            return fail(
                "phase " + std::to_string(phaseIndex) +
                " has no incoming roads.");
        }
        for (Road* road : phases[phaseIndex]) {
            if (road == nullptr || road->getEnd() != this ||
                std::find(incomingRoads.begin(),
                          incomingRoads.end(),
                          road) == incomingRoads.end()) {
                return fail(
                    "phase " + std::to_string(phaseIndex) +
                    " references a road that is not incoming.");
            }
            if (!configuredRoadIds.insert(road->getId()).second) {
                return fail(
                    "incoming road " + std::to_string(road->getId()) +
                    " appears in more than one phase.");
            }
        }
    }
    for (const Road* road : incomingRoads) {
        if (road != nullptr &&
            configuredRoadIds.count(road->getId()) == 0) {
            return fail(
                "incoming road " + std::to_string(road->getId()) +
                " is missing from the signal plan.");
        }
    }

    greenDurationSeconds_ = greenDuration;
    yellowDurationSeconds_ = yellowDuration;
    allRedDurationSeconds_ = allRedDuration;
    phaseGroups = phases;
    trafficLights.clear();
    const double redDuration = nominalRedDuration();
    for (const auto& phase : phaseGroups) {
        for (Road* road : phase) {
            trafficLights.emplace(
                road->getId(),
                std::make_unique<TrafficLight>(
                    road->getId(),
                    greenDurationSeconds_,
                    yellowDurationSeconds_,
                    redDuration,
                    LightState::RED));
        }
    }
    resetSignalCycle();
    return true;
}

bool Intersection::configureTrafficSignalsAutomatically(
    double greenDuration,
    double yellowDuration,
    double allRedDuration,
    std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) *error = message;
        return false;
    };
    if (incomingRoads.empty()) {
        return fail(
            "automatic signal plan requires at least one incoming road.");
    }
    if (!std::isfinite(greenDuration) || greenDuration <= 0.0 ||
        !std::isfinite(yellowDuration) || yellowDuration <= 0.0 ||
        !std::isfinite(allRedDuration) || allRedDuration <= 0.0) {
        return fail(
            "green, yellow and all-red durations must be positive.");
    }

    greenDurationSeconds_ = greenDuration;
    yellowDurationSeconds_ = yellowDuration;
    allRedDurationSeconds_ = allRedDuration;
    trafficLights.clear();
    for (Road* road : incomingRoads) {
        if (road == nullptr) continue;
        trafficLights.emplace(
            road->getId(),
            std::make_unique<TrafficLight>(
                road->getId(),
                greenDurationSeconds_,
                yellowDurationSeconds_,
                greenDurationSeconds_ + yellowDurationSeconds_ +
                    allRedDurationSeconds_,
                LightState::RED));
    }

    rebuildPhaseGroups();
    if (phaseGroups.empty()) {
        trafficLights.clear();
        return fail(
            "automatic signal plan could not form a phase group.");
    }

    synchronizeSignalHeads();
    return true;
}

TrafficLight* Intersection::getMutableLightForIncomingRoad(
    int roadId) {
    auto it = trafficLights.find(roadId);
    if (it != trafficLights.end()) {
        return it->second.get();
    }
    return nullptr;
}
 
const TrafficLight* Intersection::getLightForIncomingRoad(
    int roadId) const {
    auto it = trafficLights.find(roadId);
    return it != trafficLights.end()
        ? it->second.get()
        : nullptr;
}

const TrafficLight* Intersection::getLightForIncomingRoad(
    const Road* road) const {
    if (road == nullptr) return nullptr;
    return getLightForIncomingRoad(road->getId());
}

void Intersection::unregisterIncomingLight(Road* road) {
    if (road == nullptr) return;
    trafficLights.erase(road->getId());
    rebuildPhaseGroups();
}

bool Intersection::hasTrafficLights() const {
    return !trafficLights.empty();
}


void Intersection::rebuildPhaseGroups() {
    phaseGroups.clear();

    std::vector<Road*> activeIncomingRoads;
    for (Road* road : incomingRoads) {
        if (getLightForIncomingRoad(road) != nullptr) {
            activeIncomingRoads.push_back(road);
        }
    }

    const std::size_t n = activeIncomingRoads.size();
    if (n == 0) {
        resetSignalCycle();
        return;
    }

    std::vector<bool> assigned(n, false);

    auto approachAngle = [this](Road* road) -> double {
        // Direction from which traffic arrives, i.e. vector from this
        // intersection back towards where the road started.
        const Intersection* from = road->getStart();
        const double dx = from->getX() - x;
        const double dy = from->getY() - y;
        return std::atan2(dy, dx);
    };

    for (std::size_t i = 0; i < n; ++i) {
        if (assigned[i]) continue;
        std::vector<Road*> group;
        group.push_back(activeIncomingRoads[i]);
        assigned[i] = true;

        const double angleI = approachAngle(activeIncomingRoads[i]);

        for (std::size_t j = i + 1; j < n; ++j) {
            if (assigned[j]) continue;
            const double angleJ = approachAngle(activeIncomingRoads[j]);

            double diff = std::fabs(angleI - angleJ);
            if (diff > PI) diff = 2.0 * PI - diff;
            const double distanceFromOpposite = std::fabs(diff - PI);

            if (distanceFromOpposite <= OPPOSITE_TOLERANCE_RAD) {
                group.push_back(activeIncomingRoads[j]);
                assigned[j] = true;
                break; // each approach pairs with at most one opposite partner
            }
        }

        phaseGroups.push_back(std::move(group));
    }
    const double redDuration = nominalRedDuration();
    for (auto& entry : trafficLights) {
        entry.second->setDurations(
            greenDurationSeconds_,
            yellowDurationSeconds_,
            redDuration);
    }
    resetSignalCycle();
}

void Intersection::resetSignalCycle() {
    activePhaseGroup = 0;
    signalStage_ = SignalStage::GREEN;
    stageRemainingSeconds_ =
        phaseGroups.empty() ? 0.0 : greenDurationSeconds_;
    emergencyApproach_ = {};
    emergencyPriorityRemainingSeconds_ = 0.0;
    synchronizeSignalHeads();
}

double Intersection::nominalRedDuration() const {
    if (phaseGroups.empty()) return allRedDurationSeconds_;
    const double otherPhaseCount =
        static_cast<double>(phaseGroups.size() - 1u);
    return static_cast<double>(phaseGroups.size()) *
               allRedDurationSeconds_ +
           otherPhaseCount *
               (greenDurationSeconds_ + yellowDurationSeconds_);
}

std::size_t Intersection::nextScheduledPhase() const {
    if (phaseGroups.empty()) return 0;
    return (activePhaseGroup + 1) % phaseGroups.size();
}

void Intersection::synchronizeSignalHeads() {
    if (phaseGroups.empty()) return;

    const auto timeUntilGreen =
        [this](std::size_t targetPhase) {
            double remaining = 0.0;
            switch (signalStage_) {
                case SignalStage::GREEN:
                    remaining =
                        stageRemainingSeconds_ +
                        yellowDurationSeconds_ +
                        allRedDurationSeconds_;
                    break;
                case SignalStage::YELLOW:
                    remaining =
                        stageRemainingSeconds_ +
                        allRedDurationSeconds_;
                    break;
                case SignalStage::ALL_RED:
                    remaining = stageRemainingSeconds_;
                    break;
            }

            std::size_t phase = nextScheduledPhase();
            while (phase != targetPhase) {
                remaining +=
                    greenDurationSeconds_ +
                    yellowDurationSeconds_ +
                    allRedDurationSeconds_;
                phase = (phase + 1) % phaseGroups.size();
            }
            return std::max(0.0, remaining);
        };

    for (std::size_t phaseIndex = 0;
         phaseIndex < phaseGroups.size();
         ++phaseIndex) {
        LightState state = LightState::RED;
        double remaining = timeUntilGreen(phaseIndex);
        if (phaseIndex == activePhaseGroup &&
            signalStage_ == SignalStage::GREEN) {
            state = LightState::GREEN;
            remaining = stageRemainingSeconds_;
        } else if (phaseIndex == activePhaseGroup &&
                   signalStage_ == SignalStage::YELLOW) {
            state = LightState::YELLOW;
            remaining = stageRemainingSeconds_;
        }
        for (Road* road : phaseGroups[phaseIndex]) {
            TrafficLight* light =
                getMutableLightForIncomingRoad(road->getId());
            if (light != nullptr) {
                light->synchronize(state, remaining);
            }
        }
    }
}

bool Intersection::areRoadsInSamePhase(const Road* a, const Road* b) const {
    if (a == nullptr || b == nullptr) return true;
    if (a == b) return true;

    for (const auto& group : phaseGroups) {
        bool hasA = std::find(group.begin(), group.end(), a) != group.end();
        bool hasB = std::find(group.begin(), group.end(), b) != group.end();
        if (hasA && hasB) return true;
        if (hasA || hasB) return false; // found one but not the other -> different groups
    }
    return true;
}

void Intersection::updateTrafficLights(double dt) {
    updateEmergencyPriority(dt);
    if (phaseGroups.empty() ||
        !std::isfinite(dt) ||
        dt <= 0.0) {
        synchronizeSignalHeads();
        return;
    }

    double remainingDt = dt;
    constexpr double epsilon = 1e-9;
    while (remainingDt > epsilon) {
        if (stageRemainingSeconds_ > remainingDt + epsilon) {
            stageRemainingSeconds_ -= remainingDt;
            remainingDt = 0.0;
            break;
        }

        const double consumed =
            std::max(0.0, stageRemainingSeconds_);
        remainingDt =
            std::max(0.0, remainingDt - consumed);

        if (signalStage_ == SignalStage::GREEN) {
            signalStage_ = SignalStage::YELLOW;
            stageRemainingSeconds_ =
                yellowDurationSeconds_;
        } else if (signalStage_ == SignalStage::YELLOW) {
            signalStage_ = SignalStage::ALL_RED;
            stageRemainingSeconds_ = allRedDurationSeconds_;
        } else {
            const std::size_t nextPhase =
                nextScheduledPhase();
            activePhaseGroup = nextPhase;
            signalStage_ = SignalStage::GREEN;
            stageRemainingSeconds_ = greenDurationSeconds_;
        }
    }
    synchronizeSignalHeads();
}

void Intersection::clearEmergencyPriority() {
    emergencyApproach_ = {};
    emergencyPriorityRemainingSeconds_ = 0.0;
}

void Intersection::updateEmergencyPriority(double dt) {
    if (!hasActiveEmergencyPriority() ||
        !std::isfinite(dt) ||
        dt <= 0.0) {
        return;
    }
    emergencyPriorityRemainingSeconds_ =
        std::max(
            0.0,
            emergencyPriorityRemainingSeconds_ - dt);
    if (emergencyPriorityRemainingSeconds_ <= 1e-9) {
        clearEmergencyPriority();
    }
}

bool Intersection::hasActiveEmergencyPriority() const {
    return emergencyPriorityRemainingSeconds_ > 1e-9 &&
           emergencyApproach_.isValid();
}

bool Intersection::isPrioritizedEmergencyVehicle(
    int vehicleId,
    const Road* incomingRoad) const {
    return hasActiveEmergencyPriority() &&
           emergencyApproach_.vehicleId == vehicleId &&
           emergencyApproach_.path.incomingRoad == incomingRoad;
}

bool Intersection::isEmergencyAdmissionBlocked(
    int vehicleId,
    const Road* incomingRoad) const {
    return hasActiveEmergencyPriority() &&
           !isPrioritizedEmergencyVehicle(
               vehicleId, incomingRoad) &&
           incomingRoad !=
               emergencyApproach_.path.incomingRoad;
}

void Intersection::requestEmergencyPriority(
    int vehicleId,
    const Road* incomingRoad,
    int incomingLane,
    const Road* outgoingRoad,
    int outgoingLane,
    double vehicleWidthMetres,
    double holdDuration) {
    if (vehicleId < 0 ||
        incomingRoad == nullptr ||
        incomingRoad->getEnd() != this ||
        incomingLane < 0 ||
        incomingLane >= incomingRoad->getLaneCount() ||
        !std::isfinite(vehicleWidthMetres) ||
        vehicleWidthMetres <= 0.0 ||
        !std::isfinite(holdDuration) ||
        holdDuration <= 0.0) {
        return;
    }
    if (hasActiveEmergencyPriority() &&
        emergencyApproach_.vehicleId != vehicleId) {
        return;
    }

    const bool validOutgoing =
        outgoingRoad != nullptr &&
        outgoingRoad->getStart() == this &&
        outgoingLane >= 0 &&
        outgoingLane < outgoingRoad->getLaneCount();
    emergencyApproach_ = {
        vehicleId,
        {
            incomingRoad,
            incomingLane,
            validOutgoing ? outgoingRoad : nullptr,
            validOutgoing ? outgoingLane : -1,
            vehicleWidthMetres
        }
    };
    emergencyPriorityRemainingSeconds_ =
        std::max(
            emergencyPriorityRemainingSeconds_,
            holdDuration);
    // Emergency priority affects right-of-way and junction reservations, not
    // the fixed signal clock.
}

bool Intersection::mustStopForRoad(const Road* road) const {
    const TrafficLight* light = getLightForIncomingRoad(road);
    // Unsignalized approaches are not controlled by this signal plan.
    return (light != nullptr) && light->mustStop();
}

JunctionDecision Intersection::getMovementDecision(
    const Road* incomingRoad,
    int incomingLane,
    const Road* outgoingRoad,
    MovementType movement) const {
    if (incomingRoad == nullptr || outgoingRoad == nullptr ||
        incomingRoad->getEnd() != this ||
        outgoingRoad->getStart() != this ||
        incomingLane < 0 ||
        incomingLane >= incomingRoad->getLaneCount()) {
        return JunctionDecision::Stop;
    }
    const TrafficLight* light =
        getLightForIncomingRoad(incomingRoad);
    if (light == nullptr || light->getState() == LightState::GREEN) {
        return JunctionDecision::Proceed;
    }
    if (!allowRightTurnOnRed_ ||
        light->getState() != LightState::RED ||
        movement != MovementType::Right ||
        !incomingRoad->isCurbLane(incomingLane)) {
        return JunctionDecision::Stop;
    }
    return JunctionDecision::Yield;
}

bool Intersection::hasPriorityVehicleApproaching(
    const Road* yieldingRoad) const {
    // Use the per-frame cache to avoid O(N^2) behavior.
    // clearFrameCache() is called once per frame from
    // TrafficSimulator::update() before the vehicle update loop.
    if (priorityVehicleCacheValid_) {
        auto it = priorityVehicleCache_.find(yieldingRoad);
        if (it != priorityVehicleCache_.end()) {
            return it->second;
        }
    }
    for (const Road* road : getIncomingRoads()) {
        if (road == nullptr || road == yieldingRoad ||
            mustStopForRoad(road)) {
            continue;
        }
        for (const Lane& lane : road->getLanes()) {
            for (const Vehicle* vehicle : lane.getVehicles()) {
                if (vehicle == nullptr ||
                    vehicle->getCurrentRoad() != road) {
                    continue;
                }
                const double distanceToStop =
                    std::max(
                        0.0,
                        RoadGeometry::stopLineProgressMetres(*road) -
                            vehicle->getProgressOnRoad());
                const double speed =
                    std::max(0.0, vehicle->getCurrentSpeed());
                if (distanceToStop <=
                        RIGHT_ON_RED_PRIORITY_LOOKAHEAD_METRES &&
                    (speed <= 1e-6 ||
                     distanceToStop / speed <=
                         RIGHT_ON_RED_PRIORITY_TIME_SECONDS)) {
                    priorityVehicleCache_[yieldingRoad] = true;
                    priorityVehicleCacheValid_ = true;
                    return true;
                }
            }
        }
    }
    priorityVehicleCache_[yieldingRoad] = false;
    priorityVehicleCacheValid_ = true;
    return false;
}

std::shared_ptr<const JunctionConnector> Intersection::createConnector(
    const Road& incoming,
    int incomingLane,
    const Road& outgoing,
    int outgoingLane) const {
    const Vec2 start =
        RoadGeometry::laneEndpoint(incoming, incomingLane, false);
    const Vec2 finish =
        RoadGeometry::laneEndpoint(outgoing, outgoingLane, true);
    const Vec2 startTangent = RoadGeometry::roadDirection(incoming);
    const Vec2 endTangent = RoadGeometry::roadDirection(outgoing);
    const MovementType movement =
        TurnLanePolicy::classify(incoming, outgoing);
    const double chord = distance(start, finish);
    const double junctionRadiusWorld =
        RoadGeometry::junctionBoundaryRadiusWorld(*this);

    double movementFactor = 0.75;
    switch (movement) {
        case MovementType::Straight: movementFactor = 0.45; break;
        case MovementType::Right: movementFactor = 0.60; break;
        case MovementType::Left: movementFactor = 0.85; break;
        case MovementType::UTurn: movementFactor = 1.15; break;
    }
    const double scale =
        RoadGeometry::metresPerWorldUnit(*this);
    const double controlDistance = std::max({
        chord * 0.30,
        junctionRadiusWorld * movementFactor,
        junctionRadiusWorld > 1e-9
            ? RoadGeometry::LANE_WIDTH_METRES / scale
            : 0.0
    });

    auto path = std::make_shared<BezierJunctionPath>(
        start,
        start + startTangent * controlDistance,
        finish - endTangent * controlDistance,
        finish,
        scale);
    const ConnectorKey key{
        incoming.getId(), incomingLane,
        outgoing.getId(), outgoingLane
    };
    return std::make_shared<JunctionConnector>(
        key,
        &incoming,
        &outgoing,
        movement,
        startTangent,
        endTangent,
        RoadGeometry::metresPerWorldUnit(outgoing),
        std::move(path));
}

std::shared_ptr<const JunctionConnector> Intersection::getConnector(
    const Road* incoming,
    int incomingLane,
    const Road* outgoing,
    int outgoingLane) const {
    if (incoming == nullptr || outgoing == nullptr ||
        incomingLane < 0 || incomingLane >= incoming->getLaneCount() ||
        outgoingLane < 0 || outgoingLane >= outgoing->getLaneCount() ||
        incoming->getEnd() != this || outgoing->getStart() != this) {
        return nullptr;
    }

    const ConnectorKey key{
        incoming->getId(), incomingLane,
        outgoing->getId(), outgoingLane
    };
    const auto found = connectorCache_.find(key);
    if (found != connectorCache_.end()) {
        return found->second;
    }

    auto connector = createConnector(
        *incoming, incomingLane, *outgoing, outgoingLane);
    if (connector != nullptr) {
        connectorCache_.emplace(key, connector);
    }
    return connector;
}

void Intersection::invalidateConnectorCache() {
    connectorCache_.clear();
    movementGeometryCache_.valid = false;
}

void Intersection::markReservationStateChanged() {
    ++reservationRevision_;
    movementGeometryCache_.valid = false;
}

// --- Intersection-box reservation ---
// Deliberately independent from the traffic-light phase logic above: a
// green light only means "your approach's turn according to the signal
// cycle", it says nothing about whether the physical box in the middle of
// the junction is currently occupied by a vehicle arriving from another
// approach. This is the missing piece that stops vehicles from different
// roads/lanes rendering on top of each other inside the junction.

bool Intersection::canEnter(int vehicleId, const Road* fromRoad) const {
    if (occupants_.count(vehicleId) > 0) {
        return true; // dang giu cho roi
    }
    if (isEmergencyAdmissionBlocked(vehicleId, fromRoad)) {
        return false;
    }
    for (const auto& occupant : occupants_) {
        if (!areRoadsInSamePhase(
                fromRoad, occupant.second.fromRoad)) {
            return false; 
        }
    }
    return static_cast<int>(occupants_.size()) < capacity_;
}

bool Intersection::canEnterMovement(
    int vehicleId,
    const std::shared_ptr<const JunctionConnector>& connector,
    double requiredGapMetres,
    double vehicleLengthMetres,
    double vehicleWidthMetres) const {
    if (connector == nullptr) {
        return false;
    }
    const Road* incomingRoad = connector->getIncomingRoad();
    if (!canEnter(vehicleId, incomingRoad)) {
        return false;
    }

    const double metricScale = RoadGeometry::metresPerWorldUnit(*this);
    const double step = std::max(0.5, std::min(vehicleLengthMetres, vehicleWidthMetres) * 0.5);
    const double length1 = connector->getLength();
    
    // Quick out for empty intersection
    if (occupants_.empty() || (occupants_.size() == 1 && occupants_.count(vehicleId) > 0)) {
        return true;
    }

    if (movementGeometryCache_.valid &&
        movementGeometryCache_.reservationRevision ==
            reservationRevision_ &&
        movementGeometryCache_.vehicleId == vehicleId &&
        movementGeometryCache_.connector == connector.get() &&
        movementGeometryCache_.requiredGapMetres ==
            requiredGapMetres &&
        movementGeometryCache_.vehicleLengthMetres ==
            vehicleLengthMetres &&
        movementGeometryCache_.vehicleWidthMetres ==
            vehicleWidthMetres) {
        return movementGeometryCache_.canEnter;
    }

    const auto cacheGeometryResult = [&](bool canEnter) {
        movementGeometryCache_.valid = true;
        movementGeometryCache_.reservationRevision =
            reservationRevision_;
        movementGeometryCache_.vehicleId = vehicleId;
        movementGeometryCache_.connector = connector.get();
        movementGeometryCache_.requiredGapMetres =
            requiredGapMetres;
        movementGeometryCache_.vehicleLengthMetres =
            vehicleLengthMetres;
        movementGeometryCache_.vehicleWidthMetres =
            vehicleWidthMetres;
        movementGeometryCache_.canEnter = canEnter;
        return canEnter;
    };

    // Precompute occupant OBB samples once, so the inner loop does not
    // redundantly call makeVehicleBounds (which calls cos/sin) for every
    // (candidate_position, occupant_position) pair. This reduces OBB
    // creation from O(steps1 * steps2) to O(steps1 + steps2) per occupant.
    struct OccupantPath {
        std::vector<OrientedVehicleBounds> samples;
        uint64_t entryId;
    };
    std::vector<OccupantPath> occupantPaths;
    occupantPaths.reserve(occupants_.size());
    
    uint64_t candidateEntryId = std::numeric_limits<uint64_t>::max();
    auto candIt = occupants_.find(vehicleId);
    if (candIt != occupants_.end()) {
        candidateEntryId = candIt->second.entryId;
    }

    for (const auto& entry : occupants_) {
        if (entry.first == vehicleId) continue;
        const Reservation& res = entry.second;
        if (res.connector == nullptr) continue;
        if (connectorsPreserveLaneOrder(
                *connector, *res.connector) ||
            oppositeApproachMovementsAreNonCrossing(
                *connector, *res.connector)) {
            continue;
        }
        OccupantPath path;
        path.entryId = res.entryId;
        const double length2 = res.connector->getLength();
        const std::size_t estimatedSteps =
            static_cast<std::size_t>(
                std::ceil(length2 / step)) + 1u;
        path.samples.reserve(estimatedSteps);
        for (double p2 = res.progressMetres;
             p2 <= length2; p2 += step) {
            path.samples.push_back(makeVehicleBounds(
                res.connector->sampleByDistance(p2),
                metricScale,
                res.vehicleLengthMetres,
                res.vehicleWidthMetres,
                0.5));
        }
        occupantPaths.push_back(std::move(path));
    }

    for (double p1 = 0.0; p1 <= length1; p1 += step) {
        const OrientedVehicleBounds candidate = makeVehicleBounds(
            connector->sampleByDistance(p1),
            metricScale,
            vehicleLengthMetres,
            vehicleWidthMetres,
            0.5);

        for (const auto& path : occupantPaths) {
            for (const auto& other : path.samples) {
                if (boundsOverlap(candidate, other)) {
                    if (candidateEntryId < path.entryId) {
                        continue; // Candidate entered first, ignore overlap
                    }
                    return cacheGeometryResult(
                        false); // Paths overlap, must yield
                }
            }
        }
    }
    return cacheGeometryResult(true);
}

bool Intersection::canEnterYieldingMovement(
    int vehicleId,
    const std::shared_ptr<const JunctionConnector>& connector,
    double requiredGapMetres,
    double vehicleLengthMetres,
    double vehicleWidthMetres) const {
    if (connector == nullptr ||
        getMovementDecision(
            connector->getIncomingRoad(),
            connector->getIncomingLane(),
            connector->getOutgoingRoad(),
            connector->getMovementType()) !=
            JunctionDecision::Yield ||
        hasPriorityVehicleApproaching(
            connector->getIncomingRoad())) {
        return false;
    }
    return canEnterMovement(
        vehicleId,
        connector,
        requiredGapMetres,
        vehicleLengthMetres,
        vehicleWidthMetres);
}

bool Intersection::tryEnter(int vehicleId, const Road* fromRoad) {
    if (occupants_.count(vehicleId) > 0) {
        return true; // already holding a slot, nothing to do
    }
    if (!canEnter(vehicleId, fromRoad)) {
        return false;
    }
    occupants_.emplace(
        vehicleId,
        Reservation{fromRoad, nullptr, 0.0, 4.5, 1.8, ++nextEntryId_});
    markReservationStateChanged();
    return true;
}

bool Intersection::tryEnterMovement(
    int vehicleId,
    const std::shared_ptr<const JunctionConnector>& connector,
    double requiredGapMetres,
    double vehicleLengthMetres,
    double vehicleWidthMetres) {
    auto existing = occupants_.find(vehicleId);
    if (existing != occupants_.end()) {
        const auto previousConnector = existing->second.connector;
        const double previousLength =
            existing->second.vehicleLengthMetres;
        const double previousWidth =
            existing->second.vehicleWidthMetres;
        if (connector != nullptr) {
            existing->second.connector = connector;
        }
        existing->second.vehicleLengthMetres =
            std::max(0.1, vehicleLengthMetres);
        existing->second.vehicleWidthMetres =
            std::max(0.1, vehicleWidthMetres);
        if (existing->second.connector != previousConnector ||
            existing->second.vehicleLengthMetres != previousLength ||
            existing->second.vehicleWidthMetres != previousWidth) {
            markReservationStateChanged();
        }
        return true;
    }
    if (!canEnterMovement(
            vehicleId,
            connector,
            requiredGapMetres,
            vehicleLengthMetres,
            vehicleWidthMetres)) {
        return false;
    }
    occupants_.emplace(
        vehicleId,
        Reservation{
            connector->getIncomingRoad(),
            connector,
            0.0,
            std::max(0.1, vehicleLengthMetres),
            std::max(0.1, vehicleWidthMetres),
            ++nextEntryId_
        });
    markReservationStateChanged();
    return true;
}

bool Intersection::tryEnterYieldingMovement(
    int vehicleId,
    const std::shared_ptr<const JunctionConnector>& connector,
    double requiredGapMetres,
    double vehicleLengthMetres,
    double vehicleWidthMetres) {
    if (occupants_.count(vehicleId) > 0) {
        return tryEnterMovement(
            vehicleId,
            connector,
            requiredGapMetres,
            vehicleLengthMetres,
            vehicleWidthMetres);
    }
    if (!canEnterYieldingMovement(
            vehicleId,
            connector,
            requiredGapMetres,
            vehicleLengthMetres,
            vehicleWidthMetres)) {
        return false;
    }
    occupants_.emplace(
        vehicleId,
        Reservation{
            connector->getIncomingRoad(),
            connector,
            0.0,
            std::max(0.1, vehicleLengthMetres),
            std::max(0.1, vehicleWidthMetres),
            ++nextEntryId_
        });
    markReservationStateChanged();
    return true;
}

void Intersection::updateReservationProgress(
    int vehicleId,
    double progressMetres) {
    auto found = occupants_.find(vehicleId);
    if (found != occupants_.end()) {
        const double updatedProgress =
            std::max(0.0, progressMetres);
        if (found->second.progressMetres != updatedProgress) {
            found->second.progressMetres = updatedProgress;
            markReservationStateChanged();
        }
    }
}

double Intersection::limitTraversalAdvance(
    int vehicleId,
    const std::shared_ptr<const JunctionConnector>& connector,
    double currentProgressMetres,
    double desiredAdvanceMetres,
    double vehicleLengthMetres,
    double vehicleWidthMetres,
    double clearanceMetres) const {
    if (connector == nullptr ||
        desiredAdvanceMetres <= 0.0 ||
        occupants_.size() <= 1) {
        return std::max(0.0, desiredAdvanceMetres);
    }

    const double metricScale =
        RoadGeometry::metresPerWorldUnit(*this);
    uint64_t candidateEntryId = std::numeric_limits<uint64_t>::max();
    auto candIt = occupants_.find(vehicleId);
    if (candIt != occupants_.end()) {
        candidateEntryId = candIt->second.entryId;
    }

    struct OccupantBounds {
        OrientedVehicleBounds bounds;
        uint64_t entryId = 0;
    };
    std::vector<OccupantBounds> occupantBounds;
    occupantBounds.reserve(occupants_.size() - 1);
    for (const auto& entry : occupants_) {
        if (entry.first == vehicleId) continue;
        const Reservation& reservation = entry.second;
        if (reservation.connector == nullptr) {
            return 0.0;
        }
        occupantBounds.push_back(OccupantBounds{
            makeVehicleBounds(
                reservation.connector->sampleByDistance(
                    reservation.progressMetres),
                metricScale,
                reservation.vehicleLengthMetres,
                reservation.vehicleWidthMetres,
                clearanceMetres),
            reservation.entryId});
    }

    const auto isSafeAt = [&](double progressMetres) {
        const OrientedVehicleBounds candidate =
            makeVehicleBounds(
                connector->sampleByDistance(progressMetres),
                metricScale,
                vehicleLengthMetres,
                vehicleWidthMetres,
                clearanceMetres);
        for (const OccupantBounds& other : occupantBounds) {
            if (boundsOverlap(candidate, other.bounds)) {
                if (candidateEntryId < other.entryId) {
                    continue; // Candidate entered first, ignore overlap
                }
                return false;
            }
        }
        return true;
    };

    if (!isSafeAt(currentProgressMetres)) {
        return 0.0;
    }

    // Sweep in short increments so a large caller step cannot tunnel through
    // another rectangle merely because both end positions are collision-free.
    const double sweepStepMetres = std::max(
        0.25,
        std::min(
            std::max(0.1, vehicleLengthMetres),
            std::max(0.1, vehicleWidthMetres)) * 0.25);
    const int sweepSteps = std::max(
        1,
        static_cast<int>(std::ceil(
            desiredAdvanceMetres / sweepStepMetres)));
    double safeAdvance = 0.0;
    for (int step = 1; step <= sweepSteps; ++step) {
        const double candidateAdvance =
            desiredAdvanceMetres *
            static_cast<double>(step) /
            static_cast<double>(sweepSteps);
        if (isSafeAt(
                currentProgressMetres + candidateAdvance)) {
            safeAdvance = candidateAdvance;
            continue;
        }

        double unsafeAdvance = candidateAdvance;
        for (int iteration = 0; iteration < 14; ++iteration) {
            const double candidate =
                (safeAdvance + unsafeAdvance) * 0.5;
            if (isSafeAt(
                    currentProgressMetres + candidate)) {
                safeAdvance = candidate;
            } else {
                unsafeAdvance = candidate;
            }
        }
        return safeAdvance;
    }
    return desiredAdvanceMetres;
}

double Intersection::constrainIncomingStopPosition(
    const Road* incomingRoad,
    int incomingLane,
    double nominalStopPositionMetres,
    double waitingVehicleLengthMetres,
    double requiredClearanceMetres) const {
    if (incomingRoad == nullptr || incomingLane < 0) {
        return std::max(0.0, nominalStopPositionMetres);
    }

    double stopPosition =
        std::max(0.0, nominalStopPositionMetres);
    const double waitingHalfLength =
        std::max(0.1, waitingVehicleLengthMetres) * 0.5;
    const double clearance =
        std::max(0.0, requiredClearanceMetres);
    for (const auto& entry : occupants_) {
        const Reservation& reservation = entry.second;
        if (reservation.connector == nullptr ||
            reservation.connector->getIncomingRoad() != incomingRoad ||
            reservation.connector->getIncomingLane() != incomingLane) {
            continue;
        }

        // Connector progress is measured from the incoming lane endpoint.
        // Until the reserved vehicle's rear bumper clears that endpoint, its
        // body still occupies part of the approach lane even though it has
        // already been removed from Lane::vehicles.
        const double reservedRearFromBoundary =
            reservation.progressMetres -
            reservation.vehicleLengthMetres * 0.5;
        const double safeFollowerCentre =
            incomingRoad->getDistance() +
            reservedRearFromBoundary -
            clearance -
            waitingHalfLength;
        stopPosition = std::min(
            stopPosition,
            std::max(0.0, safeFollowerCentre));
    }
    return stopPosition;
}

bool Intersection::isOutgoingLaneReserved(
    const Road* outgoingRoad,
    int outgoingLane) const {
    if (outgoingRoad == nullptr || outgoingLane < 0) {
        return false;
    }
    for (const auto& entry : occupants_) {
        const auto& connector = entry.second.connector;
        if (connector != nullptr &&
            connector->getOutgoingRoad() == outgoingRoad &&
            connector->getOutgoingLane() == outgoingLane) {
            return true;
        }
    }
    return false;
}

void Intersection::exit(int vehicleId) {
    if (occupants_.erase(vehicleId) > 0) {
        markReservationStateChanged();
    }
    if (hasActiveEmergencyPriority() &&
        emergencyApproach_.vehicleId == vehicleId) {
        clearEmergencyPriority();
    }
}

bool Intersection::isFull() const {
    return static_cast<int>(occupants_.size()) >= capacity_;
}

void Intersection::setCapacity(int cap) {
    capacity_ = std::max(1, cap);
}

//utility method
std::string Intersection::toString() const {
    return "Intersection[ID: " + std::to_string(id) + 
           ", X: " + std::to_string(x) + 
           ", Y: " + std::to_string(y) + 
           ", Type: " + getIntersectionTypeLabel() +
           ", Incoming: " + std::to_string(incomingRoads.size()) + 
           ", Outgoing: " + std::to_string(outgoingRoads.size()) +
           ", Lights: " + std::to_string(trafficLights.size()) +
           ", PhaseGroups: " + std::to_string(phaseGroups.size()) + "]";
}

Intersection::~Intersection() {
}
