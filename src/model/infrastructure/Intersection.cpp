#include "Intersection.h"
#include "Crosswalk.h"
#include "Road.h" 
#include "TrafficLight.h"
#include "LaneMapping.h"
#include "MotionPath.h"
#include "RoadGeometry.h"
#include "Vehicle.h"
#include <algorithm>
#include <cmath>

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
    explicitSignalPlan_ = false;
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
    explicitSignalPlan_ = false;
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
        !std::isfinite(allRedDuration) || allRedDuration < 0.0) {
        return fail(
            "green/yellow durations must be positive and all-red "
            "duration must be non-negative.");
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
    allRedDurationSeconds_ =
        phases.size() > 1 ? allRedDuration : 0.0;
    phaseGroups = phases;
    trafficLights.clear();
    const double maximumRedDuration =
        static_cast<double>(phaseGroups.size()) *
        (greenDurationSeconds_ + yellowDurationSeconds_ +
         allRedDurationSeconds_);
    for (const auto& phase : phaseGroups) {
        for (Road* road : phase) {
            trafficLights.emplace(
                road->getId(),
                std::make_unique<TrafficLight>(
                    road->getId(),
                    greenDurationSeconds_,
                    yellowDurationSeconds_,
                    maximumRedDuration,
                    LightState::RED));
        }
    }
    explicitSignalPlan_ = true;
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
        !std::isfinite(allRedDuration) || allRedDuration < 0.0) {
        return fail(
            "green/yellow durations must be positive and all-red "
            "duration must be non-negative.");
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

    explicitSignalPlan_ = false;
    rebuildPhaseGroups();
    if (phaseGroups.empty()) {
        trafficLights.clear();
        return fail(
            "automatic signal plan could not form a phase group.");
    }

    const double maximumRedDuration =
        static_cast<double>(phaseGroups.size()) *
        (greenDurationSeconds_ + yellowDurationSeconds_ +
         allRedDurationSeconds_);
    for (auto& entry : trafficLights) {
        entry.second->setDurations(
            greenDurationSeconds_,
            yellowDurationSeconds_,
            maximumRedDuration);
    }
    synchronizeSignalHeads();
    return true;
}

TrafficLight* Intersection::getLightForIncomingRoad(int roadId) const {
    auto it = trafficLights.find(roadId);
    if (it != trafficLights.end()) {
        return it->second.get();
    }
    return nullptr;
}
 
TrafficLight* Intersection::getLightForIncomingRoad(const Road* road) const {
    if (road == nullptr) return nullptr;
    return getLightForIncomingRoad(road->getId());
}

void Intersection::registerCrosswalk(Crosswalk* crosswalk) {
    if (crosswalk == nullptr ||
        crosswalk->getIntersection() != this ||
        crosswalk->getIncomingRoad() == nullptr ||
        crosswalk->getIncomingRoad()->getEnd() != this ||
        std::find(
            crosswalks_.begin(),
            crosswalks_.end(),
            crosswalk) != crosswalks_.end()) {
        return;
    }
    crosswalks_.push_back(crosswalk);
}

void Intersection::unregisterCrosswalk(
    Crosswalk* crosswalk) {
    if (crosswalk == nullptr) return;
    crosswalk->synchronizeSignalState(
        PedestrianSignalState::DontWalk);
    crosswalks_.erase(
        std::remove(
            crosswalks_.begin(),
            crosswalks_.end(),
            crosswalk),
        crosswalks_.end());
    pedestrianPhasePending_ =
        hasEligiblePedestrianRequest();
}

const std::vector<Crosswalk*>&
Intersection::getCrosswalks() const {
    return crosswalks_;
}

Crosswalk* Intersection::getCrosswalkForIncomingRoad(
    const Road* road) const {
    if (road == nullptr) return nullptr;
    const auto found = std::find_if(
        crosswalks_.begin(),
        crosswalks_.end(),
        [road](const Crosswalk* crosswalk) {
            return crosswalk != nullptr &&
                   crosswalk->getIncomingRoad() == road;
        });
    return found != crosswalks_.end()
        ? *found
        : nullptr;
}

void Intersection::unregisterIncomingLight(Road* road) {
    if (road == nullptr) return;
    trafficLights.erase(road->getId());
    explicitSignalPlan_ = false;
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
    allRedDurationSeconds_ =
        phaseGroups.size() > 1
            ? std::max(0.0, allRedDurationSeconds_)
            : 0.0;
    resetSignalCycle();
}

void Intersection::resetSignalCycle() {
    activePhaseGroup = 0;
    signalStage_ = SignalStage::GREEN;
    stageRemainingSeconds_ =
        phaseGroups.empty() ? 0.0 : greenDurationSeconds_;
    preemptedRoad_ = nullptr;
    preemptionHoldSeconds_ = 0.0;
    emergencyApproach_ = {};
    emergencyPriorityRemainingSeconds_ = 0.0;
    pedestrianPhasePending_ = false;
    for (Crosswalk* crosswalk : crosswalks_) {
        if (crosswalk != nullptr) {
            crosswalk->synchronizeSignalState(
                PedestrianSignalState::DontWalk);
        }
    }
    synchronizeSignalHeads();
}

bool Intersection::hasEligiblePedestrianRequest() const {
    return std::any_of(
        crosswalks_.begin(),
        crosswalks_.end(),
        [](const Crosswalk* crosswalk) {
            return crosswalk != nullptr &&
                   crosswalk->hasEligibleRequest();
        });
}

bool Intersection::hasOccupiedCrosswalk() const {
    return std::any_of(
        crosswalks_.begin(),
        crosswalks_.end(),
        [](const Crosswalk* crosswalk) {
            return crosswalk != nullptr &&
                   crosswalk->isOccupied();
        });
}

bool Intersection::hasIntersectionOccupants() const {
    return !occupants_.empty();
}

double Intersection::pedestrianWalkDuration() const {
    double duration = 0.0;
    for (const Crosswalk* crosswalk : crosswalks_) {
        if (crosswalk != nullptr &&
            (crosswalk->getSignalState() ==
                 PedestrianSignalState::Walk ||
             crosswalk->hasEligibleRequest())) {
            duration = std::max(
                duration,
                crosswalk->getTiming().
                    walkDurationSeconds);
        }
    }
    return duration;
}

double Intersection::pedestrianClearanceDuration() const {
    double duration = 0.0;
    for (const Crosswalk* crosswalk : crosswalks_) {
        if (crosswalk != nullptr &&
            (crosswalk->getSignalState() ==
                 PedestrianSignalState::Walk ||
             crosswalk->getSignalState() ==
                 PedestrianSignalState::Clearance)) {
            duration = std::max(
                duration,
                crosswalk->
                    getClearanceDurationSeconds());
        }
    }
    return duration;
}

void Intersection::beginPedestrianWalk() {
    for (Crosswalk* crosswalk : crosswalks_) {
        if (crosswalk == nullptr) continue;
        crosswalk->synchronizeSignalState(
            crosswalk->hasEligibleRequest()
                ? PedestrianSignalState::Walk
                : PedestrianSignalState::DontWalk);
    }
    signalStage_ = SignalStage::PEDESTRIAN_WALK;
    stageRemainingSeconds_ =
        std::max(0.01, pedestrianWalkDuration());
    pedestrianPhasePending_ = false;
}

void Intersection::beginPedestrianClearance() {
    for (Crosswalk* crosswalk : crosswalks_) {
        if (crosswalk != nullptr &&
            crosswalk->getSignalState() ==
                PedestrianSignalState::Walk) {
            crosswalk->synchronizeSignalState(
                PedestrianSignalState::Clearance);
        }
    }
    signalStage_ =
        SignalStage::PEDESTRIAN_CLEARANCE;
    stageRemainingSeconds_ = std::max(
        0.0,
        pedestrianClearanceDuration());
}

void Intersection::endPedestrianPhase() {
    for (Crosswalk* crosswalk : crosswalks_) {
        if (crosswalk != nullptr) {
            crosswalk->synchronizeSignalState(
                PedestrianSignalState::DontWalk);
        }
    }
}

std::size_t Intersection::phaseIndexForRoad(
    const Road* road) const {
    if (road == nullptr) return phaseGroups.size();
    for (std::size_t index = 0;
         index < phaseGroups.size();
         ++index) {
        if (std::find(
                phaseGroups[index].begin(),
                phaseGroups[index].end(),
                road) != phaseGroups[index].end()) {
            return index;
        }
    }
    return phaseGroups.size();
}

std::size_t Intersection::nextScheduledPhase() const {
    if (phaseGroups.empty()) return 0;
    const std::size_t preempted =
        phaseIndexForRoad(preemptedRoad_);
    if (preempted < phaseGroups.size()) {
        return preempted;
    }
    return (activePhaseGroup + 1) % phaseGroups.size();
}

bool Intersection::hasConflictingReservationForPhase(
    std::size_t phaseIndex) const {
    if (phaseIndex >= phaseGroups.size()) return true;
    for (const auto& entry : occupants_) {
        const Road* occupiedFrom = entry.second.fromRoad;
        if (occupiedFrom == nullptr) return true;
        if (phaseIndexForRoad(occupiedFrom) != phaseIndex) {
            return true;
        }
    }
    return false;
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
                case SignalStage::PEDESTRIAN_WALK:
                    remaining =
                        stageRemainingSeconds_ +
                        pedestrianClearanceDuration();
                    break;
                case SignalStage::PEDESTRIAN_CLEARANCE:
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
            TrafficLight* light = getLightForIncomingRoad(road);
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
        return;
    }
    if (signalStage_ !=
            SignalStage::PEDESTRIAN_WALK &&
        signalStage_ !=
            SignalStage::PEDESTRIAN_CLEARANCE &&
        hasEligiblePedestrianRequest()) {
        const bool newlyPending =
            !pedestrianPhasePending_;
        pedestrianPhasePending_ = true;
        if (newlyPending &&
            signalStage_ == SignalStage::ALL_RED &&
            !hasActiveEmergencyPriority()) {
            stageRemainingSeconds_ = std::max(
                stageRemainingSeconds_,
                1.5);
        }
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
            stageRemainingSeconds_ =
                pedestrianPhasePending_ &&
                        !hasActiveEmergencyPriority()
                    ? std::max(
                          allRedDurationSeconds_,
                          1.5)
                    : allRedDurationSeconds_;
        } else if (signalStage_ == SignalStage::ALL_RED) {
            if (pedestrianPhasePending_ &&
                !hasActiveEmergencyPriority()) {
                if (hasIntersectionOccupants()) {
                    stageRemainingSeconds_ = 0.0;
                    break;
                }
                beginPedestrianWalk();
                continue;
            }
            const std::size_t nextPhase =
                nextScheduledPhase();
            if (hasConflictingReservationForPhase(nextPhase)) {
                stageRemainingSeconds_ = 0.0;
                break;
            }
            activePhaseGroup = nextPhase;
            signalStage_ = SignalStage::GREEN;
            stageRemainingSeconds_ =
                std::max(
                    greenDurationSeconds_,
                    preemptionHoldSeconds_);
            preemptedRoad_ = nullptr;
            preemptionHoldSeconds_ = 0.0;
        } else if (
            signalStage_ ==
            SignalStage::PEDESTRIAN_WALK) {
            beginPedestrianClearance();
        } else {
            if (hasOccupiedCrosswalk()) {
                stageRemainingSeconds_ = 0.0;
                break;
            }
            endPedestrianPhase();
            const std::size_t nextPhase =
                nextScheduledPhase();
            activePhaseGroup = nextPhase;
            signalStage_ = SignalStage::GREEN;
            stageRemainingSeconds_ =
                std::max(
                    greenDurationSeconds_,
                    preemptionHoldSeconds_);
            preemptedRoad_ = nullptr;
            preemptionHoldSeconds_ = 0.0;
        }
    }
    synchronizeSignalHeads();
}

void Intersection::clearEmergencyPriority() {
    emergencyApproach_ = {};
    emergencyPriorityRemainingSeconds_ = 0.0;
    preemptedRoad_ = nullptr;
    preemptionHoldSeconds_ = 0.0;
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
           emergencyApproach_.incomingRoad == incomingRoad;
}

const EmergencyApproach*
Intersection::getEmergencyApproach() const {
    return hasActiveEmergencyPriority()
        ? &emergencyApproach_
        : nullptr;
}

bool Intersection::isEmergencyPathClear(
    int vehicleId) const {
    if (!hasActiveEmergencyPriority() ||
        emergencyApproach_.vehicleId != vehicleId) {
        return false;
    }
    return std::all_of(
        crosswalks_.begin(),
        crosswalks_.end(),
        [this](const Crosswalk* crosswalk) {
            return crosswalk == nullptr ||
                   crosswalk->isEmergencyPathClear(
                       emergencyApproach_);
        });
}

void Intersection::requestEmergencyPreemption(
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
        incomingRoad,
        incomingLane,
        validOutgoing ? outgoingRoad : nullptr,
        validOutgoing ? outgoingLane : -1,
        vehicleWidthMetres
    };
    emergencyPriorityRemainingSeconds_ =
        std::max(
            emergencyPriorityRemainingSeconds_,
            holdDuration);

    if (getLightForIncomingRoad(incomingRoad) == nullptr) {
        return;
    }

    const std::size_t requestedPhase =
        phaseIndexForRoad(incomingRoad);
    if (requestedPhase >= phaseGroups.size()) return;

    preemptionHoldSeconds_ =
        std::max(preemptionHoldSeconds_, holdDuration);
    if (requestedPhase == activePhaseGroup &&
        signalStage_ == SignalStage::GREEN) {
        stageRemainingSeconds_ =
            std::max(stageRemainingSeconds_, holdDuration);
        preemptedRoad_ = nullptr;
        synchronizeSignalHeads();
        return;
    }

    preemptedRoad_ = incomingRoad;
    if (signalStage_ == SignalStage::GREEN) {
        signalStage_ = SignalStage::YELLOW;
        stageRemainingSeconds_ = yellowDurationSeconds_;
    }
    synchronizeSignalHeads();
}

bool Intersection::mustStopForRoad(const Road *road) const{
    TrafficLight* light = getLightForIncomingRoad(road);
    //không có đèn -> mặc định ko bắt dừng
    return (light != nullptr) && light->mustStop();
}

JunctionDecision Intersection::getMovementDecision(
    const Road* incomingRoad,
    const Road* outgoingRoad,
    MovementType movement) const {
    if (incomingRoad == nullptr || outgoingRoad == nullptr ||
        incomingRoad->getEnd() != this ||
        outgoingRoad->getStart() != this) {
        return JunctionDecision::Stop;
    }
    if (!mustStopForRoad(incomingRoad)) {
        return JunctionDecision::Proceed;
    }
    if (!allowRightTurnOnRed_ ||
        movement != MovementType::Right ||
        signalStage_ == SignalStage::PEDESTRIAN_WALK ||
        signalStage_ == SignalStage::PEDESTRIAN_CLEARANCE ||
        std::any_of(
            crosswalks_.begin(),
            crosswalks_.end(),
            [](const Crosswalk* crosswalk) {
                return crosswalk != nullptr &&
                       crosswalk->getSignalState() !=
                           PedestrianSignalState::DontWalk;
            })) {
        return JunctionDecision::Stop;
    }
    return JunctionDecision::Yield;
}

namespace {

bool hasPriorityVehicleApproaching(
    const Intersection& intersection,
    const Road* yieldingRoad) {
    for (const Road* road : intersection.getIncomingRoads()) {
        if (road == nullptr || road == yieldingRoad ||
            intersection.mustStopForRoad(road)) {
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
                    return true;
                }
            }
        }
    }
    return false;
}

} // namespace

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
    const bool prioritizedEmergency =
        isPrioritizedEmergencyVehicle(
            vehicleId, fromRoad);
    if (hasActiveEmergencyPriority() &&
        !prioritizedEmergency &&
        fromRoad != emergencyApproach_.incomingRoad) {
        return false;
    }
    if (prioritizedEmergency &&
        !isEmergencyPathClear(vehicleId)) {
        return false;
    }
    if (signalStage_ ==
            SignalStage::PEDESTRIAN_WALK ||
        signalStage_ ==
            SignalStage::PEDESTRIAN_CLEARANCE) {
        if (!prioritizedEmergency) {
            return false;
        }
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
    if (connector == nullptr || !canEnter(vehicleId, connector->getIncomingRoad())) {
        return false;
    }

    const double metricScale = RoadGeometry::metresPerWorldUnit(*this);
    const double step = std::max(0.5, std::min(vehicleLengthMetres, vehicleWidthMetres) * 0.5);
    const double length1 = connector->getLength();
    
    // Quick out for empty intersection
    if (occupants_.empty() || (occupants_.size() == 1 && occupants_.count(vehicleId) > 0)) {
        return true;
    }

    for (double p1 = 0.0; p1 <= length1; p1 += step) {
        const OrientedVehicleBounds candidate = makeVehicleBounds(
            connector->sampleByDistance(p1), metricScale, vehicleLengthMetres, vehicleWidthMetres, 0.5);

        for (const auto& entry : occupants_) {
            if (entry.first == vehicleId) continue;
            const Reservation& res = entry.second;
            if (res.connector == nullptr) continue;
            if (connectorsPreserveLaneOrder(
                    *connector, *res.connector) ||
                oppositeApproachMovementsAreNonCrossing(
                    *connector, *res.connector)) {
                // Lane-preserving movements and non-turning/right-turning
                // traffic from opposite green approaches do not cross.
                // Avoid resampling both complete connector paths for every
                // waiting vehicle tick. Live OBB checks still guard actual
                // traversal spacing.
                continue;
            }

            const double length2 = res.connector->getLength();
            for (double p2 = res.progressMetres; p2 <= length2; p2 += step) {
                const OrientedVehicleBounds other = makeVehicleBounds(
                    res.connector->sampleByDistance(p2), metricScale, res.vehicleLengthMetres, res.vehicleWidthMetres, 0.5);
                
                if (boundsOverlap(candidate, other)) {
                    return false; // Paths overlap, must yield
                }
            }
        }
    }
    return true;
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
            connector->getOutgoingRoad(),
            connector->getMovementType()) !=
            JunctionDecision::Yield ||
        hasPriorityVehicleApproaching(
            *this, connector->getIncomingRoad())) {
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
        Reservation{fromRoad, nullptr, 0.0, 4.5, 1.8});
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
        if (connector != nullptr) {
            existing->second.connector = connector;
        }
        existing->second.vehicleLengthMetres =
            std::max(0.1, vehicleLengthMetres);
        existing->second.vehicleWidthMetres =
            std::max(0.1, vehicleWidthMetres);
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
            std::max(0.1, vehicleWidthMetres)
        });
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
            std::max(0.1, vehicleWidthMetres)
        });
    return true;
}

void Intersection::updateReservationProgress(
    int vehicleId,
    double progressMetres) {
    auto found = occupants_.find(vehicleId);
    if (found != occupants_.end()) {
        found->second.progressMetres =
            std::max(0.0, progressMetres);
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
    const auto isSafeAt = [&](double progressMetres) {
        const OrientedVehicleBounds candidate =
            makeVehicleBounds(
                connector->sampleByDistance(progressMetres),
                metricScale,
                vehicleLengthMetres,
                vehicleWidthMetres,
                clearanceMetres);
        for (const auto& entry : occupants_) {
            if (entry.first == vehicleId) continue;
            const Reservation& reservation = entry.second;
            if (reservation.connector == nullptr) {
                return false;
            }
            const OrientedVehicleBounds other =
                makeVehicleBounds(
                    reservation.connector->sampleByDistance(
                        reservation.progressMetres),
                    metricScale,
                    reservation.vehicleLengthMetres,
                    reservation.vehicleWidthMetres,
                    clearanceMetres);
            if (boundsOverlap(candidate, other)) {
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
    occupants_.erase(vehicleId);
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
