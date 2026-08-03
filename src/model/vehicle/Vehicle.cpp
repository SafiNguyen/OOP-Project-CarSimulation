#include <iostream>
#include "Vehicle.h"
#include "Road.h"
#include "Intersection.h"
#include "TrafficLight.h"
#include "Graph.h"
#include "JunctionConnector.h"
#include "LaneMapping.h"
#include "RoadGeometry.h"
#include "algorithm/PathFindingStrategy.h"
#include "PointOfInterest.h"
#include "SpawnPoint.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdlib>
#include "VehicleMath.h"
using namespace VehicleMath;



Vehicle::Vehicle(int id, double speed, Intersection* start, Intersection* dest)
    : id(id),
      baseSpeed(speed),
      spawnPoint(start),
      destination(dest),
      currentRoad(nullptr),
      progressOnCurrentRoad(0.0),
      currentSpeed(0.0),
      currentRouteIndex(0),
      paused(false),
      pauseReason(PauseReason::None),
      laneChangeCooldownTimer(4.0) {
    patienceThreshold =
        3.0 + static_cast<double>(std::rand() % 50) / 10.0;
    recalculateTimer =
        5.0 + static_cast<double>(std::rand() % 100) / 10.0;
    rerouteRandomState_ =
        static_cast<std::uint32_t>(id) * 747796405u + 2891336453u;
}

double Vehicle::nextRerouteDelaySeconds() {
    // Small per-vehicle xorshift generator. Its state is included in the
    // snapshot so continuing from a rewound point remains deterministic.
    std::uint32_t value = rerouteRandomState_;
    value ^= value << 13u;
    value ^= value >> 17u;
    value ^= value << 5u;
    rerouteRandomState_ = value != 0u ? value : 0x9E3779B9u;
    return 10.0 +
        static_cast<double>(rerouteRandomState_ % 50u) / 10.0;
}

Vehicle::~Vehicle() {
    releaseSpawnSlot();
    if (reservedIntersection_ != nullptr) {
        reservedIntersection_->exit(getId());
        reservedIntersection_ = nullptr;
    }
    if (currentRoad != nullptr) {
        if (isMergingFromPOI) {
            currentRoad->removeMergingVehicle(this);
        } else if (currentLaneIndex >= 0 &&
                   currentLaneIndex < currentRoad->getLaneCount()) {
            currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        }
        currentRoad = nullptr;
    }
}

void Vehicle::recordTravelDistance(double distanceMetres) {
    if (std::isfinite(distanceMetres) && distanceMetres > 0.0) {
        distanceTravelledLastUpdateMetres_ += distanceMetres;
    }
}



bool Vehicle::hasReachedDestination() const {
    if (targetPOI != nullptr && currentRoad == targetPOI->getConnectedRoad()) {
        if (currentRouteIndex == static_cast<int>(currentRoute.size()) - 1) {
            if (progressOnCurrentRoad >= targetPOI->getProgressOffset()) {
                int targetLane = targetPOI->getAccessLaneIndex();
                if (targetLane < 0) targetLane = currentRoad->getCurbLaneIndex();
                if (currentLaneIndex == targetLane) {
                    if (isEnteringPOI) {
                        return poiAnimationTimer <= 0.0;
                    }
                }
                return false;
            }
        }
    }
    // If we missed it entirely and have no more road, we MUST vanish, otherwise we leak.
    if (routeAssigned && currentRoad == nullptr && currentRouteIndex >= static_cast<int>(currentRoute.size())) {
        return true;
    }

    return false;
}

PauseReason Vehicle::getIntersectionControlReason() const {
    if (currentRoad == nullptr) {
        return PauseReason::None;
    }

    Intersection* nextIntersection = currentRoad->getEnd();
    if (nextIntersection == nullptr) {
        return PauseReason::None;
    }

    const bool prioritizedEmergency =
        nextIntersection->
            isPrioritizedEmergencyVehicle(
                getId(), currentRoad);
    const LaneMapping movementMapping =
        getJunctionEntryLaneMapping();
    Road* outgoingRoad = getNextRoad();
    const JunctionDecision movementDecision =
        movementMapping.valid && outgoingRoad != nullptr
            ? getJunctionDecision(
                  nextIntersection,
                  movementMapping,
                  outgoingRoad)
            : (prioritizedEmergency
                   ? JunctionDecision::Proceed
                   : nextIntersection->mustStopForRoad(currentRoad)
                         ? JunctionDecision::Stop
                         : JunctionDecision::Proceed);
    if (movementDecision == JunctionDecision::Stop) {
        return PauseReason::TrafficLight;
    }

    if (reservedIntersection_ != nextIntersection &&
        !currentRoad->getLane(currentLaneIndex).isBlocked()) {
        if (outgoingRoad != nullptr) {
            if (!movementMapping.valid ||
                currentLaneIndex !=
                    movementMapping.incomingLane) {
                return PauseReason::Intersection;
            }
            Vehicle* outgoingLeader =
                outgoingRoad->getFirstVehicleInLane(
                    movementMapping.outgoingLane);
            if (outgoingLeader != nullptr &&
                outgoingReleaseGap(
                    *this, *outgoingLeader) <
                    std::max(
                        getMinGap(),
                        outgoingLeader->getMinGap())) {
                return PauseReason::Intersection;
            }
            auto connector = nextIntersection->getConnector(
                currentRoad,
                movementMapping.incomingLane,
                outgoingRoad,
                movementMapping.outgoingLane);
            const double requiredGap =
                getMinGap() + currentSpeed * getTimeHeadway();
            const bool canEnter =
                movementDecision == JunctionDecision::Yield
                    ? nextIntersection->canEnterYieldingMovement(
                          getId(),
                          connector,
                          requiredGap,
                          getLength(),
                          getWidth())
                    : nextIntersection->canEnterMovement(
                          getId(),
                          connector,
                          requiredGap,
                          getLength(),
                          getWidth());
            if (!canEnter) {
                return PauseReason::Intersection;
            }
        } else if (!nextIntersection->canEnter(
                       getId(), currentRoad)) {
            return PauseReason::Intersection;
        }
    }

    return PauseReason::None;
}

double Vehicle::getIntersectionStopPosition() const {
    if (currentRoad == nullptr) {
        return 0.0;
    }
    const double nominalStopPosition = std::max(
        0.0,
        RoadGeometry::stopLineProgressMetres(*currentRoad) -
            getLength() * 0.5);
    Intersection* intersection = currentRoad->getEnd();
    if (intersection == nullptr) {
        return nominalStopPosition;
    }
    return intersection->constrainIncomingStopPosition(
        currentRoad,
        currentLaneIndex,
        nominalStopPosition,
        getLength(),
        getMinGap());
}

void Vehicle::beginPause(PauseReason reason) {
    if (reason == PauseReason::None) {
        clearPause();
        return;
    }

    const bool pauseJustStarted = !paused || pauseReason != reason;
    paused = true;
    pauseReason = reason;
    movementState_ = reason == PauseReason::BusStop
        ? MovementState::DwellingAtBusStop
        : MovementState::WaitingAtIntersection;
    currentSpeed = 0.0;
    if (pauseJustStarted) {
        onPauseStarted();
    }
}

void Vehicle::clearPause() {
    paused = false;
    pauseReason = PauseReason::None;
    if (movementState_ != MovementState::TraversingJunction) {
        movementState_ = MovementState::OnRoad;
    }
}

bool Vehicle::shouldPauseAt(double currentPos,
                            double projectedPos,
                            double& pausePos) {
    if (currentRoad == nullptr) return false;

    const double nominalStopPosition = std::max(
        0.0,
        RoadGeometry::stopLineProgressMetres(*currentRoad) -
            getLength() * 0.5);
    if (projectedPos + 1e-9 < nominalStopPosition) {
        return false;
    }

    const PauseReason controlReason = getIntersectionControlReason();
    if (controlReason == PauseReason::None) return false;

    // Position of the nominal stop line a short distance before the road end.
    double stopLinePos = getIntersectionStopPosition();

    // A red-light queue is still waiting for the signal. By contrast, a
    // vehicle held behind an intersection-waiting leader is stopped by that
    // leader, not directly by the box reservation.
    if (controlReason == PauseReason::TrafficLight) {
        Vehicle* leader = currentRoad->findLeader(currentLaneIndex, this);
        if (leader != nullptr && leader->isPaused()) {
            const double requiredGap =
                std::max(getMinGap(), leader->getMinGap());
            const double desiredPos =
                leader->getProgressOnRoad() -
                combinedHalfLength(*this, *leader) -
                requiredGap;
            // Don't allow desiredPos to go past the nominal stop line further upstream
            if (desiredPos < stopLinePos) {
                stopLinePos = desiredPos;
            }
        }
    }

    // Pause only when this movement step actually reaches the stop line.
    if (stopLinePos > currentPos && projectedPos < stopLinePos) return false;

    // Never move backwards if the control state changed after the vehicle
    // had already crossed the nominal stop line.
    pausePos = std::max(currentPos, stopLinePos);
    pauseReason = controlReason;
    return true;
}

void Vehicle::setRoute(const std::vector<Road*>& route) {
    setRouteAt(route, -1, 0.0);
}

bool Vehicle::setRouteAt(const std::vector<Road*>& route,
                         int initialLaneIndex,
                         double initialProgressMetres) {
    if (reservedIntersection_ != nullptr) {
        reservedIntersection_->exit(getId());
        reservedIntersection_ = nullptr;
    }
    if (currentRoad != nullptr &&
        movementState_ != MovementState::TraversingJunction) {
        if (isMergingFromPOI) {
            currentRoad->removeMergingVehicle(this);
        } else if (currentLaneIndex >= 0 &&
                   currentLaneIndex < currentRoad->getLaneCount()) {
            currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        }
    }
    currentRoute = route;
    currentRouteIndex = 0;
    progressOnCurrentRoad = 0.0;
    currentSpeed = 0.0;
    clearPause();
    activeConnector_.reset();
    junctionOutgoingRoad_ = nullptr;
    junctionProgressMetres_ = 0.0;
    movementState_ = MovementState::OnRoad;
    clearLaneChangeIntent();
    junctionTurnSignal_ = TurnSignal::Off;
    routeAssigned = true;

    if (!currentRoute.empty()) {
        currentRoad = currentRoute[0];
        if (currentRoad) {
            if (initialLaneIndex >= 0 &&
                initialLaneIndex < currentRoad->getLaneCount() &&
                !currentRoad->getLane(initialLaneIndex).isBlocked()) {
                currentLaneIndex = initialLaneIndex;
            } else {
                currentLaneIndex = currentRoad->getFreestLaneIndex();
            }
            progressOnCurrentRoad = std::clamp(
                initialProgressMetres,
                0.0,
                currentRoad->getDistance());
            
            if (isMergingFromPOI) {
                currentRoad->addMergingVehicle(this);
                poiMergePhase_ =
                    PoiMergePhase::ApproachingYieldLine;
                poiAnimationTimer =
                    getPoiMergePhaseDuration(
                        poiMergePhase_);
            } else {
                currentRoad->getLane(currentLaneIndex).addVehicle(this);
                spawnLifecycleState_ =
                    SpawnLifecycleState::Active;
            }
        }
    } else {
        currentRoad = nullptr;
    }

    onRoadChanged();
    refreshTurnSignal();
    return currentRoute.empty() || currentRoad != nullptr;
}


double Vehicle::getProgressRatio() const {
    if (currentRoad == nullptr || currentRoad->getDistance() <= 0.0) {
        return 0.0;
    }
    return progressOnCurrentRoad / currentRoad->getDistance();
}

Pose2D Vehicle::getPose() const {
    if (movementState_ == MovementState::TraversingJunction &&
        activeConnector_ != nullptr) {
        return activeConnector_->sampleByDistance(
            junctionProgressMetres_);
    }
    if (poseTransitionTimer_ > 0.0 &&
        poseTransitionDuration_ > 0.0) {
        const double linearRatio = 1.0 -
            std::clamp(
                poseTransitionTimer_ / poseTransitionDuration_,
                0.0,
                1.0);
        const double easedRatio = smoothStep(linearRatio);
        const Pose2D fromPose = poseTransitionFrom_;
        const Pose2D toPose = poseTransitionTo_;
        const double headingDelta = std::remainder(
            toPose.headingRadians - fromPose.headingRadians,
            2.0 * 3.14159265358979323846);
        Pose2D pose;
        pose.position = {
            fromPose.position.x +
                (toPose.position.x - fromPose.position.x) * easedRatio,
            fromPose.position.y +
                (toPose.position.y - fromPose.position.y) * easedRatio
        };
        pose.headingRadians = fromPose.headingRadians +
            headingDelta * easedRatio;
        if (poseTransitionDuration_ <=
            LANE_CHANGE_POSE_TRANSITION_SECONDS + 1e-9) {
            const Vec2 transitionDelta =
                toPose.position - fromPose.position;
            const Vec2 forwardDirection{
                std::cos(fromPose.headingRadians),
                std::sin(fromPose.headingRadians)};
            const double leanDirection =
                dot(transitionDelta, rightNormal(forwardDirection)) >= 0.0
                    ? 1.0
                    : -1.0;
            pose.headingRadians += leanDirection *
                laneChangeLeanMagnitude(easedRatio);
        }
        return pose;
    }
    if (currentRoad == nullptr) {
        return {};
    }

    Pose2D roadPose = RoadGeometry::sampleLane(
        *currentRoad, currentLaneIndex, progressOnCurrentRoad);

    const PointOfInterest* mergeSource =
        mergeSourcePOI_ != nullptr
            ? mergeSourcePOI_
            : spawnPOI;
    if (isMergingFromPOI && mergeSource != nullptr) {
        const RoadGeometry::RoadAccessPath accessPath =
            RoadGeometry::makeRoadAccessPath(
                *currentRoad,
                currentLaneIndex,
                progressOnCurrentRoad,
                {mergeSource->getX(),
                 mergeSource->getY()});
        const double yieldRatio =
            getPoiMergeYieldPathRatio();
        double ratio = yieldRatio;
        if (poiMergePhase_ ==
            PoiMergePhase::ApproachingYieldLine) {
            const double duration =
                getPoiMergePhaseDuration(
                    poiMergePhase_);
            const double linearRatio = duration > 0.0
                ? 1.0 - poiAnimationTimer / duration
                : 1.0;
            ratio = yieldRatio * smoothStep(linearRatio);
        } else if (poiMergePhase_ ==
                   PoiMergePhase::Committed) {
            const double duration =
                getPoiMergePhaseDuration(
                    poiMergePhase_);
            const double linearRatio = duration > 0.0
                ? 1.0 - poiAnimationTimer / duration
                : 1.0;
            ratio = yieldRatio +
                (1.0 - yieldRatio) *
                    smoothStep(linearRatio);
        }
        return RoadGeometry::sampleRoadAccessPath(
            accessPath, ratio);
    }
    
    if (isEnteringPOI && targetPOI != nullptr) {
        if (poiAnimationTimer > 0) {
            double linearRatio = 1.0 - (poiAnimationTimer / poiAnimationDuration);
            // Decelerate into the destination along the same driveway used
            // for departures, sampled in reverse.
            double ratio = linearRatio * (2.0 - linearRatio);
            const RoadGeometry::RoadAccessPath accessPath =
                RoadGeometry::makeRoadAccessPath(
                    *currentRoad,
                    currentLaneIndex,
                    progressOnCurrentRoad,
                    {targetPOI->getX(),
                     targetPOI->getY()});
            Pose2D pose =
                RoadGeometry::sampleRoadAccessPath(
                    accessPath, 1.0 - ratio);
            pose.headingRadians +=
                3.14159265358979323846;
            return pose;
        }
    }

    Pose2D pose = roadPose;
    if (laneChangeState_ != LaneChangeState::Idle &&
        laneChangeTargetLane_ != currentLaneIndex) {
        const double signalLeadTime = hasTrafficPriority()
            ? PRIORITY_SIGNAL_LEAD_TIME_SECONDS
            : MIN_SIGNAL_LEAD_TIME_SECONDS;
        const double signalRatio = laneChangeState_ == LaneChangeState::Signaling
            ? smoothStep(signalLeadTime > 0.0
                ? laneChangeSignalElapsedSeconds_ / signalLeadTime
                : 1.0)
            : 1.0;
        pose.headingRadians +=
            laneChangeLeanDirection(
                laneChangeTargetLane_, currentLaneIndex) *
            laneChangeLeanMagnitude(signalRatio);
    }

    return pose;
}

LaneMapping Vehicle::getUpcomingLaneMapping() const {
    Road* nextRoad = getNextRoad();
    if (currentRoad == nullptr || nextRoad == nullptr) {
        return {};
    }
    bool isDestinationRoad = false;
    if (!currentRoute.empty() && currentRouteIndex + 1 == static_cast<int>(currentRoute.size()) - 1) {
        isDestinationRoad = true;
    }
    return TurnLanePolicy::map(
        *currentRoad, currentLaneIndex, *nextRoad, true, isDestinationRoad);
}

LaneMapping Vehicle::getJunctionEntryLaneMapping() const {
    const LaneMapping preferred = getUpcomingLaneMapping();
    if (!preferred.valid ||
        preferred.incomingLane == currentLaneIndex) {
        return preferred;
    }

    Road* nextRoad = getNextRoad();
    if (currentRoad == nullptr || nextRoad == nullptr) {
        return {};
    }

    bool isDestinationRoad = false;
    if (!currentRoute.empty() && currentRouteIndex + 1 == static_cast<int>(currentRoute.size()) - 1) {
        isDestinationRoad = true;
    }

    // Lane preparation remains the preferred behavior. If a red-light queue
    // prevented the merge, use the vehicle's actual lane at the stop line
    // instead of leaving that lane permanently blocked after green.
    return TurnLanePolicy::mapFromCurrentLane(
        *currentRoad,
        currentLaneIndex,
        *nextRoad,
        true,
        isDestinationRoad);
}

JunctionDecision Vehicle::getJunctionDecision(
    Intersection* intersection,
    const LaneMapping& mapping,
    Road* outgoingRoad) const {
    if (intersection == nullptr || currentRoad == nullptr ||
        outgoingRoad == nullptr || !mapping.valid) {
        return JunctionDecision::Stop;
    }
    if (intersection->isPrioritizedEmergencyVehicle(
            getId(), currentRoad)) {
        return JunctionDecision::Proceed;
    }

    const JunctionDecision decision =
        intersection->getMovementDecision(
            currentRoad,
            mapping.incomingLane,
            outgoingRoad,
            mapping.movement);
    if (decision == JunctionDecision::Yield &&
        !canTurnRightOnRed()) {
        return JunctionDecision::Stop;
    }
    return decision;
}

int Vehicle::getRedLightCurbYieldLane() const {
    if (getVehicleKind() != VehicleKind::Car ||
        currentRoad == nullptr || getNextRoad() == nullptr ||
        movementState_ == MovementState::TraversingJunction ||
        currentRoad->getLaneCount() <= 1) {
        return -1;
    }

    Intersection* intersection = currentRoad->getEnd();
    const TrafficLight* light = intersection != nullptr
        ? intersection->getLightForIncomingRoad(currentRoad)
        : nullptr;
    if (intersection == nullptr || light == nullptr ||
        light->getState() != LightState::RED ||
        !intersection->allowsRightTurnOnRed()) {
        return -1;
    }

    const double distanceToJunction =
        currentRoad->getDistance() - progressOnCurrentRoad;
    const double yieldDistance = std::max(
        RED_LIGHT_CURB_YIELD_DISTANCE_METRES,
        std::max(0.0, currentSpeed) *
            (MIN_SIGNAL_LEAD_TIME_SECONDS + 1.0));
    if (distanceToJunction > yieldDistance) {
        return -1;
    }

    const auto outgoingRoads = intersection->getOutgoingRoads();
    const bool hasRightTurnExit = std::any_of(
        outgoingRoads.begin(),
        outgoingRoads.end(),
        [this, intersection](const Road* road) {
            return road != nullptr &&
                   road->getStart() == intersection &&
                   TurnLanePolicy::classify(
                       *currentRoad, *road) ==
                       MovementType::Right;
        });
    if (!hasRightTurnExit) {
        return -1;
    }

    const int curbLane = currentRoad->getCurbLaneIndex();
    const int preferredLane = std::min(
        currentLaneIndex, curbLane - 1);
    for (int offset = 0; offset < curbLane; ++offset) {
        const int lowerLane = preferredLane - offset;
        if (lowerLane >= 0 &&
            !currentRoad->getLane(lowerLane).isBlocked()) {
            return lowerLane;
        }
        const int upperLane = preferredLane + offset;
        if (offset > 0 && upperLane < curbLane &&
            !currentRoad->getLane(upperLane).isBlocked()) {
            return upperLane;
        }
    }
    return -1;
}

bool Vehicle::beginJunctionTraversal(
    const LaneMapping& mapping,
    Intersection* intersection) {
    return junctionTraversalState.beginTraversal(*this, mapping, intersection);
}

void Vehicle::completeJunctionTraversal(
    double outgoingProgressMetres) {
    junctionTraversalState.completeTraversal(*this, outgoingProgressMetres);
    laneChangeCooldownTimer = 4.0; // Delay lane change immediately after intersection
}

double Vehicle::advanceJunction(double availableTime) {
    return junctionTraversalState.advance(*this, availableTime);
}

bool Vehicle::requestLaneChange(
    int targetLaneIndex,
    TurnSignalReason reason) {
    if (currentRoad == nullptr ||
        targetLaneIndex < 0 ||
        targetLaneIndex >= currentRoad->getLaneCount() ||
        targetLaneIndex == currentLaneIndex) {
        return false;
    }
    if (laneChangeState_ == LaneChangeState::Idle ||
        laneChangeTargetLane_ != targetLaneIndex ||
        laneChangeReason_ != reason) {
        laneChangeTargetLane_ = targetLaneIndex;
        laneChangeReason_ = reason;
        laneChangeSignalElapsedSeconds_ = 0.0;
        laneChangeState_ = LaneChangeState::Signaling;
        refreshTurnSignal();
    }
    return true;
}

void Vehicle::startPoseTransition(
    const Pose2D& fromPose,
    const Pose2D& toPose,
    double durationSeconds) {
    poseTransitionFrom_ = fromPose;
    poseTransitionTo_ = toPose;
    poseTransitionDuration_ = std::max(0.0, durationSeconds);
    poseTransitionTimer_ = poseTransitionDuration_;
}


TurnSignal Vehicle::deriveUpcomingJunctionSignal() const {
    const LaneMapping mapping = getUpcomingLaneMapping();
    if (!mapping.valid) {
        return TurnSignal::Off;
    }
    switch (mapping.movement) {
        case MovementType::Right:
            return TurnSignal::Right;
        case MovementType::Left:
        case MovementType::UTurn:
            return TurnSignal::Left;
        case MovementType::Straight:
        default:
            return TurnSignal::Off;
    }
}


bool Vehicle::isTurnSignalBlinkOn() const {
    if (turnSignal_ == TurnSignal::Off) {
        return false;
    }
    const double phase = std::fmod(
        std::max(0.0, simulationTimeSeconds_),
        TURN_SIGNAL_BLINK_PERIOD_SECONDS);
    return phase < TURN_SIGNAL_BLINK_PERIOD_SECONDS * 0.5;
}

bool Vehicle::tryRequiredLaneChange(
    int requiredLaneIndex,
    TurnSignalReason reason) {
    if (currentRoad == nullptr ||
        !canChangeLanes() ||
        requiredLaneIndex < 0 ||
        requiredLaneIndex >= currentRoad->getLaneCount() ||
        requiredLaneIndex == currentLaneIndex) {
        if (requiredLaneIndex == currentLaneIndex) {
            clearLaneChangeIntent();
            refreshTurnSignal();
        }
        return false;
    }

    requestLaneChange(requiredLaneIndex, reason);
    const double signalLeadTime = hasTrafficPriority()
        ? PRIORITY_SIGNAL_LEAD_TIME_SECONDS
        : MIN_SIGNAL_LEAD_TIME_SECONDS;
    if (laneChangeSignalElapsedSeconds_ + 1e-9 < signalLeadTime) {
        laneChangeState_ = LaneChangeState::Signaling;
        return false;
    }
    laneChangeState_ = LaneChangeState::WaitingForGap;
    const int adjacentLane = currentLaneIndex +
        (requiredLaneIndex > currentLaneIndex ? 1 : -1);
    const LaneChangeCandidate candidate = laneChangePolicy.assess(
        *this,
        *currentRoad,
        adjacentLane,
        LANE_CHANGE_REAR_SAFETY_TIME,
        LANE_CHANGE_MIN_TTC);
    if (!candidate.safe) {
        return false;
    }

    currentRoad->getLane(currentLaneIndex).removeVehicle(this);
    const Pose2D fromPose = RoadGeometry::sampleLane(
        *currentRoad,
        currentLaneIndex,
        progressOnCurrentRoad);
    const Pose2D toPose = RoadGeometry::sampleLane(
        *currentRoad,
        adjacentLane,
        progressOnCurrentRoad);
    currentLaneIndex = adjacentLane;
    currentRoad->getLane(currentLaneIndex).addVehicle(this);
    startPoseTransition(
        fromPose,
        toPose,
        LANE_CHANGE_POSE_TRANSITION_SECONDS);
    if (currentLaneIndex == requiredLaneIndex) {
        clearLaneChangeIntent();
        laneChangeCooldownTimer = hasTrafficPriority()
            ? PRIORITY_LANE_CHANGE_COOLDOWN
            : LANE_CHANGE_COOLDOWN;
    } else {
        laneChangeSignalElapsedSeconds_ = 0.0;
        laneChangeState_ = LaneChangeState::Signaling;
    }
    refreshTurnSignal();
    return true;
}



Road* Vehicle::getNextRoad() const {
    if (currentRouteIndex + 1 < static_cast<int>(currentRoute.size())) {
        return currentRoute[currentRouteIndex + 1];
    }
    return nullptr;
}


bool Vehicle::isRoadInUpcomingRoute(int roadId) const {
    for (size_t i = currentRouteIndex + 1; i < currentRoute.size(); ++i) {
        if (currentRoute[i]->getId() == roadId) {
            return true;
        }
    }
    return false;
}


bool Vehicle::performUTurn(const Graph& graph, PathFindingStrategy* strategy) {
    return routeFollower.performUTurn(*this, graph, strategy);
}

double Vehicle::getAcceleration() const { return 3.0; }
double Vehicle::getDeceleration() const { return 5.0; }
double Vehicle::getMaxLateralAcceleration() const { return 3.0; }
VehicleKind Vehicle::getVehicleKind() const { return VehicleKind::Car; }
bool Vehicle::canChangeLanes() const { return true; }
PauseUpdateResult Vehicle::updatePause(double availableTime) {
    return {true, availableTime};
}
double Vehicle::getYieldSpeedFactor() const { return YIELD_SPEED_FACTOR; }
double Vehicle::getYieldEscapeSpeedFactor() const { return YIELD_ESCAPE_SPEED_FACTOR; }
double Vehicle::getLength() const { return 4.5; }    // metres
double Vehicle::getWidth() const { return 1.8; }     // metres
double Vehicle::getHeight() const { return 1.5; }    // metres
double Vehicle::getWeight() const { return 1.5; }    // tonnes
double Vehicle::getMiniGap() const { return 0.0; }
double Vehicle::getMinGap() const { return getMiniGap(); }
double Vehicle::getTimeHeadway() const { return 1.5; }

double Vehicle::getPoiMergeYieldPathRatio() const {
    const PointOfInterest* source =
        mergeSourcePOI_ != nullptr
            ? mergeSourcePOI_
            : spawnPOI;
    if (currentRoad == nullptr || source == nullptr ||
        mergeLaneIndex < 0 ||
        mergeLaneIndex >= currentRoad->getLaneCount()) {
        return 0.5;
    }

    const RoadGeometry::RoadAccessPath accessPath =
        RoadGeometry::makeRoadAccessPath(
            *currentRoad,
            mergeLaneIndex,
            mergeProgressOffset,
            {source->getX(), source->getY()});
    const Vec2 inward = normalized(
        accessPath.lanePose.position - accessPath.curb,
        rightNormal(RoadGeometry::roadDirection(*currentRoad)));
    const double centreClearanceWorld =
        (getLength() * 0.5 +
         POI_MERGE_YIELD_BUFFER_METRES) /
        RoadGeometry::metresPerWorldUnit(*currentRoad);
    const Vec2 yieldPosition =
        accessPath.curb - inward * centreClearanceWorld;
    return RoadGeometry::roadAccessPathProgressAt(
        accessPath,
        yieldPosition);
}

double Vehicle::getPoiMergePhaseDuration(
    PoiMergePhase phase) const {
    const double yieldRatio =
        getPoiMergeYieldPathRatio();
    if (phase == PoiMergePhase::ApproachingYieldLine) {
        return std::max(
            MIN_POI_MERGE_PHASE_SECONDS,
            poiAnimationDuration * yieldRatio);
    }
    if (phase == PoiMergePhase::Committed) {
        return std::max(
            MIN_POI_MERGE_PHASE_SECONDS,
            poiAnimationDuration * (1.0 - yieldRatio));
    }
    return 0.0;
}

bool Vehicle::canCommitPoiMerge() const {
    if (currentRoad == nullptr ||
        mergeLaneIndex < 0 ||
        mergeLaneIndex >= currentRoad->getLaneCount()) {
        return false;
    }

    const Lane& mergeLane =
        currentRoad->getLane(mergeLaneIndex);
    if (mergeLane.isBlocked()) {
        return false;
    }

    // Lane capacity is an admission/queueing limit, not a spatial gap.
    // A long lane can reach that count while every vehicle is far away from
    // this driveway.  The leader/follower checks below are the authoritative
    // local safety test and prevent a POI vehicle from starving outside an
    // otherwise empty merge area.

    Intersection* entrance = currentRoad->getStart();
    if (entrance != nullptr &&
        entrance->isOutgoingLaneReserved(
            currentRoad,
            mergeLaneIndex)) {
        return false;
    }

    Vehicle* leader =
        currentRoad->findLeader(
            mergeLaneIndex, this);
    if (leader != nullptr) {
        const double frontGap =
            leader->getProgressOnRoad() -
            mergeProgressOffset -
            combinedHalfLength(*this, *leader);
        const double requiredFrontGap =
            std::max(getMinGap(), leader->getMinGap());
        if (frontGap + 1e-6 < requiredFrontGap) {
            return false;
        }
    }

    Vehicle* follower =
        currentRoad->findFollower(
            mergeLaneIndex, this);
    if (follower == nullptr) {
        return true;
    }

    const double rearGap =
        mergeProgressOffset -
        follower->getProgressOnRoad() -
        combinedHalfLength(*this, *follower);
    const double followerSpeed =
        std::max(0.0, follower->getCurrentSpeed());
    const double baseGap =
        std::max(getMinGap(), follower->getMinGap());
    const double reactionDistance =
        followerSpeed * LANE_CHANGE_REACTION_TIME;
    const double brakingDistance =
        followerSpeed * followerSpeed /
        (2.0 * std::max(
            follower->getDeceleration(), 1e-6));
    const double requiredRearGap =
        baseGap + reactionDistance * 0.5 + brakingDistance * 0.4; // Aggressive merge: require less gap for POI vehicles
    if (rearGap + 1e-6 < requiredRearGap) {
        return false;
    }

    if (followerSpeed > 1e-6 &&
        rearGap / followerSpeed + 1e-9 <
            LANE_CHANGE_REAR_SAFETY_TIME) {
        return false;
    }
    return true;
}

void Vehicle::updatePoiAnimation(double dt) {
    if (poiAnimationTimer > 0.0) {
        poiAnimationTimer = std::max(
            0.0,
            poiAnimationTimer - std::max(0.0, dt));
    }
}



double Vehicle::getIntersectionTransitionProgress() const {
    if (movementState_ != MovementState::TraversingJunction ||
        activeConnector_ == nullptr) {
        return 0.0;
    }
    const double length = activeConnector_->getLength();
    if (length <= 0.0) {
        return 1.0;
    }
    return junctionProgressMetres_ / length;
}

bool Vehicle::isStuckInJam(int depth) const {
    if (depth > 100) return true; // safety limit against deep recursion
    if (currentSpeed > 0.01) return false;
    if (isWaitingForLight_) return false;
    if (currentLeader_ != nullptr) return currentLeader_->isStuckInJam(depth + 1);
    return true; 
}

int Vehicle::getRequiredLaneIndex() const {
    if (targetPOI != nullptr && currentRoad == targetPOI->getConnectedRoad() &&
        currentRouteIndex == static_cast<int>(currentRoute.size()) - 1) {
        int targetLane = targetPOI->getAccessLaneIndex();
        if (targetLane < 0) targetLane = currentRoad->getCurbLaneIndex();
        return targetLane;
    }
    return -1;
}

// --- Snapshot support (Memento pattern) ---

void Vehicle::captureSnapshot(VehicleSnapshot& snap,
                              const Graph& graph) const {
    snap.id = id;
    snap.kind = getVehicleKind();
    snap.currentRoadId = currentRoad != nullptr
        ? std::optional<int>(currentRoad->getId())
        : std::nullopt;
    snap.currentLaneIndex = currentLaneIndex;
    snap.progressOnCurrentRoad = progressOnCurrentRoad;
    snap.currentSpeed = currentSpeed;

    snap.currentRoute.clear();
    snap.currentRoute.reserve(currentRoute.size());
    for (const Road* road : currentRoute) {
        snap.currentRoute.push_back(
            road != nullptr
                ? std::optional<int>(road->getId())
                : std::nullopt);
    }
    snap.currentRouteIndex = currentRouteIndex;

    snap.travelHistory.clear();
    snap.travelHistory.reserve(travelHistory.size());
    for (const Road* road : travelHistory) {
        snap.travelHistory.push_back(
            road != nullptr
                ? std::optional<int>(road->getId())
                : std::nullopt);
    }

    snap.paused = paused;
    snap.pauseReason = pauseReason;
    snap.movementState = movementState_;

    snap.junctionIncomingRoadId = currentRoad != nullptr
        ? std::optional<int>(currentRoad->getId())
        : std::nullopt;
    snap.junctionIncomingLane = incomingLaneIndex_;
    snap.junctionOutgoingRoadId = junctionOutgoingRoad_ != nullptr
        ? std::optional<int>(junctionOutgoingRoad_->getId())
        : std::nullopt;
    snap.junctionOutgoingLane = outgoingLaneIndex_;
    snap.junctionProgressMetres = junctionProgressMetres_;
    snap.reservedIntersectionId =
        reservedIntersection_ != nullptr ? reservedIntersection_->getId() : -1;

    snap.laneChangeCooldownTimer = laneChangeCooldownTimer;
    snap.yielding = yielding;
    snap.yieldCooldownTimer = yieldCooldownTimer;
    snap.emergencyLaneToAvoid = emergencyLaneToAvoid;
    snap.laneChangeState = laneChangeState_;
    snap.laneChangeTargetLane = laneChangeTargetLane_;
    snap.laneChangeReason = laneChangeReason_;
    snap.laneChangeSignalElapsedSeconds = laneChangeSignalElapsedSeconds_;
    snap.turnSignal = turnSignal_;
    snap.turnSignalReason = turnSignalReason_;
    snap.junctionTurnSignal = junctionTurnSignal_;

    snap.poseTransitionFromX = poseTransitionFrom_.position.x;
    snap.poseTransitionFromY = poseTransitionFrom_.position.y;
    snap.poseTransitionFromHeading = poseTransitionFrom_.headingRadians;
    snap.poseTransitionToX = poseTransitionTo_.position.x;
    snap.poseTransitionToY = poseTransitionTo_.position.y;
    snap.poseTransitionToHeading = poseTransitionTo_.headingRadians;
    snap.poseTransitionTimer = poseTransitionTimer_;
    snap.poseTransitionDuration = poseTransitionDuration_;

    snap.stuckTimer = stuckTimer;
    snap.patienceThreshold = patienceThreshold;
    snap.uTurnCooldownTimer = uTurnCooldownTimer;
    snap.simulationTimeSeconds = simulationTimeSeconds_;
    snap.recalculateTimer = recalculateTimer;
    snap.isWaitingForLight = isWaitingForLight_;

    snap.isMergingFromPOI = isMergingFromPOI;
    snap.poiMergePhase = poiMergePhase_;
    snap.isEnteringPOI = isEnteringPOI;
    snap.mergeSourcePOIId =
        mergeSourcePOI_ != nullptr ? mergeSourcePOI_->getId() : -1;
    snap.mergeProgressOffset = mergeProgressOffset;
    snap.mergeLaneIndex = mergeLaneIndex;
    snap.poiAnimationTimer = poiAnimationTimer;
    snap.poiAnimationDuration = poiAnimationDuration;

    snap.spawnPointId =
        spawnPoint != nullptr ? spawnPoint->getId() : -1;
    snap.destinationId =
        destination != nullptr ? destination->getId() : -1;
    snap.baseSpeed = baseSpeed;
    snap.spawnPOIId = spawnPOI != nullptr ? spawnPOI->getId() : -1;
    snap.targetPOIId = targetPOI != nullptr ? targetPOI->getId() : -1;
    snap.spawnLifecycleState = spawnLifecycleState_;
    snap.reservedSpawnPointId =
        reservedSpawnPoint_ != nullptr ? reservedSpawnPoint_->getId() : -1;
    snap.rerouteRandomState = rerouteRandomState_;
}

void Vehicle::restoreSnapshot(const VehicleSnapshot& snap,
                              Graph& graph,
                              bool restoreReservations) {
    id = snap.id;
    baseSpeed = snap.baseSpeed;
    spawnPoint = snap.spawnPointId >= 0
        ? graph.getIntersection(snap.spawnPointId)
        : nullptr;
    destination = snap.destinationId >= 0
        ? graph.getIntersection(snap.destinationId)
        : nullptr;
    currentLaneIndex = snap.currentLaneIndex;
    progressOnCurrentRoad = snap.progressOnCurrentRoad;
    currentSpeed = snap.currentSpeed;

    // Restore route (road id -> Road*).
    currentRoute.clear();
    currentRoute.reserve(snap.currentRoute.size());
    for (const std::optional<int>& roadId : snap.currentRoute) {
        currentRoute.push_back(
            roadId.has_value() ? graph.getRoad(*roadId) : nullptr);
    }
    currentRouteIndex = snap.currentRouteIndex;
    routeAssigned = true;

    paused = snap.paused;
    pauseReason = snap.pauseReason;
    movementState_ = snap.movementState;

    // currentRoadId is authoritative. Pending vehicles can already own a
    // resolved route while still being detached from every Road.
    currentRoad = snap.currentRoadId.has_value()
        ? graph.getRoad(*snap.currentRoadId)
        : nullptr;

    travelHistory.clear();
    travelHistory.reserve(snap.travelHistory.size());
    for (const std::optional<int>& roadId : snap.travelHistory) {
        travelHistory.push_back(
            roadId.has_value() ? graph.getRoad(*roadId) : nullptr);
    }

    incomingLaneIndex_ = snap.junctionIncomingLane;
    outgoingLaneIndex_ = snap.junctionOutgoingLane;
    junctionProgressMetres_ = snap.junctionProgressMetres;
    junctionOutgoingRoad_ = snap.junctionOutgoingRoadId.has_value()
        ? graph.getRoad(*snap.junctionOutgoingRoadId)
        : nullptr;
    reservedIntersection_ =
        snap.reservedIntersectionId >= 0
            ? graph.getIntersection(snap.reservedIntersectionId)
            : nullptr;

    activeConnector_.reset();
    if (movementState_ == MovementState::TraversingJunction &&
        currentRoad != nullptr &&
        junctionOutgoingRoad_ != nullptr) {
        Intersection* intersection = currentRoad->getEnd();
        if (intersection != nullptr) {
            activeConnector_ = intersection->getConnector(
                currentRoad,
                incomingLaneIndex_,
                junctionOutgoingRoad_,
                outgoingLaneIndex_);
        }
    }

    laneChangeCooldownTimer = snap.laneChangeCooldownTimer;
    yielding = snap.yielding;
    yieldCooldownTimer = snap.yieldCooldownTimer;
    emergencyLaneToAvoid = snap.emergencyLaneToAvoid;
    laneChangeState_ = snap.laneChangeState;
    laneChangeTargetLane_ = snap.laneChangeTargetLane;
    laneChangeReason_ = snap.laneChangeReason;
    laneChangeSignalElapsedSeconds_ = snap.laneChangeSignalElapsedSeconds;
    turnSignal_ = snap.turnSignal;
    turnSignalReason_ = snap.turnSignalReason;
    junctionTurnSignal_ = snap.junctionTurnSignal;

    poseTransitionFrom_.position = {
        snap.poseTransitionFromX, snap.poseTransitionFromY};
    poseTransitionFrom_.headingRadians = snap.poseTransitionFromHeading;
    poseTransitionTo_.position = {
        snap.poseTransitionToX, snap.poseTransitionToY};
    poseTransitionTo_.headingRadians = snap.poseTransitionToHeading;
    poseTransitionTimer_ = snap.poseTransitionTimer;
    poseTransitionDuration_ = snap.poseTransitionDuration;

    stuckTimer = snap.stuckTimer;
    patienceThreshold = snap.patienceThreshold;
    uTurnCooldownTimer = snap.uTurnCooldownTimer;
    simulationTimeSeconds_ = snap.simulationTimeSeconds;
    recalculateTimer = snap.recalculateTimer;
    isWaitingForLight_ = snap.isWaitingForLight;

    isMergingFromPOI = snap.isMergingFromPOI;
    poiMergePhase_ = snap.poiMergePhase;
    isEnteringPOI = snap.isEnteringPOI;
    mergeSourcePOI_ = snap.mergeSourcePOIId >= 0
        ? graph.getPointOfInterest(snap.mergeSourcePOIId)
        : nullptr;
    mergeProgressOffset = snap.mergeProgressOffset;
    mergeLaneIndex = snap.mergeLaneIndex;
    poiAnimationTimer = snap.poiAnimationTimer;
    poiAnimationDuration = snap.poiAnimationDuration;

    spawnPOI = snap.spawnPOIId >= 0
        ? graph.getPointOfInterest(snap.spawnPOIId)
        : nullptr;
    targetPOI = snap.targetPOIId >= 0
        ? graph.getPointOfInterest(snap.targetPOIId)
        : nullptr;
    spawnLifecycleState_ = snap.spawnLifecycleState;
    rerouteRandomState_ = snap.rerouteRandomState != 0u
        ? snap.rerouteRandomState
        : static_cast<std::uint32_t>(id) * 747796405u + 2891336453u;
    currentLeader_ = nullptr;

    reservedSpawnPoint_ = nullptr;
    if (restoreReservations && snap.reservedSpawnPointId >= 0) {
        PointOfInterest* poi =
            graph.getPointOfInterest(snap.reservedSpawnPointId);
        const auto* spawn = dynamic_cast<const SpawnPoint*>(poi);
        if (spawn != nullptr && spawn->tryReserveSpawnSlot()) {
            reservedSpawnPoint_ = spawn;
        }
    }
}

