#include <iostream>
#include "Vehicle.h"
#include "Road.h"
#include "Intersection.h"
#include "TrafficLight.h"
#include "Graph.h"
#include "JunctionConnector.h"
#include "LaneMapping.h"
#include "RoadGeometry.h"
#include "Crosswalk.h"
#include "algorithm/PathFindingStrategy.h"
#include "PointOfInterest.h"
#include "SpawnPoint.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <chrono>

namespace {

double combinedHalfLength(const Vehicle& first,
                          const Vehicle& second) {
    return (first.getLength() + second.getLength()) * 0.5;
}

double bumperGap(const Vehicle& follower,
                 const Vehicle& leader) {
    return leader.getProgressOnRoad()
         - follower.getProgressOnRoad()
         - combinedHalfLength(follower, leader);
}

double outgoingReleaseGap(const Vehicle& entering,
                          const Vehicle& leader) {
    // A connector remains reserved until the entering vehicle's rear bumper
    // clears the junction boundary. At release, its centre is half its own
    // length down the outgoing road, which must be included in the headroom.
    return leader.getProgressOnRoad()
         - entering.getLength() * 0.5
         - combinedHalfLength(entering, leader);
}

struct LaneChangeCandidate {
    int laneIndex = -1;
    double gapAhead = -std::numeric_limits<double>::infinity();
    double gapBehind = -std::numeric_limits<double>::infinity();
    bool safe = false;
};

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

LaneChangeCandidate assessLaneChange(const Vehicle& vehicle,
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

bool isBetterCandidate(const LaneChangeCandidate& candidate,
                       const LaneChangeCandidate& best) {
    if (!candidate.safe) return false;
    if (!best.safe) return true;
    if (candidate.gapAhead != best.gapAhead) {
        return candidate.gapAhead > best.gapAhead;
    }
    return candidate.gapBehind > best.gapBehind;
}

} // namespace


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
      pauseReason(PauseReason::None) {
    patienceThreshold = 3.0 + static_cast<double>(std::rand() % 50) / 10.0;
    recalculateTimer = 5.0 + static_cast<double>(std::rand() % 100) / 10.0;
}

Vehicle::~Vehicle() {
    releaseSpawnSlot();
    if (reservedIntersection_ != nullptr) {
        reservedIntersection_->exit(getId());
        reservedIntersection_ = nullptr;
    }
    if (currentRoad != nullptr) {
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        currentRoad = nullptr;
    }
}

bool Vehicle::tryReserveSpawnSlot() {
    if (spawnPOI == nullptr) {
        return true;
    }
    if (reservedSpawnPoint_ != nullptr) {
        return true;
    }
    const auto* spawnPoint =
        dynamic_cast<const SpawnPoint*>(spawnPOI);
    if (spawnPoint == nullptr) {
        return false;
    }
    if (!spawnPoint->tryReserveSpawnSlot()) {
        spawnLifecycleState_ =
            SpawnLifecycleState::
                WaitingForSourceCapacity;
        return false;
    }
    reservedSpawnPoint_ = spawnPoint;
    return true;
}

void Vehicle::releaseSpawnSlot() {
    if (reservedSpawnPoint_ == nullptr) {
        return;
    }
    reservedSpawnPoint_->releaseSpawnSlot();
    reservedSpawnPoint_ = nullptr;
}

bool Vehicle::hasReachedDestination() const {
    if (targetPOI != nullptr && currentRoad == targetPOI->getConnectedRoad()) {
        if (currentRouteIndex == static_cast<int>(currentRoute.size()) - 1) {
            if (progressOnCurrentRoad >= targetPOI->getProgressOffset()) {
                if (isEnteringPOI) {
                    return poiAnimationTimer <= 0.0;
                }
                return false;
            }
        }
    }
    return routeAssigned && currentRoad == nullptr && currentRouteIndex >= static_cast<int>(currentRoute.size());
}

PauseReason Vehicle::getIntersectionControlReason() const {
    if (currentRoad == nullptr) {
        return PauseReason::None;
    }

    Intersection* nextIntersection = currentRoad->getEnd();
    if (nextIntersection == nullptr) {
        return PauseReason::None;
    }

    const Crosswalk* crosswalk =
        nextIntersection->
            getCrosswalkForIncomingRoad(currentRoad);
    const bool prioritizedEmergency =
        nextIntersection->
            isPrioritizedEmergencyVehicle(
                getId(), currentRoad);
    if (prioritizedEmergency &&
        !nextIntersection->
             isEmergencyPathClear(getId())) {
        return PauseReason::PedestrianCrossing;
    }
    if (crosswalk != nullptr &&
        (crosswalk->getSignalState() ==
             PedestrianSignalState::Walk ||
         crosswalk->getSignalState() ==
             PedestrianSignalState::Clearance) &&
        !prioritizedEmergency) {
        return PauseReason::PedestrianCrossing;
    }

    const LaneMapping movementMapping =
        getJunctionEntryLaneMapping();
    const bool prioritizedMovement =
        nextIntersection->isPrioritizedEmergencyVehicle(
            getId(), currentRoad);
    const JunctionDecision movementDecision =
        prioritizedMovement
            ? JunctionDecision::Proceed
            : movementMapping.valid && getNextRoad() != nullptr
            ? nextIntersection->getMovementDecision(
                  currentRoad,
                  getNextRoad(),
                  movementMapping.movement)
            : (nextIntersection->mustStopForRoad(currentRoad)
                   ? JunctionDecision::Stop
                   : JunctionDecision::Proceed);
    if (movementDecision == JunctionDecision::Stop ||
        (movementDecision == JunctionDecision::Yield &&
         rightOnRedStoppedSeconds_ + 1e-9 <
             RIGHT_ON_RED_MIN_STOP_SECONDS)) {
        return PauseReason::TrafficLight;
    }

    if (reservedIntersection_ != nextIntersection &&
        !currentRoad->getLane(currentLaneIndex).isBlocked()) {
        Road* outgoing = getNextRoad();
        if (outgoing != nullptr) {
            const LaneMapping mapping =
                getJunctionEntryLaneMapping();
            if (!mapping.valid ||
                currentLaneIndex != mapping.incomingLane) {
                return PauseReason::Intersection;
            }
            Vehicle* outgoingLeader =
                outgoing->getFirstVehicleInLane(
                    mapping.outgoingLane);
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
                mapping.incomingLane,
                outgoing,
                mapping.outgoingLane);
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

    const PauseReason controlReason = getIntersectionControlReason();
    if (controlReason == PauseReason::None) return false;

    // Position of the nominal stop line a short distance before the road end.
    double stopLinePos = getIntersectionStopPosition();

    // A red-light queue is still waiting for the signal. By contrast, a
    // vehicle held behind an intersection-waiting leader is stopped by that
    // leader, not directly by the box reservation.
    if (controlReason == PauseReason::TrafficLight ||
        controlReason ==
            PauseReason::PedestrianCrossing) {
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
        movementState_ != MovementState::TraversingJunction &&
        currentLaneIndex >= 0 &&
        currentLaneIndex < currentRoad->getLaneCount()) {
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
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
    rightOnRedStoppedSeconds_ = 0.0;
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

bool Vehicle::advanceToNextRoad() {
    if (currentRoad != nullptr) {
        addTravelHistory(currentRoad);
    }
    ++currentRouteIndex;

    if (currentRouteIndex < static_cast<int>(currentRoute.size())) {
        if (currentRoad) {
            currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        }
        currentRoad = currentRoute[currentRouteIndex];
        if (currentRoad) {
            const int maxLaneIndex = currentRoad->getLaneCount() - 1;
            if (currentLaneIndex > maxLaneIndex) {
                currentLaneIndex = maxLaneIndex;
            }
            if (currentRoad->getLane(currentLaneIndex).isBlocked()) {
                currentLaneIndex = currentRoad->getFreestLaneIndex();
            }
            currentRoad->getLane(currentLaneIndex).addVehicle(this);
        }
        onRoadChanged();
        return true;
    }

    if (currentRoad) {
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
    }
    currentRoad = nullptr;
    progressOnCurrentRoad = 0.0;
    clearLaneChangeIntent();
    junctionTurnSignal_ = TurnSignal::Off;
    rightOnRedStoppedSeconds_ = 0.0;
    onRoadChanged();
    refreshTurnSignal();
    return false;
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
        double linearRatio = 1.0;
        if (poiAnimationDuration > 0) {
            linearRatio = std::clamp(1.0 - (poiAnimationTimer / poiAnimationDuration), 0.0, 1.0);
        }
        // Accelerate while leaving the source, but sample the same
        // right-angle access path that is rendered beneath the vehicle.
        // The final sample is the exact centre of the configured lane.
        double ratio = linearRatio * linearRatio;
        const RoadGeometry::RoadAccessPath accessPath =
            RoadGeometry::makeRoadAccessPath(
                *currentRoad,
                currentLaneIndex,
                progressOnCurrentRoad,
                {mergeSource->getX(),
                 mergeSource->getY()});
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

    return roadPose;
}

LaneMapping Vehicle::getUpcomingLaneMapping() const {
    Road* nextRoad = getNextRoad();
    if (currentRoad == nullptr || nextRoad == nullptr) {
        return {};
    }
    return TurnLanePolicy::map(
        *currentRoad, currentLaneIndex, *nextRoad, true);
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

    // Lane preparation remains the preferred behavior. If a red-light queue
    // prevented the merge, use the vehicle's actual lane at the stop line
    // instead of leaving that lane permanently blocked after green.
    return TurnLanePolicy::mapFromCurrentLane(
        *currentRoad,
        currentLaneIndex,
        *nextRoad,
        true);
}

bool Vehicle::beginJunctionTraversal(
    const LaneMapping& mapping,
    Intersection* intersection) {
    if (currentRoad == nullptr || junctionOutgoingRoad_ != nullptr ||
        intersection == nullptr || !mapping.valid ||
        currentLaneIndex != mapping.incomingLane) {
        return false;
    }

    Road* outgoing = getNextRoad();
    if (outgoing == nullptr) return false;
    Vehicle* outgoingLeader =
        outgoing->getFirstVehicleInLane(mapping.outgoingLane);
    if (outgoingLeader != nullptr) {
        const double availableAtExit =
            outgoingReleaseGap(*this, *outgoingLeader);
        if (availableAtExit <
            std::max(
                getMinGap(), outgoingLeader->getMinGap())) {
            return false;
        }
    }
    auto connector = intersection->getConnector(
        currentRoad,
        mapping.incomingLane,
        outgoing,
        mapping.outgoingLane);
    if (connector == nullptr) return false;

    const double requiredGap =
        getMinGap() + currentSpeed * getTimeHeadway();
    const JunctionDecision decision =
        intersection->isPrioritizedEmergencyVehicle(
            getId(), currentRoad)
            ? JunctionDecision::Proceed
            : intersection->getMovementDecision(
                  currentRoad,
                  outgoing,
                  mapping.movement);
    const bool entered =
        decision == JunctionDecision::Yield
            ? intersection->tryEnterYieldingMovement(
                  getId(),
                  connector,
                  requiredGap,
                  getLength(),
                  getWidth())
            : intersection->tryEnterMovement(
                  getId(),
                  connector,
                  requiredGap,
                  getLength(),
                  getWidth());
    if (!entered) {
        return false;
    }

    reservedIntersection_ = intersection;
    incomingLaneIndex_ = mapping.incomingLane;
    outgoingLaneIndex_ = mapping.outgoingLane;
    junctionOutgoingRoad_ = outgoing;
    activeConnector_ = std::move(connector);
    junctionProgressMetres_ = 0.0;
    currentRoad->getLane(incomingLaneIndex_).removeVehicle(this);
    progressOnCurrentRoad = currentRoad->getDistance();
    paused = false;
    pauseReason = PauseReason::None;
    movementState_ = MovementState::TraversingJunction;
    junctionTurnSignal_ =
        mapping.movement == MovementType::Right
            ? TurnSignal::Right
            : (mapping.movement == MovementType::Left ||
               mapping.movement == MovementType::UTurn
                   ? TurnSignal::Left
                   : TurnSignal::Off);
    clearLaneChangeIntent();
    refreshTurnSignal();
    return true;
}

void Vehicle::completeJunctionTraversal(
    double outgoingProgressMetres) {
    Road* incoming = currentRoad;
    Road* outgoing = junctionOutgoingRoad_;
    if (incoming != nullptr) {
        addTravelHistory(incoming);
    }
    ++currentRouteIndex;
    currentRoad = outgoing;
    currentLaneIndex = outgoingLaneIndex_;
    progressOnCurrentRoad = std::max(0.0, outgoingProgressMetres);

    if (reservedIntersection_ != nullptr) {
        reservedIntersection_->exit(getId());
        reservedIntersection_ = nullptr;
    }
    activeConnector_.reset();
    junctionOutgoingRoad_ = nullptr;
    junctionProgressMetres_ = 0.0;
    movementState_ = MovementState::OnRoad;
    paused = false;
    pauseReason = PauseReason::None;
    junctionTurnSignal_ = TurnSignal::Off;
    rightOnRedStoppedSeconds_ = 0.0;
    clearLaneChangeIntent();

    if (currentRoad != nullptr) {
        progressOnCurrentRoad = std::min(
            progressOnCurrentRoad, currentRoad->getDistance());
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
    }
    onRoadChanged();
    refreshTurnSignal();
}

double Vehicle::advanceJunction(double availableTime) {
    if (movementState_ != MovementState::TraversingJunction ||
        activeConnector_ == nullptr || availableTime <= 0.0) {
        return 0.0;
    }

    const double pathLength = activeConnector_->getLength();
    const double clearanceDistance =
        pathLength > 1e-6 ? getLength() * 0.5 : 0.0;
    const double completionDistance =
        pathLength + clearanceDistance;
    if (completionDistance <= junctionProgressMetres_ + 1e-9) {
        completeJunctionTraversal(
            std::max(0.0, junctionProgressMetres_ - pathLength));
        return 0.0;
    }

    const double subDt =
        std::min(availableTime, MAX_PHYSICS_SUBSTEP);
    double targetSpeed = std::max(
        1.0,
        std::min(baseSpeed, currentRoad->getSpeedLimit()) /
            std::max(1.0, currentRoad->getCongestionLevel()));
    const Pose2D pose = activeConnector_->sampleByDistance(
        junctionProgressMetres_);
    const double curvature = std::fabs(pose.curvature);
    if (curvature > 1e-8) {
        const double curveSpeed = std::sqrt(
            getMaxLateralAcceleration() / curvature);
        targetSpeed = std::min(targetSpeed, curveSpeed);
    }

    if (currentSpeed < targetSpeed) {
        currentSpeed = std::min(
            targetSpeed,
            currentSpeed + getAcceleration() * subDt);
    } else if (currentSpeed > targetSpeed) {
        currentSpeed = std::max(
            targetSpeed,
            currentSpeed - getDeceleration() * subDt);
    }
    currentSpeed = std::max(currentSpeed, 1e-3);

    const double remainingDistance =
        completionDistance - junctionProgressMetres_;
    const double possibleDistance = currentSpeed * subDt;
    double travelled =
        std::min(remainingDistance, possibleDistance);
    bool constrainedByOccupant = false;
    if (reservedIntersection_ != nullptr) {
        const double safeTravelled =
            reservedIntersection_->limitTraversalAdvance(
                getId(),
                activeConnector_,
                junctionProgressMetres_,
                travelled,
                getLength(),
                getWidth(),
                getMinGap());
        constrainedByOccupant =
            safeTravelled + 1e-9 < travelled;
        travelled = safeTravelled;
        if (constrainedByOccupant) {
            currentSpeed = travelled > 1e-9
                ? travelled / subDt
                : 0.0;
        }
    }
    const double consumedTime = constrainedByOccupant
        ? subDt
        : (currentSpeed > 1e-9
               ? travelled / currentSpeed
               : subDt);
    junctionProgressMetres_ += travelled;
    if (reservedIntersection_ != nullptr) {
        reservedIntersection_->updateReservationProgress(
            getId(), junctionProgressMetres_);
    }

    if (junctionProgressMetres_ + 1e-9 >= completionDistance) {
        completeJunctionTraversal(
            std::max(0.0, junctionProgressMetres_ - pathLength));
    }
    return consumedTime;
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

void Vehicle::clearLaneChangeIntent() {
    laneChangeState_ = LaneChangeState::Idle;
    laneChangeTargetLane_ = -1;
    laneChangeReason_ = TurnSignalReason::None;
    laneChangeSignalElapsedSeconds_ = 0.0;
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

void Vehicle::refreshTurnSignal() {
    if (laneChangeState_ != LaneChangeState::Idle &&
        currentRoad != nullptr &&
        laneChangeTargetLane_ != currentLaneIndex) {
        turnSignal_ = laneChangeTargetLane_ > currentLaneIndex
            ? TurnSignal::Right
            : TurnSignal::Left;
        turnSignalReason_ = laneChangeReason_;
        return;
    }
    if (movementState_ == MovementState::TraversingJunction) {
        turnSignal_ = junctionTurnSignal_;
        turnSignalReason_ =
            turnSignal_ == TurnSignal::Off
                ? TurnSignalReason::None
                : TurnSignalReason::Junction;
        return;
    }
    if (currentRoad != nullptr &&
        currentRoad->getDistance() - progressOnCurrentRoad <=
            getJunctionLanePreparationDistance()) {
        turnSignal_ = deriveUpcomingJunctionSignal();
        turnSignalReason_ =
            turnSignal_ == TurnSignal::Off
                ? TurnSignalReason::None
                : TurnSignalReason::Junction;
        return;
    }
    turnSignal_ = TurnSignal::Off;
    turnSignalReason_ = TurnSignalReason::None;
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

bool Vehicle::isRightTurnOnRedYield() const {
    if (currentRoad == nullptr ||
        movementState_ == MovementState::TraversingJunction) {
        return false;
    }
    Road* outgoing = getNextRoad();
    Intersection* intersection = currentRoad->getEnd();
    const LaneMapping mapping = getJunctionEntryLaneMapping();
    return outgoing != nullptr &&
           intersection != nullptr &&
           mapping.valid &&
           intersection->getMovementDecision(
               currentRoad,
               outgoing,
               mapping.movement) ==
               JunctionDecision::Yield;
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
    if (laneChangeSignalElapsedSeconds_ + 1e-9 <
        MIN_SIGNAL_LEAD_TIME_SECONDS) {
        laneChangeState_ = LaneChangeState::Signaling;
        return false;
    }
    laneChangeState_ = LaneChangeState::WaitingForGap;
    const int adjacentLane = currentLaneIndex +
        (requiredLaneIndex > currentLaneIndex ? 1 : -1);
    const LaneChangeCandidate candidate = assessLaneChange(
        *this,
        *currentRoad,
        adjacentLane,
        LANE_CHANGE_REAR_SAFETY_TIME,
        LANE_CHANGE_MIN_TTC);
    if (!candidate.safe) {
        return false;
    }

    currentRoad->getLane(currentLaneIndex).removeVehicle(this);
    currentLaneIndex = adjacentLane;
    currentRoad->getLane(currentLaneIndex).addVehicle(this);
    if (currentLaneIndex == requiredLaneIndex) {
        clearLaneChangeIntent();
        laneChangeCooldownTimer = LANE_CHANGE_COOLDOWN;
    } else {
        laneChangeSignalElapsedSeconds_ = 0.0;
        laneChangeState_ = LaneChangeState::Signaling;
    }
    refreshTurnSignal();
    return true;
}

void Vehicle::tryLaneChange(double freeFlowSpeed) {
    if (currentRoad == nullptr || !canChangeLanes()) {
        return;
    }
    if (currentRoad->getLaneCount() <= 1) {
        return; // chi co 1 lane, khong co gi de doi
    }

    // 1) Chi xet doi lane khi dang THUC SU bi can tro o lane hien tai -
    //    tuc la gap phia truoc nho hon "khoang cach thoai mai" mong muon.
    //    Neu dang chay tu do, khong co ly do gi de doi lane.
    Vehicle* currentLeader = currentRoad->findLeader(currentLaneIndex, this);
    const double currentGapAhead = gapAheadOf(*this, currentLeader);

    const double minGap = getMinGap();
    const double desiredGap = minGap + freeFlowSpeed * getTimeHeadway();

    const bool isCurrentLaneBlocked = currentRoad->getLane(currentLaneIndex).isBlocked();

    if (!isCurrentLaneBlocked && currentGapAhead >= desiredGap) {
        if (laneChangeReason_ ==
            TurnSignalReason::LaneChange) {
            clearLaneChangeIntent();
            refreshTurnSignal();
        }
        return; // khong bi can tro dang ke, khong can doi lane
    }

    // Avoid opportunistic weaving while entering an intersection. Escaping a
    // blocked lane remains allowed; emergency yielding is handled separately.
    const double distanceToIntersection = currentRoad->getDistance() - progressOnCurrentRoad;
    const double noChangeDistance = std::min(
        NO_LANE_CHANGE_DISTANCE,
        currentRoad->getDistance() * NO_LANE_CHANGE_ROAD_FRACTION);
    if (!isCurrentLaneBlocked && distanceToIntersection <= noChangeDistance) {
        return;
    }

    // 2) Xet 2 lane lan can (trai/phai). Chon lane tot nhat trong so cac
    //    lane thoa dieu kien "tot hon dang ke" (LANE_CHANGE_GAP_IMPROVEMENT_FACTOR)
    //    VA an toan cho xe phia sau o lane do.
    LaneChangeCandidate bestCandidate;
    const double requiredGapAhead = isCurrentLaneBlocked
        ? minGap
        : currentGapAhead * LANE_CHANGE_GAP_IMPROVEMENT_FACTOR;

    const int candidateLanes[2] = { currentLaneIndex - 1, currentLaneIndex + 1 };
    for (int candidateLane : candidateLanes) {
        if (candidateLane < 0 || candidateLane >= currentRoad->getLaneCount()) {
            continue;
        }
        // A yielding vehicle may still use the normal lane-change logic when
        // its own lane is obstructed. However, it must never choose the lane
        // currently being cleared for the approaching emergency vehicle.
        if (yielding && candidateLane == emergencyLaneToAvoid) {
            continue;
        }
        LaneChangeCandidate candidate = assessLaneChange(
            *this, *currentRoad, candidateLane,
            LANE_CHANGE_REAR_SAFETY_TIME, LANE_CHANGE_MIN_TTC);
        if (!candidate.safe || candidate.gapAhead <= requiredGapAhead) continue;

        if (isBetterCandidate(candidate, bestCandidate)) {
            bestCandidate = candidate;
        }
    }

    if (bestCandidate.safe) {
        requestLaneChange(
            bestCandidate.laneIndex,
            TurnSignalReason::LaneChange);
        if (laneChangeSignalElapsedSeconds_ + 1e-9 <
            MIN_SIGNAL_LEAD_TIME_SECONDS) {
            laneChangeState_ = LaneChangeState::Signaling;
            return;
        }
        laneChangeState_ = LaneChangeState::WaitingForGap;
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        currentLaneIndex = bestCandidate.laneIndex;
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
        clearLaneChangeIntent();
        refreshTurnSignal();
        laneChangeCooldownTimer = LANE_CHANGE_COOLDOWN;
    } else {
        laneChangeCooldownTimer = 0.5; // Short cooldown when lane change is skipped/fails
    }
}

void Vehicle::tryYieldLaneChange() {
    if (currentRoad == nullptr || !canChangeLanes()) return;
    if (currentRoad->getLaneCount() <= 1) return;
    if (emergencyLaneToAvoid < 0 || currentLaneIndex != emergencyLaneToAvoid) return;

    LaneChangeCandidate bestCandidate;
    const int candidateLanes[2] = { currentLaneIndex - 1, currentLaneIndex + 1 };
    for (int candidateLane : candidateLanes) {
        LaneChangeCandidate candidate = assessLaneChange(
            *this, *currentRoad, candidateLane,
            LANE_CHANGE_REAR_SAFETY_TIME * 0.5,
            YIELD_LANE_CHANGE_MIN_TTC);
        if (isBetterCandidate(candidate, bestCandidate)) {
            bestCandidate = candidate;
        }
    }

    if (bestCandidate.safe) {
        requestLaneChange(
            bestCandidate.laneIndex,
            TurnSignalReason::LaneChange);
        if (laneChangeSignalElapsedSeconds_ + 1e-9 <
            MIN_SIGNAL_LEAD_TIME_SECONDS) {
            laneChangeState_ = LaneChangeState::Signaling;
            return;
        }
        laneChangeState_ = LaneChangeState::WaitingForGap;
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        currentLaneIndex = bestCandidate.laneIndex;
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
        clearLaneChangeIntent();
        refreshTurnSignal();
        laneChangeCooldownTimer = LANE_CHANGE_COOLDOWN * 0.5;
    } else {
        laneChangeCooldownTimer = 0.25; // Short cooldown when yield lane change fails
    }
}

bool Vehicle::mustStopForTrafficLight(Intersection* nextIntersection) const {
    if (nextIntersection == nullptr || currentRoad == nullptr) {
        return false;
    }
    Road* outgoing = getNextRoad();
    const LaneMapping mapping = getJunctionEntryLaneMapping();
    if (outgoing == nullptr || !mapping.valid) {
        return nextIntersection->mustStopForRoad(currentRoad);
    }
    return nextIntersection->getMovementDecision(
               currentRoad,
               outgoing,
               mapping.movement) !=
           JunctionDecision::Proceed;
}

Road* Vehicle::getNextRoad() const {
    if (currentRouteIndex + 1 < static_cast<int>(currentRoute.size())) {
        return currentRoute[currentRouteIndex + 1];
    }
    return nullptr;
}

void Vehicle::update(double dt,Graph* graph,PathFindingStrategy* strategy,bool allowDynamicReroute) {
    if (hasReachedDestination() || currentRoad == nullptr) {
        clearLaneChangeIntent();
        junctionTurnSignal_ = TurnSignal::Off;
        refreshTurnSignal();
        return;
    }

    const double nonNegativeDt = std::max(0.0, dt);
    simulationTimeSeconds_ += nonNegativeDt;
    if (laneChangeState_ != LaneChangeState::Idle) {
        laneChangeSignalElapsedSeconds_ += nonNegativeDt;
    }
    if (!isRightTurnOnRedYield()) {
        rightOnRedStoppedSeconds_ = 0.0;
    }
    refreshTurnSignal();

    if (isMergingFromPOI) {
        if (poiAnimationTimer > 0) {
            updatePoiAnimation(dt);
            // Xe đang chạy từ toà nhà ra mép đường, chưa check gap vội
            return;
        }

        if (mergeLaneIndex < 0 ||
            mergeLaneIndex >=
                currentRoad->getLaneCount()) {
            return;
        }
        const Lane& mergeLane =
            currentRoad->getLane(mergeLaneIndex);
        if (mergeLane.isBlocked() ||
            mergeLane.getVehicleCount() >=
                mergeLane.getCapacity()) {
            return;
        }
        Intersection* entrance =
            currentRoad->getStart();
        if (entrance != nullptr &&
            entrance->isOutgoingLaneReserved(
                currentRoad,
                mergeLaneIndex)) {
            return;
        }

        Vehicle* follower = currentRoad->findFollower(mergeLaneIndex, this);
        Vehicle* leader = currentRoad->findLeader(mergeLaneIndex, this);
        bool safeToMerge = true;
        
        if (follower != nullptr) {
            double gapBehind = mergeProgressOffset - follower->getProgressOnRoad() - (getLength() + follower->getLength()) * 0.5;
            double followerSpeed = follower->getCurrentSpeed();
            double requiredGap = follower->getMinGap() + followerSpeed * LANE_CHANGE_REAR_SAFETY_TIME;
            // Add a small epsilon to prevent floating point issues when follower stops exactly at requiredGap
            if (gapBehind < requiredGap - 1e-4) {
                safeToMerge = false;
            }
        }
        if (leader != nullptr) {
            double gapAhead = leader->getProgressOnRoad() - mergeProgressOffset - (getLength() + leader->getLength()) * 0.5;
            if (gapAhead < getMinGap() - 1e-4) {
                safeToMerge = false;
            }
        }

        if (safeToMerge) {
            currentRoad->removeMergingVehicle(this);
            currentRoad->getLane(mergeLaneIndex).addVehicle(this);
            isMergingFromPOI = false;
            currentLaneIndex = mergeLaneIndex;
            progressOnCurrentRoad = mergeProgressOffset;
            currentSpeed = 0.0;
            releaseSpawnSlot();
            spawnLifecycleState_ =
                SpawnLifecycleState::Active;
        } else {
            return; // Wait for gap
        }
    }
    
    if (targetPOI != nullptr && currentRoad == targetPOI->getConnectedRoad()) {
        if (currentRouteIndex == static_cast<int>(currentRoute.size()) - 1) {
            if (!isEnteringPOI && progressOnCurrentRoad >= targetPOI->getProgressOffset()) {
                setEnteringPOI(true);
                currentRoad->getLane(currentLaneIndex).removeVehicle(this); // Stop blocking road while entering
                return;
            }
            
            if (isEnteringPOI) {
                if (poiAnimationTimer > 0) {
                    updatePoiAnimation(dt);
                }
                return; // Don't move on the road anymore
            }
        }
    }

    double remainingTime = std::max(0.0, dt);
    if (remainingTime <= 0.0) {
        return;
    }
    const double elapsedTimeThisUpdate = remainingTime;

    if (allowsDynamicRerouting()) {
        recalculateTimer -= elapsedTimeThisUpdate;
        if (recalculateTimer <= 0.0 && allowDynamicReroute) {
            recalculateTimer = 10.0 +
                static_cast<double>(
                    std::rand() % 50) /
                    10.0; // 10-15s
            if (graph && strategy && currentRoad &&
                movementState_ ==
                    MovementState::OnRoad) {
                bool hasCongestion = false;
                {
                    const size_t maxLookAhead = 50; // Check only the next 50 roads for congestion
                    const size_t end = std::min(
                        currentRouteIndex + 1 + maxLookAhead,
                        currentRoute.size());
                    for (size_t i = currentRouteIndex + 1;
                         i < end; ++i) {
                        if (currentRoute[i]->
                                    getDynamicCongestionLevel() >
                                1.5 ||
                            currentRoute[i]->isBlocked()) {
                            hasCongestion = true;
                            break;
                        }
                    }
                }
                if (hasCongestion) {
                                    auto start = std::chrono::high_resolution_clock::now();

                    bool ok = recalculateRoute(*graph, strategy);

                    auto us =
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::high_resolution_clock::now() - start)
                            .count();

                    if (us > 1000) {
                        std::cout
                            << "[REROUTE] vehicle "
                            << getId()
                            << " took "
                            << us / 1000.0
                            << " ms\n";
                    }
                }
            }
        }
    }

    if (laneChangeCooldownTimer > 0.0) {
        laneChangeCooldownTimer -= elapsedTimeThisUpdate;
        if (laneChangeCooldownTimer < 0.0) {
            laneChangeCooldownTimer = 0.0;
        }
    }

    if (uTurnCooldownTimer > 0.0) {
        uTurnCooldownTimer -= elapsedTimeThisUpdate;
    }

    if (yielding) {
        yieldCooldownTimer -= elapsedTimeThisUpdate;
        if (yieldCooldownTimer <= 0.0) {
            yieldCooldownTimer = 0.0;
            yielding = false;
            emergencyLaneToAvoid = -1; 
        }
    }

    if (paused) {
        if (pauseReason == PauseReason::BusStop) {
            const PauseUpdateResult result = updatePause(remainingTime);
            remainingTime = std::clamp(
                result.remainingTime, 0.0, remainingTime);
            if (!result.resumed) {
                return;
            }
            clearPause();
        } else {
            if (isRightTurnOnRedYield() &&
                rightOnRedStoppedSeconds_ + 1e-9 <
                    RIGHT_ON_RED_MIN_STOP_SECONDS) {
                const double needed =
                    RIGHT_ON_RED_MIN_STOP_SECONDS -
                    rightOnRedStoppedSeconds_;
                const double consumed =
                    std::min(remainingTime, needed);
                rightOnRedStoppedSeconds_ += consumed;
                remainingTime -= consumed;
                if (rightOnRedStoppedSeconds_ + 1e-9 <
                        RIGHT_ON_RED_MIN_STOP_SECONDS ||
                    remainingTime <= 0.0) {
                    return;
                }
            }
            const PauseReason currentControlReason =
                getIntersectionControlReason();
            if (currentControlReason != PauseReason::None) {
                beginPause(currentControlReason);
                return;
            }
            clearPause();
        }
    }

    int loopCount = 0;
    while (remainingTime > 0.0 && currentRoad != nullptr && !paused) {
        
        ++loopCount;

        if (loopCount > 1000) {
            std::cout << "[STUCK] Vehicle " << getId()
                    << " loopCount=" << loopCount
                    << " remainingTime=" << remainingTime
                    << " speed=" << currentSpeed
                    << " road=" << (currentRoad ? currentRoad->getId() : -1)
                    << '\n';
            break;
        }
            
        if (movementState_ == MovementState::TraversingJunction) {
            const double consumed = advanceJunction(remainingTime);
            remainingTime =
                std::max(0.0, remainingTime - consumed);
            if (consumed <= 0.0 &&
                movementState_ == MovementState::TraversingJunction) {
                break;
            }
            continue;
        }

        const double freeFlowSpeed = calculateCurrentSpeed();
        double targetSpeed = freeFlowSpeed;

        // ambulance yielding constraints
        if (yielding && emergencyLaneToAvoid >= 0
            && currentLaneIndex == emergencyLaneToAvoid) {
            // Keep moving while looking for a safe adjacent gap. Vehicles that
            // already cleared the emergency lane keep their normal speed.
            const double escapeSpeed = std::min(
                currentRoad->getSpeedLimit(),
                freeFlowSpeed * getYieldEscapeSpeedFactor());
            targetSpeed = std::max(targetSpeed, escapeSpeed);
        }
        if (currentRoad != nullptr) {
            Intersection* nextIntersectionForLight = currentRoad->getEnd();
            if (nextIntersectionForLight != nullptr) {
                const double distanceToIntersection =
                    currentRoad->getDistance() -
                    progressOnCurrentRoad;
                if (nextIntersectionForLight->
                        hasActiveEmergencyPriority() &&
                    distanceToIntersection <=
                        EMERGENCY_JUNCTION_CAUTION_DISTANCE) {
                    targetSpeed = std::min(
                        targetSpeed,
                        getEmergencyJunctionSpeedLimit(
                            freeFlowSpeed));
                }
                const PauseReason controlReason =
                    getIntersectionControlReason();
                if (controlReason != PauseReason::None) {
                    const double distToStopLine =
                        currentRoad->getDistance() -
                        progressOnCurrentRoad;
                    const double stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * std::max(getDeceleration(), 1e-6));
                    const double safetyBuffer = 1.0; // metres: small margin
                    if (distToStopLine <= stoppingDistance + safetyBuffer) {
                        targetSpeed = 0.0;
                    }
                }
            }
        }
        
        // Slow down when approaching destination POI
        if (targetPOI != nullptr && currentRoad == targetPOI->getConnectedRoad() && currentRouteIndex == static_cast<int>(currentRoute.size()) - 1) {
            double distToPOI = targetPOI->getProgressOffset() - progressOnCurrentRoad;
            if (distToPOI > 0.0) {
                const double stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * std::max(getDeceleration(), 1e-6));
                const double safetyBuffer = 3.0; // metres
                if (distToPOI <= stoppingDistance + safetyBuffer) {
                    targetSpeed = std::min(targetSpeed, 2.0); // Decelerate to 2 m/s before turning into POI
                }
            }
        }
        
        if (yielding && laneChangeCooldownTimer <= 0.0) {
            tryYieldLaneChange();
        }

        const LaneMapping upcomingMapping =
            getUpcomingLaneMapping();
        const double distanceToJunction =
            currentRoad->getDistance() - progressOnCurrentRoad;
        const bool preparingForJunction =
            upcomingMapping.valid &&
            distanceToJunction <=
                getJunctionLanePreparationDistance();
        const int stopOrServiceLane = getRequiredLaneIndex();
        const int requiredLaneIndex =
            stopOrServiceLane >= 0
                ? stopOrServiceLane
                : (preparingForJunction
                       ? upcomingMapping.incomingLane
                       : -1);
        const bool hasRequiredLane =
            requiredLaneIndex >= 0 &&
            requiredLaneIndex < currentRoad->getLaneCount();
        if (laneChangeCooldownTimer <= 0.0) {
            if (hasRequiredLane) {
                if (requiredLaneIndex != currentLaneIndex) {
                    tryRequiredLaneChange(
                        requiredLaneIndex,
                        stopOrServiceLane >= 0
                            ? TurnSignalReason::BusStop
                            : TurnSignalReason::Junction);
                } else if (
                    laneChangeState_ != LaneChangeState::Idle) {
                    clearLaneChangeIntent();
                    refreshTurnSignal();
                }
            } else {
                if (laneChangeState_ != LaneChangeState::Idle &&
                    (laneChangeReason_ ==
                         TurnSignalReason::BusStop ||
                     laneChangeReason_ ==
                         TurnSignalReason::Junction)) {
                    clearLaneChangeIntent();
                    refreshTurnSignal();
                }
                if (!(yielding &&
                      emergencyLaneToAvoid >= 0 &&
                      currentLaneIndex ==
                          emergencyLaneToAvoid)) {
                    tryLaneChange(freeFlowSpeed);
                }
            }
        }
        const LaneMapping entryMapping =
            getJunctionEntryLaneMapping();
        if (preparingForJunction &&
            currentLaneIndex != upcomingMapping.incomingLane &&
            (!entryMapping.valid ||
             entryMapping.incomingLane != currentLaneIndex) &&
            distanceToJunction <=
                RoadGeometry::STOP_LINE_SETBACK_METRES + 0.25) {
            targetSpeed = 0.0;
        }

        targetSpeed = std::min(
            targetSpeed,
            std::clamp(
                getLanePreparationSpeedLimit(freeFlowSpeed),
                0.0,
                freeFlowSpeed));

        Vehicle* leader = currentRoad->findLeader(currentLaneIndex, this);
        double minGap = getMinGap();
        double gapToLeader = std::numeric_limits<double>::infinity();

        if (leader != nullptr) {
            gapToLeader = bumperGap(*this, *leader);
        } else {
            Road* nextRoad = getNextRoad();
            if (nextRoad != nullptr) {
                const LaneMapping leaderMapping =
                    getJunctionEntryLaneMapping();
                const int nextLaneIndex = leaderMapping.valid
                    ? leaderMapping.outgoingLane
                    : std::clamp(
                          currentLaneIndex,
                          0,
                          nextRoad->getLaneCount() - 1);
                Vehicle* nextLeader = nextRoad->getFirstVehicleInLane(nextLaneIndex);
                if (nextLeader != nullptr) {
                    const double distToEndOfCurrentRoad = currentRoad->getDistance() - progressOnCurrentRoad;
                    gapToLeader = distToEndOfCurrentRoad
                                + nextLeader->getProgressOnRoad()
                                - combinedHalfLength(
                                      *this, *nextLeader);
                    leader = nextLeader; 
                }
            }
        }

        if (leader != nullptr) {
            minGap = std::max(minGap, leader->getMinGap());
            double leaderTargetSpeed = freeFlowSpeed;
            if (gapToLeader <= minGap) {
                targetSpeed = 0.0;
            } else {
                const double desiredGap = minGap + freeFlowSpeed * getTimeHeadway();
                if (gapToLeader < desiredGap && desiredGap > minGap) {
                    leaderTargetSpeed = freeFlowSpeed * (gapToLeader - minGap) / (desiredGap - minGap);
                }
            }

            targetSpeed = std::min(targetSpeed, leaderTargetSpeed);

            const double stoppingDistance =
                (currentSpeed * currentSpeed) / (2.0 * std::max(getDeceleration(), 1e-6));
            if (gapToLeader - minGap < stoppingDistance) {
                targetSpeed = 0.0;
            }
        }

        const double subDt = std::min(remainingTime, MAX_PHYSICS_SUBSTEP);
        if (currentSpeed < targetSpeed) {
            currentSpeed = std::min(targetSpeed, currentSpeed + getAcceleration() * subDt);
        } else if (currentSpeed > targetSpeed) {
            currentSpeed = std::max(targetSpeed, currentSpeed - getDeceleration() * subDt);
        }

        double speed = currentSpeed;
        if (speed <= 0.0) {
            const PauseReason controlReason =
                getIntersectionControlReason();
            const bool atIntersectionStopPosition =
                progressOnCurrentRoad + 1e-6 >=
                getIntersectionStopPosition();
            if (controlReason != PauseReason::None &&
                atIntersectionStopPosition) {
                beginPause(controlReason);
                break;
            }

            stuckTimer += subDt;
            if (allowsUTurn() &&
                stuckTimer > patienceThreshold &&
                graph != nullptr && strategy != nullptr) {
                // Check if it's safe to U-turn (no vehicle closely behind)
                Vehicle* follower = currentRoad->findFollower(currentLaneIndex, this);
                bool safeToUTurn = true;
                if (follower != nullptr) {
                    const double gapBehind =
                        bumperGap(*follower, *this);
                    if (gapBehind < 5.0 && follower->getCurrentSpeed() > 0.01) {
                        safeToUTurn = false;
                    } else if (gapBehind < 1.0) { // they are very close, still unsafe
                        safeToUTurn = false;
                    }
                }
                
                if (safeToUTurn && currentLaneIndex == 0 && uTurnCooldownTimer <= 0.0) {
                    if (performUTurn(*graph, strategy)) {
                        stuckTimer = 0.0;
                        uTurnCooldownTimer = 30.0; // 30s cooldown
                    } else {
                        // If U-turn failed (e.g. no path), wait longer
                        patienceThreshold += 2.0; 
                    }
                }
            }
            break;
        } else {
            stuckTimer = 0.0;
        }

        double distanceThisTick = speed * subDt;
        if (leader != nullptr) {
            const double maxAdvance = std::max(0.0, gapToLeader - minGap);
            distanceThisTick = std::min(distanceThisTick, maxAdvance);
        }
        double currentPos = progressOnCurrentRoad;
        double projectedPos = currentPos + distanceThisTick;

        double pausePos = -1.0;
        if (shouldPauseAt(currentPos, projectedPos, pausePos)
            && pausePos >= currentPos
            && pausePos <= currentRoad->getDistance()) {
            progressOnCurrentRoad = pausePos;
            beginPause(pauseReason);
            break;
        }

        if (projectedPos < currentRoad->getDistance()) {
            progressOnCurrentRoad = projectedPos;
            remainingTime -= subDt;
        } else {
            Intersection* nextIntersection = currentRoad->getEnd();

            const PauseReason controlReason =
                getIntersectionControlReason();
            if (controlReason != PauseReason::None) {
                progressOnCurrentRoad = std::min(
                    currentRoad->getDistance(),
                    std::max(currentPos, getIntersectionStopPosition()));
                beginPause(controlReason);
                break;
            }

            const bool hasNextRoad =
                currentRouteIndex + 1 <
                static_cast<int>(currentRoute.size());
            if (hasNextRoad) {
                const LaneMapping mapping =
                    getJunctionEntryLaneMapping();
                if (!mapping.valid ||
                    currentLaneIndex != mapping.incomingLane ||
                    !beginJunctionTraversal(
                        mapping, nextIntersection)) {
                    progressOnCurrentRoad = std::min(
                        currentRoad->getDistance(),
                        std::max(currentPos, getIntersectionStopPosition()));
                    beginPause(PauseReason::Intersection);
                    break;
                }
            } else if (nextIntersection != nullptr &&
                       reservedIntersection_ != nextIntersection) {
                if (nextIntersection->tryEnter(
                        getId(), currentRoad)) {
                    reservedIntersection_ = nextIntersection;
                } else {
                    progressOnCurrentRoad = std::min(
                        currentRoad->getDistance(),
                        std::max(currentPos, getIntersectionStopPosition()));
                    beginPause(PauseReason::Intersection);
                    break;
                }
            }
            const double distToEnd =
                currentRoad->getDistance() - currentPos;
            const double timeToEnd =
                (speed > 0.0) ? distToEnd / speed : 0.0;

            remainingTime -= std::max(subDt, timeToEnd);
            remainingTime = std::max(0.0, remainingTime);

            if (hasNextRoad) {
                continue;
            } else {
                if (reservedIntersection_ != nullptr) {
                    reservedIntersection_->exit(getId());
                    reservedIntersection_ = nullptr;
                }

                if (!advanceToNextRoad() || hasReachedDestination()) {
                    break;
                }
            }
        }
    }
}

bool Vehicle::isRoadInUpcomingRoute(int roadId) const {
    for (size_t i = currentRouteIndex + 1; i < currentRoute.size(); ++i) {
        if (currentRoute[i]->getId() == roadId) {
            return true;
        }
    }
    return false;
}

bool Vehicle::recalculateRoute(const Graph& graph, PathFindingStrategy* strategy) {
    if (!allowsDynamicRerouting() ||
        currentRoad == nullptr || destination == nullptr ||
        movementState_ == MovementState::TraversingJunction) {
        return false;
    }

    int startNodeId = currentRoad->getEnd()->getId();
    int destNodeId = destination->getId();

    PathResult result = strategy->findPath(graph, startNodeId, destNodeId);

    if (!result.found) {
        return false;
    }

    std::vector<Road*> newRoute;
    newRoute.reserve(static_cast<size_t>(currentRouteIndex) + 1 + result.roadPath.size());
    for (int i = 0; i <= currentRouteIndex; ++i) {
        newRoute.push_back(currentRoute[i]);
    }

    for (Road* r : result.roadPath) {
        newRoute.push_back(r);
    }

    currentRoute = newRoute;
    return true;
}

bool Vehicle::performUTurn(const Graph& graph, PathFindingStrategy* strategy) {
    if (!allowsUTurn() ||
        currentRoad == nullptr || destination == nullptr ||
        movementState_ == MovementState::TraversingJunction) {
        return false;
    }

    int startId = currentRoad->getStart()->getId();
    int endId = currentRoad->getEnd()->getId();
    Road* reverseRoad = nullptr;

    for (Road* r : graph.getAllRoads()) {
        if (r->getStart()->getId() == endId && r->getEnd()->getId() == startId) {
            reverseRoad = r;
            break;
        }
    }

    if (reverseRoad == nullptr) {
        return false;
    }

    PathResult result = strategy->findPath(graph, startId, destination->getId());
    if (!result.found) {
        return false;
    }

    currentRoad->getLane(currentLaneIndex).removeVehicle(this);
    currentRoad = reverseRoad;
    currentLaneIndex = currentRoad->getLane(0).isBlocked()
        ? currentRoad->getFreestLaneIndex()
        : 0;
    currentRoad->getLane(currentLaneIndex).addVehicle(this);

    progressOnCurrentRoad = std::clamp(
        currentRoad->getDistance() - progressOnCurrentRoad,
        0.0,
        currentRoad->getDistance());

    currentSpeed = 0.0;
    clearPause();

    std::vector<Road*> newRoute;
    for (int i = 0; i < currentRouteIndex; ++i) {
        newRoute.push_back(currentRoute[i]);
    }

    newRoute.push_back(currentRoad);
    for (Road* r : result.roadPath) {
        newRoute.push_back(r);
    }

    currentRoute = newRoute;
    onRoadChanged();
    return true;
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
void Vehicle::notifyEmergencyApproaching(int emergencyLaneIndex) {
    yielding = true;
    yieldCooldownTimer = YIELD_COOLDOWN_DURATION;
    emergencyLaneToAvoid = emergencyLaneIndex;
}
double Vehicle::getLength() const { return 4.5; }    // metres
double Vehicle::getWidth() const { return 1.8; }     // metres
double Vehicle::getHeight() const { return 1.5; }    // metres
double Vehicle::getWeight() const { return 1.5; }    // tonnes
double Vehicle::getMiniGap() const { return 0.0; }
double Vehicle::getMinGap() const { return getMiniGap(); }
double Vehicle::getTimeHeadway() const { return 1.5; }

void Vehicle::updatePoiAnimation(double dt) {
    if (poiAnimationTimer > 0) poiAnimationTimer -= dt;
}

void Vehicle::setMergingFromPOI(bool merging, double offset, int laneIdx) {
    isMergingFromPOI = merging;
    mergeProgressOffset = offset;
    mergeLaneIndex = laneIdx;
    if (merging) {
        poiAnimationTimer = poiAnimationDuration;
        spawnLifecycleState_ =
            SpawnLifecycleState::Merging;
    }
}

void Vehicle::setEnteringPOI(bool entering) {
    isEnteringPOI = entering;
    if (entering) {
        poiAnimationTimer = poiAnimationDuration;
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

