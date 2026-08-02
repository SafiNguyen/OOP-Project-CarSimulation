#include "Vehicle.h"
#include "Road.h"
#include "Lane.h"
#include "Intersection.h"
#include "RoadGeometry.h"
#include "VehicleMath.h"
using namespace VehicleMath;
#include "JunctionConnector.h"
#include "PointOfInterest.h"

void Vehicle::clearLaneChangeIntent() {
    laneChangeState_ = LaneChangeState::Idle;
    laneChangeTargetLane_ = -1;
    laneChangeReason_ = TurnSignalReason::None;
    laneChangeSignalElapsedSeconds_ = 0.0;
}
void Vehicle::refreshTurnSignal() {
    if (laneChangeState_ != LaneChangeState::Idle &&
        currentRoad != nullptr &&
        laneChangeTargetLane_ != currentLaneIndex) {
        const TurnSignal junctionIntent =
            laneChangeReason_ == TurnSignalReason::Junction
                ? deriveUpcomingJunctionSignal()
                : TurnSignal::Off;
        turnSignal_ = junctionIntent != TurnSignal::Off
            ? junctionIntent
            : (laneChangeTargetLane_ > currentLaneIndex
                   ? TurnSignal::Right
                   : TurnSignal::Left);
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
void Vehicle::tryLaneChange(double freeFlowSpeed) {
    if (currentRoad == nullptr || !canChangeLanes()) {
        return;
    }
    if (currentRoad->getLaneCount() <= 1) {
        return; // chi co 1 lane, khong co gi de doi
    }

    // Rule 1 & 3: Cannot change if already transitioning, at stop line, or waiting for light
    if (isTransitioningPose() || movementState_ == MovementState::TraversingJunction || isWaitingForLight_) {
        return;
    }

    const bool isCurrentLaneBlocked = currentRoad->getLane(currentLaneIndex).isBlocked();

    // Rule 2: Cannot change just after entering road
    if (!isCurrentLaneBlocked && progressOnCurrentRoad < std::min(20.0, currentRoad->getDistance() * 0.2)) {
        return;
    }

    // 1) Chi xet doi lane khi dang THUC SU bi can tro o lane hien tai -
    //    tuc la gap phia truoc nho hon "khoang cach thoai mai" mong muon.
    //    Neu dang chay tu do, khong co ly do gi de doi lane.
    Vehicle* currentLeader = currentRoad->findLeader(currentLaneIndex, this);
    const double currentGapAhead = gapAheadOf(*this, currentLeader);

    const double minGap = getMinGap();
    const double desiredGap = minGap + freeFlowSpeed * getTimeHeadway();

    if (!isCurrentLaneBlocked && currentGapAhead >= desiredGap) {
        if (laneChangeReason_ ==
            TurnSignalReason::LaneChange) {
            clearLaneChangeIntent();
            refreshTurnSignal();
        }
        return; // khong bi can tro dang ke, khong can doi lane
    }

    // Ordinary vehicles avoid opportunistic weaving while entering an
    // intersection. A traffic-priority vehicle may still use a safe adjacent
    // lane to clear a queue.
    const double distanceToIntersection = currentRoad->getDistance() - progressOnCurrentRoad;
    const double noChangeDistance = std::min(
        NO_LANE_CHANGE_DISTANCE,
        currentRoad->getDistance() * NO_LANE_CHANGE_ROAD_FRACTION);
    if (!isCurrentLaneBlocked &&
        !hasTrafficPriority() &&
        distanceToIntersection <= noChangeDistance) {
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
    
    auto hasTransitioningVehicleNearby = [&](int targetLane) {
        auto checkLane = [&](int laneIdx) {
            const auto& vehicles = currentRoad->getLane(laneIdx).getVehicles();
            for (Vehicle* v : vehicles) {
                if (v != this && v->isTransitioningPose() && std::abs(v->getProgressOnRoad() - progressOnCurrentRoad) < 15.0) {
                    return true;
                }
            }
            return false;
        };
        return checkLane(currentLaneIndex) || checkLane(targetLane);
    };

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
        
        // Rule 4: No crossing lane changes
        if (hasTransitioningVehicleNearby(candidateLane)) {
            continue;
        }

        LaneChangeCandidate candidate = laneChangePolicy.assess(
            *this, *currentRoad, candidateLane,
            LANE_CHANGE_REAR_SAFETY_TIME, LANE_CHANGE_MIN_TTC);
        if (!candidate.safe || candidate.gapAhead <= requiredGapAhead) continue;

        if (laneChangePolicy.isBetter(candidate, bestCandidate)) {
            bestCandidate = candidate;
        }
    }

    if (bestCandidate.safe) {
        requestLaneChange(
            bestCandidate.laneIndex,
            TurnSignalReason::LaneChange);
        const double signalLeadTime = hasTrafficPriority()
            ? PRIORITY_SIGNAL_LEAD_TIME_SECONDS
            : MIN_SIGNAL_LEAD_TIME_SECONDS;
        // if (laneChangeSignalElapsedSeconds_ + 1e-9 < signalLeadTime) {
        //     laneChangeState_ = LaneChangeState::Signaling;
        //     return;
        // }
        laneChangeState_ = LaneChangeState::WaitingForGap;
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        const Pose2D fromPose = RoadGeometry::sampleLane(
            *currentRoad,
            currentLaneIndex,
            progressOnCurrentRoad);
        const Pose2D toPose = RoadGeometry::sampleLane(
            *currentRoad,
            bestCandidate.laneIndex,
            progressOnCurrentRoad);
        currentLaneIndex = bestCandidate.laneIndex;
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
        startPoseTransition(
            fromPose,
            toPose,
            LANE_CHANGE_POSE_TRANSITION_SECONDS);
        clearLaneChangeIntent();
        refreshTurnSignal();
        laneChangeCooldownTimer = hasTrafficPriority()
            ? PRIORITY_LANE_CHANGE_COOLDOWN
            : LANE_CHANGE_COOLDOWN;
    } else {
        laneChangeCooldownTimer = LANE_CHANGE_POSE_TRANSITION_SECONDS + 3.0; // Wait longer before consecutive lane changes
    }
}

void Vehicle::tryYieldLaneChange() {
    if (currentRoad == nullptr || !canChangeLanes()) return;
    if (currentRoad->getLaneCount() <= 1) return;
    if (emergencyLaneToAvoid < 0 || currentLaneIndex != emergencyLaneToAvoid) return;

    LaneChangeCandidate bestCandidate;
    const int candidateLanes[2] = { currentLaneIndex - 1, currentLaneIndex + 1 };
    for (int candidateLane : candidateLanes) {
        LaneChangeCandidate candidate = laneChangePolicy.assess(
            *this, *currentRoad, candidateLane,
            LANE_CHANGE_REAR_SAFETY_TIME * 0.5,
            YIELD_LANE_CHANGE_MIN_TTC);
        if (laneChangePolicy.isBetter(candidate, bestCandidate)) {
            bestCandidate = candidate;
        }
    }

    if (bestCandidate.safe) {
        requestLaneChange(
            bestCandidate.laneIndex,
            TurnSignalReason::LaneChange);
        // if (laneChangeSignalElapsedSeconds_ + 1e-9 <
        //     MIN_SIGNAL_LEAD_TIME_SECONDS) {
        //     laneChangeState_ = LaneChangeState::Signaling;
        //     return;
        // }
        laneChangeState_ = LaneChangeState::WaitingForGap;
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        const Pose2D fromPose = RoadGeometry::sampleLane(
            *currentRoad,
            currentLaneIndex,
            progressOnCurrentRoad);
        const Pose2D toPose = RoadGeometry::sampleLane(
            *currentRoad,
            bestCandidate.laneIndex,
            progressOnCurrentRoad);
        currentLaneIndex = bestCandidate.laneIndex;
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
        startPoseTransition(
            fromPose,
            toPose,
            LANE_CHANGE_POSE_TRANSITION_SECONDS);
        clearLaneChangeIntent();
        refreshTurnSignal();
        laneChangeCooldownTimer = LANE_CHANGE_COOLDOWN * 0.5;
    } else {
        laneChangeCooldownTimer = 0.25; // Short cooldown when yield lane change fails
    }
}
void Vehicle::notifyEmergencyApproaching(int emergencyLaneIndex) {
    yielding = true;
    yieldCooldownTimer = YIELD_COOLDOWN_DURATION;
    emergencyLaneToAvoid = emergencyLaneIndex;
}
