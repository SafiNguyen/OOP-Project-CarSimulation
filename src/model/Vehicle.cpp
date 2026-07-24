#include <iostream>
#include "Vehicle.h"
#include "Road.h"
#include "Intersection.h"
#include "TrafficLight.h"
#include "Graph.h"
#include "algorithm/PathFindingStrategy.h"
#include <algorithm>
#include <limits>
#include <cstdlib>

namespace {

constexpr double INTERSECTION_STOP_LINE_OFFSET = 1.5;

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
    return leader->getProgressOnRoad()
         - vehicle.getProgressOnRoad()
         - leader->getLength();
}

double gapBehindOf(const Vehicle& vehicle, const Vehicle* follower) {
    if (follower == nullptr) {
        return std::numeric_limits<double>::infinity();
    }
    return vehicle.getProgressOnRoad()
         - follower->getProgressOnRoad()
         - vehicle.getLength();
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
      pauseReason(PauseReason::None),
      awaitingIntersectionTransition(false),
      intersectionTransitionTimer(0.0) {
    patienceThreshold = 3.0 + static_cast<double>(std::rand() % 50) / 10.0;
    recalculateTimer = 5.0 + static_cast<double>(std::rand() % 100) / 10.0;
}

Vehicle::~Vehicle() {
    if (reservedIntersection_ != nullptr) {
        reservedIntersection_->exit(getId());
        reservedIntersection_ = nullptr;
    }
    if (currentRoad != nullptr) {
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        currentRoad = nullptr;
    }
}

PauseReason Vehicle::getIntersectionControlReason() const {
    if (currentRoad == nullptr) {
        return PauseReason::None;
    }

    Intersection* nextIntersection = currentRoad->getEnd();
    if (nextIntersection == nullptr) {
        return PauseReason::None;
    }

    if (mustStopForTrafficLight(nextIntersection)) {
        return PauseReason::TrafficLight;
    }

    if (reservedIntersection_ != nextIntersection &&
        !currentRoad->getLane(currentLaneIndex).isBlocked() &&
        !nextIntersection->canEnter(getId(), currentRoad)) {
        return PauseReason::Intersection;
    }

    return PauseReason::None;
}

double Vehicle::getIntersectionStopPosition() const {
    if (currentRoad == nullptr) {
        return 0.0;
    }
    return std::max(
        0.0,
        currentRoad->getDistance() - INTERSECTION_STOP_LINE_OFFSET);
}

void Vehicle::beginPause(PauseReason reason) {
    if (reason == PauseReason::None) {
        clearPause();
        return;
    }

    const bool pauseJustStarted = !paused || pauseReason != reason;
    paused = true;
    pauseReason = reason;
    currentSpeed = 0.0;
    if (pauseJustStarted) {
        onPauseStarted();
    }
}

void Vehicle::clearPause() {
    paused = false;
    pauseReason = PauseReason::None;
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
    if (controlReason == PauseReason::TrafficLight) {
        Vehicle* leader = currentRoad->findLeader(currentLaneIndex, this);
        if (leader != nullptr && leader->isPaused()) {
            double leaderBack = leader->getProgressOnRoad() - leader->getLength();
            double desiredPos = leaderBack - getMinGap();
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
    currentRoute = route;
    currentRouteIndex = 0;
    progressOnCurrentRoad = 0.0;
    currentSpeed = 0.0;
    clearPause();
    routeAssigned = true;

    if (!currentRoute.empty()) {
        currentRoad = currentRoute[0];
        if (currentRoad) {
            currentLaneIndex = currentRoad->getFreestLaneIndex();
            currentRoad->getLane(currentLaneIndex).addVehicle(this);
        }
    } else {
        currentRoad = nullptr;
    }

    onRoadChanged();
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
    onRoadChanged();
    return false;
}

double Vehicle::getProgressRatio() const {
    if (currentRoad == nullptr || currentRoad->getDistance() <= 0.0) {
        return 0.0;
    }
    return progressOnCurrentRoad / currentRoad->getDistance();
}

bool Vehicle::tryRequiredLaneChange(int requiredLaneIndex) {
    if (currentRoad == nullptr ||
        !canChangeLanes() ||
        requiredLaneIndex < 0 ||
        requiredLaneIndex >= currentRoad->getLaneCount() ||
        requiredLaneIndex == currentLaneIndex) {
        return false;
    }

    const int adjacentLane = currentLaneIndex +
        (requiredLaneIndex > currentLaneIndex ? 1 : -1);
    const LaneChangeCandidate candidate = assessLaneChange(
        *this,
        *currentRoad,
        adjacentLane,
        LANE_CHANGE_REAR_SAFETY_TIME,
        LANE_CHANGE_MIN_TTC);
    if (!candidate.safe) {
        laneChangeCooldownTimer = 0.25;
        return false;
    }

    currentRoad->getLane(currentLaneIndex).removeVehicle(this);
    currentLaneIndex = adjacentLane;
    currentRoad->getLane(currentLaneIndex).addVehicle(this);
    laneChangeCooldownTimer = LANE_CHANGE_COOLDOWN;
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
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        currentLaneIndex = bestCandidate.laneIndex;
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
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
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        currentLaneIndex = bestCandidate.laneIndex;
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
        laneChangeCooldownTimer = LANE_CHANGE_COOLDOWN * 0.5;
    } else {
        laneChangeCooldownTimer = 0.25; // Short cooldown when yield lane change fails
    }
}

bool Vehicle::mustStopForTrafficLight(Intersection* nextIntersection) const {
    if (nextIntersection == nullptr || currentRoad == nullptr) {
        return false;
    }
    return nextIntersection->mustStopForRoad(currentRoad);
}

Road* Vehicle::getNextRoad() const {
    if (currentRouteIndex + 1 < static_cast<int>(currentRoute.size())) {
        return currentRoute[currentRouteIndex + 1];
    }
    return nullptr;
}

void Vehicle::update(double dt, Graph* graph, PathFindingStrategy* strategy) {
    if (hasReachedDestination() || currentRoad == nullptr) {
        return;
    }

    double remainingTime = std::max(0.0, dt);
    if (remainingTime <= 0.0) {
        return;
    }
    const double elapsedTimeThisUpdate = remainingTime;

    recalculateTimer -= elapsedTimeThisUpdate;
    if (recalculateTimer <= 0.0) {
        recalculateTimer = 10.0 + static_cast<double>(std::rand() % 50) / 10.0; // 10-15s
        if (graph && strategy && currentRoad) {
            bool hasCongestion = false;
            for (size_t i = currentRouteIndex + 1; i < currentRoute.size(); ++i) {
                if (currentRoute[i]->getDynamicCongestionLevel() > 1.5 || currentRoute[i]->isBlocked()) {
                    hasCongestion = true;
                    break;
                }
            }
            if (hasCongestion) {
                recalculateRoute(*graph, strategy);
            }
        }
    }

    if (awaitingIntersectionTransition) {
        intersectionTransitionTimer -= elapsedTimeThisUpdate;
        if (intersectionTransitionTimer <= 0.0) {
            intersectionTransitionTimer = 0.0;
            awaitingIntersectionTransition = false;
            if (reservedIntersection_ != nullptr) {
                reservedIntersection_->exit(getId());
                reservedIntersection_ = nullptr;
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
            const PauseReason currentControlReason =
                getIntersectionControlReason();
            if (currentControlReason != PauseReason::None) {
                beginPause(currentControlReason);
                return;
            }
            clearPause();
        }
    }

    while (remainingTime > 0.0 && currentRoad != nullptr && !paused) {
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
                const PauseReason controlReason =
                    getIntersectionControlReason();
                if (controlReason != PauseReason::None) {
                    const double distToStopLine = currentRoad->getDistance() - progressOnCurrentRoad;
                    const double stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * std::max(getDeceleration(), 1e-6));
                    const double safetyBuffer = 1.0; // metres: small margin
                    if (distToStopLine <= stoppingDistance + safetyBuffer) {
                        targetSpeed = 0.0;
                    }
                }
            }
        }
        
        if (yielding && laneChangeCooldownTimer <= 0.0) {
            tryYieldLaneChange();
        }

        const int requiredLaneIndex = getRequiredLaneIndex();
        const bool hasRequiredLane =
            requiredLaneIndex >= 0 &&
            requiredLaneIndex < currentRoad->getLaneCount();
        if (laneChangeCooldownTimer <= 0.0) {
            if (hasRequiredLane) {
                if (requiredLaneIndex != currentLaneIndex) {
                    tryRequiredLaneChange(requiredLaneIndex);
                }
            } else {
                tryLaneChange(freeFlowSpeed);
            }
        }


        Vehicle* leader = currentRoad->findLeader(currentLaneIndex, this);
        const double minGap = getMinGap();
        double gapToLeader = std::numeric_limits<double>::infinity();

        if (leader != nullptr) {
            gapToLeader = leader->getProgressOnRoad() - progressOnCurrentRoad - leader->getLength();
        } else {
            Road* nextRoad = getNextRoad();
            if (nextRoad != nullptr) {
                int nextLaneIndex = currentLaneIndex;
                if (nextLaneIndex >= nextRoad->getLaneCount()) {
                    nextLaneIndex = nextRoad->getLaneCount() - 1; // clamp phong khi so lane khac nhau
                }
                Vehicle* nextLeader = nextRoad->getFirstVehicleInLane(nextLaneIndex);
                if (nextLeader != nullptr) {
                    const double distToEndOfCurrentRoad = currentRoad->getDistance() - progressOnCurrentRoad;
                    gapToLeader = distToEndOfCurrentRoad
                                + nextLeader->getProgressOnRoad()
                                - nextLeader->getLength();
                    leader = nextLeader; 
                }
            }
        }

        if (leader != nullptr) {
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
            if (stuckTimer > patienceThreshold && graph != nullptr && strategy != nullptr) {
                // Check if it's safe to U-turn (no vehicle closely behind)
                Vehicle* follower = currentRoad->findFollower(currentLaneIndex, this);
                bool safeToUTurn = true;
                if (follower != nullptr) {
                    double gapBehind = progressOnCurrentRoad - getLength() - follower->getProgressOnRoad();
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

            if (nextIntersection != nullptr && reservedIntersection_ != nextIntersection) {
                if (nextIntersection->tryEnter(getId(), currentRoad)) {
                    reservedIntersection_ = nextIntersection;
                } else {
                    progressOnCurrentRoad = std::min(
                        currentRoad->getDistance(),
                        std::max(currentPos, getIntersectionStopPosition()));
                    beginPause(PauseReason::Intersection);
                    break;
                }
            }
            double distToEnd = currentRoad->getDistance() - currentPos;
            double timeToEnd = (speed > 0.0) ? distToEnd / speed : 0.0;

            remainingTime -= timeToEnd;
            if (remainingTime < 0.0) {
                remainingTime = 0.0;
            }
            progressOnCurrentRoad = 0.0;


            if (currentRouteIndex + 1 < static_cast<int>(currentRoute.size())) {
                awaitingIntersectionTransition = true;
                intersectionTransitionTimer = INTERSECTION_TRANSITION_DURATION;
                if (!advanceToNextRoad()) break;
                continue;
            } else {
                if (reservedIntersection_ != nullptr) {
                    reservedIntersection_->exit(getId());
                    reservedIntersection_ = nullptr;
                }

                if (!advanceToNextRoad()) {
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
    if (currentRoad == nullptr || destination == nullptr) {
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
    if (currentRoad == nullptr || destination == nullptr) {
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
