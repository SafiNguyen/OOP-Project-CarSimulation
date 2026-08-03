#include "Vehicle.h"
#include "Road.h"
#include "Lane.h"
#include "Intersection.h"
#include "RoadGeometry.h"
#include "VehicleMath.h"
using namespace VehicleMath;
#include "JunctionConnector.h"
#include "PointOfInterest.h"
#include <iostream>

bool Vehicle::advanceToNextRoad() {
    return routeFollower.advanceToNextRoad(*this);
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
    if (poseTransitionTimer_ > 0.0) {
        poseTransitionTimer_ = std::max(
            0.0,
            poseTransitionTimer_ - nonNegativeDt);
    }
    refreshTurnSignal();

    if (isMergingFromPOI) {
        if (poiMergePhase_ == PoiMergePhase::None) {
            poiMergePhase_ =
                PoiMergePhase::ApproachingYieldLine;
            poiAnimationTimer =
                getPoiMergePhaseDuration(
                    poiMergePhase_);
        }

        if (poiMergePhase_ ==
            PoiMergePhase::ApproachingYieldLine) {
            updatePoiAnimation(nonNegativeDt);
            if (poiAnimationTimer <= 0.0) {
                poiMergePhase_ =
                    PoiMergePhase::WaitingForGap;
            }
            return;
        }

        if (poiMergePhase_ ==
            PoiMergePhase::WaitingForGap) {
            if (canCommitPoiMerge()) {
                poiMergePhase_ =
                    PoiMergePhase::Committed;
                poiAnimationTimer =
                    getPoiMergePhaseDuration(
                        poiMergePhase_);
            }
            return;
        }

        if (poiMergePhase_ != PoiMergePhase::Committed) {
            return;
        }

        updatePoiAnimation(nonNegativeDt);
        if (poiAnimationTimer > 0.0) {
            return;
        }

        // The reservation was visible to followers throughout the crossing.
        // Replace it atomically with normal lane membership so collision
        // queries never lose sight of the vehicle for a frame.
        Road* mergedRoad = currentRoad;
        const int mergedLane = mergeLaneIndex;
        const double mergedProgress = mergeProgressOffset;
        setMergingFromPOI(false);
        currentLaneIndex = mergedLane;
        progressOnCurrentRoad = mergedProgress;
        currentSpeed = 0.0;
        mergedRoad->getLane(mergedLane).addVehicle(this);
        releaseSpawnSlot();
        spawnLifecycleState_ =
            SpawnLifecycleState::Active;
        laneChangeCooldownTimer = 4.0; // Delay lane change immediately after POI merge
    }
    
    if (targetPOI != nullptr && currentRoad == targetPOI->getConnectedRoad()) {
        if (currentRouteIndex == static_cast<int>(currentRoute.size()) - 1) {
            int targetLane = targetPOI->getAccessLaneIndex();
            if (targetLane < 0) targetLane = currentRoad->getCurbLaneIndex();
            
            if (currentLaneIndex == targetLane && !isEnteringPOI && progressOnCurrentRoad >= targetPOI->getProgressOffset()) {
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
            recalculateTimer = nextRerouteDelaySeconds();
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
                    recalculateRoute(*graph, strategy);
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
            const int curbYieldLane =
                getRedLightCurbYieldLane();
            if (laneChangeCooldownTimer <= 0.0 &&
                curbYieldLane >= 0 &&
                curbYieldLane != currentLaneIndex) {
                tryRequiredLaneChange(
                    curbYieldLane,
                    TurnSignalReason::Junction);
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
                const double stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * std::max(getDeceleration(), 1e-6));
                const double safetyBuffer = 1.0; // metres: small margin
                if (distanceToIntersection <=
                    stoppingDistance + safetyBuffer) {
                    const PauseReason controlReason =
                        getIntersectionControlReason();
                    if (controlReason !=
                        PauseReason::None) {
                        targetSpeed = 0.0;
                    }
                }
            }
        }
        
        // Slow down when approaching destination POI
        if (targetPOI != nullptr && currentRoad == targetPOI->getConnectedRoad() && currentRouteIndex == static_cast<int>(currentRoute.size()) - 1) {
            double distToPOI = targetPOI->getProgressOffset() - progressOnCurrentRoad;
            int targetLane = targetPOI->getAccessLaneIndex();
            if (targetLane < 0) targetLane = currentRoad->getCurbLaneIndex();

            if (currentLaneIndex != targetLane) {
                // We are in the wrong lane. We must change lanes BEFORE reaching the POI.
                // Pick a point 25 meters before the POI to stop and wait for a clear gap.
                double waitPoint = std::max(0.0, targetPOI->getProgressOffset() - 25.0);
                double distToWaitPoint = waitPoint - progressOnCurrentRoad;

                if (distToWaitPoint <= 0.0) {
                    targetSpeed = 0.0;
                    currentSpeed = 0.0;
                } else {
                    const double stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * std::max(getDeceleration(), 1e-6));
                    const double safetyBuffer = 3.0; // metres
                    if (distToWaitPoint <= stoppingDistance + safetyBuffer) {
                        targetSpeed = 0.0;
                    }
                }
            } else {
                if (distToPOI > 0.0) {
                    const double stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * std::max(getDeceleration(), 1e-6));
                    const double safetyBuffer = 3.0; // metres
                    if (distToPOI <= stoppingDistance + safetyBuffer) {
                        targetSpeed = std::min(targetSpeed, 2.0); // Decelerate to 2 m/s before turning into POI
                    }
                } else {
                    targetSpeed = 0.0;
                    currentSpeed = 0.0;
                    progressOnCurrentRoad = targetPOI->getProgressOffset();
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
            (laneChangePolicy.usesDedicatedEdgeLane(static_cast<int>(upcomingMapping.movement)) ||
             distanceToJunction <=
                 getJunctionLanePreparationDistance());
        const int stopOrServiceLane = getRequiredLaneIndex();
        const int redLightCurbYieldLane =
            getRedLightCurbYieldLane();
        const int targetPoiLane = getRequiredLaneIndex();
        const int requiredLaneIndex =
            targetPoiLane >= 0
                ? targetPoiLane
                : stopOrServiceLane >= 0
                    ? stopOrServiceLane
                    : redLightCurbYieldLane >= 0
                          ? redLightCurbYieldLane
                          : (preparingForJunction
                                 ? upcomingMapping.incomingLane
                                 : -1);
        const bool clearingEmergencyLane =
            yielding && emergencyLaneToAvoid >= 0 &&
            currentLaneIndex == emergencyLaneToAvoid;
        const bool bypassingQueueForPriority =
            upcomingMapping.valid &&
            shouldBypassQueueBeforeJunction(upcomingMapping);
        const bool hasRequiredLane =
            !clearingEmergencyLane &&
            !bypassingQueueForPriority &&
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
                
        if (laneChangeState_ != LaneChangeState::Idle) {
            targetSpeed *= 0.85; // Giảm tốc độ khi đổi lane
        }

        Vehicle* leader = currentRoad->findLeader(currentLaneIndex, this);
        double minGap = getMinGap();
        double gapToLeader = std::numeric_limits<double>::infinity();

        if (leader != nullptr) {
            gapToLeader = bumperGap(*this, *leader);
        } else {
            Road* nextRoad = getNextRoad();
            if (nextRoad != nullptr) {
                const int nextLaneIndex = entryMapping.valid
                    ? entryMapping.outgoingLane
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
        currentLeader_ = leader;

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
            const bool atIntersectionStopPosition =
                progressOnCurrentRoad + 1e-6 >=
                getIntersectionStopPosition();

            isWaitingForLight_ = false;
            if (atIntersectionStopPosition) {
                const PauseReason controlReason =
                    getIntersectionControlReason();
                isWaitingForLight_ =
                    controlReason != PauseReason::None;
                if (controlReason != PauseReason::None) {
                    beginPause(controlReason);
                    break;
                }
            }

            if (isStuckInJam()) {
                stuckTimer += subDt;
            } else {
                stuckTimer = 0.0;
            }

            bool isWaitingForPOI = (targetPOI != nullptr && currentRoad == targetPOI->getConnectedRoad() && currentRouteIndex == static_cast<int>(currentRoute.size()) - 1);
            if (!isWaitingForPOI && allowsUTurn() &&
                stuckTimer > patienceThreshold &&
                graph != nullptr && strategy != nullptr) {
                // Check if it's safe to U-turn (no vehicle closely behind)
                Vehicle* follower = currentRoad->findFollower(currentLaneIndex, this);
                bool safeToUTurn = true;
                if (follower != nullptr) {
                    const double gapBehind =
                        bumperGap(*follower, *this);
                    if (follower->getCurrentSpeed() <= 0.01) {
                        safeToUTurn = true; // Xe sau dung yen thi cho phep u-turn
                    } else if (gapBehind < 5.0 && follower->getCurrentSpeed() > 0.01) {
                        safeToUTurn = false;
                    } else if (gapBehind < 1.0) { // they are very close, still unsafe
                        safeToUTurn = false;
                    }
                }
                
                if (safeToUTurn && currentLaneIndex == 0 && uTurnCooldownTimer <= 0.0) {
                    if (performUTurn(*graph, strategy)) {
                        stuckTimer = 0.0;
                        uTurnCooldownTimer = 30.0; // 30s cooldown
                        laneChangeCooldownTimer = LANE_CHANGE_COOLDOWN;
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
                if (!entryMapping.valid ||
                    currentLaneIndex != entryMapping.incomingLane ||
                    !beginJunctionTraversal(
                        entryMapping, nextIntersection)) {
                    progressOnCurrentRoad = std::min(
                        currentRoad->getDistance(),
                        std::max(currentPos, getIntersectionStopPosition()));
                    targetSpeed = 0.0;
                    currentSpeed = 0.0;
                    remainingTime -= subDt;
                    continue;
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
                    targetSpeed = 0.0;
                    currentSpeed = 0.0;
                    remainingTime -= subDt;
                    continue;
                }
            }
            const double distToEnd =
                currentRoad->getDistance() - currentPos;
            const double timeToEnd =
                (speed > 0.0) ? distToEnd / speed : subDt;

            remainingTime -= std::min(subDt, timeToEnd);

            if (hasNextRoad) {
                continue;
            } else {
                if (reservedIntersection_ != nullptr) {
                    reservedIntersection_->exit(getId());
                    reservedIntersection_ = nullptr;
                }

                // If we reached the end of our route but haven't entered the target POI, we missed it!
                if (targetPOI != nullptr && !hasReachedDestination()) {
                    if (graph != nullptr && strategy != nullptr && performUTurn(*graph, strategy)) {
                        stuckTimer = 0.0;
                        uTurnCooldownTimer = 30.0;
                        laneChangeCooldownTimer = LANE_CHANGE_COOLDOWN;
                        continue;
                    }
                }

                if (!advanceToNextRoad() || hasReachedDestination()) {
                    break;
                }
            }
        }
    }
}
bool Vehicle::recalculateRoute(const Graph& graph, PathFindingStrategy* strategy) {
    return routeFollower.recalculateRoute(*this, graph, strategy);
}
