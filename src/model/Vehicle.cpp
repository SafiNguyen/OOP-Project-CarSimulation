#include <iostream>
#include "Vehicle.h"
#include "Road.h"
#include "Intersection.h"
#include "TrafficLight.h"
#include "Graph.h"
#include "algorithm/PathFindingStrategy.h"
#include <algorithm>
#include <limits>


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
      awaitingIntersectionTransition(false),
      intersectionTransitionTimer(0.0) {
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

bool Vehicle::shouldPauseAt(double currentPos,
                            double projectedPos,
                            double& pausePos) {
    if (currentRoad == nullptr) return false;

    Intersection* nextIntersection = currentRoad->getEnd();
    if (nextIntersection == nullptr) return false;

    // Only consider traffic-light enforced stops here.
    if (!mustStopForTrafficLight(nextIntersection)) return false;

    // Position of the nominal stop line a short distance before the road end.
    constexpr double STOP_LINE_OFFSET = 1.5; // metres before road end
    const double roadEnd = currentRoad->getDistance();
    double stopLinePos = std::max(0.0, roadEnd - STOP_LINE_OFFSET);

    // If there's a paused leader ahead on this lane, queue behind it
    Vehicle* leader = currentRoad->findLeader(currentLaneIndex, this);
    if (leader != nullptr && leader->isPaused()) {
        double leaderBack = leader->getProgressOnRoad() - leader->getLength();
        double desiredPos = leaderBack - getMinGap();
        // Don't allow desiredPos to go past the nominal stop line further upstream
        if (desiredPos < stopLinePos) {
            stopLinePos = desiredPos;
        }
    }

    if (stopLinePos <= currentPos) return false;

    // We only pause if our projected position reaches or passes the pausePos
    pausePos = std::min(stopLinePos, projectedPos);
    return pausePos > currentPos;
}

void Vehicle::setRoute(const std::vector<Road*>& route) {
    currentRoute = route;
    currentRouteIndex = 0;
    progressOnCurrentRoad = 0.0;
    currentSpeed = 0.0;
    paused = false;
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
    return false;
}

double Vehicle::getProgressRatio() const {
    if (currentRoad == nullptr || currentRoad->getDistance() <= 0.0) {
        return 0.0;
    }
    return progressOnCurrentRoad / currentRoad->getDistance();
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
    double currentGapAhead = std::numeric_limits<double>::infinity();
    if (currentLeader != nullptr) {
        currentGapAhead = currentLeader->getProgressOnRoad() - progressOnCurrentRoad - currentLeader->getLength();
    }

    const double minGap = getMinGap();
    const double desiredGap = minGap + freeFlowSpeed * getTimeHeadway();

    if (currentGapAhead >= desiredGap) {
        return; // khong bi can tro dang ke, khong can doi lane
    }

    // 2) Xet 2 lane lan can (trai/phai). Chon lane tot nhat trong so cac
    //    lane thoa dieu kien "tot hon dang ke" (LANE_CHANGE_GAP_IMPROVEMENT_FACTOR)
    //    VA an toan cho xe phia sau o lane do.
    int bestLaneIndex = -1;
    double bestGapAhead = currentGapAhead;

    const int candidateLanes[2] = { currentLaneIndex - 1, currentLaneIndex + 1 };
    for (int candidateLane : candidateLanes) {
        if (candidateLane < 0 || candidateLane >= currentRoad->getLaneCount()) {
            continue;
        }

        Vehicle* candidateLeader = currentRoad->findLeader(candidateLane, this);
        double gapAhead = std::numeric_limits<double>::infinity();
        if (candidateLeader != nullptr) {
            gapAhead = candidateLeader->getProgressOnRoad() - progressOnCurrentRoad - candidateLeader->getLength();
        }

        if (gapAhead <= bestGapAhead * LANE_CHANGE_GAP_IMPROVEMENT_FACTOR) {
            continue;
        }

        Vehicle* candidateFollower = currentRoad->findFollower(candidateLane, this);
        if (candidateFollower != nullptr) {
            const double gapBehind = progressOnCurrentRoad - candidateFollower->getProgressOnRoad() - getLength();
            const double followerSpeed = candidateFollower->getCurrentSpeed();
            const double requiredGapBehind = minGap + followerSpeed * LANE_CHANGE_REAR_SAFETY_TIME;
            if (gapBehind < requiredGapBehind) {
                continue; 
            }
        }

        bestLaneIndex = candidateLane;
        bestGapAhead = gapAhead;
    }

    if (bestLaneIndex != -1 && bestGapAhead > minGap ) {
        currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        currentLaneIndex = bestLaneIndex;
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
        laneChangeCooldownTimer = LANE_CHANGE_COOLDOWN;
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

void Vehicle::update(double dt) {
    if (hasReachedDestination() || currentRoad == nullptr) {
        return;
    }

    if (awaitingIntersectionTransition) {
        intersectionTransitionTimer -= dt;
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
        laneChangeCooldownTimer -= dt;
        if (laneChangeCooldownTimer < 0.0) {
            laneChangeCooldownTimer = 0.0;
        }
    }

    if (paused) {
        if (updatePause(dt)) {
            paused = false;
        }
        return;
    }

    double remainingTime = dt;

    while (remainingTime > 0.0 && currentRoad != nullptr && !paused) {
        const double freeFlowSpeed = calculateCurrentSpeed();
        double targetSpeed = freeFlowSpeed;
        if (currentRoad != nullptr) {
            Intersection* nextIntersectionForLight = currentRoad->getEnd();
            if (nextIntersectionForLight != nullptr) {
                TrafficLight* upcomingLight = nextIntersectionForLight->getLightForIncomingRoad(currentRoad);
                bool lightRequiresStop = (upcomingLight != nullptr && upcomingLight->mustStop());
                bool boxRequiresStop = (reservedIntersection_ != nextIntersectionForLight)
                                       && nextIntersectionForLight->isFull();

                if (lightRequiresStop || boxRequiresStop) {
                    const double distToStopLine = currentRoad->getDistance() - progressOnCurrentRoad;
                    const double stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * std::max(getDeceleration(), 1e-6));
                    const double safetyBuffer = 1.0; // metres: small margin
                    if (distToStopLine <= stoppingDistance + safetyBuffer) {
                        targetSpeed = 0.0;
                    }
                }
            }
        }

        if (laneChangeCooldownTimer <= 0.0) {
            tryLaneChange(freeFlowSpeed);
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
            break;
        }

        double distanceThisTick = speed * remainingTime;
        if (leader != nullptr) {
            const double maxAdvance = std::max(0.0, gapToLeader - minGap);
            distanceThisTick = std::min(distanceThisTick, maxAdvance);
        }
        double currentPos = progressOnCurrentRoad;
        double projectedPos = currentPos + distanceThisTick;

        double pausePos = -1.0;
        if (shouldPauseAt(currentPos, projectedPos, pausePos)
            && pausePos > currentPos
            && pausePos <= currentRoad->getDistance()) {
            progressOnCurrentRoad = pausePos;
            paused = true;
            onPauseStarted();
            break;
        }

        if (projectedPos < currentRoad->getDistance()) {
            progressOnCurrentRoad = projectedPos;
            remainingTime -= subDt;
        } else {
            Intersection* nextIntersection = currentRoad->getEnd();

            bool boxBlocked = (nextIntersection != nullptr)
                              && (reservedIntersection_ != nextIntersection)
                              && nextIntersection->isFull();

            if (mustStopForTrafficLight(nextIntersection) || boxBlocked) {
                progressOnCurrentRoad = currentRoad->getDistance();
                currentSpeed = 0.0;
                remainingTime = 0.0;
                break;
            }

            if (nextIntersection != nullptr && reservedIntersection_ != nextIntersection) {
                nextIntersection->tryEnter(getId());
                reservedIntersection_ = nextIntersection;
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
    paused = false;
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

    currentRoad = reverseRoad;
    progressOnCurrentRoad = currentRoad->getDistance() - progressOnCurrentRoad;
    if (progressOnCurrentRoad < 0) {
        progressOnCurrentRoad = 0;
    }

    currentSpeed = 0.0;
    paused = false;

    std::vector<Road*> newRoute;
    for (int i = 0; i < currentRouteIndex; ++i) {
        newRoute.push_back(currentRoute[i]);
    }

    newRoute.push_back(currentRoad);
    for (Road* r : result.roadPath) {
        newRoute.push_back(r);
    }

    currentRoute = newRoute;
    return true;
}