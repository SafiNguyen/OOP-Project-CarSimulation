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
        // Early braking for traffic lights: if the upcoming intersection
        // has a light requiring stop (RED or YELLOW) and the vehicle
        // cannot safely clear the stop line given current speed and
        // braking capability, begin braking now (set targetSpeed=0).
        if (currentRoad != nullptr) {
            Intersection* nextIntersectionForLight = currentRoad->getEnd();
            if (nextIntersectionForLight != nullptr) {
                TrafficLight* upcomingLight = nextIntersectionForLight->getLightForIncomingRoad(currentRoad);
                bool lightRequiresStop = (upcomingLight != nullptr && upcomingLight->mustStop());

                // Intersection-box reservation: even on a green light, if the
                // physical box is already occupied by a vehicle coming from
                // another road/lane, we still have to brake for it - this is
                // what stops vehicles from different approaches rendering on
                // top of each other inside the junction.
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
        // Tim xe ngay phia truoc trong cung lane. Neu co, gioi han
        // targetSpeed theo khoang cach con lai (gap) so voi minGap va
        // "khoang cach thoai mai" o toc do mong muon (desiredGap).
        //   gap <= minGap            -> dung han (targetSpeed = 0)
        //   gap >= desiredGap        -> chay tu do (targetSpeed = freeFlowSpeed)
        //   minGap < gap < desiredGap -> giam toc tuyen tinh theo ty le gap
        Vehicle* leader = currentRoad->findLeader(currentLaneIndex, this);
        const double minGap = getMinGap();
        double gapToLeader = std::numeric_limits<double>::infinity();

        if (leader != nullptr) {
            gapToLeader = leader->getProgressOnRoad() - progressOnCurrentRoad - leader->getLength();
        } else {
            // Không có ai phía trước trên road hiện tại -> thử nhìn sang road kế tiếp
            // (Task 1.2 nâng cấp: tránh xe "phóng" hết road rồi mới phát hiện vật cản
            // ngay khi vừa đổi road, gây tunneling/dồn xe tại nút giao).
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
            if (gapToLeader <= minGap) {
                targetSpeed = 0.0;
            } else {
                const double desiredGap = minGap + freeFlowSpeed * getTimeHeadway();
                if (gapToLeader < desiredGap && desiredGap > minGap) {
                    targetSpeed = freeFlowSpeed * (gapToLeader - minGap) / (desiredGap - minGap);
                }
            }

            const double stoppingDistance =
                (currentSpeed * currentSpeed) / (2.0 * std::max(getDeceleration(), 1e-6));
            if (gapToLeader - minGap < stoppingDistance) {
                targetSpeed = 0.0;
            }
        }

        if (currentSpeed < targetSpeed) {
            currentSpeed = std::min(targetSpeed, currentSpeed + getAcceleration() * remainingTime);
        } else if (currentSpeed > targetSpeed) {
            currentSpeed = std::max(targetSpeed, currentSpeed - getDeceleration() * remainingTime);
        }

        double speed = currentSpeed;
        if (speed <= 0.0) {
            break;
        }

        double distanceThisTick = speed * remainingTime;
        // Hard safety clamp: bat ke toc do/dt tinh ra la bao nhieu, xe
        // KHONG BAO GIO duoc phep tien qua (vi tri xe truoc - minGap)
        // trong 1 lan goi update() nay. Can thiet vi cong thuc "soft" o
        // tren gia dinh dt nho; khi speedMultiplier cao (Task 5), remainingTime
        // co the len toi vai giay, va ap dung targetSpeed hang so cho ca
        // khoang thoi gian lon do co the khien xe "xuyen" qua xe truoc
        // (buoc nhay Euler qua lon so voi 1 buoc vat ly lien tuc).
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
            remainingTime = 0.0;
        } else {
            Intersection* nextIntersection = currentRoad->getEnd();

            // Same idea as the two intersections up above: a green light is
            // not enough on its own, the box itself must have a free slot
            // too (unless we already reserved one for this intersection).
            bool boxBlocked = (nextIntersection != nullptr)
                              && (reservedIntersection_ != nextIntersection)
                              && nextIntersection->isFull();

            if (mustStopForTrafficLight(nextIntersection) || boxBlocked) {
                progressOnCurrentRoad = currentRoad->getDistance();
                currentSpeed = 0.0;
                remainingTime = 0.0;
                break;
            }

            // Clear to cross: claim a slot in the box before leaving this
            // road, so no other vehicle can be granted the same slot in the
            // same tick. Released once the intersection-transition animation
            // finishes (see the awaitingIntersectionTransition block above)
            // or in the destructor if the vehicle is removed mid-transition.
            if (nextIntersection != nullptr && reservedIntersection_ != nextIntersection) {
                nextIntersection->tryEnter(getId());
                reservedIntersection_ = nextIntersection;
            }

            if (currentRouteIndex + 1 < static_cast<int>(currentRoute.size())) {
                double distToEnd = currentRoad->getDistance() - currentPos;
                double timeToEnd = (speed > 0.0) ? distToEnd / speed : 0.0;
                remainingTime -= timeToEnd;
                if (remainingTime < 0.0) {
                    remainingTime = 0.0;
                }
                progressOnCurrentRoad = 0.0;
                awaitingIntersectionTransition = true;
                intersectionTransitionTimer = INTERSECTION_TRANSITION_DURATION;
                if (!advanceToNextRoad()) break;
                continue;   
            }

            double distToEnd = currentRoad->getDistance() - currentPos;
            double timeToEnd = (speed > 0.0) ? distToEnd / speed : 0.0;
            remainingTime -= timeToEnd;
            progressOnCurrentRoad = 0.0;

            // Journey ends at this intersection - there is no further road
            // to turn onto, so there is no transition animation to wait
            // out. Release the box slot right away instead of leaking it.
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