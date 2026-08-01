#ifndef BUS_H
#define BUS_H

#include "BusService.h"
#include "BusStop.h"
#include "BusTripPlan.h"
#include "Road.h"
#include "SpawnPoint.h"
#include "Vehicle.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

enum class BusTripState {
    WaitingAtOrigin,
    Departing,
    EnRoute,
    Dwelling,
    Arrived
};

class Bus : public Vehicle {
public:
    static constexpr double CRUISE_FACTOR = 0.85;
    static constexpr double DEFAULT_DWELL = 15.0;
    static constexpr double STOP_LANE_APPROACH_DISTANCE = 60.0;
    static constexpr double STOP_LANE_SAFETY_BUFFER = 15.0;
    static constexpr double STOP_LANE_PREPARATION_TIME = 1.0;
    static constexpr double STOP_LANE_MIN_ROLLING_SPEED = 1.5;

private:
    double dwellTime;
    double dwellTimer;
    const BusStop* nextStop;
    double nextStopPos;

    std::string fleetCode_;
    const BusService* service_;
    const BusStation* originStation_;
    const BusStation* destinationStation_;
    std::vector<const BusStop*> assignedStops_;
    std::vector<std::size_t> assignedStopRouteIndices_;
    double scheduledDepartureTime_;
    std::size_t scheduledStopIndex_;
    std::vector<int> servedStopIds_;
    std::vector<int> missedStopIds_;
    BusTripState tripState_;
    bool departureSlotHeld_;

    bool hasValidNextStopAhead() const {
        return currentRoad != nullptr &&
               nextStop != nullptr &&
               nextStop->getRoad() == currentRoad &&
               nextStopPos > progressOnCurrentRoad;
    }

    const BusStop* scheduledStop() const {
        if (service_ == nullptr) {
            return nextStop;
        }
        return scheduledStopIndex_ <
                   assignedStops_.size()
            ? assignedStops_[scheduledStopIndex_]
            : nullptr;
    }

    std::size_t scheduledStopRouteIndex() const {
        if (service_ == nullptr ||
            scheduledStopIndex_ >=
                assignedStopRouteIndices_.size()) {
            return currentRoute.size();
        }
        return assignedStopRouteIndices_[
            scheduledStopIndex_];
    }

    static BusTripPlan makeDefaultTripPlan(
        const BusService& service) {
        BusTripPlan plan;
        plan.roadRoute = service.getRoadRoute();
        plan.orderedStops =
            service.getOrderedStops();

        std::size_t searchFrom = 0u;
        for (const BusStop* stop :
             plan.orderedStops) {
            const auto found = std::find(
                plan.roadRoute.begin() +
                    static_cast<std::ptrdiff_t>(
                        std::min(
                            searchFrom,
                            plan.roadRoute.size())),
                plan.roadRoute.end(),
                stop != nullptr
                    ? stop->getRoad()
                    : nullptr);
            if (found == plan.roadRoute.end()) {
                plan.stopRouteIndices.push_back(
                    plan.roadRoute.size());
                continue;
            }
            searchFrom =
                static_cast<std::size_t>(
                    std::distance(
                        plan.roadRoute.begin(),
                        found));
            plan.stopRouteIndices.push_back(
                searchFrom);
        }
        return plan;
    }

    static bool isValidTripPlan(
        const BusService& service,
        const BusTripPlan& plan) {
        if (plan.roadRoute.empty() ||
            !std::isfinite(
                plan.scheduledDepartureTime) ||
            plan.scheduledDepartureTime < 0.0 ||
            plan.orderedStops.size() !=
                plan.stopRouteIndices.size() ||
            plan.roadRoute.front() !=
                service.getOriginStation().
                    getDepartureRoad() ||
            plan.roadRoute.back() !=
                service.getDestinationStation().
                    getArrivalRoad()) {
            return false;
        }

        for (std::size_t routeIndex = 0u;
             routeIndex < plan.roadRoute.size();
             ++routeIndex) {
            Road* road = plan.roadRoute[routeIndex];
            if (road == nullptr ||
                (routeIndex > 0u &&
                 plan.roadRoute[routeIndex - 1u]->
                         getEnd() !=
                     road->getStart())) {
                return false;
            }
        }

        for (std::size_t stopIndex = 0u;
             stopIndex < plan.orderedStops.size();
             ++stopIndex) {
            const BusStop* stop =
                plan.orderedStops[stopIndex];
            const std::size_t routeIndex =
                plan.stopRouteIndices[stopIndex];
            if (stop == nullptr ||
                routeIndex >=
                    plan.roadRoute.size() ||
                stop->getRoad() !=
                    plan.roadRoute[routeIndex] ||
                (stopIndex > 0u &&
                 routeIndex <
                     plan.stopRouteIndices[
                         stopIndex - 1u]) ||
                (stopIndex > 0u &&
                 routeIndex ==
                     plan.stopRouteIndices[
                         stopIndex - 1u] &&
                 stop->getPositionOnRoad() <=
                     plan.orderedStops[
                         stopIndex - 1u]->
                             getPositionOnRoad())) {
                return false;
            }
        }
        return true;
    }

    void markScheduledStopMissed() {
        const BusStop* stop = scheduledStop();
        if (service_ == nullptr || stop == nullptr) {
            return;
        }
        missedStopIds_.push_back(stop->getId());
        ++scheduledStopIndex_;
        nextStop = nullptr;
        nextStopPos = -1.0;
    }

    void markStopsMissedBeforeCurrentPosition() {
        if (service_ == nullptr || currentRoad == nullptr) {
            return;
        }
        const std::size_t currentRoadIndex =
            static_cast<std::size_t>(
                std::max(0, currentRouteIndex));
        while (const BusStop* stop = scheduledStop()) {
            const std::size_t stopRoadIndex =
                scheduledStopRouteIndex();
            const bool roadWasPassed =
                stopRoadIndex < currentRoadIndex;
            const bool positionWasPassed =
                stopRoadIndex == currentRoadIndex &&
                stop->getRoad() == currentRoad &&
                stop->getPositionOnRoad() <=
                    progressOnCurrentRoad;
            if (!roadWasPassed && !positionWasPassed) {
                break;
            }
            markScheduledStopMissed();
        }
    }

    void markRemainingStopsMissed() {
        while (scheduledStop() != nullptr) {
            markScheduledStopMissed();
        }
    }

    double getStopLaneApproachDistance() const {
        if (!hasValidNextStopAhead()) {
            return 0.0;
        }

        const int stopLane = nextStop->getLaneIndex();
        if (stopLane < 0 ||
            stopLane >= currentRoad->getLaneCount()) {
            return 0.0;
        }

        const int remainingLaneChanges =
            std::abs(stopLane - currentLaneIndex);
        const double speed = std::max(0.0, currentSpeed);
        const double brakingDistance =
            speed * speed /
            (2.0 * std::max(getDeceleration(), 1e-6));
        const double laneChangeDistance =
            speed * LANE_CHANGE_COOLDOWN *
            remainingLaneChanges;
        const double speedBuffer =
            speed * STOP_LANE_PREPARATION_TIME;
        const double desiredDistance = std::max(
            STOP_LANE_APPROACH_DISTANCE,
            brakingDistance + laneChangeDistance +
                speedBuffer + STOP_LANE_SAFETY_BUFFER);

        return std::min(
            desiredDistance,
            std::max(
                0.0,
                currentRoad->getDistance() -
                    progressOnCurrentRoad));
    }

    bool isApproachingNextStop() const {
        return hasValidNextStopAhead() &&
               nextStopPos - progressOnCurrentRoad <=
                   getStopLaneApproachDistance();
    }

    void refreshNextStop() {
        nextStop = nullptr;
        nextStopPos = -1.0;
        if (currentRoad == nullptr) {
            return;
        }

        const BusStop* candidate = nullptr;
        if (service_ != nullptr) {
            candidate = scheduledStop();
            if (candidate == nullptr ||
                scheduledStopRouteIndex() !=
                    static_cast<std::size_t>(
                        std::max(
                            0,
                            currentRouteIndex)) ||
                candidate->getRoad() != currentRoad ||
                candidate->getPositionOnRoad() <=
                    progressOnCurrentRoad) {
                return;
            }
        } else {
            candidate = currentRoad->getNextBusStopInfo(
                progressOnCurrentRoad,
                currentRoad->getDistance());
        }

        if (candidate != nullptr &&
            candidate->getRoad() == currentRoad) {
            nextStop = candidate;
            nextStopPos = candidate->getPositionOnRoad();
        }
        assert(
            nextStop == nullptr ||
            nextStop->getRoad() == currentRoad);
    }

protected:
    int getRequiredLaneIndex() const override {
        if (!isApproachingNextStop()) {
            return -1;
        }

        const int stopLane = nextStop->getLaneIndex();
        if (stopLane < 0 ||
            stopLane >= currentRoad->getLaneCount()) {
            return -1;
        }
        return stopLane;
    }

    double getLanePreparationSpeedLimit(
        double freeFlowSpeed) const override {
        if (!isApproachingNextStop() ||
            nextStop->getLaneIndex() ==
                currentLaneIndex) {
            return freeFlowSpeed;
        }

        const int remainingLaneChanges =
            std::abs(
                nextStop->getLaneIndex() -
                currentLaneIndex);
        const double distanceToStop =
            std::max(
                0.0,
                nextStopPos - progressOnCurrentRoad);
        const double preparationTime =
            STOP_LANE_PREPARATION_TIME +
            LANE_CHANGE_COOLDOWN * remainingLaneChanges;
        const double timeBasedLimit =
            distanceToStop /
            std::max(preparationTime, 1e-6);
        const double brakingBasedLimit = std::sqrt(
            2.0 *
            std::max(getDeceleration(), 1e-6) *
            std::max(
                0.0,
                distanceToStop -
                    STOP_LANE_SAFETY_BUFFER));
        const double minimumRollingSpeed =
            service_ != nullptr
                ? 0.0
                : STOP_LANE_MIN_ROLLING_SPEED;
        const double controlledLimit = std::max(
            minimumRollingSpeed,
            std::min(timeBasedLimit, brakingBasedLimit));

        return std::min(
            freeFlowSpeed,
            controlledLimit);
    }

public:
    Bus(int id,
        double speed,
        Intersection* start,
        Intersection* dest,
        double dwellTime = DEFAULT_DWELL)
        : Vehicle(id, speed, start, dest),
          dwellTime(std::max(0.0, dwellTime)),
          dwellTimer(0.0),
          nextStop(nullptr),
          nextStopPos(-1.0),
          service_(nullptr),
          originStation_(nullptr),
          destinationStation_(nullptr),
          scheduledDepartureTime_(0.0),
          scheduledStopIndex_(0),
          tripState_(BusTripState::WaitingAtOrigin),
          departureSlotHeld_(false) {
    }

    Bus(int id,
        std::string fleetCode,
        const BusService& service,
        double speed,
        double dwellTime = DEFAULT_DWELL)
        : Bus(
              id,
              std::move(fleetCode),
              service,
              makeDefaultTripPlan(service),
              speed,
              dwellTime) {
    }

    Bus(int id,
        std::string fleetCode,
        const BusService& service,
        BusTripPlan tripPlan,
        double speed,
        double dwellTime = DEFAULT_DWELL)
        : Vehicle(
              id,
              speed,
              service.getOriginStation().
                  getAccessIntersection(),
              service.getDestinationStation().
                  getAccessIntersection()),
          dwellTime(std::max(0.0, dwellTime)),
          dwellTimer(0.0),
          nextStop(nullptr),
          nextStopPos(-1.0),
          fleetCode_(std::move(fleetCode)),
          service_(&service),
          originStation_(&service.getOriginStation()),
          destinationStation_(
              &service.getDestinationStation()),
          scheduledDepartureTime_(
              tripPlan.scheduledDepartureTime),
          scheduledStopIndex_(0),
          tripState_(BusTripState::WaitingAtOrigin),
          departureSlotHeld_(false) {
        if (fleetCode_.empty() ||
            !isValidTripPlan(
                service,
                tripPlan)) {
            throw std::invalid_argument(
                "Transit Bus requires a fleet code "
                "and a valid trip plan.");
        }
        currentRoute =
            std::move(tripPlan.roadRoute);
        assignedStops_ =
            std::move(tripPlan.orderedStops);
        assignedStopRouteIndices_ =
            std::move(
                tripPlan.stopRouteIndices);
        currentRouteIndex = 0;
        routeAssigned = true;
    }

    ~Bus() override {
        releaseDepartureSlot();
    }

    // --- Snapshot support (Memento pattern) ---
    void captureSnapshot(VehicleSnapshot& snap,
                         const Graph& graph) const override;
    void restoreSnapshot(const VehicleSnapshot& snap,
                         Graph& graph) override;

    double calculateCurrentSpeed() const override {
        if (currentRoad == nullptr) {
            return 0.0;
        }
        if (currentRoad->getLane(
                currentLaneIndex).isBlocked()) {
            return 0.0;
        }

        const double cap = std::min(
            baseSpeed,
            currentRoad->getSpeedLimit()) *
            CRUISE_FACTOR;
        return cap /
               currentRoad->getCongestionLevel();
    }

    void update(
        double dt,
        Graph* graph = nullptr,
        PathFindingStrategy* strategy = nullptr,
        bool allowDynamicReroute = true) override {
        if (service_ != nullptr &&
            currentRoad != nullptr &&
            tripState_ ==
                BusTripState::WaitingAtOrigin) {
            tripState_ = BusTripState::Departing;
        }
        Vehicle::update(dt, graph, strategy, allowDynamicReroute);
        if (service_ != nullptr &&
            currentRoad != nullptr &&
            tripState_ == BusTripState::Departing) {
            tripState_ = BusTripState::EnRoute;
        }
    }

    void onRoadChanged() override {
        dwellTimer = 0.0;
        nextStop = nullptr;
        nextStopPos = -1.0;
        clearPause();

        if (service_ == nullptr) {
            refreshNextStop();
            return;
        }
        if (tripState_ == BusTripState::Arrived) {
            return;
        }
        if (currentRoad == nullptr) {
            if (routeAssigned &&
                currentRouteIndex >=
                    static_cast<int>(
                        currentRoute.size())) {
                markRemainingStopsMissed();
                tripState_ = BusTripState::Arrived;
            }
            return;
        }

        tripState_ = currentRouteIndex == 0
            ? BusTripState::Departing
            : BusTripState::EnRoute;
        markStopsMissedBeforeCurrentPosition();
        refreshNextStop();
    }

    bool shouldPauseAt(
        double currentPos,
        double projectedPos,
        double& pausePos) override {
        if (service_ != nullptr) {
            markStopsMissedBeforeCurrentPosition();
            if (nextStop != nullptr &&
                nextStopPos <= currentPos) {
                markScheduledStopMissed();
                refreshNextStop();
            } else if (nextStop == nullptr) {
                refreshNextStop();
            }
        } else {
            if (nextStop != nullptr &&
                nextStop->getRoad() != currentRoad) {
                refreshNextStop();
            }
            if (nextStop != nullptr &&
                nextStopPos <= currentPos) {
                refreshNextStop();
            }
        }

        double controlPausePos = -1.0;
        const bool controlPause =
            Vehicle::shouldPauseAt(
                currentPos,
                projectedPos,
                controlPausePos);
        const PauseReason controlReason =
            pauseReason;

        const int stopLane =
            nextStop != nullptr
                ? nextStop->getLaneIndex()
                : -1;
        const bool busStopPause =
            nextStop != nullptr &&
            stopLane >= 0 &&
            stopLane < currentRoad->getLaneCount() &&
            currentLaneIndex == stopLane &&
            !currentRoad->getLane(
                 stopLane).isBlocked() &&
            nextStopPos > currentPos &&
            projectedPos >= nextStopPos;

        if (busStopPause &&
            (!controlPause ||
             nextStopPos <= controlPausePos)) {
            pausePos = nextStopPos;
            pauseReason = PauseReason::BusStop;
            return true;
        }

        if (controlPause) {
            pausePos = controlPausePos;
            pauseReason = controlReason;
            return true;
        }

        pauseReason = PauseReason::None;
        return false;
    }

    void onPauseStarted() override {
        if (pauseReason == PauseReason::BusStop &&
            nextStop != nullptr &&
            nextStop->getRoad() == currentRoad &&
            currentLaneIndex ==
                nextStop->getLaneIndex()) {
            dwellTimer =
                nextStop->hasConfiguredDwellTime()
                    ? nextStop->getDwellTime()
                    : dwellTime;
            if (service_ != nullptr) {
                tripState_ = BusTripState::Dwelling;
            }
        } else {
            dwellTimer = 0.0;
        }
    }

    PauseUpdateResult updatePause(
        double availableTime) override {
        availableTime =
            std::max(0.0, availableTime);
        if (pauseReason != PauseReason::BusStop) {
            return {true, availableTime};
        }

        const double consumedTime =
            std::min(dwellTimer, availableTime);
        dwellTimer = std::max(
            0.0,
            dwellTimer - consumedTime);
        if (dwellTimer > 0.0) {
            return {false, 0.0};
        }

        if (service_ != nullptr) {
            const BusStop* stop = scheduledStop();
            if (stop != nullptr &&
                stop == nextStop) {
                servedStopIds_.push_back(
                    stop->getId());
                ++scheduledStopIndex_;
            }
            tripState_ = BusTripState::EnRoute;
        }
        refreshNextStop();

        return {
            true,
            std::max(
                0.0,
                availableTime - consumedTime)
        };
    }

    bool tryAcquireDepartureSlot() {
        if (service_ == nullptr ||
            departureSlotHeld_) {
            return true;
        }
        departureSlotHeld_ =
            originStation_->
                tryAcquireDepartureSlot();
        return departureSlotHeld_;
    }

    void releaseDepartureSlot() {
        if (!departureSlotHeld_ ||
            originStation_ == nullptr) {
            return;
        }
        originStation_->releaseDepartureSlot();
        departureSlotHeld_ = false;
    }

    void notifyActivatedFromStation() {
        releaseDepartureSlot();
        if (service_ != nullptr) {
            tripState_ = BusTripState::Departing;
        }
    }

    double getAcceleration() const override {
        return 8.0;
    }
    double getDeceleration() const override {
        return 12.0;
    }
    double getMaxLateralAcceleration() const override {
        return 1.5;
    }
    VehicleKind getVehicleKind() const override {
        return VehicleKind::Bus;
    }
    bool allowsDynamicRerouting() const override {
        return service_ == nullptr;
    }
    bool allowsUTurn() const override {
        return service_ == nullptr;
    }
    double getJunctionLanePreparationDistance()
        const override {
        return 60.0;
    }

    double getLength() const override { return 12.0; }
    double getWidth() const override { return 2.5; }
    double getHeight() const override { return 3.5; }
    double getWeight() const override { return 12.0; }
    double getMinGap() const override { return 4.0; }

    bool isDwelling() const {
        return isPaused() &&
               pauseReason == PauseReason::BusStop;
    }
    double getDwellTimer() const { return dwellTimer; }
    double getDwellTime() const { return dwellTime; }
    void setDwellTime(double t) {
        dwellTime = std::max(0.0, t);
    }
    double getNextStopPos() const {
        return nextStop != nullptr &&
               nextStop->getRoad() == currentRoad
            ? nextStopPos
            : -1.0;
    }
    const BusStop* getNextBusStop() const {
        return nextStop != nullptr &&
               nextStop->getRoad() == currentRoad
            ? nextStop
            : nullptr;
    }

    bool hasTransitService() const {
        return service_ != nullptr;
    }
    const std::string& getFleetCode() const {
        return fleetCode_;
    }
    const BusService* getService() const {
        return service_;
    }
    const BusStation* getOriginStation() const {
        return originStation_;
    }
    const BusStation* getDestinationStation() const {
        return destinationStation_;
    }
    const std::vector<const BusStop*>&
    getAssignedStops() const {
        return assignedStops_;
    }
    const std::vector<std::size_t>&
    getAssignedStopRouteIndices() const {
        return assignedStopRouteIndices_;
    }
    double getScheduledDepartureTime() const {
        return scheduledDepartureTime_;
    }
    const BusStop* getNextScheduledStop() const {
        return scheduledStop();
    }
    std::size_t getScheduledStopIndex() const {
        return scheduledStopIndex_;
    }
    const std::vector<int>& getServedStopIds() const {
        return servedStopIds_;
    }
    const std::vector<int>& getMissedStopIds() const {
        return missedStopIds_;
    }
    BusTripState getTripState() const {
        return tripState_;
    }
    bool hasDepartureSlot() const {
        return departureSlotHeld_;
    }
};

#endif
