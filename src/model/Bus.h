#ifndef BUS_H
#define BUS_H

#include "Vehicle.h"
#include "Road.h"
#include "BusStop.h"
#include <algorithm>
#include <cassert>
#include <cmath>


class Bus : public Vehicle {
public:

    static constexpr double CRUISE_FACTOR    = 0.85;
    static constexpr double DEFAULT_DWELL    = 15.0;
    static constexpr double STOP_LANE_APPROACH_DISTANCE = 60.0;
    static constexpr double STOP_LANE_SAFETY_BUFFER = 15.0;
    static constexpr double STOP_LANE_PREPARATION_TIME = 1.0;
    static constexpr double STOP_LANE_MIN_ROLLING_SPEED = 1.5;

private:
    double dwellTime;   ///< seconds to wait at each stop

    double dwellTimer;      ///< counts down during a dwell; 0 when not dwelling
    const BusStop* nextStop;
    double nextStopPos;     ///< position of the upcoming stop on currentRoad;
                            ///< negative → no stop ahead on this road

    bool hasValidNextStopAhead() const {
        return currentRoad != nullptr &&
               nextStop != nullptr &&
               nextStop->getRoad() == currentRoad &&
               nextStopPos > progressOnCurrentRoad;
    }

    double getStopLaneApproachDistance() const {
        if (!hasValidNextStopAhead()) {
            return 0.0;
        }

        const int stopLane = nextStop->getLaneIndex();
        if (stopLane < 0 || stopLane >= currentRoad->getLaneCount()) {
            return 0.0;
        }

        const int remainingLaneChanges =
            std::abs(stopLane - currentLaneIndex);
        const double speed = std::max(0.0, currentSpeed);
        const double brakingDistance =
            speed * speed / (2.0 * std::max(getDeceleration(), 1e-6));
        const double laneChangeDistance =
            speed * LANE_CHANGE_COOLDOWN * remainingLaneChanges;
        const double speedBuffer = speed * STOP_LANE_PREPARATION_TIME;
        const double desiredDistance = std::max(
            STOP_LANE_APPROACH_DISTANCE,
            brakingDistance + laneChangeDistance +
                speedBuffer + STOP_LANE_SAFETY_BUFFER);

        return std::min(
            desiredDistance,
            std::max(0.0, currentRoad->getDistance() -
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

        const BusStop* candidate = currentRoad->getNextBusStopInfo(
            progressOnCurrentRoad,
            currentRoad->getDistance());
        if (candidate != nullptr && candidate->getRoad() == currentRoad) {
            nextStop = candidate;
            nextStopPos = candidate->getPositionOnRoad();
        }
        assert(nextStop == nullptr || nextStop->getRoad() == currentRoad);
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

    double getLanePreparationSpeedLimit(double freeFlowSpeed) const override {
        if (!isApproachingNextStop() ||
            nextStop->getLaneIndex() == currentLaneIndex) {
            return freeFlowSpeed;
        }

        const int remainingLaneChanges =
            std::abs(nextStop->getLaneIndex() - currentLaneIndex);
        const double distanceToStop =
            std::max(0.0, nextStopPos - progressOnCurrentRoad);
        const double preparationTime =
            STOP_LANE_PREPARATION_TIME +
            LANE_CHANGE_COOLDOWN * remainingLaneChanges;
        const double timeBasedLimit =
            distanceToStop / std::max(preparationTime, 1e-6);
        const double brakingBasedLimit = std::sqrt(
            2.0 * std::max(getDeceleration(), 1e-6) *
            std::max(0.0, distanceToStop - STOP_LANE_SAFETY_BUFFER));
        const double controlledLimit = std::max(
            STOP_LANE_MIN_ROLLING_SPEED,
            std::min(timeBasedLimit, brakingBasedLimit));

        return std::min(freeFlowSpeed, controlledLimit);
    }

public:
    //  Constructor 
    Bus(int id, double speed,
        Intersection* start, Intersection* dest,
        double dwellTime = DEFAULT_DWELL)
        : Vehicle(id, speed, start, dest),
          dwellTime(std::max(0.0, dwellTime)),
          dwellTimer(0.0),
          nextStop(nullptr),
          nextStopPos(-1.0)
    {}

    double calculateCurrentSpeed() const override {
        if (currentRoad == nullptr)     return 0.0;
        if (currentRoad->getLane(currentLaneIndex).isBlocked())   return 0.0;

        double cap = std::min(baseSpeed, currentRoad->getSpeedLimit()) * CRUISE_FACTOR;
        return cap / currentRoad->getCongestionLevel();
    }

    void onRoadChanged() override {
        dwellTimer = 0.0;
        nextStop = nullptr;
        nextStopPos = -1.0;
        clearPause();
        refreshNextStop();
    }

    bool shouldPauseAt(double currentPos,
                       double projectedPos,
                       double& pausePos) override {
        if (nextStop != nullptr && nextStop->getRoad() != currentRoad) {
            refreshNextStop();
        }
        if (nextStop != nullptr && nextStopPos <= currentPos) {
            refreshNextStop();
        }

        double controlPausePos = -1.0;
        const bool controlPause = Vehicle::shouldPauseAt(
            currentPos, projectedPos, controlPausePos);
        const PauseReason controlReason = pauseReason;

        const int stopLane =
            nextStop != nullptr ? nextStop->getLaneIndex() : -1;
        const bool busStopPause =
            nextStop != nullptr &&
            stopLane >= 0 &&
            stopLane < currentRoad->getLaneCount() &&
            currentLaneIndex == stopLane &&
            !currentRoad->getLane(stopLane).isBlocked() &&
            nextStopPos > currentPos &&
            projectedPos >= nextStopPos;

        if (busStopPause && (!controlPause || nextStopPos <= controlPausePos)) {
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
            currentLaneIndex == nextStop->getLaneIndex()) {
            dwellTimer = nextStop->hasConfiguredDwellTime()
                ? nextStop->getDwellTime()
                : dwellTime;
        } else {
            dwellTimer = 0.0;
        }
    }


    PauseUpdateResult updatePause(double availableTime) override {
        availableTime = std::max(0.0, availableTime);
        if (pauseReason != PauseReason::BusStop) {
            return {true, availableTime};
        }

        const double consumedTime = std::min(dwellTimer, availableTime);
        dwellTimer = std::max(0.0, dwellTimer - consumedTime);
        if (dwellTimer > 0.0) {
            return {false, 0.0};
        }

        refreshNextStop();

        return {true, std::max(0.0, availableTime - consumedTime)};
    }

    // Buses are heavy and carry passengers: gentler acceleration/braking
    // than a car so standing passengers aren't thrown around.
    double getAcceleration() const override { return 8.0; }
    double getDeceleration() const override { return 12.0; }

    // Buses are long vehicles and keep a slightly larger safety gap so
    // passengers aren't jolted by sudden stops behind other traffic.
    double getLength() const override { return 12.0; }
    double getHeight() const override { return 3.5; }  // metres — tall vehicle
    double getWeight() const override { return 12.0; } // tonnes — heavy vehicle
    double getMinGap() const override { return 3.0; }

    bool   isDwelling()    const {
        return isPaused() && pauseReason == PauseReason::BusStop;
    }
    double getDwellTimer() const { return dwellTimer; }
    double getDwellTime()  const { return dwellTime; }
    void   setDwellTime(double t) { dwellTime = std::max(0.0, t); }
    double getNextStopPos() const {
        return nextStop != nullptr && nextStop->getRoad() == currentRoad
            ? nextStopPos
            : -1.0;
    }
    const BusStop* getNextBusStop() const {
        return nextStop != nullptr && nextStop->getRoad() == currentRoad
            ? nextStop
            : nullptr;
    }
};

#endif
