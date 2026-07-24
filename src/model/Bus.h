#ifndef BUS_H
#define BUS_H

#include "Vehicle.h"
#include "Road.h"
#include "BusStop.h"
#include <algorithm>
#include <cassert>


class Bus : public Vehicle {
public:

    static constexpr double CRUISE_FACTOR    = 0.85;
    static constexpr double DEFAULT_DWELL    = 15.0;

private:
    double dwellTime;   ///< seconds to wait at each stop

    double dwellTimer;      ///< counts down during a dwell; 0 when not dwelling
    const BusStop* nextStop;
    double nextStopPos;     ///< position of the upcoming stop on currentRoad;
                            ///< negative → no stop ahead on this road
    void refreshNextStop() {
        if (currentRoad == nullptr) {
            nextStop = nullptr;
            nextStopPos = -1.0;
            return;
        }
        nextStop = currentRoad->getNextBusStopInfo(
            progressOnCurrentRoad,
            currentRoad->getDistance());
        nextStopPos = nextStop != nullptr ? nextStop->getPositionOnRoad() : -1.0;
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
        dwellTimer  = 0.0;
        pauseReason = PauseReason::None;
        refreshNextStop();
    }

    bool shouldPauseAt(double currentPos,
                       double projectedPos,
                       double& pausePos) override {
        double trafficPausePos = -1.0;
        const bool trafficPause = Vehicle::shouldPauseAt(
            currentPos, projectedPos, trafficPausePos);

        const bool busStopPause =
            nextStop != nullptr &&
            nextStopPos > currentPos &&
            projectedPos >= nextStopPos;

        if (busStopPause && (!trafficPause || nextStopPos <= trafficPausePos)) {
            pausePos = nextStopPos;
            pauseReason = PauseReason::BusStop;
            return true;
        }

        if (trafficPause) {
            pausePos = trafficPausePos;
            pauseReason = PauseReason::TrafficLight;
            return true;
        }

        pauseReason = PauseReason::None;
        return false;
    }


    void onPauseStarted() override {
        if (pauseReason == PauseReason::BusStop && nextStop != nullptr) {
            dwellTimer = nextStop->hasConfiguredDwellTime()
                ? nextStop->getDwellTime()
                : dwellTime;
        } else {
            dwellTimer = 0.0;
        }
    }


    bool updatePause(double dt) override {
        if (pauseReason != PauseReason::BusStop) {
            return true;
        }

        dwellTimer -= dt;
        if (dwellTimer > 0.0) return false;

        dwellTimer = 0.0;

        refreshNextStop();

        return true; // resume
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
    double getNextStopPos() const { return nextStopPos; }
    const BusStop* getNextBusStop() const { return nextStop; }
};

#endif
