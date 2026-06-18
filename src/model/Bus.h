#ifndef BUS_H
#define BUS_H

#include "Vehicle.h"
#include "Road.h"
#include <algorithm>
#include <cassert>


class Bus : public Vehicle {
public:

    static constexpr double CRUISE_FACTOR    = 0.85;
    static constexpr double DEFAULT_DWELL    = 15.0;

private:
    double dwellTime;   ///< seconds to wait at each stop

    double dwellTimer;      ///< counts down during a dwell; 0 when not dwelling
    double nextStopPos;     ///< position of the upcoming stop on currentRoad;
                            ///< negative → no stop ahead on this road
    void refreshNextStop() {
        if (currentRoad == nullptr) {
            nextStopPos = -1.0;
            return;
        }
        nextStopPos = currentRoad->getNextBusStop(
            progressOnCurrentRoad,
            currentRoad->getDistance());
    }

public:
    //  Constructor 
    Bus(int id, double speed,
        Intersection* start, Intersection* dest,
        double dwellTime = DEFAULT_DWELL)
        : Vehicle(id, speed, start, dest),
          dwellTime(std::max(0.0, dwellTime)),
          dwellTimer(0.0),
          nextStopPos(-1.0)
    {}

    double calculateCurrentSpeed() const override {
        if (currentRoad == nullptr)     return 0.0;
        if (currentRoad->isBlocked())   return 0.0;

        double cap = std::min(baseSpeed, currentRoad->getSpeedLimit()) * CRUISE_FACTOR;
        return cap / currentRoad->getCongestionLevel();
    }

    void onRoadChanged() override {
        dwellTimer  = 0.0;
        refreshNextStop();
    }

    bool shouldPauseAt(double currentPos,
                       double projectedPos,
                       double& pausePos) override {
        if (nextStopPos < 0.0)          return false; // no stop ahead
        if (nextStopPos <= currentPos)  return false; // stop already passed
        if (projectedPos < nextStopPos) return false; // not there yet

        pausePos = nextStopPos;
        return true;
    }


    void onPauseStarted() override {
        dwellTimer = dwellTime;
    }


    bool updatePause(double dt) override {
        dwellTimer -= dt;
        if (dwellTimer > 0.0) return false;

        dwellTimer = 0.0;

        refreshNextStop();

        return true; // resume
    }

    bool   isDwelling()    const { return isPaused(); }
    double getDwellTimer() const { return dwellTimer; }
    double getDwellTime()  const { return dwellTime; }
    void   setDwellTime(double t) { dwellTime = std::max(0.0, t); }
    double getNextStopPos() const { return nextStopPos; }
};

#endif 