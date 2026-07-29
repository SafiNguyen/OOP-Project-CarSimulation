#include "BusStop.h"

#include <algorithm>
#include <utility>

#include "Road.h"

BusStop::BusStop(int id,
                 std::string name,
                 Road* road,
                 double positionOnRoad,
                 int laneIndex,
                 double dwellTime,
                 bool hasConfiguredDwellTime)
    : id_(id),
      name_(std::move(name)),
      road_(road),
      positionOnRoad_(positionOnRoad),
      laneIndex_(laneIndex),
      dwellTime_(std::max(0.0, dwellTime)),
      hasConfiguredDwellTime_(hasConfiguredDwellTime) {
}

int BusStop::getRoadId() const {
    return road_ != nullptr ? road_->getId() : 0;
}

double BusStop::getPositionRatio() const {
    if (road_ == nullptr || road_->getDistance() <= 0.0) {
        return 0.0;
    }
    return positionOnRoad_ / road_->getDistance();
}
