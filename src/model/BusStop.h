#ifndef BUS_STOP_H
#define BUS_STOP_H

#include <string>

class Road;

class BusStop {
public:
    static constexpr double DEFAULT_DWELL_TIME = 15.0;
    static constexpr double MIN_SPACING = 0.01;

    BusStop(int id,
            std::string name,
            Road* road,
            double positionOnRoad,
            int laneIndex,
            double dwellTime = DEFAULT_DWELL_TIME,
            bool hasConfiguredDwellTime = true);

    BusStop(const BusStop&) = delete;
    BusStop& operator=(const BusStop&) = delete;
    BusStop(BusStop&&) = delete;
    BusStop& operator=(BusStop&&) = delete;

    int getId() const { return id_; }
    const std::string& getName() const { return name_; }
    Road* getRoad() const { return road_; }
    int getRoadId() const;
    double getPositionOnRoad() const { return positionOnRoad_; }
    double getPositionRatio() const;
    int getLaneIndex() const { return laneIndex_; }
    double getDwellTime() const { return dwellTime_; }
    bool hasConfiguredDwellTime() const { return hasConfiguredDwellTime_; }

private:
    int id_;
    std::string name_;
    Road* road_;  // Non-owning. The Road owns this BusStop.
    double positionOnRoad_;
    int laneIndex_;
    double dwellTime_;
    bool hasConfiguredDwellTime_;
};

#endif
