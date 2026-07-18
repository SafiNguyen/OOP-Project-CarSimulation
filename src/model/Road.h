#ifndef ROAD_H
#define ROAD_H

#include <string>
#include <vector>
#include "Lane.h"
class Intersection;
class Vehicle;

class Road {
private:
    int id;
    std::string name;
    Intersection* start;
    Intersection* end;
    double distance;  //met (m)
    double speedLimit;  // m/s
    double congestionLevel;
    bool blocked; // accident status

    int laneCount;
    std::vector<Lane> lanes;
    std::vector<double> busStopPositions;   

public:
    Road(int id, const std::string& name, Intersection* start, Intersection* end, 
         double distance, 
         double speedLimit, 
         double congestionLevel = 1.0,
         int laneCount = 1);

    Road(const Road&) = delete;
    Road& operator=(const Road&) = delete;
    
    //getters
    int getId() const;
    const std::string& getName() const;
    void setName(const std::string& newName);
    Intersection* getStart() const;
    Intersection* getEnd() const;
    double getDistance() const;
    double getSpeedLimit() const;
    double getCongestionLevel() const;
    double getDynamicCongestionLevel() const;
    bool isBlocked() const;
    virtual bool isBridge() const { return false; }
    virtual bool isTunnel() const { return false; }

    int getLaneCount () const;
    const std::vector<Lane>& getLanes() const;
    const Lane& getLane(int laneIndex) const;
    Lane& getLane(int laneIndex);

    int getFreestLaneIndex() const;

    // Car-Following Model support: returns the vehicle immediately ahead of
    // `self` within lane `laneIndex` on this road (i.e. the vehicle with the
    // smallest progress-on-road strictly greater than self's), or nullptr if
    // `self` is the lead vehicle in that lane / laneIndex is invalid.
    Vehicle* findLeader(int laneIndex, const Vehicle* self) const;

    double getTravelCost() const;
    double getTravelTime() const;        
    
    void updateCongestionLevel(double newLevel);
    void blockRoad();
    void unblockRoad();

    void addBusStop(double position);
    void clearBusStops();
    const std::vector<double>& getBusStopPositions() const;
    double getNextBusStop(double fromPosition, double toPosition) const;


    std::string toString() const;

    ~Road();
};

#endif