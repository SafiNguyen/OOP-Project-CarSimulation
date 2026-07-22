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
    bool hasBlockedLane() const;
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
    // Doi xung voi findLeader — tra ve xe gan nhat PHIA SAU self trong lane
    // nay (progress nho hon self, lon nhat trong so do), hoac nullptr neu
    // self la xe cuoi cung trong lane / laneIndex khong hop le. Can thiet
    // cho lane-changing: truoc khi tat dau vao 1 lane, phai biet xe phia
    // sau o lane do co bi buoc phanh gap khong.
    Vehicle* findFollower(int laneIndex, const Vehicle* self) const;
    Vehicle* getFirstVehicleInLane(int laneIndex) const;

    double getTravelCost() const;
    double getTravelTime() const;        

    // --- Speed-aware weighted routing cost ---
    // General-purpose edge cost for pathfinding strategies (Dijkstra / A*)
    // that lets the caller blend between "shortest distance" and
    // "fastest travel time" (which already factors in this road's
    // speedLimit and current congestion):
    //
    //   cost = speedPreference * travelTime + (1 - speedPreference) * distance
    //
    //   speedPreference = 1.0  -> pure travel-time optimization (equivalent
    //                             to getTravelCost() / getTravelTime());
    //                             a road with a higher speedLimit is
    //                             cheaper even if it is physically longer.
    //   speedPreference = 0.0  -> pure distance optimization; speedLimit is
    //                             ignored entirely.
    //   0 < speedPreference < 1 -> a blend of both.
    //
    // Returns +infinity if the road is blocked, same as getTravelTime().
    double getWeightedCost(double speedPreference = 1.0) const;
    
    void updateCongestionLevel(double newLevel);
    void blockRoad();
    void unblockRoad();
    void blockLane(int laneIndex);
    void unblockLane(int laneIndex);
    void addLanes(int count);

    void addBusStop(double position);
    void clearBusStops();
    const std::vector<double>& getBusStopPositions() const;
    double getNextBusStop(double fromPosition, double toPosition) const;

    // ambulance yielding support
    std::vector<Vehicle*> getVehiclesInProgressRange(double fromProgress,
                                                       double toProgress) const;
    
    
    std::string toString() const;

    virtual ~Road();
};

#endif
