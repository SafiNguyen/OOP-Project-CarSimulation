#ifndef ROAD_H
#define ROAD_H

#include <string>
#include <vector>
class Intersection;

class Road {
private:
    int id;
    Intersection* start;
    Intersection* end;
    double distance;
    double speedLimit;
    double congestionLevel;
    bool blocked; // accident status

    std::vector<double> busStopPositions;   

public:
    Road(int id, Intersection* start, Intersection* end, 
         double distance, 
         double speedLimit, 
         double congestionLevel = 1.0);

    Road(const Road&) = delete;
    Road& operator=(const Road&) = delete;
    
    //getters
    int getId() const;
    Intersection* getStart() const;
    Intersection* getEnd() const;
    double getDistance() const;
    double getSpeedLimit() const;
    double getCongestionLevel() const;
    bool isBlocked() const;

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