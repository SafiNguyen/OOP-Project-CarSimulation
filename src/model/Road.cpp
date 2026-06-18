#include "Road.h"
#include "Intersection.h" 
#include <limits>
#include <stdexcept>
#include <cmath>    
#include <algorithm> 

Road::Road(int id, Intersection* start, Intersection* end, 
           double distance, double speedLimit, double congestionLevel) {
    this->id = id;
    this->start = start;
    this->end = end;
    this->distance = distance;
    this->speedLimit = speedLimit;
    if (congestionLevel < 1.0) {
        this->congestionLevel = 1.0; 
    } else {
        this->congestionLevel = congestionLevel;
    }    
    this->blocked = false; 
}


int Road::getId() const { return id; }
Intersection* Road::getStart() const { return start; }
Intersection* Road::getEnd() const { return end; }
double Road::getDistance() const { return distance; }
double Road::getSpeedLimit() const { return speedLimit; }
double Road::getCongestionLevel() const { return congestionLevel; }
bool Road::isBlocked() const { return blocked; }


double Road::getTravelTime() const {
    if (blocked) {
        return std::numeric_limits<double>::infinity(); 
    }
    return distance / (speedLimit / congestionLevel); 
}

double Road::getTravelCost() const {
    return getTravelTime();
}

void Road::updateCongestionLevel(double newLevel) {
    if (newLevel < 1.0) {
        this->congestionLevel = 1.0;
    } else {
        this->congestionLevel = newLevel;
    }
}

void Road::blockRoad() {
    this->blocked = true;
}

void Road::unblockRoad() {
    this->blocked = false;
}

void Road::addBusStop(double position) {
    if (position <= 0.0 || position >= distance) {
        return;
    }
 
    for (double pos : busStopPositions) {
        if (std::fabs(pos - position) < 0.01) {
            return;
        }
    }
 
    busStopPositions.push_back(position);
    std::sort(busStopPositions.begin(), busStopPositions.end());
}
 
void Road::clearBusStops() {
    busStopPositions.clear();
}
 
const std::vector<double>& Road::getBusStopPositions() const {
    return busStopPositions;
}
 
double Road::getNextBusStop(double fromPosition, double toPosition) const {
    if (busStopPositions.empty()) {
        return -1.0;
    }
 
    for (double stopPos : busStopPositions) {
        if (stopPos > toPosition + 0.001) {
            break; 
        }

        if (stopPos > fromPosition + 0.001) {
            return stopPos; 
        }
    }
 
    return -1.0; 
}


std::string Road::toString() const {
    std::string blockStatus = blocked ? "true" : "false";
    
    std::string stops = "[";
    for (size_t i = 0; i < busStopPositions.size(); ++i) {
        stops += std::to_string(busStopPositions[i]);
        if (i + 1 < busStopPositions.size()) stops += ", ";
    }
    stops += "]";

    return "Road[ID: " + std::to_string(id) + 
           ", Start ID: " + std::to_string(start->getId()) + 
           ", End ID: " + std::to_string(end->getId()) + 
           ", Distance: " + std::to_string(distance) + 
           ", SpeedLimit: " + std::to_string(speedLimit) + 
           ", Congestion: " + std::to_string(congestionLevel) + 
           ", Blocked: " + blockStatus +
           ", BusStops: " + stops + "]";
}


Road::~Road() {
}