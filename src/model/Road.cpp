#include "Road.h"
#include "Intersection.h"
#include "Vehicle.h" 
#include <limits>
#include <stdexcept>
#include <cmath>    
#include <algorithm> 

Road::Road(int id, const std::string& name, Intersection* start, Intersection* end, 
           double distance, double speedLimit, double congestionLevel, int laneCount) 
           : id(id), name(name), start(start), end(end), distance(distance), speedLimit(speedLimit), laneCount((laneCount >= 1) ? laneCount : 1) {
    if (congestionLevel < 1.0) {
        this->congestionLevel = 1.0; 
    } else {
        this->congestionLevel = congestionLevel;
    }    
    this->blocked = false; 
    lanes.reserve(this -> laneCount);
    for (int i = 0; i < this->laneCount; ++i) {
        lanes.emplace_back(i);
    }
}


int Road::getId() const { return id; }
const std::string& Road::getName() const { return name; }
void Road::setName(const std::string& newName) { name = newName; }
Intersection* Road::getStart() const { return start; }
Intersection* Road::getEnd() const { return end; }
double Road::getDistance() const { return distance; }
double Road::getSpeedLimit() const { return speedLimit; }
double Road::getCongestionLevel() const { return congestionLevel; }

double Road::getDynamicCongestionLevel() const {
    double totalVehicles = 0.0, totalCapacity = 0.0;
    for (const Lane& lane : lanes) {
        totalVehicles += lane.getVehicleCount();
        totalCapacity += lane.getCapacity();
    }
    if (totalCapacity <= 0.0) return congestionLevel;
    double occupancy = totalVehicles / totalCapacity;
    return congestionLevel * (1.0 + occupancy);
}

bool Road::isBlocked() const { return blocked; }
int Road::getLaneCount() const { return laneCount; }
 
const std::vector<Lane>& Road::getLanes() const { return lanes; }
 
const Lane& Road::getLane(int laneIndex) const {
    if (laneIndex < 0 || laneIndex >= static_cast<int>(lanes.size())) {
        throw std::out_of_range("Road::getLane: laneIndex out of range");
    }
    return lanes[laneIndex];
}

Lane& Road::getLane(int laneIndex) {
    if (laneIndex < 0 || laneIndex >= static_cast<int>(lanes.size())) {
        throw std::out_of_range("Road::getLane: laneIndex out of range");
    }
    return lanes[laneIndex];
}
 
int Road::getFreestLaneIndex() const {
    int freestIndex = lanes.front().getIndex(); 
    double lowestOccupancy = std::numeric_limits<double>::infinity();
 
    for (const Lane& lane : lanes) {
        double capacity = lane.getCapacity();
        double occupancy = (capacity > 0.0)
            ? static_cast<double>(lane.getVehicleCount()) / capacity
            : std::numeric_limits<double>::infinity();
 
        if (occupancy < lowestOccupancy) {
            lowestOccupancy = occupancy;
            freestIndex = lane.getIndex();
        }
    }
 
    return freestIndex;
}

Vehicle* Road::findLeader(int laneIndex, const Vehicle* self) const {
    if (self == nullptr || laneIndex < 0 || laneIndex >= static_cast<int>(lanes.size())) {
        return nullptr;
    }

    const Lane& lane = lanes[laneIndex];
    const double selfProgress = self->getProgressOnRoad();

    Vehicle* leader = nullptr;
    double bestProgress = std::numeric_limits<double>::infinity();

    for (Vehicle* candidate : lane.getVehicles()) {
        if (candidate == self) {
            continue;
        }
        const double candidateProgress = candidate->getProgressOnRoad();
        // Only consider vehicles strictly ahead of self on this road.
        if (candidateProgress > selfProgress && candidateProgress < bestProgress) {
            bestProgress = candidateProgress;
            leader = candidate;
        }
    }

    return leader;
}

Vehicle* Road::findFollower(int laneIndex, const Vehicle* self) const {
    if (self == nullptr || laneIndex < 0 || laneIndex >= static_cast<int>(lanes.size())) {
        return nullptr;
    }
 
    const Lane& lane = lanes[laneIndex];
    const double selfProgress = self->getProgressOnRoad();
 
    Vehicle* follower = nullptr;
    double bestProgress = -std::numeric_limits<double>::infinity();
 
    for (Vehicle* candidate : lane.getVehicles()) {
        if (candidate == self) {
            continue;
        }
        const double candidateProgress = candidate->getProgressOnRoad();
        // Only consider vehicles strictly behind self on this road.
        if (candidateProgress < selfProgress && candidateProgress > bestProgress) {
            bestProgress = candidateProgress;
            follower = candidate;
        }
    }
 
    return follower;
}

Vehicle* Road::getFirstVehicleInLane(int laneIndex) const {
    if (laneIndex < 0 || laneIndex >= static_cast<int>(lanes.size())) {
        return nullptr;
    }

    const Lane& lane = lanes[laneIndex];
    Vehicle* first = nullptr;
    double bestProgress = std::numeric_limits<double>::infinity();

    for (Vehicle* candidate : lane.getVehicles()) {
        const double p = candidate->getProgressOnRoad();
        if (p < bestProgress) {
            bestProgress = p;
            first = candidate;
        }
    }

    return first;
}

double Road::getTravelTime() const {
    if (blocked) {
        return std::numeric_limits<double>::infinity(); 
    }
    return distance / (speedLimit / getDynamicCongestionLevel()); 
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
           ", BusStops: " + stops +
           ", Lanes: " + std::to_string(laneCount) + "]";
}

std::vector<Vehicle*> Road::getVehiclesInProgressRange(double fromProgress,
                                                          double toProgress) const {
    std::vector<Vehicle*> result;
    if (toProgress <= fromProgress) {
        return result;
    }

    for (const Lane& lane : lanes) {
        for (Vehicle* v : lane.getVehicles()) {
            double p = v->getProgressOnRoad();
            if (p > fromProgress && p <= toProgress) {
                result.push_back(v);
            }
        }
    }
    return result;
}

Road::~Road() {
}