#include "Road.h"
#include "BusStop.h"
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
    lanes.reserve(this->laneCount);
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

bool Road::isBlocked() const {
    if (lanes.empty()) return false;
    for (const Lane& lane : lanes) {
        if (!lane.isBlocked()) return false;
    }
    return true; // All lanes blocked
}

bool Road::hasBlockedLane() const {
    if (lanes.empty()) return false;
    for (const Lane& lane : lanes) {
        if (lane.isBlocked()) return true;
    }
    return false;
}
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
        if (lane.isBlocked()) {
            continue; // khong xet lane dang bi block khi tim lane trong nhat
        }
        double capacity = lane.getCapacity();
        double occupancy = (capacity > 0.0)
            ? static_cast<double>(lane.getVehicleCount()) / capacity
            : std::numeric_limits<double>::infinity();  
 
        if (occupancy < lowestOccupancy) {
            lowestOccupancy = occupancy;
            freestIndex = lane.getIndex();
        }
    }
 
    return (freestIndex != -1) ? freestIndex : lanes.front().getIndex();
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
        // Only consider vehicles strictly ahead of self on this road, or use ID for tie-break if overlapping.
        if ((candidateProgress > selfProgress || (candidateProgress == selfProgress && candidate->getId() < self->getId())) && candidateProgress < bestProgress) {
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
        // Only consider vehicles strictly behind self on this road, or use ID for tie-break if overlapping.
        if ((candidateProgress < selfProgress || (candidateProgress == selfProgress && candidate->getId() > self->getId())) && candidateProgress > bestProgress) {
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
    if (isBlocked()) {
        return std::numeric_limits<double>::infinity(); 
    }
    double time = distance / (speedLimit / getDynamicCongestionLevel()); 
    return time;
}

double Road::getTravelCost() const {
    return getTravelTime();
}

double Road::getWeightedCost(double speedPreference) const {
    if (isBlocked()) {
        return std::numeric_limits<double>::infinity();
    }

    // Clamp defensively: callers passing a slider value or similar should
    // never be able to push this outside the intended [0, 1] blend range.
    const double alpha = std::clamp(speedPreference, 0.0, 1.0);

    // alpha == 1.0 reduces exactly to getTravelTime()'s formula, so
    // existing "pure time-optimal" callers keep identical behaviour.
    const double travelTime = distance / (speedLimit / getDynamicCongestionLevel());
    const double distanceCost = distance;

    return alpha * travelTime + (1.0 - alpha) * distanceCost;
}

void Road::updateCongestionLevel(double newLevel) {
    if (newLevel < 1.0) {
        this->congestionLevel = 1.0;
    } else {
        this->congestionLevel = newLevel;
    }
}

void Road::blockRoad() {
    for (Lane& lane : lanes) lane.block();
}

void Road::unblockRoad() {
    for (Lane& lane : lanes) lane.unblock();
}

void Road::blockLane(int laneIndex) {
    if (laneIndex >= 0 && laneIndex < laneCount) {
        lanes[laneIndex].block();
    }
}

void Road::unblockLane(int laneIndex) {
    if (laneIndex >= 0 && laneIndex < laneCount) {
        lanes[laneIndex].unblock();
    }
}

void Road::addLanes(int count) {
    if (count <= 0) return;
    int oldLaneCount = laneCount;
    laneCount += count;
    lanes.reserve(laneCount);
    for (int i = 0; i < count; ++i) {
        lanes.emplace_back(oldLaneCount + i);
    }
}

bool Road::addBusStop(std::unique_ptr<BusStop> busStop) {
    if (busStop == nullptr ||
        busStop->getRoad() != this ||
        busStop->getPositionOnRoad() <= 0.0 ||
        busStop->getPositionOnRoad() >= distance ||
        busStop->getLaneIndex() < 0 ||
        busStop->getLaneIndex() >= laneCount ||
        findBusStopById(busStop->getId()) != nullptr) {
        return false;
    }

    for (const auto& existing : busStops) {
        if (std::fabs(existing->getPositionOnRoad() - busStop->getPositionOnRoad())
            < BusStop::MIN_SPACING) {
            return false;
        }
    }

    busStops.push_back(std::move(busStop));
    std::sort(busStops.begin(), busStops.end(),
              [](const std::unique_ptr<BusStop>& lhs,
                 const std::unique_ptr<BusStop>& rhs) {
                  return lhs->getPositionOnRoad() < rhs->getPositionOnRoad();
              });

    busStopPositions.clear();
    busStopPositions.reserve(busStops.size());
    for (const auto& stop : busStops) {
        busStopPositions.push_back(stop->getPositionOnRoad());
    }
    return true;
}

void Road::addBusStop(double position) {
    // Compatibility API for existing code/tests. Negative IDs are local,
    // generated identities and are never produced by the JSON loader.
    int generatedId = -1;
    while (findBusStopById(generatedId) != nullptr) {
        --generatedId;
    }
    addBusStop(std::make_unique<BusStop>(
        generatedId,
        "Bus Stop",
        this,
        position,
        0,
        BusStop::DEFAULT_DWELL_TIME,
        false));
}

void Road::clearBusStops() {
    busStops.clear();
    busStopPositions.clear();
}

const std::vector<std::unique_ptr<BusStop>>& Road::getBusStops() const {
    return busStops;
}

const BusStop* Road::findBusStopById(int stopId) const {
    for (const auto& stop : busStops) {
        if (stop->getId() == stopId) {
            return stop.get();
        }
    }
    return nullptr;
}

const BusStop* Road::getNextBusStopInfo(double fromPosition, double toPosition) const {
    for (const auto& stop : busStops) {
        const double stopPos = stop->getPositionOnRoad();
        if (stopPos > toPosition + 0.001) {
            break;
        }
        if (stopPos > fromPosition + 0.001) {
            return stop.get();
        }
    }
    return nullptr;
}
 
const std::vector<double>& Road::getBusStopPositions() const {
    return busStopPositions;
}
 
double Road::getNextBusStop(double fromPosition, double toPosition) const {
    const BusStop* stop = getNextBusStopInfo(fromPosition, toPosition);
    return stop != nullptr ? stop->getPositionOnRoad() : -1.0;
}


std::string Road::toString() const {
    std::string blockStatus = isBlocked() ? "true" : "false";
    
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
