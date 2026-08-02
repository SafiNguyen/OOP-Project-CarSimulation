#include "Lane.h"
#include <algorithm>


Lane::Lane(int laneIndex, double capacity)
    : laneIndex(laneIndex),
      capacity(capacity > 0.0 ? capacity : DEFAULT_LANE_CAPACITY),
      blocked(false) {
}
 
int Lane::getIndex() const {
    return laneIndex;
}
 
double Lane::getCapacity() const {
    return capacity;
}
 
void Lane::addVehicle(Vehicle* v) {
    if (v == nullptr) return;
    if (std::find(vehicles.begin(), vehicles.end(), v) != vehicles.end()) {
        return; 
    }
    vehicles.push_back(v);
}
 
void Lane::removeVehicle(Vehicle* v) {
    if (v == nullptr) return;
    vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), v), vehicles.end());
}
 
int Lane::getVehicleCount() const {
    return static_cast<int>(vehicles.size());
}
 
const std::vector<Vehicle*>& Lane::getVehicles() const {
    return vehicles;
}

bool Lane::isBlocked() const {
    return blocked;
}

void Lane::block() {
    blocked = true;
}

void Lane::unblock() {
    blocked = false;
}
 
Lane::~Lane() {
}

void Lane::clearVehicles() {
    vehicles.clear();
}
