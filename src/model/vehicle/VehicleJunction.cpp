#include "Vehicle.h"
#include "Road.h"
#include "Lane.h"
#include "Intersection.h"
#include "RoadGeometry.h"
#include "VehicleMath.h"
using namespace VehicleMath;
#include "JunctionConnector.h"
#include "PointOfInterest.h"
#include "SpawnPoint.h"

bool Vehicle::tryReserveSpawnSlot() {
    if (spawnPOI == nullptr) {
        return true;
    }
    if (reservedSpawnPoint_ != nullptr) {
        return true;
    }
    const auto* spawnPoint =
        dynamic_cast<const SpawnPoint*>(spawnPOI);
    if (spawnPoint == nullptr) {
        return false;
    }
    if (!spawnPoint->tryReserveSpawnSlot()) {
        spawnLifecycleState_ =
            SpawnLifecycleState::
                WaitingForSourceCapacity;
        return false;
    }
    reservedSpawnPoint_ = spawnPoint;
    return true;
}
void Vehicle::releaseSpawnSlot() {
    if (reservedSpawnPoint_ == nullptr) {
        return;
    }
    reservedSpawnPoint_->releaseSpawnSlot();
    reservedSpawnPoint_ = nullptr;
}
