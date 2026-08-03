#include "SnapshotManager.h"
#include "TrafficSimulator.h"

namespace {

std::size_t estimateVehicleBytes(const VehicleSnapshot& vehicle) {
    return sizeof(VehicleSnapshot) +
           vehicle.currentRoute.capacity() * sizeof(std::optional<int>) +
           vehicle.travelHistory.capacity() * sizeof(std::optional<int>) +
           vehicle.fleetCode.capacity() +
           vehicle.assignedStopIds.capacity() * sizeof(int) +
           vehicle.assignedStopRouteIndices.capacity() * sizeof(std::size_t) +
           vehicle.servedStopIds.capacity() * sizeof(int) +
           vehicle.missedStopIds.capacity() * sizeof(int);
}

std::size_t estimateSnapshotBytes(const SimulationSnapshot& snapshot) {
    std::size_t bytes = sizeof(SimulationSnapshot) +
        snapshot.vehicles.capacity() * sizeof(VehicleSnapshot) +
        snapshot.pendingVehicles.capacity() * sizeof(PendingVehicleSnapshot) +
        snapshot.intersections.capacity() * sizeof(IntersectionSnapshot) +
        snapshot.roads.capacity() * sizeof(RoadRuntimeSnapshot) +
        snapshot.activeEvents.capacity() * sizeof(TrafficEventSnapshot) +
        snapshot.failedRecalcIds.capacity() * sizeof(int);

    for (const VehicleSnapshot& vehicle : snapshot.vehicles) {
        bytes += estimateVehicleBytes(vehicle) - sizeof(VehicleSnapshot);
    }
    for (const PendingVehicleSnapshot& pending : snapshot.pendingVehicles) {
        bytes += estimateVehicleBytes(pending.vehicle) - sizeof(VehicleSnapshot);
        bytes += pending.route.capacity() * sizeof(std::optional<int>);
    }
    for (const IntersectionSnapshot& intersection : snapshot.intersections) {
        bytes += intersection.occupants.size() *
            (sizeof(int) + sizeof(IntersectionSnapshot::ReservationData) + 32u);
    }
    for (const RoadRuntimeSnapshot& road : snapshot.roads) {
        bytes += (road.blockedLanes.capacity() + 7u) / 8u;
    }
    bytes += snapshot.statistics.algorithmMetrics.size() * 128u;
    bytes += snapshot.statistics.travelMetrics.size() * 96u;
    for (const auto& entry : snapshot.mergingFromPOIByRoad) {
        bytes += sizeof(entry) + entry.second.capacity() * sizeof(int) + 32u;
    }
    bytes += snapshot.nextSpawnTimeByRoad.size() * 48u;
    bytes += snapshot.nextSpawnTimeBySource.size() * 48u;
    bytes += snapshot.nextTransitDepartureTimeByService.size() * 48u;
    return bytes;
}

} // namespace

SnapshotManager::SnapshotManager(std::size_t capacity)
    : capacity_(capacity > 0 ? capacity : 1) {
}

void SnapshotManager::setCapacity(std::size_t capacity) {
    capacity_ = capacity > 0 ? capacity : 1;
    enforceLimits();
}

void SnapshotManager::setMemoryBudgetBytes(std::size_t bytes) {
    memoryBudgetBytes_ = bytes > 0u ? bytes : 1u;
    enforceLimits();
}

void SnapshotManager::evictOldest() {
    if (snapshots_.empty()) {
        return;
    }
    if (!snapshotBytes_.empty()) {
        const std::size_t oldestBytes = snapshotBytes_.front();
        estimatedMemoryBytes_ = oldestBytes <= estimatedMemoryBytes_
            ? estimatedMemoryBytes_ - oldestBytes
            : 0u;
        snapshotBytes_.pop_front();
    }
    snapshots_.pop_front();
}

void SnapshotManager::enforceLimits() {
    while (snapshots_.size() > capacity_) {
        evictOldest();
    }
    while (snapshots_.size() > 2u &&
           estimatedMemoryBytes_ > memoryBudgetBytes_) {
        evictOldest();
    }
}

std::size_t SnapshotManager::capture(const TrafficSimulator& simulator) {
    // Use the most recent snapshot as a conservative forecast and release
    // old history before allocating another full copy under memory pressure.
    const std::size_t forecast = snapshotBytes_.empty()
        ? 0u
        : snapshotBytes_.back();
    while (snapshots_.size() > 1u &&
           estimatedMemoryBytes_ + forecast > memoryBudgetBytes_) {
        evictOldest();
    }
    while (!snapshots_.empty() && snapshots_.size() >= capacity_) {
        evictOldest();
    }

    // Capture into a temporary first. If allocation/copying throws, the
    // remaining history is still valid and no partial snapshot is published.
    SimulationSnapshot captured = simulator.captureSnapshot();
    const std::size_t capturedBytes = estimateSnapshotBytes(captured);
    snapshots_.emplace_back(std::move(captured));
    snapshotBytes_.push_back(capturedBytes);
    estimatedMemoryBytes_ += capturedBytes;
    enforceLimits();
    return snapshots_.size() - 1;
}

bool SnapshotManager::restore(TrafficSimulator& simulator,
                              std::size_t index,
                              std::string* error) {
    if (index >= snapshots_.size()) {
        if (error != nullptr) {
            *error = "Snapshot index is out of range.";
        }
        return false;
    }
    return simulator.restoreSnapshot(snapshots_[index], error);
}

void SnapshotManager::truncateAfter(std::size_t index) {
    if (index >= snapshots_.size()) {
        return;
    }
    while (snapshots_.size() > index + 1u) {
        const std::size_t newestBytes = snapshotBytes_.back();
        estimatedMemoryBytes_ = newestBytes <= estimatedMemoryBytes_
            ? estimatedMemoryBytes_ - newestBytes
            : 0u;
        snapshotBytes_.pop_back();
        snapshots_.pop_back();
    }
}
