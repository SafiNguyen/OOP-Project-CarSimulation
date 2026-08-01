#include "SnapshotManager.h"
#include "TrafficSimulator.h"

SnapshotManager::SnapshotManager(std::size_t capacity)
    : capacity_(capacity > 0 ? capacity : 1) {
}

std::size_t SnapshotManager::capture(const TrafficSimulator& simulator) {
    if (snapshots_.size() >= capacity_) {
        snapshots_.pop_front();
    }
    snapshots_.emplace_back(simulator.captureSnapshot());
    return snapshots_.size() - 1;
}

bool SnapshotManager::restore(TrafficSimulator& simulator, std::size_t index) {
    if (index >= snapshots_.size()) {
        return false;
    }
    simulator.restoreSnapshot(snapshots_[index]);
    return true;
}