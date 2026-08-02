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
    snapshots_.erase(
        snapshots_.begin() + static_cast<std::ptrdiff_t>(index + 1u),
        snapshots_.end());
}
