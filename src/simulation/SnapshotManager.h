#ifndef SNAPSHOT_MANAGER_H
#define SNAPSHOT_MANAGER_H

#include <cstddef>
#include <deque>
#include <memory>
#include <string>

#include "SnapshotTypes.h"

class TrafficSimulator;

/**
 * SnapshotManager
 * ---------------
 * Memento-pattern caretaker. Owns a bounded ring buffer of
 * SimulationSnapshot objects and provides O(1) capture/restore
 * operations. The ring buffer is implemented as a std::deque with
 * a fixed capacity so the oldest snapshots are evicted automatically.
 *
 * Performance contract:
 *   - capture()  : O(N) where N = active vehicles + intersections.
 *   - restore()  : O(N) same bound.
 *   - Memory     : bounded by capacity * snapshotSize.
 */
class SnapshotManager {
public:
    explicit SnapshotManager(std::size_t capacity = 600);

    // Disable copying (owns a large buffer).
    SnapshotManager(const SnapshotManager&) = delete;
    SnapshotManager& operator=(const SnapshotManager&) = delete;

    /// Captures the current simulator state into a new snapshot and
    /// appends it to the ring buffer. Returns the index of the new snapshot.
    std::size_t capture(const TrafficSimulator& simulator);

    /// Restores the simulator to the snapshot at `index`.
    /// Returns false if index is out of range.
    bool restore(TrafficSimulator& simulator,
                 std::size_t index,
                 std::string* error = nullptr);

    /// Drops snapshots newer than `index`. Used when live simulation resumes
    /// from a rewound state and creates a new timeline branch.
    void truncateAfter(std::size_t index);

    /// Returns the number of snapshots currently held.
    std::size_t size() const { return snapshots_.size(); }

    /// Returns the capacity of the ring buffer.
    std::size_t capacity() const { return capacity_; }

    /// Changes the ring-buffer capacity and evicts the oldest snapshots when
    /// the new limit is smaller than the current history.
    void setCapacity(std::size_t capacity);

    /// Bounds snapshot history by estimated heap usage as well as count.
    /// At least the two newest snapshots are retained so rewind remains
    /// useful even when one unusually large snapshot exceeds the budget.
    void setMemoryBudgetBytes(std::size_t bytes);
    std::size_t memoryBudgetBytes() const { return memoryBudgetBytes_; }
    std::size_t estimatedMemoryBytes() const { return estimatedMemoryBytes_; }

    /// Returns the index of the most recent snapshot, or npos if empty.
    std::size_t newestIndex() const {
        return snapshots_.empty() ? npos : snapshots_.size() - 1;
    }

    /// Returns the index of the oldest snapshot, or npos if empty.
    std::size_t oldestIndex() const {
        return snapshots_.empty() ? npos : 0;
    }

    /// Clears all snapshots.
    void clear() {
        snapshots_.clear();
        snapshotBytes_.clear();
        estimatedMemoryBytes_ = 0u;
    }

    /// Returns a const reference to the snapshot at `index`.
    /// Behavior is undefined if index is out of range.
    const SimulationSnapshot& at(std::size_t index) const {
        return snapshots_[index];
    }

    static constexpr std::size_t npos = static_cast<std::size_t>(-1);

private:
    std::deque<SimulationSnapshot> snapshots_;
    std::deque<std::size_t> snapshotBytes_;
    std::size_t capacity_;
    std::size_t memoryBudgetBytes_ = 384u * 1024u * 1024u;
    std::size_t estimatedMemoryBytes_ = 0u;

    void evictOldest();
    void enforceLimits();
};

#endif // SNAPSHOT_MANAGER_H
