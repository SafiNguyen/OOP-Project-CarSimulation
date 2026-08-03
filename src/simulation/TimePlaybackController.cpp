#include "TimePlaybackController.h"
#include "TrafficSimulator.h"

TimePlaybackController::TimePlaybackController(
    TrafficSimulator* simulator,
    SnapshotManager* manager)
    : simulator_(simulator),
      manager_(manager) {
}

std::size_t TimePlaybackController::rewind(std::size_t frames) {
    if (manager_ == nullptr || manager_->size() == 0) {
        return SnapshotManager::npos;
    }
    std::size_t target = currentIndex();
    if (target == SnapshotManager::npos) {
        target = manager_->newestIndex();
    }
    if (target == SnapshotManager::npos) {
        return SnapshotManager::npos;
    }
    if (target > 0) {
        const std::size_t steps = frames < target ? frames : target;
        target -= steps;
    }
    pendingIndex_ = target;
    return pendingIndex_;
}

std::size_t TimePlaybackController::forward(std::size_t frames) {
    if (manager_ == nullptr || manager_->size() == 0) {
        return SnapshotManager::npos;
    }
    std::size_t target = currentIndex();
    if (target == SnapshotManager::npos) {
        target = manager_->newestIndex();
        pendingIndex_ = target;
        return pendingIndex_;
    }
    const std::size_t newest = manager_->newestIndex();
    if (target < newest) {
        const std::size_t remaining = newest - target;
        const std::size_t steps = frames < remaining ? frames : remaining;
        target += steps;
    }
    pendingIndex_ = target;
    return pendingIndex_;
}

std::size_t TimePlaybackController::seek(std::size_t index) {
    if (manager_ == nullptr || index >= manager_->size()) {
        return SnapshotManager::npos;
    }
    pendingIndex_ = index;
    return pendingIndex_;
}

bool TimePlaybackController::applyPendingSeek(std::string* error) {
    if (!hasPendingSeek()) {
        return true;
    }
    const std::size_t target = pendingIndex_;
    pendingIndex_ = SnapshotManager::npos;
    if (manager_ == nullptr || simulator_ == nullptr) {
        if (error != nullptr) {
            *error = "Playback services are unavailable.";
        }
        return false;
    }
    try {
        if (!manager_->restore(*simulator_, target, error)) {
            return false;
        }
    } catch (const std::bad_alloc&) {
        if (error != nullptr) {
            *error = "Not enough memory to restore this snapshot.";
        }
        simulator_->pause();
        return false;
    } catch (const std::exception& exception) {
        if (error != nullptr) {
            *error = std::string("Snapshot restore failed: ") +
                     exception.what();
        }
        simulator_->pause();
        return false;
    } catch (...) {
        if (error != nullptr) {
            *error = "Snapshot restore failed with an unknown error.";
        }
        simulator_->pause();
        return false;
    }
    cursor_ = target;
    simulator_->pause();
    return true;
}

void TimePlaybackController::resumeLive() {
    pendingIndex_ = SnapshotManager::npos;
    if (manager_ != nullptr && cursor_ != SnapshotManager::npos) {
        manager_->truncateAfter(cursor_);
    }
    cursor_ = SnapshotManager::npos;
}

std::size_t TimePlaybackController::available() const {
    return manager_ != nullptr ? manager_->size() : 0;
}

bool TimePlaybackController::canRewind() const {
    if (manager_ == nullptr || manager_->size() == 0u) {
        return false;
    }
    const std::size_t index = currentIndex();
    return index == SnapshotManager::npos || index > 0u;
}

bool TimePlaybackController::canForward() const {
    if (manager_ == nullptr || manager_->size() == 0u) {
        return false;
    }
    const std::size_t index = currentIndex();
    return index != SnapshotManager::npos &&
           index < manager_->newestIndex();
}

double TimePlaybackController::timeAt(std::size_t index) const {
    if (manager_ == nullptr || index >= manager_->size()) {
        return 0.0;
    }
    return manager_->at(index).elapsedTime;
}

void TimePlaybackController::reset() {
    cursor_ = SnapshotManager::npos;
    pendingIndex_ = SnapshotManager::npos;
    if (manager_ != nullptr) {
        manager_->clear();
    }
}
