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
    if (cursor_ == SnapshotManager::npos) {
        cursor_ = manager_->newestIndex();
    }
    if (cursor_ == SnapshotManager::npos) {
        return SnapshotManager::npos;
    }
    if (cursor_ > 0) {
        const std::size_t steps = frames < cursor_ ? frames : cursor_;
        cursor_ -= steps;
        if (simulator_ != nullptr) {
            manager_->restore(*simulator_, cursor_);
            simulator_->pause();
        }
    }
    return cursor_;
}

std::size_t TimePlaybackController::forward(std::size_t frames) {
    if (manager_ == nullptr || manager_->size() == 0) {
        return SnapshotManager::npos;
    }
    if (cursor_ == SnapshotManager::npos) {
        cursor_ = manager_->newestIndex();
        if (simulator_ != nullptr && cursor_ != SnapshotManager::npos) {
            manager_->restore(*simulator_, cursor_);
            simulator_->pause();
        }
        return cursor_;
    }
    const std::size_t newest = manager_->newestIndex();
    if (cursor_ < newest) {
        const std::size_t remaining = newest - cursor_;
        const std::size_t steps = frames < remaining ? frames : remaining;
        cursor_ += steps;
        if (simulator_ != nullptr) {
            manager_->restore(*simulator_, cursor_);
            simulator_->pause();
        }
    }
    return cursor_;
}

std::size_t TimePlaybackController::seek(std::size_t index) {
    if (manager_ == nullptr || index >= manager_->size()) {
        return SnapshotManager::npos;
    }
    cursor_ = index;
    if (simulator_ != nullptr) {
        manager_->restore(*simulator_, cursor_);
        simulator_->pause();
    }
    return cursor_;
}

std::size_t TimePlaybackController::available() const {
    return manager_ != nullptr ? manager_->size() : 0;
}

double TimePlaybackController::timeAt(std::size_t index) const {
    if (manager_ == nullptr || index >= manager_->size()) {
        return 0.0;
    }
    return manager_->at(index).elapsedTime;
}

void TimePlaybackController::reset() {
    cursor_ = SnapshotManager::npos;
    if (manager_ != nullptr) {
        manager_->clear();
    }
}