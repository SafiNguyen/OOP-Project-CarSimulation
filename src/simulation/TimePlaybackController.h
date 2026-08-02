#ifndef TIME_PLAYBACK_CONTROLLER_H
#define TIME_PLAYBACK_CONTROLLER_H

#include <cstddef>
#include <string>

#include "SnapshotManager.h"

class TrafficSimulator;

/**
 * TimePlaybackController
 * ---------------------
 * High-level playback facade (Command pattern). Wraps a SnapshotManager
 * and a live TrafficSimulator to expose:
 *
 *   - rewind(speed)     : step backwards through snapshots each frame
 *   - forward(speed)    : step forwards through snapshots each frame
 *   - seek(index)       : jump to an exact snapshot
 *   - clearHistory()    : drop all snapshots (e.g. on map reload)
 *
 * While rewinding/forwarding, the simulator is held paused so the live
 * update() loop never races with the snapshot restore.
 */
class TimePlaybackController {
public:
    TimePlaybackController(TrafficSimulator* simulator,
                           SnapshotManager* manager);

    // Queues a rewind for the next frame boundary. Returns the target index.
    std::size_t rewind(std::size_t frames = 1);

    // Queues a fast-forward for the next frame boundary.
    std::size_t forward(std::size_t frames = 1);

    // Queues a jump directly to a snapshot index.
    std::size_t seek(std::size_t index);

    // Applies a queued seek. Call before simulation update/render begins.
    bool applyPendingSeek(std::string* error = nullptr);
    bool hasPendingSeek() const {
        return pendingIndex_ != SnapshotManager::npos;
    }

    // Current playback cursor, or npos if no snapshots exist.
    std::size_t currentIndex() const {
        return hasPendingSeek() ? pendingIndex_ : cursor_;
    }

    bool isSeeking() const {
        return cursor_ != SnapshotManager::npos || hasPendingSeek();
    }

    void stopSeeking() {
        cursor_ = SnapshotManager::npos;
        pendingIndex_ = SnapshotManager::npos;
    }

    // Leaves playback and truncates future history when live simulation
    // resumes from a past snapshot.
    void resumeLive();

    // How many snapshots are available.
    std::size_t available() const;

    // The timestamp of the snapshot at `index`.
    double timeAt(std::size_t index) const;

    // Clears all snapshot history and resets the cursor.
    void reset();

private:
    TrafficSimulator* simulator_;
    SnapshotManager* manager_;
    std::size_t cursor_ = SnapshotManager::npos;
    std::size_t pendingIndex_ = SnapshotManager::npos;
};

#endif // TIME_PLAYBACK_CONTROLLER_H
