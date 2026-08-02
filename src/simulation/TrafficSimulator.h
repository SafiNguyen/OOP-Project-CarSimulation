#ifndef TRAFFICSIMULATOR_H
#define TRAFFICSIMULATOR_H

#include <algorithm>
#include <vector>
#include <deque>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "SnapshotTypes.h"

class Graph;
class Road;
class Vehicle;
class EventManager;
class TrafficEvent;
class PathFindingStrategy;
class StatisticsManager;
class BusService;
class PointOfInterest;
class SnapshotManager;
class TimePlaybackController;


class TrafficSimulator {
public:
    using DeferredDemandProducer =
        std::function<void(TrafficSimulator&)>;

    struct SpawnStatistics {
        // accepted counts submissions whose ownership entered the simulator.
        // rejected/timedOut count terminal failures, including accepted
        // requests that later exhausted route retries or their queue lifetime.
        std::size_t accepted = 0;
        std::size_t activated = 0;
        std::size_t delayedAttempts = 0;
        std::size_t rejected = 0;
        std::size_t timedOut = 0;
    };
    
    TrafficSimulator(Graph* graph, PathFindingStrategy* strategy);

    ~TrafficSimulator();

   
    TrafficSimulator(const TrafficSimulator&) = delete;
    TrafficSimulator& operator=(const TrafficSimulator&) = delete;

    bool addVehicle(Vehicle* vehicle);
    // Used by explicit debug-console actions. Resolves and activates the
    // vehicle synchronously instead of placing it in the normal demand queue.
    bool addVehicleImmediately(Vehicle* vehicle);
    bool scheduleVehicleSpawn(
        Vehicle* vehicle,
        double delaySeconds);
    bool addVehicleWithFixedRoute(
        Vehicle* vehicle,
        const std::vector<Road*>& route);

    // Streams large configured demand into the simulator in short,
    // frame-budgeted batches so starting a large run never blocks the UI.
    void setDeferredDemand(
        std::size_t vehicleCount,
        DeferredDemandProducer producer);
    std::size_t getDeferredDemandCount() const {
        return deferredDemandRemaining_;
    }
    const std::string& getDeferredDemandError() const {
        return deferredDemandError_;
    }
    void reserveVehicleIdsThrough(int highestVehicleId) {
        highestReservedVehicleId_ = std::max(
            highestReservedVehicleId_,
            highestVehicleId);
    }
    int getHighestReservedVehicleId() const {
        return highestReservedVehicleId_;
    }

    void triggerEvent(std::unique_ptr<TrafficEvent> event);

    void update(double dt);


    void pause();
    void resume();
    bool isPaused() const;

    void setSpeedMultiplier(double factor);
    double getSpeedMultiplier() const;


    const std::vector<Vehicle*>& getVehicles() const;
    const std::vector<Vehicle*>& getFinishedVehicles() const;
    std::size_t getPendingVehicleCount() const;
    std::vector<Vehicle*> getPendingVehicles() const;
    void setMaximumActiveVehicles(std::size_t maximum);
    std::size_t getMaximumActiveVehicles() const;
    void setPendingVehicleTimeout(double seconds);
    double getPendingVehicleTimeout() const {
        return pendingVehicleTimeoutSeconds_;
    }
    const SpawnStatistics& getSpawnStatistics() const {
        return spawnStatistics_;
    }
    const Graph& getGraph() const;
    StatisticsManager* getStatisticsManager() const;

    double getElapsedTime() const;

    // --- Task 4: runtime pathfinding algorithm switching (UI dropdown) ---
    // Swaps the strategy used for all FUTURE routing decisions (new vehicles,
    // event-triggered recalculations, etc) and immediately recalculates the
    // route of every vehicle currently on the road so the change is visible
    // right away. Vehicles for which no new route could be found keep their
    // old route (so they don't vanish/teleport) but their id is recorded in
    // failedRecalcIds so the UI can draw a warning marker above them.
    void setPathFindingStrategy(PathFindingStrategy* strategy);
    PathFindingStrategy* getPathFindingStrategy() const;

    // Ids of vehicles whose most recent recalculation attempt failed.
    const std::set<int>& getFailedRecalcIds() const;

    // --- Snapshot support (Memento pattern) ---
    // Captures the complete simulation state into a SimulationSnapshot.
    // O(N) where N = active vehicles + intersections + pending vehicles.
    SimulationSnapshot captureSnapshot() const;
    // Restores the complete simulation state from a snapshot. O(N).
    // Destroys all live vehicles and reconstructs them from snapshot data.
    bool restoreSnapshot(const SimulationSnapshot& snapshot,
                         std::string* error = nullptr);

    // --- Snapshot auto-capture (OMIT_CAPTURE_PERFORMANCE feature) ---
    // Snapshots are taken automatically every `intervalSeconds` of
    // simulated time so playback history is always available. Setting
    // intervalSeconds <= 0 disables auto-capture.
    void setSnapshotInterval(double intervalSeconds);
    double getSnapshotInterval() const { return snapshotIntervalSeconds_; }
    // Time of the last auto-snapshot (for deciding when to capture next).
    double getLastSnapshotTime() const { return lastSnapshotTime_; }
    void notifySnapshotTaken(double atTime) { lastSnapshotTime_ = atTime; }

    // Snapshot manager / playback controller access (Memento caretaker).
    SnapshotManager* getSnapshotManager() const {
        return snapshotManager_.get();
    }
    TimePlaybackController* getPlaybackController() const {
        return playbackController_.get();
    }

    // Captures a snapshot right now (manual trigger from UI/keyboard).
    // Returns the new snapshot index.
    std::size_t captureSnapshotNow();

private:
    // Called from update() once per frame; applies auto-capture policy.
    void maybeAutoCaptureSnapshot();

    double snapshotIntervalSeconds_ = 5.0;
    double lastSnapshotTime_ = 0.0;
    std::unique_ptr<SnapshotManager> snapshotManager_;
    std::unique_ptr<TimePlaybackController> playbackController_;
    friend class SnapshotManager;
    friend class TimePlaybackController;
    struct PendingVehicle {
        Vehicle* vehicle = nullptr;
        std::vector<Road*> route;
        double earliestActivationTime = 0.0;
        double nextAttemptTime = 0.0;
        double deadlineTime = 0.0;
        bool phasedAdmission = false;
        bool routeResolved = false;
        bool fixedRoute = false;
        int routeAttempts = 0;
    };

    static constexpr double MAX_RAW_DT = 0.1;
    static constexpr double MAX_SUBSTEP = 0.05;
    static constexpr int MAX_SUBSTEPS_PER_CALL = 200;
    static constexpr double MAX_LEFTOVER_DT = 5.0;
    static constexpr double MIN_SPAWN_HEADWAY_SECONDS = 0.9;
    static constexpr double TRANSIT_NETWORK_HEADWAY_SECONDS = 4.0;
    static constexpr double TRANSIT_DEPARTURE_HEADWAY_SECONDS = 15.0;
    static constexpr int
        MAX_PHASED_ACTIVATIONS_PER_ADMISSION_PASS = 6;
    static constexpr std::size_t
        MAX_PENDING_INSPECTIONS_PER_UPDATE = 64u;
    static constexpr double
        DEFAULT_PENDING_TIMEOUT_SECONDS = 600.0;
    static constexpr int MAX_ROUTE_ATTEMPTS = 8;
    // Route searches allocate and traverse the graph. Keeping this bounded
    // prevents a synchronized reroute wave from stalling a render frame.
    static constexpr std::size_t MAX_DYNAMIC_REROUTES_PER_SUBSTEP = 2u;
    static constexpr std::size_t
        MAX_DEFERRED_DEMAND_PER_FRAME = 64u;
    static constexpr double
        DEFERRED_DEMAND_FRAME_BUDGET_MS = 3.0;

    void pumpDeferredDemand();
    void pruneMergingIndex(Road* road);
    void recalculateAllVehicleRoutes();
    void removeFinishedVehicles();
    bool addVehicleWithDelay(
        Vehicle* vehicle,
        double delaySeconds);
    bool tryActivateVehicle(
        Vehicle* vehicle,
        const std::vector<Road*>& route,
        bool bypassAdmissionGates = false);
    bool hasValidEndpoints(const Vehicle* vehicle) const;
    bool resolveRoute(
        Vehicle* vehicle,
        std::vector<Road*>& route);
    bool routeNeedsRefresh(
        const PendingVehicle& pending) const;
    void discardPendingVehicle(
        PendingVehicle& pending,
        bool timedOut);
    void activatePendingVehicles();


    Graph* graph;                                   
    PathFindingStrategy* pathFindingStrategy;  
    std::vector<Vehicle*> vehicles;
    std::size_t dynamicRerouteCursor_ = 0u;
    std::unordered_map<Road*, std::vector<Vehicle*>> mergingFromPOIByRoad_;
    std::deque<PendingVehicle> pendingVehicles;
    std::vector<Vehicle*> finishedVehicles;      
    std::size_t maximumActiveVehicles_ =
        std::numeric_limits<std::size_t>::max();
    std::unordered_map<const Road*, double>
        nextSpawnTimeByRoad_;
    std::unordered_map<const PointOfInterest*, double>
        nextSpawnTimeBySource_;
    std::unordered_map<const BusService*, double>
        nextTransitDepartureTimeByService_;
    double nextTransitNetworkDepartureTime_ = 0.0;
    SpawnStatistics spawnStatistics_;
    double pendingVehicleTimeoutSeconds_ =
        DEFAULT_PENDING_TIMEOUT_SECONDS;
    std::unique_ptr<EventManager> eventManager;     
    std::unique_ptr<StatisticsManager> statisticsManager;               
    DeferredDemandProducer deferredDemandProducer_;
    std::size_t deferredDemandRemaining_ = 0u;
    std::string deferredDemandError_;
    int highestReservedVehicleId_ = -1;

    bool paused;
    double speedMultiplier;
    double elapsedTime;
    long long tickCount;
    double leftoverDt = 0.0;


    std::set<int> failedRecalcIds;
};

#endif
