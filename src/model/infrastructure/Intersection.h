#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <vector>
#include <string>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include "JunctionConnector.h"

class Road;  
class TrafficLight;
struct IntersectionSnapshot;

struct VehicleMovementPath {
    const Road* incomingRoad = nullptr;
    int incomingLane = -1;
    const Road* outgoingRoad = nullptr;
    int outgoingLane = -1;
    double vehicleWidthMetres = 0.0;

    bool isValid() const {
        return incomingRoad != nullptr &&
               incomingLane >= 0 &&
               vehicleWidthMetres > 0.0;
    }
};

struct EmergencyApproach {
    int vehicleId = -1;
    VehicleMovementPath path;

    bool isValid() const {
        return vehicleId >= 0 && path.isValid();
    }
};

enum class IntersectionType {
    PASS_THROUGH, // 0-2 incoming roads: nothing to arbitrate between
    THREE_WAY,    // nga ba
    FOUR_WAY,     // nga tu
    COMPLEX       // 5+ incoming roads
};

enum class SignalStage {
    GREEN,
    YELLOW,
    ALL_RED
};

enum class JunctionDecision {
    Proceed,
    Yield,
    Stop
};

// attributes
class Intersection {
protected:
    struct Reservation {
        const Road* fromRoad = nullptr;
        std::shared_ptr<const JunctionConnector> connector;
        double progressMetres = 0.0;
        double vehicleLengthMetres = 4.5;
        double vehicleWidthMetres = 1.8;
        uint64_t entryId = 0;
    };

private:
    int id;
    double x;
    double y;
    
    std::vector<Road*> incomingRoads; 
    std::vector<Road*> outgoingRoads;
    // key = id cua incoming road; moi road vao co dung 1 den rieng
    std::unordered_map<int, std::unique_ptr<TrafficLight>> trafficLights;
    
    std::vector<std::vector<Road*>> phaseGroups;
    std::size_t activePhaseGroup = 0;
    SignalStage signalStage_ = SignalStage::GREEN;
    double stageRemainingSeconds_ = 30.0;
    double greenDurationSeconds_ = 30.0;
    double yellowDurationSeconds_ = 3.0;
    double allRedDurationSeconds_ = 2.0;
    bool allowRightTurnOnRed_ = true;

    void rebuildPhaseGroups();
    void resetSignalCycle();
    void synchronizeSignalHeads();
    TrafficLight* getMutableLightForIncomingRoad(int roadId);
    double nominalRedDuration() const;
    std::size_t nextScheduledPhase() const;

    // --- Intersection-box reservation (prevents multiple vehicles from
    // different roads overlapping inside the junction at the same time,
    // independent of the traffic-light phase groups above) ---
    int capacity_ = 1;

protected:
    std::unordered_map<int, Reservation> occupants_;

private:
    mutable std::unordered_map<
        ConnectorKey,
        std::shared_ptr<const JunctionConnector>,
        ConnectorKeyHash> connectorCache_;

    struct MovementGeometryCache {
        bool valid = false;
        std::uint64_t reservationRevision = 0;
        int vehicleId = -1;
        const JunctionConnector* connector = nullptr;
        double requiredGapMetres = 0.0;
        double vehicleLengthMetres = 0.0;
        double vehicleWidthMetres = 0.0;
        bool canEnter = false;
    };

    std::uint64_t nextEntryId_ = 0;
    std::uint64_t reservationRevision_ = 0;
    mutable MovementGeometryCache movementGeometryCache_;

    // Cache for hasPriorityVehicleApproaching to avoid O(N^2) per frame.
    // Cleared once per frame in TrafficSimulator::update().
    mutable std::unordered_map<const Road*, bool> priorityVehicleCache_;
    mutable bool priorityVehicleCacheValid_ = false;

    EmergencyApproach emergencyApproach_;
    double emergencyPriorityRemainingSeconds_ = 0.0;

    void clearEmergencyPriority();
    void updateEmergencyPriority(double dt);
    void markReservationStateChanged();

protected:
    virtual std::shared_ptr<const JunctionConnector> createConnector(
        const Road& incoming,
        int incomingLane,
        const Road& outgoing,
        int outgoingLane) const;
    bool isEmergencyAdmissionBlocked(
        int vehicleId,
        const Road* incomingRoad) const;

public:
    Intersection(int id, double x = 0.0, double y = 0.0);

    Intersection(const Intersection&) = delete;
    Intersection& operator=(const Intersection&) = delete;
    
    int getId() const;
    double getX() const;
    double getY() const;
    
    const std::vector<Road*>& getIncomingRoads() const; 
    const std::vector<Road*>& getOutgoingRoads() const;

    //intersection shape, derived from the number of incoming
    // approaches (nga ba / nga tu / etc).
    IntersectionType getIntersectionType() const;
    std::string getIntersectionTypeLabel() const;
    std::size_t getApproachCount() const;

    virtual bool isRoundabout() const { return false; }
    virtual double getTraversalRadiusMetres() const { return 0.0; }
    virtual double getTraversalWidthMetres() const { return 0.0; }

    //methods
    void addIncomingRoad(Road* road);
    void addOutgoingRoad(Road* road);
    void removeIncomingRoad(Road* road);
    void removeOutgoingRoad(Road* road);
    // Traffic light management 
    void registerIncomingLight(Road* road);
    bool configureTrafficSignals(
        const std::vector<std::vector<Road*>>& phases,
        double greenDuration,
        double yellowDuration,
        double allRedDuration,
        std::string* error = nullptr);
    bool configureTrafficSignalsAutomatically(
        double greenDuration,
        double yellowDuration,
        double allRedDuration,
        std::string* error = nullptr);
    void unregisterIncomingLight(Road* road);
    bool hasTrafficLights() const;
    const TrafficLight* getLightForIncomingRoad(int roadId) const;
    const TrafficLight* getLightForIncomingRoad(
        const Road* road) const;
    // Advances the fixed signal clock once per simulator tick.
    void updateTrafficLights(double dt);
    bool mustStopForRoad(const Road* road) const;
    JunctionDecision getMovementDecision(
        const Road* incomingRoad,
        int incomingLane,
        const Road* outgoingRoad,
        MovementType movement) const;
    void setAllowRightTurnOnRed(bool allow) {
        allowRightTurnOnRed_ = allow;
    }
    bool allowsRightTurnOnRed() const {
        return allowRightTurnOnRed_;
    }
    std::shared_ptr<const JunctionConnector> getConnector(
        const Road* incoming,
        int incomingLane,
        const Road* outgoing,
        int outgoingLane) const;
    void invalidateConnectorCache();
    std::size_t getConnectorCacheSize() const {
        return connectorCache_.size();
    }

    // Number of independent signal phases this intersection cycles
    // through (e.g. 2 for a typical nga tu with opposing through-roads
    // paired up, 1 if there is nothing to arbitrate).
    std::size_t getPhaseGroupCount() const { return phaseGroups.size(); }
    std::size_t getActivePhaseIndex() const { return activePhaseGroup; }
    SignalStage getSignalStage() const { return signalStage_; }
    double getSignalStageRemainingSeconds() const {
        return stageRemainingSeconds_;
    }
    // True if `a` and `b` are allowed to move at the same time (either
    // they are the same phase group, or one/both have no light at all).
    bool areRoadsInSamePhase(const Road* a, const Road* b) const;

    void requestEmergencyPriority(
        int vehicleId,
        const Road* incomingRoad,
        int incomingLane,
        const Road* outgoingRoad,
        int outgoingLane,
        double vehicleWidthMetres,
        double holdDuration);
    bool hasActiveEmergencyPriority() const;
    bool isPrioritizedEmergencyVehicle(
        int vehicleId,
        const Road* incomingRoad) const;
    virtual bool canEnter(int vehicleId, const Road* fromRoad) const;
    virtual bool canEnterMovement(
        int vehicleId,
        const std::shared_ptr<const JunctionConnector>& connector,
        double requiredGapMetres,
        double vehicleLengthMetres = 4.5,
        double vehicleWidthMetres = 1.8) const;
    bool canEnterYieldingMovement(
        int vehicleId,
        const std::shared_ptr<const JunctionConnector>& connector,
        double requiredGapMetres,
        double vehicleLengthMetres = 4.5,
        double vehicleWidthMetres = 1.8) const;

    // Checks whether any vehicle on a non-yielding, non-stopping road is
    // approaching within the lookahead distance. Uses a per-frame cache
    // to avoid O(N^2) behavior; call clearFrameCache() once per frame.
    bool hasPriorityVehicleApproaching(
        const Road* yieldingRoad) const;

    // --- Intersection-box reservation ---
    // Attempts to claim one of the `capacity_` slots for `vehicleId`. Returns
    // true if the vehicle now holds a slot (either newly granted, or it
    // already held one - calling this again is safe/idempotent). Returns
    // false if the box is full and the vehicle must keep waiting at the
    // stop line.
    virtual bool tryEnter(int vehicleId, const Road* fromRoad);
    virtual bool tryEnterMovement(
        int vehicleId,
        const std::shared_ptr<const JunctionConnector>& connector,
        double requiredGapMetres,
        double vehicleLengthMetres = 4.5,
        double vehicleWidthMetres = 1.8);
    bool tryEnterYieldingMovement(
        int vehicleId,
        const std::shared_ptr<const JunctionConnector>& connector,
        double requiredGapMetres,
        double vehicleLengthMetres = 4.5,
        double vehicleWidthMetres = 1.8);
    void updateReservationProgress(int vehicleId, double progressMetres);
    double limitTraversalAdvance(
        int vehicleId,
        const std::shared_ptr<const JunctionConnector>& connector,
        double currentProgressMetres,
        double desiredAdvanceMetres,
        double vehicleLengthMetres,
        double vehicleWidthMetres,
        double clearanceMetres) const;
    double constrainIncomingStopPosition(
        const Road* incomingRoad,
        int incomingLane,
        double nominalStopPositionMetres,
        double waitingVehicleLengthMetres,
        double requiredClearanceMetres) const;
    bool isOutgoingLaneReserved(
        const Road* outgoingRoad,
        int outgoingLane) const;
    // Releases the slot held by `vehicleId`, if any. Safe to call even if
    // the vehicle never held a slot.
    void exit(int vehicleId);
    // True if every slot is currently occupied.
    virtual bool isFull() const;
    void setCapacity(int cap);
    int getCapacity() const { return capacity_; }



    // Clears the per-frame cache used by hasPriorityVehicleApproaching.
    // Called once per frame from TrafficSimulator::update() before the
    // vehicle update loop.
    void clearFrameCache() const {
        priorityVehicleCacheValid_ = false;
        priorityVehicleCache_.clear();
        movementGeometryCache_.valid = false;
    }

    std::string toString() const;  

    // --- Snapshot support (Memento pattern) ---
    void captureSnapshot(IntersectionSnapshot& snap) const;
    void restoreSnapshot(const IntersectionSnapshot& snap);

    virtual ~Intersection(); 
};

#endif
