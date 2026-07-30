#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include "JunctionConnector.h"

class Road;  
class TrafficLight;


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

// attributes
class Intersection {
protected:
    struct Reservation {
        const Road* fromRoad = nullptr;
        std::shared_ptr<const JunctionConnector> connector;
        double progressMetres = 0.0;
        double vehicleLengthMetres = 4.5;
        double vehicleWidthMetres = 1.8;
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
    bool explicitSignalPlan_ = false;

    void rebuildPhaseGroups();
    void resetSignalCycle();
    void synchronizeSignalHeads();
    std::size_t phaseIndexForRoad(const Road* road) const;
    std::size_t nextScheduledPhase() const;
    bool hasConflictingReservationForPhase(
        std::size_t phaseIndex) const;

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

    const Road* preemptedRoad_ = nullptr;
    double preemptionHoldSeconds_ = 0.0;

protected:
    virtual std::shared_ptr<const JunctionConnector> createConnector(
        const Road& incoming,
        int incomingLane,
        const Road& outgoing,
        int outgoingLane) const;

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
    TrafficLight* getLightForIncomingRoad(int roadId) const;
    TrafficLight* getLightForIncomingRoad(const Road* road) const;
    // Goi moi tick tu TrafficSimulator::update(dt)
    void updateTrafficLights(double dt);
    bool mustStopForRoad(const Road* road) const;
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

    void requestEmergencyPreemption(const Road* incomingRoad, double holdDuration);
    virtual bool canEnter(int vehicleId, const Road* fromRoad) const;
    virtual bool canEnterMovement(
        int vehicleId,
        const std::shared_ptr<const JunctionConnector>& connector,
        double requiredGapMetres,
        double vehicleLengthMetres = 4.5,
        double vehicleWidthMetres = 1.8) const;

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



    std::string toString() const;  

    virtual ~Intersection(); 
};

#endif
