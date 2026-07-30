#ifndef EMERGENCY_VEHICLE_H
#define EMERGENCY_VEHICLE_H

#include "Vehicle.h"
#include "Road.h"
#include "Intersection.h"

class EmergencyVehicle : public Vehicle {
public:
    static constexpr double YIELD_LOOKAHEAD_DISTANCE = 60.0;
    static constexpr double PREEMPTION_LOOKAHEAD_DISTANCE = 80.0;
    static constexpr double PREEMPTION_HOLD_DURATION = 3.0;
    static constexpr double JUNCTION_CAUTION_SPEED_FACTOR = 0.85;

    EmergencyVehicle(int id,
                     double speed,
                     Intersection* start,
                     Intersection* dest)
        : Vehicle(id, speed, start, dest) {}

    double calculateCurrentSpeed() const override {
        if (currentRoad == nullptr) return 0.0;
        if (currentRoad->getLane(currentLaneIndex).isBlocked()) return 0.0;
        return baseSpeed;
    }

    double getAcceleration() const override { return 30.0; }
    double getDeceleration() const override { return 40.0; }
    double getMaxLateralAcceleration() const override { return 3.5; }
    VehicleKind getVehicleKind() const override {
        return VehicleKind::Emergency;
    }

    // Preemption reserves the emergency approach before this vehicle may
    // proceed against RED/YELLOW. Junction occupants and pedestrians inside
    // the vehicle's physical lane remain hard safety constraints.
    double getLength() const override { return 6.0; }
    double getWidth() const override { return 2.2; }
    double getHeight() const override { return 2.5; }
    double getWeight() const override { return 3.5; }
    double getMinGap() const override { return 2.5; }
    double getYieldSpeedFactor() const override { return 1.0; }
    double getEmergencyJunctionSpeedLimit(
        double freeFlowSpeed) const override {
        return freeFlowSpeed *
               JUNCTION_CAUTION_SPEED_FACTOR;
    }
    void notifyEmergencyApproaching(int /*emergencyLaneIndex*/) override {}
    bool mustStopForTrafficLight(
        Intersection* nextIntersection) const override {
        if (nextIntersection != nullptr &&
            nextIntersection->
                isPrioritizedEmergencyVehicle(
                    getId(), getCurrentRoad())) {
            return false;
        }
        return Vehicle::mustStopForTrafficLight(
            nextIntersection);
    }

    void update(double dt,
                Graph* graph = nullptr,
                PathFindingStrategy* strategy = nullptr,
                bool allowDynamicReroute = true) override {
        requestPreemptionIfNear();
        Vehicle::update(dt, graph, strategy, allowDynamicReroute);
        notifyVehiclesAhead();
    }

private:
    void requestPreemptionIfNear() const {
        Road* road = getCurrentRoad();
        if (road == nullptr) return;
        Intersection* next = road->getEnd();
        if (next == nullptr) return;

        const double distanceToEnd =
            road->getDistance() - getProgressOnRoad();
        if (distanceToEnd <= PREEMPTION_LOOKAHEAD_DISTANCE) {
            const LaneMapping mapping =
                getJunctionEntryLaneMapping();
            Road* outgoing =
                mapping.valid ? getNextRoad() : nullptr;
            next->requestEmergencyPreemption(
                getId(),
                road,
                getCurrentLaneIndex(),
                outgoing,
                mapping.valid
                    ? mapping.outgoingLane
                    : -1,
                getWidth(),
                PREEMPTION_HOLD_DURATION);
        }
    }

    void notifyVehiclesAhead() const {
        Road* road = getCurrentRoad();
        if (road == nullptr) return;

        const double selfProgress = getProgressOnRoad();
        const double remainingOnRoad =
            road->getDistance() - selfProgress;
        const int myLane = getCurrentLaneIndex();

        for (Vehicle* vehicle : road->getVehiclesInProgressRange(
                 selfProgress,
                 selfProgress + YIELD_LOOKAHEAD_DISTANCE)) {
            if (vehicle != this) {
                vehicle->notifyEmergencyApproaching(myLane);
            }
        }

        if (remainingOnRoad < YIELD_LOOKAHEAD_DISTANCE) {
            Road* next = getNextRoad();
            if (next == nullptr) return;
            const double spillover =
                YIELD_LOOKAHEAD_DISTANCE - remainingOnRoad;
            const int nextLane =
                std::min(myLane, next->getLaneCount() - 1);
            for (Vehicle* vehicle :
                 next->getVehiclesInProgressRange(0.0, spillover)) {
                if (vehicle != this) {
                    vehicle->notifyEmergencyApproaching(nextLane);
                }
            }
        }
    }
};

#endif
