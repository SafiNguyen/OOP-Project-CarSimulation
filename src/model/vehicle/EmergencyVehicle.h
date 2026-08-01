#ifndef EMERGENCY_VEHICLE_H
#define EMERGENCY_VEHICLE_H

#include "Vehicle.h"
#include "Road.h"
#include "Intersection.h"

class EmergencyVehicle : public Vehicle {
public:
    static constexpr double YIELD_LOOKAHEAD_DISTANCE = 60.0;
    static constexpr double PRIORITY_LOOKAHEAD_DISTANCE = 80.0;
    static constexpr double PRIORITY_HOLD_DURATION = 3.0;
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
    bool hasTrafficPriority() const override { return true; }

    // Priority reserves the emergency approach before this vehicle may
    // proceed against RED/YELLOW. Junction occupants remain a hard safety
    // constraint.
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
    void update(double dt,
                Graph* graph = nullptr,
                PathFindingStrategy* strategy = nullptr,
                bool allowDynamicReroute = true) override {
        requestPriorityIfNear();
        Vehicle::update(dt, graph, strategy, allowDynamicReroute);
        // A lane change may have cleared the approach during this update.
        // Re-evaluate immediately instead of delaying priority until the
        // next simulation tick.
        requestPriorityIfNear();
        notifyVehiclesAhead();
    }

protected:
    bool shouldBypassQueueBeforeJunction(
        const LaneMapping& preferredMapping) const override {
        if (currentRoad == nullptr || !preferredMapping.valid) {
            return false;
        }
        const int queuedLane =
            preferredMapping.movement == MovementType::Straight
                ? currentLaneIndex
                : preferredMapping.incomingLane;
        return currentRoad->findLeader(queuedLane, this) != nullptr;
    }

private:
    void requestPriorityIfNear() const {
        Road* road = getCurrentRoad();
        if (road == nullptr) return;
        Intersection* next = road->getEnd();
        if (next == nullptr) return;

        const double distanceToEnd =
            road->getDistance() - getProgressOnRoad();
        const bool approachLaneIsClear =
            road->findLeader(
                getCurrentLaneIndex(), this) == nullptr;
        if (distanceToEnd <= PRIORITY_LOOKAHEAD_DISTANCE &&
            approachLaneIsClear) {
            const LaneMapping mapping =
                getJunctionEntryLaneMapping();
            Road* outgoing =
                mapping.valid ? getNextRoad() : nullptr;
            next->requestEmergencyPriority(
                getId(),
                road,
                getCurrentLaneIndex(),
                outgoing,
                mapping.valid
                    ? mapping.outgoingLane
                    : -1,
                getWidth(),
                PRIORITY_HOLD_DURATION);
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

        Road* outgoing = getNextRoad();
        if (outgoing == nullptr) return;

        double distanceToOutgoing = remainingOnRoad;
        int outgoingLane = -1;
        if (movementState_ == MovementState::TraversingJunction &&
            activeConnector_ != nullptr) {
            distanceToOutgoing =
                activeConnector_->getLength() -
                junctionProgressMetres_;
            outgoingLane = outgoingLaneIndex_;
        } else {
            const LaneMapping mapping =
                getJunctionEntryLaneMapping();
            Intersection* junction = road->getEnd();
            if (!mapping.valid || junction == nullptr) return;

            const auto connector = junction->getConnector(
                road,
                mapping.incomingLane,
                outgoing,
                mapping.outgoingLane);
            if (connector == nullptr) return;

            distanceToOutgoing += connector->getLength();
            outgoingLane = mapping.outgoingLane;
        }

        if (outgoingLane < 0 ||
            outgoingLane >= outgoing->getLaneCount() ||
            distanceToOutgoing >= YIELD_LOOKAHEAD_DISTANCE) {
            return;
        }

        const double spillover =
            YIELD_LOOKAHEAD_DISTANCE - distanceToOutgoing;
        for (Vehicle* vehicle :
             outgoing->getVehiclesInProgressRange(0.0, spillover)) {
            if (vehicle != this) {
                vehicle->notifyEmergencyApproaching(outgoingLane);
            }
        }
    }
};

#endif
