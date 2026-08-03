#include "JunctionTraversalState.h"
#include "Vehicle.h"
#include "Intersection.h"
#include "Road.h"
#include "JunctionConnector.h"
#include "LaneMapping.h"
#include <algorithm>
#include <memory>

namespace {
double combinedHalfLength(const Vehicle& first, const Vehicle& second) {
    return (first.getLength() + second.getLength()) * 0.5;
}

double outgoingReleaseGap(const Vehicle& entering, const Vehicle& leader) {
    return leader.getProgressOnRoad() - entering.getLength() * 0.5 - combinedHalfLength(entering, leader);
}
}

bool JunctionTraversalState::beginTraversal(Vehicle& vehicle, const LaneMapping& mapping, Intersection* intersection) {
    if (vehicle.currentRoad == nullptr || vehicle.junctionOutgoingRoad_ != nullptr || intersection == nullptr || !mapping.valid || vehicle.currentLaneIndex != mapping.incomingLane) {
        return false;
    }

    Road* outgoing = vehicle.getNextRoad();
    if (outgoing == nullptr) return false;
    Vehicle* outgoingLeader = outgoing->getFirstVehicleInLane(mapping.outgoingLane);
    if (outgoingLeader != nullptr) {
        const double availableAtExit = outgoingReleaseGap(vehicle, *outgoingLeader);
        if (availableAtExit < std::max(vehicle.getMinGap(), outgoingLeader->getMinGap())) {
            return false;
        }
    }

    auto connector = intersection->getConnector(vehicle.currentRoad, mapping.incomingLane, outgoing, mapping.outgoingLane);
    if (connector == nullptr) return false;

    const double requiredGap = vehicle.getMinGap() + vehicle.currentSpeed * vehicle.getTimeHeadway();
    const JunctionDecision decision = vehicle.getJunctionDecision(intersection, mapping, outgoing);
    if (decision == JunctionDecision::Stop) {
        return false;
    }
    const bool entered = decision == JunctionDecision::Yield
        ? intersection->tryEnterYieldingMovement(vehicle.getId(), connector, requiredGap, vehicle.getLength(), vehicle.getWidth())
        : intersection->tryEnterMovement(vehicle.getId(), connector, requiredGap, vehicle.getLength(), vehicle.getWidth());
    if (!entered) {
        return false;
    }

    vehicle.reservedIntersection_ = intersection;
    vehicle.incomingLaneIndex_ = mapping.incomingLane;
    vehicle.outgoingLaneIndex_ = mapping.outgoingLane;
    vehicle.junctionOutgoingRoad_ = outgoing;
    vehicle.activeConnector_ = std::move(connector);
    vehicle.junctionProgressMetres_ = 0.0;
    vehicle.setClearedIncomingRoad(false);
    vehicle.progressOnCurrentRoad = vehicle.currentRoad->getDistance();
    vehicle.paused = false;
    vehicle.pauseReason = PauseReason::None;
    vehicle.movementState_ = MovementState::TraversingJunction;
    vehicle.junctionTurnSignal_ = mapping.movement == MovementType::Right ? TurnSignal::Right : (mapping.movement == MovementType::Left || mapping.movement == MovementType::UTurn ? TurnSignal::Left : TurnSignal::Off);
    vehicle.clearLaneChangeIntent();
    vehicle.refreshTurnSignal();
    return true;
}

void JunctionTraversalState::completeTraversal(Vehicle& vehicle, double outgoingProgressMetres) {
    Road* incoming = vehicle.currentRoad;
    Road* outgoing = vehicle.junctionOutgoingRoad_;
    if (incoming != nullptr) {
        vehicle.addTravelHistory(incoming);
    }
    ++vehicle.currentRouteIndex;
    vehicle.currentRoad = outgoing;
    vehicle.currentLaneIndex = vehicle.outgoingLaneIndex_;
    vehicle.progressOnCurrentRoad = std::max(0.0, outgoingProgressMetres);

    if (vehicle.reservedIntersection_ != nullptr) {
        vehicle.reservedIntersection_->exit(vehicle.getId());
        vehicle.reservedIntersection_ = nullptr;
    }
    vehicle.activeConnector_.reset();
    vehicle.junctionOutgoingRoad_ = nullptr;
    vehicle.junctionProgressMetres_ = 0.0;
    vehicle.movementState_ = MovementState::OnRoad;
    vehicle.paused = false;
    vehicle.pauseReason = PauseReason::None;
    vehicle.junctionTurnSignal_ = TurnSignal::Off;
    vehicle.clearLaneChangeIntent();

    if (vehicle.currentRoad != nullptr) {
        vehicle.progressOnCurrentRoad = std::min(vehicle.progressOnCurrentRoad, vehicle.currentRoad->getDistance());
        vehicle.currentRoad->getLane(vehicle.currentLaneIndex).addVehicle(&vehicle);
    }
    vehicle.onRoadChanged();
    vehicle.refreshTurnSignal();
}

double JunctionTraversalState::advance(Vehicle& vehicle, double availableTime) {
    if (vehicle.movementState_ != MovementState::TraversingJunction || vehicle.activeConnector_ == nullptr || availableTime <= 0.0) {
        return 0.0;
    }

    const double pathLength = vehicle.activeConnector_->getLength();
    const double clearanceDistance = pathLength > 1e-6 ? vehicle.getLength() * 0.5 : 0.0;
    const double completionDistance = pathLength + clearanceDistance;
    if (completionDistance <= vehicle.junctionProgressMetres_ + 1e-9) {
        vehicle.completeJunctionTraversal(std::max(0.0, vehicle.junctionProgressMetres_ - pathLength));
        return 0.0;
    }

    const double subDt = std::min(availableTime, Vehicle::MAX_PHYSICS_SUBSTEP);
    double targetSpeed = std::max(1.0, std::min(vehicle.baseSpeed, vehicle.currentRoad->getSpeedLimit()) / std::max(1.0, vehicle.currentRoad->getCongestionLevel()));
    const Pose2D pose = vehicle.activeConnector_->sampleByDistance(vehicle.junctionProgressMetres_);
    const double curvature = std::fabs(pose.curvature);
    if (curvature > 1e-8) {
        const double curveSpeed = std::sqrt(vehicle.getMaxLateralAcceleration() / curvature);
        targetSpeed = std::min(targetSpeed, curveSpeed);
    }

    if (vehicle.currentSpeed < targetSpeed) {
        vehicle.currentSpeed = std::min(targetSpeed, vehicle.currentSpeed + vehicle.getAcceleration() * subDt);
    } else if (vehicle.currentSpeed > targetSpeed) {
        vehicle.currentSpeed = std::max(targetSpeed, vehicle.currentSpeed - vehicle.getDeceleration() * subDt);
    }
    vehicle.currentSpeed = std::max(vehicle.currentSpeed, 1e-3);

    const double remainingDistance = completionDistance - vehicle.junctionProgressMetres_;
    const double possibleDistance = vehicle.currentSpeed * subDt;
    double travelled = std::min(remainingDistance, possibleDistance);
    
    // Check if we need to yield to a vehicle already on the outgoing road
    if (vehicle.junctionOutgoingRoad_ != nullptr) {
        const auto& vehicles = vehicle.junctionOutgoingRoad_->getLane(vehicle.outgoingLaneIndex_).getVehicles();
        Vehicle* outgoingLeader = vehicles.empty() ? nullptr : vehicles.back();
        if (outgoingLeader != nullptr && outgoingLeader != &vehicle) {
            // Our virtual progress on the outgoing road is our junction progress minus the path length
            const double myVirtualProgress = vehicle.junctionProgressMetres_ - pathLength;
            const double gapToLeader = outgoingLeader->getProgressOnRoad() - myVirtualProgress - combinedHalfLength(vehicle, *outgoingLeader);
            const double requiredGap = std::max(vehicle.getMinGap(), outgoingLeader->getMinGap());
            if (gapToLeader < requiredGap) {
                // Not enough gap, limit travel!
                double allowedTravel = std::max(0.0, gapToLeader - requiredGap);
                if (allowedTravel < travelled) {
                    travelled = allowedTravel;
                    vehicle.currentSpeed = travelled > 1e-9 ? travelled / subDt : 0.0;
                }
            }
        }
    }

    bool constrainedByOccupant = false;
    if (vehicle.reservedIntersection_ != nullptr) {
        const double safeTravelled = vehicle.reservedIntersection_->limitTraversalAdvance(vehicle.getId(), vehicle.activeConnector_, vehicle.junctionProgressMetres_, travelled, vehicle.getLength(), vehicle.getWidth(), vehicle.getMinGap());
        constrainedByOccupant = safeTravelled + 1e-9 < travelled;
        travelled = safeTravelled;
        if (constrainedByOccupant) {
            vehicle.currentSpeed = travelled > 1e-9 ? travelled / subDt : 0.0;
        }
    }
    const double consumedTime = constrainedByOccupant ? subDt : (vehicle.currentSpeed > 1e-9 ? travelled / vehicle.currentSpeed : subDt);
    vehicle.junctionProgressMetres_ += travelled;
    if (vehicle.reservedIntersection_ != nullptr) {
        vehicle.reservedIntersection_->updateReservationProgress(vehicle.getId(), vehicle.junctionProgressMetres_);
    }

    if (!vehicle.hasClearedIncomingRoad() && vehicle.junctionProgressMetres_ >= vehicle.getLength()) {
        if (vehicle.currentRoad != nullptr && vehicle.incomingLaneIndex_ >= 0) {
            vehicle.currentRoad->getLane(vehicle.incomingLaneIndex_).removeVehicle(&vehicle);
        }
        vehicle.setClearedIncomingRoad(true);
    }

    if (vehicle.junctionProgressMetres_ + 1e-9 >= completionDistance) {
        vehicle.completeJunctionTraversal(std::max(0.0, vehicle.junctionProgressMetres_ - pathLength));
    }
    return consumedTime;
}
