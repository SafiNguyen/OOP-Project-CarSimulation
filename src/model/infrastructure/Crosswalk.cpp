#include "Crosswalk.h"

#include <algorithm>
#include <cmath>

#include "Intersection.h"
#include "Pedestrian.h"
#include "Road.h"
#include "RoadGeometry.h"

namespace {

constexpr double EMERGENCY_PATH_CLEARANCE_METRES = 0.75;

}

Crosswalk::Crosswalk(int id,
                     Intersection* intersection,
                     Road* incomingRoad,
                     double widthMetres,
                     CrosswalkTiming timing)
    : id_(id),
      intersection_(intersection),
      incomingRoad_(incomingRoad),
      widthMetres_(widthMetres),
      timing_(timing) {
}

bool Crosswalk::isValid() const {
    return id_ >= 0 &&
           intersection_ != nullptr &&
           incomingRoad_ != nullptr &&
           incomingRoad_->getEnd() ==
               intersection_ &&
           intersection_->hasTrafficLights() &&
           std::isfinite(widthMetres_) &&
           widthMetres_ > 0.0 &&
           incomingRoad_->getDistance() >
               widthMetres_ +
                   JUNCTION_EDGE_GAP_METRES +
                   STOP_LINE_GAP_METRES &&
           std::isfinite(
               timing_.minimumWaitSeconds) &&
           timing_.minimumWaitSeconds >= 0.0 &&
           std::isfinite(
               timing_.walkDurationSeconds) &&
           timing_.walkDurationSeconds > 0.0 &&
           std::isfinite(
               timing_.
                   designWalkingSpeedMetresPerSecond) &&
           timing_.
                   designWalkingSpeedMetresPerSecond >
               0.0 &&
           std::isfinite(
               timing_.clearanceBufferSeconds) &&
           timing_.clearanceBufferSeconds >= 0.0;
}

int Crosswalk::getId() const {
    return id_;
}

Intersection* Crosswalk::getIntersection() const {
    return intersection_;
}

Road* Crosswalk::getIncomingRoad() const {
    return incomingRoad_;
}

Road* Crosswalk::getReverseRoad() const {
    return incomingRoad_ != nullptr
        ? incomingRoad_->getReverseRoad()
        : nullptr;
}

double Crosswalk::getWidthMetres() const {
    return widthMetres_;
}

const CrosswalkTiming& Crosswalk::getTiming() const {
    return timing_;
}

double Crosswalk::getEndProgressMetres() const {
    if (incomingRoad_ == nullptr) return 0.0;
    return std::max(
        0.0,
        incomingRoad_->getDistance() -
            JUNCTION_EDGE_GAP_METRES);
}

double Crosswalk::getStartProgressMetres() const {
    return std::max(
        0.0,
        getEndProgressMetres() - widthMetres_);
}

double Crosswalk::getCentreProgressMetres() const {
    return (getStartProgressMetres() +
            getEndProgressMetres()) * 0.5;
}

double Crosswalk::getReverseCentreProgressMetres() const {
    Road* reverse = getReverseRoad();
    if (reverse == nullptr || incomingRoad_ == nullptr ||
        incomingRoad_->getDistance() <= 1e-9) {
        return getCentreProgressMetres();
    }
    const double forwardRatio =
        getCentreProgressMetres() /
        incomingRoad_->getDistance();
    return std::clamp(
        (1.0 - forwardRatio) * reverse->getDistance(),
        0.0,
        reverse->getDistance());
}

double Crosswalk::getStopLineProgressMetres() const {
    return std::max(
        0.0,
        getStartProgressMetres() -
            STOP_LINE_GAP_METRES);
}

Vec2 Crosswalk::getSideAPosition() const {
    if (incomingRoad_ == nullptr) return {};
    return RoadGeometry::sampleSidewalk(
        *incomingRoad_,
        true,
        getCentreProgressMetres());
}

Vec2 Crosswalk::getSideBPosition() const {
    if (incomingRoad_ == nullptr) return {};
    Road* reverse = getReverseRoad();
    if (reverse != nullptr) {
        return RoadGeometry::sampleSidewalk(
            *reverse,
            true,
            getReverseCentreProgressMetres());
    }
    return RoadGeometry::sampleSidewalk(
        *incomingRoad_,
        false,
        getCentreProgressMetres());
}

double Crosswalk::getCrossingLengthMetres() const {
    if (intersection_ == nullptr) return 0.0;
    return distance(
        getSideAPosition(),
        getSideBPosition()) *
        RoadGeometry::metresPerWorldUnit(*intersection_);
}

double Crosswalk::getClearanceDurationSeconds() const {
    const double speed = std::max(
        1e-6,
        timing_.designWalkingSpeedMetresPerSecond);
    return getCrossingLengthMetres() / speed +
           timing_.clearanceBufferSeconds;
}

bool Crosswalk::containsPedestrian(
    const std::vector<Pedestrian*>& pedestrians,
    int pedestrianId) {
    return std::any_of(
        pedestrians.begin(),
        pedestrians.end(),
        [pedestrianId](const Pedestrian* pedestrian) {
            return pedestrian != nullptr &&
                   pedestrian->getId() == pedestrianId;
        });
}

void Crosswalk::erasePedestrian(
    std::vector<Pedestrian*>& pedestrians,
    int pedestrianId) {
    pedestrians.erase(
        std::remove_if(
            pedestrians.begin(),
            pedestrians.end(),
            [pedestrianId](const Pedestrian* pedestrian) {
                return pedestrian == nullptr ||
                       pedestrian->getId() == pedestrianId;
            }),
        pedestrians.end());
}

void Crosswalk::requestEntry(Pedestrian& pedestrian) {
    if (containsPedestrian(waiting_, pedestrian.getId()) ||
        containsPedestrian(occupants_, pedestrian.getId())) {
        return;
    }
    waiting_.push_back(&pedestrian);
}

void Crosswalk::cancelRequest(int pedestrianId) {
    erasePedestrian(waiting_, pedestrianId);
    erasePedestrian(occupants_, pedestrianId);
}

void Crosswalk::notifyEntered(Pedestrian& pedestrian) {
    erasePedestrian(waiting_, pedestrian.getId());
    if (!containsPedestrian(
            occupants_, pedestrian.getId())) {
        occupants_.push_back(&pedestrian);
    }
}

void Crosswalk::notifyExited(Pedestrian& pedestrian) {
    erasePedestrian(occupants_, pedestrian.getId());
}

void Crosswalk::grantEligiblePedestrians() {
    if (signalState_ != PedestrianSignalState::Walk) {
        return;
    }
    const std::vector<Pedestrian*> candidates = waiting_;
    for (Pedestrian* pedestrian : candidates) {
        if (pedestrian != nullptr &&
            canStartCrossing(*pedestrian)) {
            pedestrian->notifyCrossingGranted();
        }
    }
}

void Crosswalk::synchronizeSignalState(
    PedestrianSignalState state) {
    signalState_ = state;
}

PedestrianSignalState Crosswalk::getSignalState() const {
    return signalState_;
}

bool Crosswalk::canStartCrossing(
    const Pedestrian& pedestrian) const {
    const EmergencyApproach* emergency =
        intersection_ != nullptr
            ? intersection_->getEmergencyApproach()
            : nullptr;
    return signalState_ == PedestrianSignalState::Walk &&
           (emergency == nullptr ||
            !isAffectedBy(*emergency)) &&
           pedestrian.getState() ==
               PedestrianState::WaitingToCross &&
           pedestrian.getCurrentCrosswalk() == this &&
           pedestrian.getCurrentCrosswalkWaitSeconds() +
                   1e-9 >=
               timing_.minimumWaitSeconds &&
           containsPedestrian(
               waiting_, pedestrian.getId());
}

bool Crosswalk::hasWaitingPedestrians() const {
    return !waiting_.empty();
}

bool Crosswalk::hasEligibleRequest() const {
    return std::any_of(
        waiting_.begin(),
        waiting_.end(),
        [this](const Pedestrian* pedestrian) {
            return pedestrian != nullptr &&
                   pedestrian->getState() ==
                       PedestrianState::WaitingToCross &&
                   pedestrian->getCurrentCrosswalkWaitSeconds() +
                           1e-9 >=
                       timing_.minimumWaitSeconds;
        });
}

bool Crosswalk::isOccupied() const {
    return !occupants_.empty();
}

double Crosswalk::getOldestRequestAgeSeconds() const {
    double oldest = 0.0;
    for (const Pedestrian* pedestrian : waiting_) {
        if (pedestrian != nullptr) {
            oldest = std::max(
                oldest,
                pedestrian->
                    getCurrentCrosswalkWaitSeconds());
        }
    }
    return oldest;
}

bool Crosswalk::isAffectedBy(
    const EmergencyApproach& approach) const {
    if (!approach.isValid() ||
        incomingRoad_ == nullptr) {
        return false;
    }
    if (incomingRoad_ == approach.incomingRoad) {
        return true;
    }
    return approach.outgoingRoad != nullptr &&
           getReverseRoad() == approach.outgoingRoad;
}

Crosswalk::ConflictZone Crosswalk::getConflictZone(
    const Pedestrian& pedestrian,
    const EmergencyApproach& approach) const {
    const PedestrianRouteSegment* segment =
        pedestrian.getCurrentSegment();
    if (segment == nullptr ||
        segment->kind !=
            PedestrianSegmentKind::Crosswalk ||
        segment->crosswalk != this ||
        segment->lengthMetres <= 1e-9 ||
        !isAffectedBy(approach)) {
        return {};
    }

    const Road* emergencyRoad = nullptr;
    int emergencyLane = -1;
    double roadProgress = 0.0;
    if (incomingRoad_ == approach.incomingRoad) {
        emergencyRoad = approach.incomingRoad;
        emergencyLane = approach.incomingLane;
        roadProgress = getCentreProgressMetres();
    } else {
        emergencyRoad = approach.outgoingRoad;
        emergencyLane = approach.outgoingLane;
        roadProgress = getReverseCentreProgressMetres();
    }
    if (emergencyRoad == nullptr ||
        emergencyLane < 0 ||
        emergencyLane >=
            emergencyRoad->getLaneCount()) {
        return {};
    }

    const Vec2 crossingVector =
        segment->end - segment->start;
    const double squaredLength =
        dot(crossingVector, crossingVector);
    if (squaredLength <= 1e-12) {
        return {};
    }
    const Vec2 laneCentre =
        RoadGeometry::sampleLane(
            *emergencyRoad,
            emergencyLane,
            roadProgress).position;
    const double ratio = std::clamp(
        dot(laneCentre - segment->start,
            crossingVector) /
            squaredLength,
        0.0,
        1.0);
    const double centreMetres =
        ratio * segment->lengthMetres;
    const double halfClearance =
        approach.vehicleWidthMetres * 0.5 +
        EMERGENCY_PATH_CLEARANCE_METRES;
    return {
        true,
        std::max(0.0, centreMetres - halfClearance),
        std::min(
            segment->lengthMetres,
            centreMetres + halfClearance)
    };
}

EmergencyCrossingGuidance
Crosswalk::getEmergencyGuidance(
    const Pedestrian& pedestrian) const {
    const EmergencyApproach* approach =
        intersection_ != nullptr
            ? intersection_->getEmergencyApproach()
            : nullptr;
    if (approach == nullptr ||
        !isAffectedBy(*approach)) {
        return EmergencyCrossingGuidance::None;
    }
    if (pedestrian.getState() ==
            PedestrianState::WaitingToCross) {
        return EmergencyCrossingGuidance::
            HoldBeforeVehiclePath;
    }
    if (pedestrian.getState() !=
            PedestrianState::Crossing) {
        return EmergencyCrossingGuidance::None;
    }

    const ConflictZone zone =
        getConflictZone(pedestrian, *approach);
    if (!zone.valid) {
        return EmergencyCrossingGuidance::
            ExpediteOutOfVehiclePath;
    }
    const double progress =
        pedestrian.getProgressOnSegmentMetres();
    if (progress + 1e-9 < zone.startMetres) {
        return EmergencyCrossingGuidance::
            HoldBeforeVehiclePath;
    }
    if (progress <= zone.endMetres + 1e-9) {
        return EmergencyCrossingGuidance::
            ExpediteOutOfVehiclePath;
    }
    return EmergencyCrossingGuidance::None;
}

bool Crosswalk::isEmergencyPathClear(
    const EmergencyApproach& approach) const {
    if (!isAffectedBy(approach)) {
        return true;
    }
    for (const Pedestrian* pedestrian : occupants_) {
        if (pedestrian == nullptr ||
            pedestrian->getState() !=
                PedestrianState::Crossing) {
            continue;
        }
        const ConflictZone zone =
            getConflictZone(*pedestrian, approach);
        if (!zone.valid) {
            return false;
        }
        const double progress =
            pedestrian->
                getProgressOnSegmentMetres();
        if (progress + 1e-9 >= zone.startMetres &&
            progress <= zone.endMetres + 1e-9) {
            return false;
        }
    }
    return true;
}
