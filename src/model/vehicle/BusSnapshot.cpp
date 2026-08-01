#include "Bus.h"
#include "BusService.h"
#include "Graph.h"
#include "Road.h"

void Bus::captureSnapshot(VehicleSnapshot& snap,
                          const Graph& graph) const {
    Vehicle::captureSnapshot(snap, graph);

    snap.dwellTime = dwellTime;
    snap.dwellTimer = dwellTimer;
    snap.nextStopId = nextStop != nullptr ? nextStop->getId() : -1;
    snap.nextStopPos = nextStopPos;
    snap.fleetCode = fleetCode_;
    snap.serviceId = service_ != nullptr ? service_->getId() : -1;
    snap.originStationId =
        originStation_ != nullptr ? originStation_->getId() : -1;
    snap.destinationStationId =
        destinationStation_ != nullptr
            ? destinationStation_->getId()
            : -1;

    snap.assignedStopIds.clear();
    snap.assignedStopIds.reserve(assignedStops_.size());
    for (const BusStop* stop : assignedStops_) {
        snap.assignedStopIds.push_back(
            stop != nullptr ? stop->getId() : -1);
    }
    snap.assignedStopRouteIndices = assignedStopRouteIndices_;
    snap.scheduledDepartureTime = scheduledDepartureTime_;
    snap.scheduledStopIndex = scheduledStopIndex_;
    snap.servedStopIds = servedStopIds_;
    snap.missedStopIds = missedStopIds_;
    snap.tripState = static_cast<int>(tripState_);
    snap.departureSlotHeld = departureSlotHeld_;
}

void Bus::restoreSnapshot(const VehicleSnapshot& snap,
                          Graph& graph) {
    Vehicle::restoreSnapshot(snap, graph);

    dwellTime = snap.dwellTime;
    dwellTimer = snap.dwellTimer;
    nextStop = nullptr;
    nextStopPos = snap.nextStopPos;
    fleetCode_ = snap.fleetCode;
    service_ = snap.serviceId >= 0
        ? graph.getBusService(snap.serviceId)
        : nullptr;
    originStation_ = snap.originStationId >= 0
        ? graph.getBusStation(snap.originStationId)
        : nullptr;
    destinationStation_ = snap.destinationStationId >= 0
        ? graph.getBusStation(snap.destinationStationId)
        : nullptr;

    assignedStops_.clear();
    assignedStops_.reserve(snap.assignedStopIds.size());
    for (const int stopId : snap.assignedStopIds) {
        if (stopId < 0 || currentRoad == nullptr) {
            assignedStops_.push_back(nullptr);
            continue;
        }
        assignedStops_.push_back(
            currentRoad->findBusStopById(stopId));
    }
    assignedStopRouteIndices_ = snap.assignedStopRouteIndices;
    scheduledDepartureTime_ = snap.scheduledDepartureTime;
    scheduledStopIndex_ = snap.scheduledStopIndex;
    servedStopIds_ = snap.servedStopIds;
    missedStopIds_ = snap.missedStopIds;
    tripState_ = static_cast<BusTripState>(snap.tripState);
    departureSlotHeld_ = snap.departureSlotHeld;

    // Restore nextStop by resolving its id against the current road.
    if (snap.nextStopId >= 0 && currentRoad != nullptr) {
        nextStop = currentRoad->findBusStopById(snap.nextStopId);
        if (nextStop == nullptr ||
            nextStop->getRoad() != currentRoad) {
            nextStop = nullptr;
            nextStopPos = -1.0;
        }
    }
}