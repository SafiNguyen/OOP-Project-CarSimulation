#include "Vehicle.h"
#include "Road.h"
#include "Lane.h"
#include "Intersection.h"
#include "RoadGeometry.h"
#include "VehicleMath.h"
using namespace VehicleMath;
#include "JunctionConnector.h"
#include "PointOfInterest.h"
#include "SpawnPoint.h"

void Vehicle::setMergingFromPOI(bool merging, double offset, int laneIdx) {
    if (!merging && isMergingFromPOI && currentRoad != nullptr) {
        currentRoad->removeMergingVehicle(this);
    }
    isMergingFromPOI = merging;
    mergeProgressOffset = offset;
    mergeLaneIndex = laneIdx;
    if (merging) {
        poiMergePhase_ =
            PoiMergePhase::ApproachingYieldLine;
        poiAnimationTimer =
            getPoiMergePhaseDuration(
                poiMergePhase_);
        spawnLifecycleState_ =
            SpawnLifecycleState::Merging;
    } else {
        poiMergePhase_ = PoiMergePhase::None;
        poiAnimationTimer = 0.0;
    }
}
void Vehicle::setEnteringPOI(bool entering) {
    isEnteringPOI = entering;
    if (entering) {
        poiAnimationTimer = poiAnimationDuration;
    }
}
