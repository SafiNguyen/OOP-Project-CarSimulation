#include "Pedestrian.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "Crosswalk.h"

Pedestrian::Pedestrian(
    int id,
    double walkingSpeedMetresPerSecond,
    std::vector<PedestrianRouteSegment> route)
    : id_(id),
      walkingSpeedMetresPerSecond_(
          walkingSpeedMetresPerSecond),
      route_(std::move(route)) {
    const bool hasCrosswalk =
        std::any_of(
            route_.begin(),
            route_.end(),
            [](const PedestrianRouteSegment& segment) {
                return segment.kind ==
                       PedestrianSegmentKind::Crosswalk;
            });
    valid_ =
        id_ >= 0 &&
        std::isfinite(walkingSpeedMetresPerSecond_) &&
        walkingSpeedMetresPerSecond_ > 0.0 &&
        route_.size() >= 3 &&
        route_.front().kind ==
            PedestrianSegmentKind::Sidewalk &&
        route_.back().kind ==
            PedestrianSegmentKind::Sidewalk &&
        hasCrosswalk &&
        std::all_of(
            route_.begin(),
            route_.end(),
            [](const PedestrianRouteSegment& segment) {
                return segment.isValid();
            });
    if (valid_) {
        pose_ = route_.front().sample(0.0);
    } else {
        state_ = PedestrianState::Arrived;
    }
}

Pedestrian::~Pedestrian() {
    detachFromCrosswalk();
}

bool Pedestrian::isValid() const {
    return valid_;
}

void Pedestrian::update(double dt) {
    if (!valid_ ||
        state_ == PedestrianState::Arrived ||
        !std::isfinite(dt) ||
        dt <= 0.0) {
        return;
    }

    if (state_ == PedestrianState::WaitingToCross) {
        currentCrosswalkWaitSeconds_ += dt;
        totalWaitingTimeSeconds_ += dt;
        return;
    }
    advanceAlongCurrentSegment(dt);
}

void Pedestrian::advanceAlongCurrentSegment(
    double availableTime) {
    double remainingTime = availableTime;
    constexpr double epsilon = 1e-9;
    while (remainingTime > epsilon &&
           state_ != PedestrianState::Arrived &&
           state_ != PedestrianState::WaitingToCross) {
        const PedestrianRouteSegment* segment =
            getCurrentSegment();
        if (segment == nullptr) {
            finishJourney();
            return;
        }

        const double remainingDistance = std::max(
            0.0,
            segment->lengthMetres -
                progressOnSegmentMetres_);
        const double timeToFinish =
            remainingDistance /
            walkingSpeedMetresPerSecond_;
        const double consumedTime =
            std::min(remainingTime, timeToFinish);
        progressOnSegmentMetres_ = std::min(
            segment->lengthMetres,
            progressOnSegmentMetres_ +
                walkingSpeedMetresPerSecond_ *
                    consumedTime);
        pose_ = segment->sample(
            progressOnSegmentMetres_);

        if (state_ == PedestrianState::Crossing) {
            totalCrossingTimeSeconds_ += consumedTime;
        } else {
            totalWalkingTimeSeconds_ += consumedTime;
        }
        remainingTime -= consumedTime;

        if (progressOnSegmentMetres_ +
                epsilon >=
            segment->lengthMetres) {
            completeCurrentSegment();
            continue;
        }
        break;
    }
}

void Pedestrian::completeCurrentSegment() {
    const PedestrianRouteSegment* completed =
        getCurrentSegment();
    if (completed != nullptr &&
        completed->kind ==
            PedestrianSegmentKind::Crosswalk &&
        completed->crosswalk != nullptr) {
        completed->crosswalk->notifyExited(*this);
        crossingRequestSubmitted_ = false;
        currentCrosswalkWaitSeconds_ = 0.0;
    }

    ++currentSegmentIndex_;
    progressOnSegmentMetres_ = 0.0;
    if (currentSegmentIndex_ >= route_.size()) {
        finishJourney();
        return;
    }

    const PedestrianRouteSegment& next =
        route_[currentSegmentIndex_];
    pose_ = next.sample(0.0);
    if (next.kind ==
        PedestrianSegmentKind::Crosswalk) {
        arriveAtCrosswalk();
    } else {
        state_ = PedestrianState::Walking;
    }
}

void Pedestrian::arriveAtCrosswalk() {
    Crosswalk* crosswalk = getCurrentCrosswalk();
    if (crosswalk == nullptr) {
        valid_ = false;
        finishJourney();
        return;
    }
    state_ = PedestrianState::WaitingToCross;
    currentCrosswalkWaitSeconds_ = 0.0;
    if (!crossingRequestSubmitted_) {
        crosswalk->requestEntry(*this);
        crossingRequestSubmitted_ = true;
    }
}

void Pedestrian::notifyCrossingGranted() {
    Crosswalk* crosswalk = getCurrentCrosswalk();
    if (state_ != PedestrianState::WaitingToCross ||
        crosswalk == nullptr ||
        !crosswalk->canStartCrossing(*this)) {
        return;
    }
    crosswalk->notifyEntered(*this);
    state_ = PedestrianState::Crossing;
}

void Pedestrian::detachFromCrosswalk() {
    Crosswalk* crosswalk = getCurrentCrosswalk();
    if (crosswalk != nullptr &&
        (crossingRequestSubmitted_ ||
         state_ == PedestrianState::Crossing)) {
        crosswalk->cancelRequest(id_);
    }
    crossingRequestSubmitted_ = false;
}

void Pedestrian::finishJourney() {
    state_ = PedestrianState::Arrived;
    currentSegmentIndex_ = route_.size();
    progressOnSegmentMetres_ = 0.0;
}

int Pedestrian::getId() const {
    return id_;
}

double Pedestrian::getWalkingSpeed() const {
    return walkingSpeedMetresPerSecond_;
}

PedestrianState Pedestrian::getState() const {
    return state_;
}

Pose2D Pedestrian::getPose() const {
    return pose_;
}

bool Pedestrian::hasArrived() const {
    return state_ == PedestrianState::Arrived;
}

std::size_t Pedestrian::getCurrentSegmentIndex() const {
    return currentSegmentIndex_;
}

double Pedestrian::getProgressOnSegmentMetres() const {
    return progressOnSegmentMetres_;
}

const PedestrianRouteSegment*
Pedestrian::getCurrentSegment() const {
    return currentSegmentIndex_ < route_.size()
        ? &route_[currentSegmentIndex_]
        : nullptr;
}

Crosswalk* Pedestrian::getCurrentCrosswalk() const {
    const PedestrianRouteSegment* segment =
        getCurrentSegment();
    return segment != nullptr
        ? segment->crosswalk
        : nullptr;
}

const Road* Pedestrian::getReferenceRoad() const {
    const PedestrianRouteSegment* segment =
        getCurrentSegment();
    return segment != nullptr
        ? segment->referenceRoad
        : nullptr;
}

double Pedestrian::getTotalWalkingTimeSeconds() const {
    return totalWalkingTimeSeconds_;
}

double Pedestrian::getTotalWaitingTimeSeconds() const {
    return totalWaitingTimeSeconds_;
}

double Pedestrian::getTotalCrossingTimeSeconds() const {
    return totalCrossingTimeSeconds_;
}

double Pedestrian::getCurrentCrosswalkWaitSeconds() const {
    return currentCrosswalkWaitSeconds_;
}
