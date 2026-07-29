#ifndef PEDESTRIAN_H
#define PEDESTRIAN_H

#include <cstddef>
#include <vector>

#include "PedestrianRoute.h"

class Crosswalk;
class Road;

enum class PedestrianState {
    Walking,
    WaitingToCross,
    Crossing,
    Arrived
};

class Pedestrian {
public:
    static constexpr double DEFAULT_WALKING_SPEED = 1.4;

    Pedestrian(
        int id,
        double walkingSpeedMetresPerSecond,
        std::vector<PedestrianRouteSegment> route);
    ~Pedestrian();

    Pedestrian(const Pedestrian&) = delete;
    Pedestrian& operator=(const Pedestrian&) = delete;

    bool isValid() const;
    void update(double dt);
    void notifyCrossingGranted();
    void detachFromCrosswalk();

    int getId() const;
    double getWalkingSpeed() const;
    PedestrianState getState() const;
    Pose2D getPose() const;
    bool hasArrived() const;

    std::size_t getCurrentSegmentIndex() const;
    double getProgressOnSegmentMetres() const;
    const PedestrianRouteSegment* getCurrentSegment() const;
    Crosswalk* getCurrentCrosswalk() const;
    const Road* getReferenceRoad() const;

    double getTotalWalkingTimeSeconds() const;
    double getTotalWaitingTimeSeconds() const;
    double getTotalCrossingTimeSeconds() const;
    double getCurrentCrosswalkWaitSeconds() const;

private:
    void advanceAlongCurrentSegment(double availableTime);
    void completeCurrentSegment();
    void arriveAtCrosswalk();
    void finishJourney();

    int id_;
    double walkingSpeedMetresPerSecond_;
    PedestrianState state_ = PedestrianState::Walking;
    std::vector<PedestrianRouteSegment> route_;
    std::size_t currentSegmentIndex_ = 0;
    double progressOnSegmentMetres_ = 0.0;
    Pose2D pose_;
    double totalWalkingTimeSeconds_ = 0.0;
    double totalWaitingTimeSeconds_ = 0.0;
    double totalCrossingTimeSeconds_ = 0.0;
    double currentCrosswalkWaitSeconds_ = 0.0;
    bool crossingRequestSubmitted_ = false;
    bool valid_ = false;
};

#endif
