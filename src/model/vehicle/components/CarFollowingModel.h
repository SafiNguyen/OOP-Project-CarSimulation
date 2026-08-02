#ifndef CAR_FOLLOWING_MODEL_H
#define CAR_FOLLOWING_MODEL_H

class Vehicle;
class Road;

class CarFollowingModel {
public:
    struct FollowDecision {
        double targetSpeed = 0.0;
        double gapToLeader = 0.0;
        bool hasLeader = false;
    };

    FollowDecision evaluate(Vehicle& vehicle, Road* road, double freeFlowSpeed, double minGap, double desiredGap);
};

#endif
