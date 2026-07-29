#ifndef LANE_MAPPING_H
#define LANE_MAPPING_H

class Road;

enum class MovementType {
    Straight,
    Left,
    Right,
    UTurn
};

struct LaneMapping {
    bool valid = false;
    int incomingLane = 0;
    int outgoingLane = 0;
    MovementType movement = MovementType::Straight;
};

class TurnLanePolicy {
public:
    static constexpr double STRAIGHT_THRESHOLD_RADIANS =
        0.5235987755982988; // 30 degrees
    static constexpr double UTURN_THRESHOLD_RADIANS =
        2.6179938779914944; // 150 degrees

    static MovementType classify(const Road& incoming,
                                 const Road& outgoing);
    static LaneMapping map(const Road& incoming,
                           int currentIncomingLane,
                           const Road& outgoing,
                           bool allowUTurn = true);
    static LaneMapping mapFromCurrentLane(
        const Road& incoming,
        int currentIncomingLane,
        const Road& outgoing,
        bool allowUTurn = true);
};

#endif
