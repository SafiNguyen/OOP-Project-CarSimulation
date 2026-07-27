#ifndef JUNCTION_CONNECTOR_H
#define JUNCTION_CONNECTOR_H

#include <cstddef>
#include <memory>

#include "Geometry.h"
#include "LaneMapping.h"

class MotionPath;
class Road;

struct ConnectorKey {
    int incomingRoadId = 0;
    int incomingLane = 0;
    int outgoingRoadId = 0;
    int outgoingLane = 0;

    bool operator==(const ConnectorKey& other) const {
        return incomingRoadId == other.incomingRoadId &&
               incomingLane == other.incomingLane &&
               outgoingRoadId == other.outgoingRoadId &&
               outgoingLane == other.outgoingLane;
    }
};

struct ConnectorKeyHash {
    std::size_t operator()(const ConnectorKey& key) const noexcept;
};

class JunctionConnector {
private:
    ConnectorKey key_;
    const Road* incomingRoad_;
    const Road* outgoingRoad_;
    MovementType movement_;
    Vec2 startTangent_;
    Vec2 endTangent_;
    double outgoingMetresPerWorldUnit_;
    std::shared_ptr<const MotionPath> path_;

public:
    JunctionConnector(ConnectorKey key,
                      const Road* incomingRoad,
                      const Road* outgoingRoad,
                      MovementType movement,
                      Vec2 startTangent,
                      Vec2 endTangent,
                      double outgoingMetresPerWorldUnit,
                      std::shared_ptr<const MotionPath> path);

    const ConnectorKey& getKey() const { return key_; }
    const Road* getIncomingRoad() const { return incomingRoad_; }
    const Road* getOutgoingRoad() const { return outgoingRoad_; }
    int getIncomingLane() const { return key_.incomingLane; }
    int getOutgoingLane() const { return key_.outgoingLane; }
    MovementType getMovementType() const { return movement_; }
    Vec2 getStartTangent() const { return startTangent_; }
    Vec2 getEndTangent() const { return endTangent_; }
    double getLength() const;
    Pose2D sampleByDistance(double distanceMetres) const;
};

#endif
