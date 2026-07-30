#ifndef ROUNDABOUT_H
#define ROUNDABOUT_H

#include "Intersection.h"

/**
 * Roundabout — A specialized Intersection subclass that models a
 * traffic roundabout (bùng binh).
 *
 * Key differences from a normal Intersection:
 * - No traffic lights: vehicles yield to traffic already in the circle.
 * - Vehicles entering the roundabout slow down.
 *
 * OOP: Inheritance (Roundabout IS-A Intersection),
 *      Polymorphism (overrides traffic light behavior).
 */
class Roundabout : public Intersection {
private:
    double radiusMetres_;

protected:
    std::shared_ptr<const JunctionConnector> createConnector(
        const Road& incoming,
        int incomingLane,
        const Road& outgoing,
        int outgoingLane) const override;

public:
    Roundabout(int id,
               double x = 0.0,
               double y = 0.0,
               double radiusMetres = 20.0);

    double getRadius() const { return radiusMetres_; }
    double getTraversalRadiusMetres() const override {
        return radiusMetres_;
    }

    /// Roundabouts do not use traffic lights — vehicles yield instead.
    /// Override mustStopForRoad to always return false.
    bool isRoundabout() const override { return true; }
    bool canEnterMovement(
        int vehicleId,
        const std::shared_ptr<const JunctionConnector>& connector,
        double requiredGapMetres,
        double vehicleLengthMetres = 4.5,
        double vehicleWidthMetres = 1.8) const override;
};

#endif
