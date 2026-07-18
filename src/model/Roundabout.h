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
    double radius; // radius of the roundabout in km (affects transition time)

public:
    Roundabout(int id, double x = 0.0, double y = 0.0, double radius = 0.02)
        : Intersection(id, x, y), radius(radius) {}

    double getRadius() const { return radius; }

    /// Roundabouts do not use traffic lights — vehicles yield instead.
    /// Override mustStopForRoad to always return false.
    bool isRoundabout() const override { return true; }
};

#endif
