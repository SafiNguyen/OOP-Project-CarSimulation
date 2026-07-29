#ifndef BRIDGE_H
#define BRIDGE_H

#include "Road.h"

/**
 * Bridge — A specialized Road subclass representing an elevated road
 * structure that crosses over obstacles (rivers, valleys, other roads).
 *
 * Additional constraints:
 * - heightLimit: vehicles taller than this cannot use the bridge.
 * - weightLimit: vehicles heavier than this cannot use the bridge.
 *
 * OOP: Inheritance (Bridge IS-A Road).
 */
class Bridge : public Road {
private:
    double heightLimit; // metres — max vehicle height allowed
    double weightLimit; // tonnes — max vehicle weight allowed

public:
    Bridge(int id, const std::string& name, Intersection* start, Intersection* end,
           double distance, double speedLimit,
           double congestionLevel = 1.0, int laneCount = 1,
           double heightLimit = 4.5, double weightLimit = 30.0,
           double laneWidthMetres = 3.5)
        : Road(id, name, start, end, distance, speedLimit, congestionLevel,
               laneCount, laneWidthMetres),
          heightLimit(heightLimit), weightLimit(weightLimit) {}

    double getHeightLimit() const { return heightLimit; }
    double getWeightLimit() const { return weightLimit; }

    bool isBridge() const override { return true; }
};

#endif
