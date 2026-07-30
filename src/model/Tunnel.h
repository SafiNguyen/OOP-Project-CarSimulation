#ifndef TUNNEL_H
#define TUNNEL_H

#include "Road.h"

/**
 * Tunnel — A specialized Road subclass representing an underground passage.
 *
 * Additional constraints:
 * - heightLimit: vehicles taller than this cannot enter.
 * - Tunnels typically enforce a reduced speed limit for safety.
 *
 * OOP: Inheritance (Tunnel IS-A Road).
 */
class Tunnel : public Road {
private:
    double heightLimit; // metres — max vehicle height allowed

public:
    Tunnel(int id, const std::string& name, Intersection* start, Intersection* end,
           double distance, double speedLimit,
           double congestionLevel = 1.0, int laneCount = 1,
           double heightLimit = 3.5)
        : Road(id, name, start, end, distance, speedLimit, congestionLevel, laneCount),
          heightLimit(heightLimit) {}

    double getHeightLimit() const { return heightLimit; }

    bool isTunnel() const override { return true; }
};

#endif
