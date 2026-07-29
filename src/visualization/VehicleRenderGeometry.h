#ifndef VEHICLE_RENDER_GEOMETRY_H
#define VEHICLE_RENDER_GEOMETRY_H

#include "Vehicle.h"
#include "visualization/VisualizationEngine.h"

struct VehicleScreenSize {
    float lengthPixels = 0.0f;
    float widthPixels = 0.0f;
};

inline VehicleScreenSize getVehicleScreenSize(
    const Vehicle& vehicle,
    const VisualizationEngine& visualization) {
    const Road* road = vehicle.getCurrentRoad();
    // When the vehicle has no road (rare edge case during junction
    // traversal cleanup), fall back to sensible default pixel sizes.
    if (road == nullptr) {
        return {6.0f, 3.0f};
    }
    return {
        visualization.metresToScreenPixels(
            vehicle.getLength(), road),
        visualization.metresToScreenPixels(
            vehicle.getWidth(), road)
    };
}

#endif
