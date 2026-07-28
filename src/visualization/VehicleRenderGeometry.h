#ifndef VEHICLE_RENDER_GEOMETRY_H
#define VEHICLE_RENDER_GEOMETRY_H

#include <algorithm>

#include "model/Vehicle.h"
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
        std::clamp(
            visualization.metresToScreenPixels(
                vehicle.getLength(), road),
            3.0f,
            24.0f),
        std::clamp(
            visualization.metresToScreenPixels(
                vehicle.getWidth(), road),
            1.5f,
            12.0f)
    };
}

#endif
