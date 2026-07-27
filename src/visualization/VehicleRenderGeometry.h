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
    return {
        std::max(
            2.0f,
            visualization.metresToScreenPixels(
                vehicle.getLength(), road)),
        std::max(
            1.25f,
            visualization.metresToScreenPixels(
                vehicle.getWidth(), road))
    };
}

#endif
