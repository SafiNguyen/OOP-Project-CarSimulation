#ifndef VEHICLE_RENDER_GEOMETRY_H
#define VEHICLE_RENDER_GEOMETRY_H

#include <algorithm>

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

inline VehicleScreenSize getVehicleVisualScreenSize(
    const Vehicle& vehicle,
    const VisualizationEngine& visualization) {
    VehicleScreenSize size =
        getVehicleScreenSize(vehicle, visualization);
    float minimumLength = 14.0f;
    switch (vehicle.getVehicleKind()) {
        case VehicleKind::Bus:
            minimumLength = 20.0f;
            break;
        case VehicleKind::Motorbike:
            minimumLength = 11.0f;
            break;
        case VehicleKind::Emergency:
            minimumLength = 17.0f;
            break;
        case VehicleKind::Car:
        default:
            break;
    }

    // Keep the physical aspect ratio while guaranteeing that detailed
    // top-down art remains legible on a full-map view. This is visual only;
    // collision, following distance and lane occupancy retain real sizes.
    const float scale =
        size.lengthPixels > 0.0f
            ? std::max(
                  1.0f,
                  minimumLength / size.lengthPixels)
            : 1.0f;
    size.lengthPixels *= scale;
    size.widthPixels *= scale;
    return size;
}

#endif
