#ifndef VEHICLE_RENDER_GEOMETRY_H
#define VEHICLE_RENDER_GEOMETRY_H

#include <algorithm>

#include "Vehicle.h"
#include "visualization/VisualizationEngine.h"

struct VehicleScreenSize {
    float lengthPixels = 0.0f;
    float widthPixels = 0.0f;
};

struct TurnSignalVisualSize {
    float radius = 0.0f;
    float haloRadius = 0.0f;
};

// Turn signals are expressed in the same render-space units as the vehicle
// body. SFML therefore scales both together as the view zoom changes. The
// caps keep even the flashing halo comfortably inside the vehicle footprint.
inline TurnSignalVisualSize getTurnSignalVisualSize(
    float vehicleHalfLength,
    float vehicleHalfWidth) {
    const float safeHalfLength = std::max(0.0f, vehicleHalfLength);
    const float safeHalfWidth = std::max(0.0f, vehicleHalfWidth);
    const float radius = std::min(
        safeHalfLength * 0.16f,
        safeHalfWidth * 0.40f);
    return {
        radius,
        std::min(radius * 1.35f, safeHalfWidth * 0.58f)
    };
}

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
    if (size.lengthPixels <= 0.0f) {
        return size;
    }
    const Road* road = vehicle.getCurrentRoad();
    if (road == nullptr) {
        return size;
    }

    float minimumLength = 14.0f;
    double visualLengthMetres = vehicle.getLength();
    double visualWidthMetres = vehicle.getWidth();
    double visualGapMetres = vehicle.getMinGap();
    constexpr double CAR_VISUAL_LENGTH_METRES = 4.5;
    constexpr double CAR_VISUAL_WIDTH_METRES = 1.8;
    constexpr double CAR_VISUAL_GAP_METRES = 3.0;
    switch (vehicle.getVehicleKind()) {
        case VehicleKind::Bus:
            minimumLength = 20.0f;
            break;
        case VehicleKind::Motorbike:
            visualLengthMetres = CAR_VISUAL_LENGTH_METRES;
            visualWidthMetres = CAR_VISUAL_WIDTH_METRES;
            visualGapMetres = CAR_VISUAL_GAP_METRES;
            break;
        case VehicleKind::Emergency:
            minimumLength = 17.0f;
            break;
        case VehicleKind::Car:
        default:
            break;
    }

    size = {
        visualization.metresToScreenPixels(
            visualLengthMetres, road),
        visualization.metresToScreenPixels(
            visualWidthMetres, road)
    };
    constexpr double MAX_GAP_VISUALIZATION_SHARE = 0.5;
    const float maximumLength =
        visualization.metresToScreenPixels(
            visualLengthMetres +
                visualGapMetres *
                    MAX_GAP_VISUALIZATION_SHARE,
            road);
    const float targetLength = std::clamp(
        minimumLength,
        size.lengthPixels,
        std::max(size.lengthPixels, maximumLength));
    const float scale = targetLength / size.lengthPixels;
    size.lengthPixels *= scale;
    size.widthPixels *= scale;
    return size;
}

#endif
