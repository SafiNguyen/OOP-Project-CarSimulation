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
    const VisualizationEngine& visualization,
    float viewUnitsPerPixel = 1.0f) {
    VehicleScreenSize size =
        getVehicleScreenSize(vehicle, visualization);
    if (size.lengthPixels <= 0.0f) {
        return size;
    }

    // Vehicles are laid out from their physical metre dimensions.  Only
    // the final visual footprint receives a small screen-pixel floor, like
    // point rendering in a GIS, so kilometre-scale overview maps do not
    // make active vehicles disappear.  This does not affect collision,
    // following distance, lane occupancy, or any other simulation value.
    const float mapMinimumLengthPixels =
        visualization.getMinimumVehicleLengthPixels();
    float vehicleKindScale = 1.0f;
    switch (vehicle.getVehicleKind()) {
        case VehicleKind::Bus:
            vehicleKindScale = 1.25f;
            break;
        case VehicleKind::Motorbike:
            vehicleKindScale = 0.75f;
            break;
        case VehicleKind::Emergency:
            vehicleKindScale = 1.125f;
            break;
        case VehicleKind::Car:
        default:
            break;
    }

    const float safeViewUnitsPerPixel =
        std::max(1e-6f, viewUnitsPerPixel);
    const float targetLength = std::max(
        size.lengthPixels,
        mapMinimumLengthPixels * vehicleKindScale *
            safeViewUnitsPerPixel);
    const float scale = targetLength / size.lengthPixels;
    size.lengthPixels *= scale;
    size.widthPixels *= scale;
    return size;
}

#endif

