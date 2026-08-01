#ifndef DEBUG_CONSOLE_INTERNAL_H
#define DEBUG_CONSOLE_INTERNAL_H

// Small helpers shared by the DebugConsole*.cpp panel files. Kept out of
// DebugConsole.h on purpose - these are implementation details, not part of
// the class's public interface. `inline` so each translation unit that
// includes this header gets its own copy without violating ODR.

#include <algorithm>
#include <cmath>
#include <string>
#include <optional>

#include <SFML/System/Vector2.hpp>

#include "Graph.h"
#include "Intersection.h"
#include "Road.h"
#include "Vehicle.h"
#include "simulation/TrafficSimulator.h"
#include "visualization/VisualizationEngine.h"
#include "model/infrastructure/PointOfInterest.h"
#include "model/infrastructure/SpawnPoint.h"

namespace debugconsole_detail {

inline int nextFreeRoadId(const Graph& graph) {
    int maxId = 0;
    for (Road* r : graph.getAllRoads()) {
        maxId = std::max(maxId, std::abs(r->getId()));
    }
    return maxId + 1;
}

inline int nextFreeVehicleId(TrafficSimulator* simulator) {
    int maxId = 0;
    if (simulator) {
        for (Vehicle* v : simulator->getVehicles()) {
            maxId = std::max(maxId, v->getId());
        }
        for (Vehicle* v : simulator->getPendingVehicles()) {
            maxId = std::max(maxId, v->getId());
        }
        for (Vehicle* v : simulator->getFinishedVehicles()) {
            maxId = std::max(maxId, v->getId());
        }
    }
    return maxId + 1;
}

inline Intersection* pickIntersectionNear(const Graph& graph,
                                           const VisualizationEngine& visualization,
                                           const sf::Vector2f& screenPos,
                                           float pickRadiusPixels = 20.0f) {
    Intersection* best = nullptr;
    float bestDistSq = pickRadiusPixels * pickRadiusPixels;

    for (Intersection* candidate : graph.getAllIntersections()) {
        const sf::Vector2f p = visualization.worldToScreen(candidate->getX(), candidate->getY());
        const float dx = p.x - screenPos.x;
        const float dy = p.y - screenPos.y;
        const float distSq = dx * dx + dy * dy;
        if (distSq <= bestDistSq) {
            bestDistSq = distSq;
            best = candidate;
        }
    }
    return best;
}

inline std::string intersectionLabel(Intersection* it) {
    return "#" + std::to_string(it->getId());
}

inline PointOfInterest* pickPoiNear(const Graph& graph,
                                    const VisualizationEngine& visualization,
                                    const sf::Vector2f& screenPos,
                                    std::optional<POIType> filterType = std::nullopt,
                                    float pickRadiusPixels = 20.0f) {
    PointOfInterest* best = nullptr;
    float bestDistSq = pickRadiusPixels * pickRadiusPixels;

    for (PointOfInterest* candidate : graph.getAllPOIs()) {
        if (filterType.has_value() && candidate->getType() != filterType.value()) {
            continue;
        }
        const sf::Vector2f p = visualization.worldToScreen(candidate->getX(), candidate->getY());
        const float dx = p.x - screenPos.x;
        const float dy = p.y - screenPos.y;
        const float distSq = dx * dx + dy * dy;
        if (distSq <= bestDistSq) {
            bestDistSq = distSq;
            best = candidate;
        }
    }
    
    for (BusStation* candidate : graph.getAllBusStations()) {
        if (filterType.has_value() && candidate->getType() != filterType.value()) {
            continue;
        }
        const sf::Vector2f p = visualization.worldToScreen(candidate->getX(), candidate->getY());
        const float dx = p.x - screenPos.x;
        const float dy = p.y - screenPos.y;
        const float distSq = dx * dx + dy * dy;
        if (distSq <= bestDistSq) {
            bestDistSq = distSq;
            best = candidate;
        }
    }
    return best;
}

inline std::string poiLabel(PointOfInterest* poi) {
    return "#" + std::to_string(poi->getId()) + " (" + poi->getName() + ")";
}

} // namespace debugconsole_detail

#endif
