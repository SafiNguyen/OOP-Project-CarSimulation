#include "Rendering.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

#include <SFML/Graphics.hpp>
#include <imgui-SFML.h>
#include <imgui.h>

#include "AppContext.h"
#include "Intersection.h"
#include "JunctionConnector.h"
#include "LaneMapping.h"
#include "PointOfInterest.h"
#include "Road.h"
#include "RoadGeometry.h"
#include "Vehicle.h"
#include "simulation/StatisticsManager.h"
#include "simulation/TrafficSimulator.h"
#include "ui/DebugConsole.h"
#include "ui/StatsPanel.h"
#include "ui/VehicleInspector.h"
#include "visualization/VehicleAssets.h"
#include "visualization/VehicleRenderGeometry.h"
#include "visualization/VehicleSprite.h"
#include "visualization/VisualizationEngine.h"

namespace {

constexpr float kVehicleOutlinePixels = 0.55f;
constexpr int kDiscSegments = 10;
constexpr float kTurnSignalRadiusPixels = 1.8f;
constexpr float kTurnSignalHaloPixels = 0.6f;
const sf::Color kTurnSignalAmber(255, 165, 0);
const sf::Color kTurnSignalHalo(55, 30, 5);

struct VehicleVisual {
    float halfLength;
    float halfWidth;
    sf::Color color;
};

VehicleVisual getVehicleVisual(
    const Vehicle& vehicle,
    const VisualizationEngine& visualization,
    double simulationTime) {
    const VehicleScreenSize size =
        getVehicleVisualScreenSize(
            vehicle, visualization);
    const float halfLength = size.lengthPixels * 0.5f;
    const float halfWidth = size.widthPixels * 0.5f;
    switch (vehicle.getVehicleKind()) {
        case VehicleKind::Bus:
            return {
                halfLength,
                halfWidth,
                sf::Color(200, 162, 50)
            };
        case VehicleKind::Motorbike:
            return {
                halfLength,
                halfWidth,
                sf::Color(255, 200, 0)
            };
        case VehicleKind::Emergency: {
            const double flashPhase = std::fmod(
                simulationTime + static_cast<double>(vehicle.getId()) * 0.037,
                0.4);
            return {
                halfLength,
                halfWidth,
                flashPhase < 0.2 ? sf::Color::Red : sf::Color::Blue
            };
        }
        case VehicleKind::Car:
        default:
            return {
                halfLength,
                halfWidth,
                sf::Color(50, 150, 255)
            };
    }
}

void appendVehicleQuad(std::vector<sf::Vertex>& vertices,
                       const sf::Vector2f& center,
                       const sf::Vector2f& forward,
                       const sf::Vector2f& side,
                       float halfLength,
                       float halfWidth,
                       sf::Color color) {
    const sf::Vector2f longitudinal = forward * halfLength;
    const sf::Vector2f lateral = side * halfWidth;
    vertices.emplace_back(center - longitudinal - lateral, color);
    vertices.emplace_back(center + longitudinal - lateral, color);
    vertices.emplace_back(center + longitudinal + lateral, color);
    vertices.emplace_back(center - longitudinal + lateral, color);
}

std::size_t vehicleTextureIndex(VehicleKind kind) {
    switch (kind) {
        case VehicleKind::Bus:
            return 1u;
        case VehicleKind::Motorbike:
            return 2u;
        case VehicleKind::Emergency:
            return 3u;
        case VehicleKind::Car:
        default:
            return 0u;
    }
}

void appendTexturedVehicleQuad(
    std::vector<sf::Vertex>& vertices,
    const sf::Vector2f& center,
    const sf::Vector2f& forward,
    const sf::Vector2f& side,
    float halfLength,
    float halfWidth,
    const sf::Vector2u& textureSize) {
    const sf::Vector2f longitudinal =
        forward * halfLength;
    const sf::Vector2f lateral = side * halfWidth;
    const float textureWidth =
        static_cast<float>(textureSize.x);
    const float textureHeight =
        static_cast<float>(textureSize.y);

    // Every source image faces upward. Map its top edge to the vehicle's
    // forward edge and its horizontal axis to the vehicle's lateral axis.
    vertices.emplace_back(
        center + longitudinal - lateral,
        sf::Color::White,
        sf::Vector2f(0.0f, 0.0f));
    vertices.emplace_back(
        center + longitudinal + lateral,
        sf::Color::White,
        sf::Vector2f(textureWidth, 0.0f));
    vertices.emplace_back(
        center - longitudinal + lateral,
        sf::Color::White,
        sf::Vector2f(textureWidth, textureHeight));
    vertices.emplace_back(
        center - longitudinal - lateral,
        sf::Color::White,
        sf::Vector2f(0.0f, textureHeight));
}

void appendDisc(std::vector<sf::Vertex>& vertices,
                const sf::Vector2f& center,
                float radius,
                sf::Color color) {
    constexpr float tau = 6.28318530718f;
    for (int segment = 0;
         segment < kDiscSegments;
         ++segment) {
        const float angleA =
            tau * static_cast<float>(segment) /
            static_cast<float>(kDiscSegments);
        const float angleB =
            tau * static_cast<float>(segment + 1) /
            static_cast<float>(kDiscSegments);
        vertices.emplace_back(center, color);
        vertices.emplace_back(
            center +
                sf::Vector2f(std::cos(angleA), std::sin(angleA)) *
                    radius,
            color);
        vertices.emplace_back(
            center +
                sf::Vector2f(std::cos(angleB), std::sin(angleB)) *
                    radius,
            color);
    }
}

bool drawCachedStaticLayer(AppContext& ctx) {
    const sf::Vector2u requestedSize(
        ctx.windowW,
        ctx.windowH);
    if (requestedSize.x == 0u || requestedSize.y == 0u) {
        return false;
    }

    const std::uint64_t visualizationRevision =
        ctx.visualization.getRevision();
    const bool geometryChanged =
        !ctx.mapCacheReady ||
        ctx.mapCacheRevision != visualizationRevision ||
        ctx.mapCacheTexture.getSize() != requestedSize;

    if (geometryChanged) {
        if (ctx.mapCacheTexture.getSize() != requestedSize &&
            !ctx.mapCacheTexture.create(
                requestedSize.x,
                requestedSize.y)) {
            ctx.mapCacheReady = false;
            return false;
        }

        ctx.mapCacheTexture.setView(
            ctx.mapCacheTexture.getDefaultView());
        // Match the window clear color so translucent map details are blended
        // exactly once before this opaque cached texture is composited.
        ctx.mapCacheTexture.clear(sf::Color(34, 42, 48));
        ctx.visualization.drawStaticLayer(
            ctx.mapCacheTexture,
            ctx.graph);
        ctx.mapCacheTexture.display();
        ctx.mapCacheSprite.setTexture(
            ctx.mapCacheTexture.getTexture(),
            true);
        ctx.mapCacheRevision = visualizationRevision;
        ctx.mapCacheReady = true;
    }

    ctx.window.draw(ctx.mapCacheSprite);
    return true;
}

void drawActiveVehicles(sf::RenderWindow& window,
                        const VisualizationEngine& visualization,
                        TrafficSimulator& simulator,
                        const sf::View& view) {
    // Reuse allocations between frames and batch by texture. The four
    // vehicle types cost at most four textured draw calls even for a large
    // fleet; missing assets continue through the old rectangle fallback.
    static std::vector<sf::Vertex> fallbackVehicleVertices;
    static std::array<std::vector<sf::Vertex>, 4>
        texturedVehicleVertices;
    static std::vector<sf::Vertex> turnSignalVertices;
    static std::vector<sf::Vertex> emergencyLightVertices;
    fallbackVehicleVertices.clear();
    for (auto& vertices : texturedVehicleVertices) {
        vertices.clear();
    }
    turnSignalVertices.clear();
    emergencyLightVertices.clear();
    const std::size_t requiredVertices =
        simulator.getVehicles().size() * 8u;
    if (fallbackVehicleVertices.capacity() <
        requiredVertices) {
        fallbackVehicleVertices.reserve(requiredVertices);
    }
    for (auto& vertices : texturedVehicleVertices) {
        const std::size_t perTypeReserve =
            simulator.getVehicles().size() * 4u;
        if (vertices.capacity() < perTypeReserve) {
            vertices.reserve(perTypeReserve);
        }
    }
    const std::size_t requiredSignalVertices =
        simulator.getVehicles().size() *
        static_cast<std::size_t>(
            kDiscSegments * 6);
    if (turnSignalVertices.capacity() <
        requiredSignalVertices) {
        turnSignalVertices.reserve(
            requiredSignalVertices);
    }

    const sf::Vector2f viewCenter = view.getCenter();
    const sf::Vector2f viewSize = view.getSize();
    constexpr float margin = 60.0f;
    const float minX = viewCenter.x - viewSize.x * 0.5f - margin;
    const float maxX = viewCenter.x + viewSize.x * 0.5f + margin;
    const float minY = viewCenter.y - viewSize.y * 0.5f - margin;
    const float maxY = viewCenter.y + viewSize.y * 0.5f + margin;
    const double simulationTime = simulator.getElapsedTime();
    const sf::Vector2u windowSize = window.getSize();
    const float viewUnitsPerPixel = std::max(
        windowSize.x > 0u
            ? viewSize.x /
                  static_cast<float>(windowSize.x)
            : 1.0f,
        windowSize.y > 0u
            ? viewSize.y /
                  static_cast<float>(windowSize.y)
            : 1.0f);

    for (Vehicle* vehicle : simulator.getVehicles()) {
        if (vehicle == nullptr || vehicle->getCurrentRoad() == nullptr) {
            continue;
        }

        // One pose sample supplies both position and heading. The old sprite
        // path sampled the same pose once for culling and twice again to draw.
        const Pose2D pose = vehicle->getPose();
        const sf::Vector2f position = visualization.worldToScreen(
            pose.position.x, pose.position.y);
        if (position.x < minX || position.x > maxX ||
            position.y < minY || position.y > maxY) {
            continue;
        }

        const float cosine =
            static_cast<float>(std::cos(pose.headingRadians));
        const float sine =
            static_cast<float>(std::sin(pose.headingRadians));
        const sf::Vector2f forward(cosine, -sine);
        const sf::Vector2f side(sine, cosine);
        const VehicleVisual visual =
            getVehicleVisual(
                *vehicle, visualization, simulationTime);
        const sf::Texture* texture =
            VehicleAssets::instance().textureFor(
                vehicle->getVehicleKind());
        if (texture != nullptr) {
            appendTexturedVehicleQuad(
                texturedVehicleVertices[
                    vehicleTextureIndex(
                        vehicle->getVehicleKind())],
                position,
                forward,
                side,
                visual.halfLength,
                visual.halfWidth,
                texture->getSize());
        } else {
            appendVehicleQuad(
                fallbackVehicleVertices,
                position,
                forward,
                side,
                visual.halfLength +
                    kVehicleOutlinePixels,
                visual.halfWidth +
                    kVehicleOutlinePixels,
                sf::Color::Black);
            appendVehicleQuad(
                fallbackVehicleVertices,
                position,
                forward,
                side,
                visual.halfLength,
                visual.halfWidth,
                visual.color);
        }

        if (vehicle->getVehicleKind() ==
            VehicleKind::Emergency) {
            const float lightRadius =
                1.35f * viewUnitsPerPixel;
            const sf::Vector2f lightPosition =
                position +
                forward * (visual.halfLength * 0.22f);
            appendDisc(
                emergencyLightVertices,
                lightPosition,
                lightRadius,
                visual.color);
        }

        if (vehicle->getTurnSignal() !=
                TurnSignal::Off &&
            vehicle->isTurnSignalBlinkOn()) {
            const float radius =
                kTurnSignalRadiusPixels *
                viewUnitsPerPixel;
            const float haloRadius =
                (kTurnSignalRadiusPixels +
                 kTurnSignalHaloPixels) *
                viewUnitsPerPixel;
            const float sideSign =
                vehicle->getTurnSignal() ==
                        TurnSignal::Right
                    ? 1.0f
                    : -1.0f;
            const sf::Vector2f lampPosition =
                position -
                forward *
                    (visual.halfLength +
                     radius * 0.15f) +
                side * sideSign *
                    (visual.halfWidth +
                     radius * 0.10f);
            appendDisc(
                turnSignalVertices,
                lampPosition,
                haloRadius,
                kTurnSignalHalo);
            appendDisc(
                turnSignalVertices,
                lampPosition,
                radius,
                kTurnSignalAmber);
        }
    }

    if (!fallbackVehicleVertices.empty()) {
        window.draw(
            fallbackVehicleVertices.data(),
            fallbackVehicleVertices.size(),
            sf::Quads);
    }
    const VehicleKind textureKinds[4] = {
        VehicleKind::Car,
        VehicleKind::Bus,
        VehicleKind::Motorbike,
        VehicleKind::Emergency
    };
    for (std::size_t index = 0u;
         index < texturedVehicleVertices.size();
         ++index) {
        const auto& vertices =
            texturedVehicleVertices[index];
        const sf::Texture* texture =
            VehicleAssets::instance().textureFor(
                textureKinds[index]);
        if (texture == nullptr || vertices.empty()) {
            continue;
        }
        sf::RenderStates states;
        states.texture = texture;
        window.draw(
            vertices.data(),
            vertices.size(),
            sf::Quads,
            states);
    }
    if (!emergencyLightVertices.empty()) {
        window.draw(
            emergencyLightVertices.data(),
            emergencyLightVertices.size(),
            sf::Triangles);
    }
    if (!turnSignalVertices.empty()) {
        window.draw(
            turnSignalVertices.data(),
            turnSignalVertices.size(),
            sf::Triangles);
    }
}

void drawParkedVehicles(sf::RenderWindow& window, const VisualizationEngine& visualization,
                         TrafficSimulator& simulator) {
    // Group finished vehicles by destination
    std::map<Intersection*, std::vector<Vehicle*>> parked;
    for (Vehicle* v : simulator.getFinishedVehicles()) {
        if (v->getDestination()) {
            parked[v->getDestination()].push_back(v);
        }
    }

    static std::vector<sf::Vertex> parkedVertices;
    parkedVertices.clear();
    const ViewportBounds viewportBounds(window.getView());
    const std::size_t requiredVertices =
        simulator.getFinishedVehicles().size() * 8u;
    if (parkedVertices.capacity() < requiredVertices) {
        parkedVertices.reserve(requiredVertices);
    }

    for (const auto& pair : parked) {
        Intersection* dest = pair.first;
        const auto& list = pair.second;

        sf::Vector2f center = visualization.worldToScreen(dest->getX(), dest->getY());

        // Draw a box near the intersection (e.g. top right)
        float boxX = center.x + 20.0f;
        float boxY = center.y - 40.0f;

        // Calculate box size based on number of vehicles
        int cols = 5; // up to 5 cars per row
        int rows = (static_cast<int>(list.size()) + cols - 1) / cols;
        float cellWidth = 24.0f;
        float cellHeight = 16.0f;

        if (!viewportBounds.intersectsRectangle(
                sf::FloatRect(
                    boxX,
                    boxY,
                    cols * cellWidth + 8.0f,
                    rows * cellHeight + 8.0f),
                1.0f)) {
            continue;
        }

        sf::RectangleShape box({cols * cellWidth + 8.0f, rows * cellHeight + 8.0f});
        box.setPosition(boxX, boxY);
        box.setFillColor(sf::Color(40, 40, 40, 200));
        box.setOutlineThickness(1.0f);
        box.setOutlineColor(sf::Color(150, 150, 150));
        window.draw(box);

        // Batch all parked vehicles into one submission. This matters late in
        // a run, when the completed-trip list can contain thousands of
        // vehicles from a large configured simulation.
        for (size_t i = 0; i < list.size(); ++i) {
            int col = i % cols;
            int row = i / cols;

            sf::Vector2f vPos(boxX + 4.0f + col * cellWidth + cellWidth * 0.5f,
                              boxY + 4.0f + row * cellHeight + cellHeight * 0.5f);

            const VehicleVisual visual =
                getVehicleVisual(
                    *list[i],
                    visualization,
                    simulator.getElapsedTime());
            const sf::Vector2f forward(0.0f, -1.0f);
            const sf::Vector2f side(1.0f, 0.0f);
            appendVehicleQuad(
                parkedVertices,
                vPos,
                forward,
                side,
                visual.halfLength + kVehicleOutlinePixels,
                visual.halfWidth + kVehicleOutlinePixels,
                sf::Color::Black);
            appendVehicleQuad(
                parkedVertices,
                vPos,
                forward,
                side,
                visual.halfLength,
                visual.halfWidth,
                visual.color);
        }
    }

    if (!parkedVertices.empty()) {
        window.draw(
            parkedVertices.data(),
            parkedVertices.size(),
            sf::Quads);
    }
}

// Draws a highlight ring above whichever vehicle is currently selected in
// the VehicleInspector, so the user can see which car the panel refers to.
// Searches both active and finished vehicles (mirrors
// VehicleInspector::findSelectedVehicle) so a completed trip's vehicle can
// still be highlighted if its "parked" box is visible.
void drawSelectedVehicleHighlight(sf::RenderWindow& window,
                                   const VisualizationEngine& visualization,
                                   TrafficSimulator& simulator,
                                   const VehicleInspector& vehicleInspector) {
    if (!vehicleInspector.hasSelection()) {
        return;
    }

    const int selectedId = vehicleInspector.getSelectedId();
    Vehicle* target = nullptr;
    for (Vehicle* v : simulator.getVehicles()) {
        if (v->getId() == selectedId) {
            target = v;
            break;
        }
    }
    if (target == nullptr || target->getCurrentRoad() == nullptr) {
        return; // finished vehicles are drawn inside their "parked" box, not on the road
    }

    VehicleSprite sprite(target, &visualization);
    const sf::Vector2f pos = sprite.getPosition();

    sf::CircleShape ring(14.0f);
    ring.setOrigin(14.0f, 14.0f);
    ring.setPosition(pos);
    ring.setFillColor(sf::Color::Transparent);
    ring.setOutlineThickness(2.0f);
    ring.setOutlineColor(sf::Color::Yellow);
    window.draw(ring);
}

void drawSelectedVehicleRoute(sf::RenderWindow& window,
                              const VisualizationEngine& visualization,
                              TrafficSimulator& simulator,
                              const VehicleInspector& vehicleInspector) {
    if (!vehicleInspector.hasSelection()) {
        return;
    }

    const int selectedId = vehicleInspector.getSelectedId();
    Vehicle* target = nullptr;
    for (Vehicle* v : simulator.getVehicles()) {
        if (v->getId() == selectedId) {
            target = v;
            break;
        }
    }
    if (target == nullptr) {
        for (Vehicle* v : simulator.getFinishedVehicles()) {
            if (v->getId() == selectedId) {
                target = v;
                break;
            }
        }
    }
    
    if (target == nullptr) return;

    const auto& route = target->getCurrentRoute();
    if (route.empty()) return;

    int currentIdx = target->getCurrentRouteIndex();
    if (currentIdx < 0) currentIdx = 0;

    std::vector<sf::Vertex> routeVertices;
    const float thickness = 6.0f;
    const sf::Color color(0, 255, 255, 120); // Cyan semi-transparent

    // The route line is drawn inside the lane the vehicle will actually
    // drive in, not down the middle of the road. Start from the vehicle's
    // current lane and propagate the lane through each junction using the
    // same lane-mapping rules the vehicle follows when navigating.
    int currentLaneIndex = target->getCurrentLaneIndex();
    if (currentLaneIndex < 0) currentLaneIndex = 0;

    // Helper to append a quad for a segment between two world points.
    auto appendSegment = [&](const Vec2& a, const Vec2& b) {
        sf::Vector2f startPos =
            visualization.worldToScreen(a.x, a.y);
        sf::Vector2f endPos =
            visualization.worldToScreen(b.x, b.y);
        sf::Vector2f dir = endPos - startPos;
        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (len < 0.1f) return;
        dir /= len;
        sf::Vector2f normal(-dir.y, dir.x);
        sf::Vector2f offset = normal * (thickness * 0.5f);
        routeVertices.emplace_back(startPos - offset, color);
        routeVertices.emplace_back(startPos + offset, color);
        routeVertices.emplace_back(endPos + offset, color);
        routeVertices.emplace_back(endPos - offset, color);
    };

    // Helper to append a polyline through a junction connector path so the
    // route line follows the actual curved path through intersections and
    // roundabouts instead of jumping straight across the junction.
    auto appendConnectorPath = [&](const JunctionConnector& connector) {
        const double pathLength = connector.getLength();
        if (pathLength <= 0.0) return;
        constexpr int kSamples = 16;
        Vec2 prevPoint;
        bool hasPrev = false;
        for (int s = 0; s <= kSamples; ++s) {
            const double dist =
                pathLength * static_cast<double>(s) /
                static_cast<double>(kSamples);
            const Pose2D pose = connector.sampleByDistance(dist);
            if (hasPrev) {
                appendSegment(prevPoint, pose.position);
            }
            prevPoint = pose.position;
            hasPrev = true;
        }
    };

    // Start from the vehicle's current position so the line begins at the
    // vehicle rather than at the start of the road it is on. However, when
    // the vehicle is entering its POI/destination, its pose is on the
    // driveway (off the road), so we should not start the line from there.
    const bool isEnteringPOI = target->getIsEnteringPOI();
    const Pose2D vehiclePose = target->getPose();
    const Vec2 currentPosition = vehiclePose.position;

    // If the vehicle is currently traversing a junction or roundabout, the
    // route line should start from the vehicle's position on the connector
    // path and follow the remaining curved path through the junction.
    bool startedOnJunction = false;
    if (!isEnteringPOI &&
        target->getMovementState() == MovementState::TraversingJunction) {
        Road* incomingRoad = target->getCurrentRoad();
        Road* outgoingRoad = target->getNextRoad();
        if (incomingRoad != nullptr && outgoingRoad != nullptr) {
            Intersection* intersection = incomingRoad->getEnd();
            if (intersection != nullptr) {
                auto connector = intersection->getConnector(
                    incomingRoad,
                    target->getIncomingLaneIndex(),
                    outgoingRoad,
                    target->getOutgoingLaneIndex());
                if (connector != nullptr) {
                    const double junctionProgress =
                        target->getJunctionProgress();
                    const double pathLength = connector->getLength();
                    if (junctionProgress < pathLength) {
                        constexpr int kSamples = 16;
                        Vec2 prevPoint = currentPosition;
                        for (int s = 1; s <= kSamples; ++s) {
                            const double dist = junctionProgress +
                                (pathLength - junctionProgress) *
                                static_cast<double>(s) /
                                static_cast<double>(kSamples);
                            const Pose2D pose =
                                connector->sampleByDistance(dist);
                            appendSegment(prevPoint, pose.position);
                            prevPoint = pose.position;
                        }
                    }
                    currentLaneIndex = target->getOutgoingLaneIndex();
                    startedOnJunction = true;
                }
            }
        }
    }

    // The outgoing road is at currentIdx + 1 when starting on a junction.
    const size_t startIdx = startedOnJunction
        ? static_cast<size_t>(currentIdx) + 1
        : static_cast<size_t>(currentIdx);

    const PointOfInterest* targetPOI = target->getTargetPOI();

    for (size_t i = startIdx; i < route.size(); ++i) {
        const Road* road = route[i];
        if (road == nullptr) continue;

        const int laneCount = road->getLaneCount();
        if (laneCount < 1) continue;

        const int lane = std::clamp(currentLaneIndex, 0, laneCount - 1);

        // Sample the lane endpoints (already trimmed to the junction
        // boundary) instead of the raw intersection centres, so each segment
        // sits within the lane geometry - offset from the road median on
        // two-way carriageways and from the centre on multi-lane roads.
        const Vec2 laneStart =
            RoadGeometry::laneEndpoint(*road, lane, true);
        const Vec2 laneEnd =
            RoadGeometry::laneEndpoint(*road, lane, false);

        // If this is the last road and the vehicle has a target POI on it,
        // stop the line at the POI position instead of the end of the road.
        Vec2 segmentEnd = laneEnd;
        if (i == route.size() - 1 && targetPOI != nullptr &&
            targetPOI->getConnectedRoad() == road) {
            const Pose2D poiPose = RoadGeometry::sampleLane(
                *road, lane, targetPOI->getProgressOffset());
            segmentEnd = poiPose.position;
        }

        // For the first road in the loop (when not starting on a junction),
        // start from the vehicle's current position. However, when the
        // vehicle is entering its POI/destination, its pose is on the
        // driveway (off the road), so start from the POI position on the
        // road instead to avoid a weird connection back to the road.
        Vec2 segmentStart = laneStart;
        if (i == startIdx && !startedOnJunction) {
            if (isEnteringPOI && targetPOI != nullptr &&
                targetPOI->getConnectedRoad() == road) {
                segmentStart = segmentEnd;
            } else {
                segmentStart = currentPosition;
            }
        }

        appendSegment(segmentStart, segmentEnd);

        // Determine which lane the vehicle will use on the next road and
        // draw the junction connector path through the intersection or
        // roundabout so the route line follows the actual curved path.
        if (i + 1 < route.size()) {
            const Road* nextRoad = route[i + 1];
            if (nextRoad != nullptr) {
                const LaneMapping mapping =
                    TurnLanePolicy::mapFromCurrentLane(
                        *road, lane, *nextRoad);
                if (mapping.valid) {
                    Intersection* intersection = road->getEnd();
                    if (intersection != nullptr) {
                        auto connector = intersection->getConnector(
                            road,
                            mapping.incomingLane,
                            nextRoad,
                            mapping.outgoingLane);
                        if (connector != nullptr) {
                            appendConnectorPath(*connector);
                        }
                    }
                    currentLaneIndex = mapping.outgoingLane;
                } else {
                    currentLaneIndex = 0;
                }
            }
        }
    }

    if (!routeVertices.empty()) {
        window.draw(routeVertices.data(), routeVertices.size(), sf::Quads);
    }
}

} // namespace

void renderFrame(AppContext& ctx, DebugConsole& debugConsole,
                  std::unique_ptr<TrafficSimulator>& simulator, StatsPanel& statsPanel,
                  VehicleInspector& vehicleInspector, float dt) {
    sf::RenderWindow& window = ctx.window;

    window.setView(ctx.view);
    window.clear(sf::Color(34, 42, 48));
    if (!drawCachedStaticLayer(ctx)) {
        ctx.visualization.drawStaticLayer(
            window,
            ctx.graph);
    }

    ctx.visualization.drawDynamicLayer(
        window,
        ctx.graph);

    if (simulator) {
        drawSelectedVehicleRoute(window, ctx.visualization, *simulator, vehicleInspector);
        drawActiveVehicles(
            window, ctx.visualization, *simulator, ctx.view);
        debugConsole.drawManualSpawnMarkers(
            window,
            simulator.get(),
            ctx.visualization,
            dt);
        debugConsole.drawFailedRecalcMarkers(window, simulator.get(), ctx.visualization);
        drawSelectedVehicleHighlight(window, ctx.visualization, *simulator, vehicleInspector);
        if (ctx.showParkedVehicles) {
            drawParkedVehicles(window, ctx.visualization, *simulator);
        }
    }

    StatisticsSummary statistics;
    const StatisticsSummary* statisticsPtr = nullptr;
    if (simulator && simulator->getStatisticsManager()) {
        statistics = simulator->getStatisticsManager()->getSummary();
        statisticsPtr = &statistics;
    }
    debugConsole.draw(window, simulator, ctx.heatMapEnabled,
                       ctx.showParkedVehicles, ctx.mapPathInput, ctx.usingDemoMap, ctx.loadError,
                       statsPanel, statisticsPtr, dt);

    vehicleInspector.draw(simulator.get(), ctx.visualization, ctx);

    ImGui::SFML::Render(window);
    window.display();
}
