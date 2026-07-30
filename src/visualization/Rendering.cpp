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
#include "Pedestrian.h"
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
constexpr int kPedestrianDiscSegments = 10;
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
         segment < kPedestrianDiscSegments;
         ++segment) {
        const float angleA =
            tau * static_cast<float>(segment) /
            static_cast<float>(kPedestrianDiscSegments);
        const float angleB =
            tau * static_cast<float>(segment + 1) /
            static_cast<float>(kPedestrianDiscSegments);
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
            kPedestrianDiscSegments * 6);
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

        sf::RectangleShape box({cols * cellWidth + 8.0f, rows * cellHeight + 8.0f});
        box.setPosition(boxX, boxY);
        box.setFillColor(sf::Color(40, 40, 40, 200));
        box.setOutlineThickness(1.0f);
        box.setOutlineColor(sf::Color(150, 150, 150));
        window.draw(box);

        // Batch all parked vehicles into one submission. This matters late in
        // a run, when the completed-trip list can contain all 1000 vehicles.
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

} // namespace

void drawPedestrians(
    sf::RenderTarget& target,
    const VisualizationEngine& visualization,
    const TrafficSimulator& simulator) {
    static std::vector<sf::Vertex> pedestrianTriangles;
    static std::vector<sf::Vertex> directionLines;
    pedestrianTriangles.clear();
    directionLines.clear();
    const std::size_t pedestrianCount =
        simulator.getPedestrians().size();
    const std::size_t requiredTriangleVertices =
        pedestrianCount *
        static_cast<std::size_t>(
            kPedestrianDiscSegments * 6);
    if (pedestrianTriangles.capacity() <
        requiredTriangleVertices) {
        pedestrianTriangles.reserve(
            requiredTriangleVertices);
    }
    if (directionLines.capacity() <
        pedestrianCount * 2u) {
        directionLines.reserve(
            pedestrianCount * 2u);
    }

    for (const auto& ownedPedestrian :
         simulator.getPedestrians()) {
        const Pedestrian* pedestrian =
            ownedPedestrian.get();
        if (pedestrian == nullptr ||
            pedestrian->hasArrived()) {
            continue;
        }

        const Pose2D pose = pedestrian->getPose();
        const sf::Vector2f position =
            visualization.worldToScreen(
                pose.position.x,
                pose.position.y);
        const Road* referenceRoad =
            pedestrian->getReferenceRoad();
        const float radius = std::clamp(
            visualization.metresToScreenPixels(
                0.32,
                referenceRoad),
            3.0f,
            6.0f);
        sf::Color color(70, 175, 235);
        if (pedestrian->getState() ==
            PedestrianState::WaitingToCross) {
            color = sf::Color(245, 170, 45);
        } else if (pedestrian->getState() ==
                   PedestrianState::Crossing) {
            color = sf::Color(65, 220, 115);
        }

        appendDisc(
            pedestrianTriangles,
            position,
            radius + 1.0f,
            sf::Color(20, 24, 28));
        appendDisc(
            pedestrianTriangles,
            position,
            radius,
            color);

        const sf::Vector2f facing(
            static_cast<float>(
                std::cos(pose.headingRadians)),
            static_cast<float>(
                -std::sin(pose.headingRadians)));
        directionLines.emplace_back(
            position,
            sf::Color(25, 28, 32));
        directionLines.emplace_back(
            position + facing * radius,
            sf::Color(25, 28, 32));
    }

    if (!pedestrianTriangles.empty()) {
        target.draw(
            pedestrianTriangles.data(),
            pedestrianTriangles.size(),
            sf::Triangles);
    }
    if (!directionLines.empty()) {
        target.draw(
            directionLines.data(),
            directionLines.size(),
            sf::Lines);
    }
}

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
        drawActiveVehicles(
            window, ctx.visualization, *simulator, ctx.view);
        drawPedestrians(
            window,
            ctx.visualization,
            *simulator);
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
    debugConsole.draw(window, simulator, ctx.view, ctx.zoomFactor, ctx.heatMapEnabled,
                       ctx.showParkedVehicles, ctx.mapPathInput, ctx.usingDemoMap, ctx.loadError,
                       statsPanel, statisticsPtr, dt);

    vehicleInspector.draw(simulator.get(), ctx.visualization);

    ImGui::SFML::Render(window);
    window.display();
}