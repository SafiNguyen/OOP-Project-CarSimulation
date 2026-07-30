#include "Rendering.h"

#include <cmath>
#include <map>
#include <vector>

#include <SFML/Graphics.hpp>
#include <imgui-SFML.h>
#include <imgui.h>

#include "AppContext.h"
#include "model/Intersection.h"
#include "model/Vehicle.h"
#include "simulation/StatisticsManager.h"
#include "simulation/TrafficSimulator.h"
#include "ui/DebugConsole.h"
#include "ui/StatsPanel.h"
#include "ui/VehicleInspector.h"
#include "visualization/VehicleRenderGeometry.h"
#include "visualization/VehicleSprite.h"
#include "visualization/VisualizationEngine.h"

namespace {

constexpr float kVehicleOutlinePixels = 0.55f;

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
        getVehicleScreenSize(vehicle, visualization);
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

void drawActiveVehicles(sf::RenderWindow& window,
                        const VisualizationEngine& visualization,
                        TrafficSimulator& simulator,
                        const sf::View& view) {
    // Reuse the allocation between frames. Every vehicle contributes an
    // outline quad and a body quad, but the whole fleet is submitted in one
    // draw call instead of one SFML Shape draw per vehicle.
    static std::vector<sf::Vertex> vehicleVertices;
    vehicleVertices.clear();
    const std::size_t requiredVertices =
        simulator.getVehicles().size() * 8u;
    if (vehicleVertices.capacity() < requiredVertices) {
        vehicleVertices.reserve(requiredVertices);
    }

    const sf::Vector2f viewCenter = view.getCenter();
    const sf::Vector2f viewSize = view.getSize();
    constexpr float margin = 60.0f;
    const float minX = viewCenter.x - viewSize.x * 0.5f - margin;
    const float maxX = viewCenter.x + viewSize.x * 0.5f + margin;
    const float minY = viewCenter.y - viewSize.y * 0.5f - margin;
    const float maxY = viewCenter.y + viewSize.y * 0.5f + margin;
    const double simulationTime = simulator.getElapsedTime();

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

        appendVehicleQuad(
            vehicleVertices,
            position,
            forward,
            side,
            visual.halfLength + kVehicleOutlinePixels,
            visual.halfWidth + kVehicleOutlinePixels,
            sf::Color::Black);
        appendVehicleQuad(
            vehicleVertices,
            position,
            forward,
            side,
            visual.halfLength,
            visual.halfWidth,
            visual.color);
    }

    if (!vehicleVertices.empty()) {
        window.draw(
            vehicleVertices.data(),
            vehicleVertices.size(),
            sf::Quads);
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

        // Draw vehicles inside the box
        for (size_t i = 0; i < list.size(); ++i) {
            int col = i % cols;
            int row = i / cols;

            sf::Vector2f vPos(boxX + 4.0f + col * cellWidth + cellWidth * 0.5f,
                              boxY + 4.0f + row * cellHeight + cellHeight * 0.5f);

            VehicleSprite sprite(list[i], &visualization);
            // Draw pointing UP (angle = -90)
            sprite.drawAt(window, vPos, -90.0f);
        }
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

void renderFrame(AppContext& ctx, DebugConsole& debugConsole,
                  std::unique_ptr<TrafficSimulator>& simulator, StatsPanel& statsPanel,
                  VehicleInspector& vehicleInspector, float dt) {
    sf::RenderWindow& window = ctx.window;

    window.setView(ctx.view);
    window.clear(sf::Color(34, 42, 48));
    ctx.visualization.drawGraph(window, ctx.graph);

    if (simulator) {
        drawActiveVehicles(
            window, ctx.visualization, *simulator, ctx.view);
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
