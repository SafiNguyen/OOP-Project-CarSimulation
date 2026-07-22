#include "Rendering.h"

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
#include "visualization/VehicleSprite.h"
#include "visualization/VisualizationEngine.h"

namespace {

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

} // namespace

void renderFrame(AppContext& ctx, DebugConsole& debugConsole,
                  std::unique_ptr<TrafficSimulator>& simulator, StatsPanel& statsPanel, float dt) {
    sf::RenderWindow& window = ctx.window;

    window.setView(ctx.view);
    window.clear(sf::Color(30, 30, 30));
    ctx.visualization.drawGraph(window, ctx.graph);

    if (simulator) {
        const sf::Vector2f viewCenter = ctx.view.getCenter();
        const sf::Vector2f viewSize = ctx.view.getSize();
        const float margin = 60.0f;
        const float minX = viewCenter.x - viewSize.x * 0.5f - margin;
        const float maxX = viewCenter.x + viewSize.x * 0.5f + margin;
        const float minY = viewCenter.y - viewSize.y * 0.5f - margin;
        const float maxY = viewCenter.y + viewSize.y * 0.5f + margin;

        for (Vehicle* v : simulator->getVehicles()) {
            VehicleSprite sprite(v, &ctx.visualization);
            const sf::Vector2f pos = sprite.getPosition();
            if (pos.x >= minX && pos.x <= maxX && pos.y >= minY && pos.y <= maxY) {
                sprite.update(dt);
                sprite.draw(window);
            }
        }
        debugConsole.drawFailedRecalcMarkers(window, simulator.get(), ctx.visualization);
        if (ctx.showParkedVehicles) {
            drawParkedVehicles(window, ctx.visualization, *simulator);
        }
    }

    if (simulator && simulator->getStatisticsManager()) {
        statsPanel.draw(simulator->getStatisticsManager()->getSummary(), window.getSize());
    }

    debugConsole.draw(window, simulator, ctx.view, ctx.zoomFactor, ctx.heatMapEnabled,
                       ctx.showParkedVehicles, ctx.mapPathInput, ctx.usingDemoMap, ctx.loadError);

    ImGui::SFML::Render(window);
    window.display();
}
