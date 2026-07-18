#include <SFML/Graphics.hpp>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <imgui-SFML.h>
#include <imgui.h>

#include "AppContext.h"
#include "visualization/Camera.h"
#include "InputHandling.h"
#include "ui/MapFileDialog.h"
#include "MapLoading.h"
#include "visualization/Rendering.h"
#include "visualization/SimulatorFactory.h"
#include "model/Graph.h"
#include "simulation/TrafficSimulator.h"
#include "ui/DebugConsole.h"
#include "ui/StatsPanel.h"
#include "visualization/VisualizationEngine.h"

namespace {

std::string resolveInitialMapPath(int argc, char** argv) {
    if (argc > 1) {
        return argv[1];
    }
    if (std::filesystem::exists("map.json")) {
        return "map.json";
    }
    return "";
}

} // namespace

int main(int argc, char** argv) {
    const unsigned int windowW = 800;
    const unsigned int windowH = 600;
    const std::string path = resolveInitialMapPath(argc, argv);

    sf::RenderWindow window(sf::VideoMode(windowW, windowH), "Urban Traffic Simulator - Debug Console");
    window.setFramerateLimit(60);

    if (!ImGui::SFML::Init(window)) {
        std::cerr << "Failed to initialize ImGui-SFML." << std::endl;
        return 1;
    }

    ImGui::GetStyle().WindowRounding = 8.0f;
    ImGui::GetStyle().FrameRounding = 6.0f;
    ImGui::GetStyle().GrabRounding = 6.0f;

    Graph graph;
    VisualizationEngine visualization({windowW, windowH});

    AppContext ctx(graph, visualization, window, windowW, windowH);
    ctx.view = window.getDefaultView();
    ctx.mapPathInput = path.empty() ? "map.json" : path;
    visualization.setHeatMapEnabled(ctx.heatMapEnabled);

    std::function<std::unique_ptr<TrafficSimulator>()> resetSimulation;

    DebugConsole debugConsole(
        graph, visualization,
        [&](const std::string& requestedPath) { loadAndRefresh(ctx, requestedPath); },
        [&]() { return resetSimulation(); },
        [&]() { clampViewToMap(ctx); },
        openMapFileDialog);

    resetSimulation = [&]() {
        return createDemoSimulator(graph, debugConsole.getSelectedStrategy());
    };

    loadAndRefresh(ctx, path);

    std::unique_ptr<TrafficSimulator> simulator = resetSimulation();
    StatsPanel statsPanel;

    sf::Clock clock;

    while (window.isOpen()) {
        const float dt = clock.restart().asSeconds();

        sf::Event event;
        while (window.pollEvent(event)) {
            handleEvent(event, ctx, debugConsole, simulator);
        }

        updateCamera(ctx, dt);

        ImGui::SFML::Update(window, sf::seconds(dt));

        if (simulator) {
            simulator->update(dt);
        }

        renderFrame(ctx, debugConsole, simulator, statsPanel, dt);
    }

    ImGui::SFML::Shutdown();
    return 0;
}
