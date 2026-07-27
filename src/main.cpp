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
#include "ui/UiTheme.h"
#include "ui/VehicleInspector.h"
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
    const unsigned int windowW = 1200;
    const unsigned int windowH = 720;
    const std::string path = resolveInitialMapPath(argc, argv);

    sf::RenderWindow window(sf::VideoMode(windowW, windowH), "Urban Traffic Simulator");
    window.setFramerateLimit(60);

    if (!ImGui::SFML::Init(window)) {
        std::cerr << "Failed to initialize ImGui-SFML." << std::endl;
        return 1;
    }

    UiTheme::apply();

    // Prefer the same clear UI family used by the native map labels when it
    // is available. The bundled ImGui font remains a safe fallback.
    const std::string uiFontPaths[] = {
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf"
    };
    constexpr float uiFontSize = 17.0f;
    bool imguiFontLoaded = false;
    for (const auto& fontPath : uiFontPaths) {
        if (std::filesystem::exists(fontPath)) {
            if (ImFont* uiFont = ImGui::GetIO().Fonts->AddFontFromFileTTF(
                    fontPath.c_str(), uiFontSize)) {
                ImGui::GetIO().FontDefault = uiFont;
                imguiFontLoaded = true;
                break;
            }
        }
    }
    if (!imguiFontLoaded) {
        ImFontConfig fallbackConfig;
        fallbackConfig.SizePixels = uiFontSize;
        ImGui::GetIO().FontDefault = ImGui::GetIO().Fonts->AddFontDefault(&fallbackConfig);
    }
    ImGui::SFML::UpdateFontTexture();

    Graph graph;
    VisualizationEngine visualization({windowW, windowH});

    sf::Font font;
    bool fontLoaded = false;
    const std::string fontPaths[] = {
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/calibri.ttf",
        "arial.ttf",
        "build/_deps/sfml-src/examples/android/app/src/main/assets/tuffy.ttf"
    };
    for (const auto& fontPath : fontPaths) {
        if (font.loadFromFile(fontPath)) {
            visualization.setFont(font);
            fontLoaded = true;
            break;
        }
    }
    if (!fontLoaded) {
        std::cerr << "Warning: Could not load any font. Road names and POIs will not be rendered." << std::endl;
    }

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
    VehicleInspector vehicleInspector;

    sf::Clock clock;

    while (window.isOpen()) {
        const float dt = clock.restart().asSeconds();

        sf::Event event;
        while (window.pollEvent(event)) {
            handleEvent(event, ctx, debugConsole, simulator, vehicleInspector);
        }

        updateCamera(ctx, dt);

        ImGui::SFML::Update(window, sf::seconds(dt));

        if (simulator) {
            simulator->update(dt);
        }

        renderFrame(ctx, debugConsole, simulator, statsPanel, vehicleInspector, dt);
    }

    ImGui::SFML::Shutdown();
    return 0;
}
