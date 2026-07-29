#include <SFML/Graphics.hpp>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include <imgui-SFML.h>
#include <imgui.h>

#include "AppContext.h"
#include "visualization/Camera.h"
#include "InputHandling.h"
#include "ui/MapFileDialog.h"
#include "MapLoading.h"
#include "Mapload.h"
#include "algorithm/DijkstraStrategy.h"
#include "visualization/Rendering.h"
#include "visualization/SimulatorFactory.h"
#include "visualization/VehicleSprite.h"
#include "Graph.h"
#include "simulation/TrafficSimulator.h"
#include "ui/DebugConsole.h"
#include "ui/StatsPanel.h"
#include "ui/UiTheme.h"
#include "ui/VehicleInspector.h"
#include "visualization/VisualizationEngine.h"

namespace {

struct SnapshotOptions {
    bool enabled = false;
    std::string outputPath;
    unsigned int width = 800u;
    unsigned int height = 600u;
    double wallSeconds = 0.0;
    double speedMultiplier = 1.0;
    bool paused = false;
};

SnapshotOptions parseSnapshotOptions(
    int argc,
    char** argv) {
    SnapshotOptions options;
    for (int index = 2; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto readValue =
            [&](const char* option) -> std::optional<std::string> {
                if (argument != option || index + 1 >= argc) {
                    return std::nullopt;
                }
                return std::string(argv[++index]);
            };
        if (const auto value = readValue("--snapshot")) {
            options.enabled = true;
            options.outputPath = *value;
        } else if (const auto value = readValue("--width")) {
            options.width = static_cast<unsigned int>(
                std::max(1, std::stoi(*value)));
        } else if (const auto value = readValue("--height")) {
            options.height = static_cast<unsigned int>(
                std::max(1, std::stoi(*value)));
        } else if (const auto value =
                       readValue("--wall-seconds")) {
            options.wallSeconds =
                std::max(0.0, std::stod(*value));
        } else if (const auto value = readValue("--speed")) {
            options.speedMultiplier =
                std::max(0.01, std::stod(*value));
        } else if (argument == "--paused") {
            options.paused = true;
        }
    }
    return options;
}

std::string resolveInitialMapPath(int argc, char** argv) {
    if (argc > 1) {
        return argv[1];
    }
    if (std::filesystem::exists("map.json")) {
        return "map.json";
    }
    return "";
}

bool loadSnapshotFont(sf::Font& font) {
    const std::vector<std::string> paths = {
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    };
    for (const std::string& path : paths) {
        if (std::filesystem::exists(path) &&
            font.loadFromFile(path)) {
            return true;
        }
    }
    return false;
}

int renderSnapshot(const std::string& mapPath,
                   const SnapshotOptions& options) {
    if (mapPath.empty() || options.outputPath.empty()) {
        std::cerr
            << "Snapshot mode requires a map path and --snapshot output."
            << std::endl;
        return 2;
    }

    Graph graph;
    std::string loadError;
    if (!MapLoad::loadGraphFromJsonFile(
            mapPath, graph, &loadError)) {
        std::cerr << loadError << std::endl;
        return 3;
    }

    VisualizationEngine visualization(
        {options.width, options.height});
    sf::Font font;
    if (loadSnapshotFont(font)) {
        visualization.setFont(font);
    }
    visualization.prepare(graph);

    DijkstraStrategy strategy;
    auto simulator =
        createDemoSimulator(graph, &strategy);
    simulator->setSpeedMultiplier(
        options.speedMultiplier);
    if (options.paused) {
        simulator->pause();
    }
    double wallRemaining = options.wallSeconds;
    while (wallRemaining > 1e-9) {
        const double step =
            std::min(0.05, wallRemaining);
        simulator->update(step);
        wallRemaining -= step;
    }

    sf::RenderTexture target;
    if (!target.create(options.width, options.height)) {
        std::cerr << "Could not create snapshot render target."
                  << std::endl;
        return 4;
    }
    target.clear(sf::Color(34, 42, 48));
    visualization.drawGraph(target, graph);
    for (Vehicle* vehicle : simulator->getVehicles()) {
        VehicleSprite sprite(vehicle, &visualization);
        sprite.draw(target);
    }
    drawPedestrians(
        target,
        visualization,
        *simulator);
    target.display();
    if (!target.getTexture().copyToImage().saveToFile(
            options.outputPath)) {
        std::cerr << "Could not save snapshot: "
                  << options.outputPath << std::endl;
        return 5;
    }

    std::cout << "Snapshot " << options.width << "x"
              << options.height << " saved to "
              << options.outputPath
              << " at simulation time "
              << simulator->getElapsedTime() << " s"
              << (simulator->isPaused() ? " (paused)" : "")
              << std::endl;
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const std::string path = resolveInitialMapPath(argc, argv);
    const SnapshotOptions snapshotOptions =
        parseSnapshotOptions(argc, argv);
    if (snapshotOptions.enabled) {
        return renderSnapshot(path, snapshotOptions);
    }

    const unsigned int windowW = 1200;
    const unsigned int windowH = 720;

    sf::RenderWindow window(sf::VideoMode(windowW, windowH), "Urban Traffic Simulator");
    window.setFramerateLimit(60);

    if (!ImGui::SFML::Init(window)) {
        std::cerr << "Failed to initialize ImGui-SFML." << std::endl;
        return 1;
    }

    UiTheme::apply();

    // Prefer the same clear UI family used by the native map labels when it
    // is available. The bundled ImGui font remains a safe fallback.
    const std::vector<std::string> uiFontPaths = {
        // Linux fonts
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
        // Windows fonts
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
        // macOS fonts
        "/Library/Fonts/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc"
    };
    constexpr float uiFontSize = 17.0f;
    bool imguiFontLoaded = false;
    for (const auto& fontPath : uiFontPaths) {
        if (std::filesystem::exists(fontPath)) {
            if (ImFont* uiFont = ImGui::GetIO().Fonts->AddFontFromFileTTF(
                    fontPath.c_str(), uiFontSize)) {
                ImGui::GetIO().FontDefault = uiFont;
                imguiFontLoaded = true;
                std::cout << "Loaded ImGui font from: " << fontPath << std::endl;
                break;
            }
        }
    }
    if (!imguiFontLoaded) {
        ImFontConfig fallbackConfig;
        fallbackConfig.SizePixels = uiFontSize;
        ImGui::GetIO().FontDefault = ImGui::GetIO().Fonts->AddFontDefault(&fallbackConfig);
        std::cout << "Using fallback ImGui font" << std::endl;
    }
    ImGui::SFML::UpdateFontTexture();

    Graph graph;
    VisualizationEngine visualization({windowW, windowH});

    sf::Font font;
    bool fontLoaded = false;
    
    // Cross-platform font loading: try Linux, Windows, macOS, and relative paths
    const std::vector<std::string> fontPaths = {
        // Linux system fonts
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
        "/usr/share/fonts/opentype/liberation/LiberationSans-Regular.otf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
        // Windows fonts
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/calibri.ttf",
        // macOS fonts
        "/Library/Fonts/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        // Relative paths
        "arial.ttf",
        "./assets/fonts/arial.ttf",
        "../assets/fonts/arial.ttf",
        "build/_deps/sfml-src/examples/android/app/src/main/assets/tuffy.ttf"
    };
    
    for (const auto& fontPath : fontPaths) {
        if (font.loadFromFile(fontPath)) {
            visualization.setFont(font);
            fontLoaded = true;
            std::cout << "Loaded font from: " << fontPath << std::endl;
            break;
        }
    }
    
    if (!fontLoaded) {
        std::cerr << "Warning: Could not load any font. Road names and POIs will not be rendered." << std::endl;
        std::cerr << "On Linux: sudo apt install fonts-liberation" << std::endl;
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
