#include <SFML/Graphics.hpp>
#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <imgui-SFML.h>
#include <imgui.h>

#include "ui/DebugConsole.h"
#include "mapload.h"
#include "model/Car.h"
#include "model/Bus.h"
#include "model/Motorbike.h"
#include "model/EmergencyVehicle.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "simulation/TrafficSimulator.h"
#include "visualization/StatsPanel.h"
#include "visualization/VehicleSprite.h"
#include "visualization/VisualizationEngine.h"

namespace {

void populateDemoGraph(Graph& graph) {
    graph.clearGraph();

    Intersection* a = new Intersection(1, 15.0, 18.0);
    Intersection* b = new Intersection(2, 84.0, 20.0);
    Intersection* c = new Intersection(3, 90.0, 60.0);
    Intersection* d = new Intersection(4, 55.0, 90.0);
    Intersection* e = new Intersection(5, 16.0, 70.0);

    graph.addIntersection(a);
    graph.addIntersection(b);
    graph.addIntersection(c);
    graph.addIntersection(d);
    graph.addIntersection(e);

    graph.addRoad(new Road(1, a, b, 70.0, 50.0, 1.0));
    graph.addRoad(new Road(2, b, c, 45.0, 45.0, 1.8));
    graph.addRoad(new Road(3, c, d, 48.0, 40.0, 3.0));
    graph.addRoad(new Road(4, d, e, 42.0, 35.0, 2.1));
    graph.addRoad(new Road(5, e, a, 52.0, 50.0, 1.2));
    graph.addRoad(new Road(6, b, d, 68.0, 40.0, 4.0));
}

bool openMapFileDialog(std::string& selectedPath) {
#if defined(__linux__)
    const std::string command = "zenity --file-selection --title='Select map JSON' 2>/dev/null";
    std::array<char, 2048> buffer{};
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        return false;
    }

    std::string result;
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        result += buffer.data();
    }

    const int status = pclose(pipe);
    if (status != 0) {
        return false;
    }

    while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
        result.pop_back();
    }

    if (!result.empty()) {
        selectedPath = result;
        return true;
    }
#endif
    return false;
}

} // namespace

int main(int argc, char** argv) {
    const unsigned int windowW = 800;
    const unsigned int windowH = 600;
    std::string path;
    if (argc > 1) {
        path = argv[1];
    } else if (std::filesystem::exists("map.json")) {
        path = "map.json";
    } else {
        std::cout << "Enter path to JSON map (leave blank to skip): ";
        std::getline(std::cin, path);
    }

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
    std::string loadError;
    std::string mapPathInput = path.empty() ? "map.json" : path;
    bool usingDemoMap = false;
    bool heatMapEnabled = true;

    VisualizationEngine visualization({windowW, windowH});
    visualization.setHeatMapEnabled(heatMapEnabled);

    sf::View view = window.getDefaultView();
    float zoomFactor = 1.0f;
    bool isDragging = false;
    sf::Vector2i lastMousePixel;

    float mapMinX = 0.0f;
    float mapMinY = 0.0f;
    float mapMaxX = 100.0f;
    float mapMaxY = 100.0f;

    auto clampViewToMap = [&]() {
        const sf::Vector2f size = view.getSize();
        const float halfWidth = size.x * 0.5f;
        const float halfHeight = size.y * 0.5f;

        const float minCenterX = mapMinX + halfWidth;
        const float maxCenterX = mapMaxX - halfWidth;
        const float minCenterY = mapMinY + halfHeight;
        const float maxCenterY = mapMaxY - halfHeight;

        sf::Vector2f center = view.getCenter();

        if (minCenterX <= maxCenterX) {
            center.x = std::clamp(center.x, minCenterX, maxCenterX);
        } else {
            center.x = (mapMinX + mapMaxX) * 0.5f;
        }

        if (minCenterY <= maxCenterY) {
            center.y = std::clamp(center.y, minCenterY, maxCenterY);
        } else {
            center.y = (mapMinY + mapMaxY) * 0.5f;
        }

        view.setCenter(center);
    };

    auto refreshViewBounds = [&]() {
        const auto& routePoints = visualization.getRoutePoints();
        if (routePoints.empty()) {
            mapMinX = 0.0f;
            mapMinY = 0.0f;
            mapMaxX = 100.0f;
            mapMaxY = 100.0f;
            return;
        }

        mapMinX = routePoints.front().x;
        mapMinY = routePoints.front().y;
        mapMaxX = routePoints.front().x;
        mapMaxY = routePoints.front().y;
        for (const auto& point : routePoints) {
            mapMinX = std::min(mapMinX, point.x);
            mapMinY = std::min(mapMinY, point.y);
            mapMaxX = std::max(mapMaxX, point.x);
            mapMaxY = std::max(mapMaxY, point.y);
        }

        const float cameraPadding = 60.0f;
        mapMinX -= cameraPadding;
        mapMinY -= cameraPadding;
        mapMaxX += cameraPadding;
        mapMaxY += cameraPadding;
        clampViewToMap();
    };

    auto loadGraphFromPath = [&](const std::string& requestedPath) -> bool {
        if (requestedPath.empty()) {
            populateDemoGraph(graph);
            usingDemoMap = true;
            loadError = "No map path supplied, so the built-in demo map is active.";
            return false;
        }

        std::string searchPath = requestedPath;
        if (searchPath.length() < 5 || searchPath.substr(searchPath.length() - 5) != ".json") {
            searchPath += ".json";
        }

        std::string fileLoadError;
        std::vector<std::string> searchPaths = {
            searchPath,
            "../" + searchPath,
            "../../" + searchPath,
            "../../../" + searchPath
        };

        bool loaded = false;
        for (const auto& p : searchPaths) {
            if (MapLoad::loadGraphFromJsonFile(p, graph, &fileLoadError)) {
                usingDemoMap = false;
                loadError.clear();
                mapPathInput = requestedPath;
                loaded = true;
                break;
            }
        }

        if (loaded) return true;

        populateDemoGraph(graph);
        usingDemoMap = true;
        loadError = fileLoadError;
        return false;
    };

    auto loadAndRefresh = [&](const std::string& requestedPath) {
        loadGraphFromPath(requestedPath);
        visualization.prepare(graph);
        refreshViewBounds();
        view = window.getDefaultView();
        clampViewToMap();
    };

    // Declared before DebugConsole so the console can capture it by reference
    // even though its actual implementation (which needs debugConsole itself,
    // to seed the initial strategy) is assigned just below.
    std::function<std::unique_ptr<TrafficSimulator>()> resetSimulation;

    DebugConsole debugConsole(graph, visualization, loadAndRefresh,
                              [&]() { return resetSimulation(); }, clampViewToMap, openMapFileDialog);

    resetSimulation = [&]() {
        std::unique_ptr<TrafficSimulator> newSimulator;
        newSimulator = std::make_unique<TrafficSimulator>(&graph, debugConsole.getSelectedStrategy());
        auto intersections = graph.getAllIntersections();
        if (intersections.size() >= 2) {
            std::mt19937 rng(42);
            std::uniform_int_distribution<size_t> dist(0, intersections.size() - 1);
            for (int i = 0; i < 40; ++i) {
                Intersection* start = intersections[dist(rng)];
                Intersection* end = intersections[dist(rng)];
                while (start == end) {
                    end = intersections[dist(rng)];
                }
                Vehicle* v = nullptr;
                int type = rng() % 4;
                if (type == 0) {
                    v = new Car(i, 20.0, start, end);
                } else if (type == 1) {
                    v = new Motorbike(i, 30.0, start, end);
                } else if (type == 2) {
                    v = new Bus(i, 15.0, start, end);
                } else {
                    v = new EmergencyVehicle(i, 35.0, start, end);
                }
                newSimulator->addVehicle(v);
            }
        }
        return newSimulator;
    };

    loadAndRefresh(path);

    std::unique_ptr<TrafficSimulator> simulator = resetSimulation();
    StatsPanel statsPanel;

    sf::Clock clock;

    while (window.isOpen()) {
        const float dt = clock.restart().asSeconds();

        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed) {
                window.close();
                continue;
            }
            if (ImGui::GetIO().WantCaptureMouse || ImGui::GetIO().WantCaptureKeyboard) {
                continue;
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
                window.close();
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space) {
                if (simulator && simulator->isPaused()) {
                    simulator->resume();
                } else if (simulator) {
                    simulator->pause();
                }
            }
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Middle) {
                isDragging = true;
                lastMousePixel = sf::Mouse::getPosition(window);
            }
            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Middle) {
                isDragging = false;
            }
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left
                && debugConsole.isPicking()) {
                const sf::Vector2i pixel(event.mouseButton.x, event.mouseButton.y);
                const sf::Vector2f worldPixel = window.mapPixelToCoords(pixel, view);
                debugConsole.handleMapClick(graph, visualization, worldPixel);
            }
            if (event.type == sf::Event::MouseMoved && isDragging) {
                sf::Vector2i mousePixel = sf::Mouse::getPosition(window);
                sf::Vector2i delta = mousePixel - lastMousePixel;
                lastMousePixel = mousePixel;

                view.move(-static_cast<float>(delta.x) * zoomFactor,
                          -static_cast<float>(delta.y) * zoomFactor);
                clampViewToMap();
            }
            if (event.type == sf::Event::MouseWheelScrolled) {
                const float factor = (event.mouseWheelScroll.delta > 0.0f) ? 0.9f : 1.1f;
                zoomFactor = std::clamp(zoomFactor * factor, 0.35f, 2.5f);
                view.setSize(static_cast<float>(windowW) * zoomFactor,
                             static_cast<float>(windowH) * zoomFactor);
                clampViewToMap();
            }
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Add || event.key.code == sf::Keyboard::Equal) {
                    zoomFactor = std::clamp(zoomFactor * 0.9f, 0.35f, 2.5f);
                    view.setSize(static_cast<float>(windowW) * zoomFactor,
                                 static_cast<float>(windowH) * zoomFactor);
                    clampViewToMap();
                }
                if (event.key.code == sf::Keyboard::Subtract || event.key.code == sf::Keyboard::Hyphen) {
                    zoomFactor = std::clamp(zoomFactor * 1.1f, 0.35f, 2.5f);
                    view.setSize(static_cast<float>(windowW) * zoomFactor,
                                 static_cast<float>(windowH) * zoomFactor);
                    clampViewToMap();
                }
                if (event.key.code == sf::Keyboard::R) {
                    zoomFactor = 1.0f;
                    view = window.getDefaultView();
                    clampViewToMap();
                }
            }
        }

        ImGui::SFML::Update(window, sf::seconds(dt));

        if (simulator) {
            simulator->update(dt);
        }

        window.setView(view);
        window.clear(sf::Color(30, 30, 30));
        visualization.drawGraph(window, graph);

        if (simulator) {
            for (Vehicle* v : simulator->getVehicles()) {
                VehicleSprite sprite(v, &visualization);
                sprite.update(dt);
                sprite.draw(window);
            }
            debugConsole.drawFailedRecalcMarkers(window, simulator.get(), visualization);

            // Group finished vehicles by destination
            std::map<Intersection*, std::vector<Vehicle*>> parked;
            for (Vehicle* v : simulator->getFinishedVehicles()) {
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

        window.setView(window.getDefaultView());
        if (simulator && simulator->getStatisticsManager()) {
            statsPanel.update(simulator->getStatisticsManager()->getSummary());
            statsPanel.draw(window);
        }

        debugConsole.draw(window, simulator, view, zoomFactor, heatMapEnabled, mapPathInput, usingDemoMap, loadError);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
    return 0;
}