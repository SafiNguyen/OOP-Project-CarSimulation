#include <SFML/Graphics.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <imgui-SFML.h>
#include <imgui.h>

#include "algorithm/AStarStrategy.h"
#include "mapload.h"
#include "model/Car.h"
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
    std::string error;
    std::string mapPathInput = path.empty() ? "map.json" : path;
    std::array<char, 1024> mapPathBuffer{};
    bool usingDemoMap = false;
    bool heatMapEnabled = true;
    bool showDebugBar = true;
    bool debugBarCollapsed = false;

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
            error = "No map path supplied, so the built-in demo map is active.";
            return false;
        }

        std::string loadError;
        if (MapLoad::loadGraphFromJsonFile(requestedPath, graph, &loadError)) {
            usingDemoMap = false;
            error.clear();
            mapPathInput = requestedPath;
            return true;
        }

        populateDemoGraph(graph);
        usingDemoMap = true;
        error = loadError;
        return false;
    };

    AStarStrategy aStar;

    auto resetSimulation = [&]() {
        std::unique_ptr<TrafficSimulator> newSimulator;
        newSimulator = std::make_unique<TrafficSimulator>(&graph, &aStar);
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
                Car* car = new Car(i, 40.0, start, end);
                newSimulator->addVehicle(car);
            }
        }
        return newSimulator;
    };

    auto loadAndRefresh = [&](const std::string& requestedPath) {
        loadGraphFromPath(requestedPath);
        visualization.prepare(graph);
        refreshViewBounds();
        view = window.getDefaultView();
        clampViewToMap();
    };

    std::copy_n(mapPathInput.begin(), std::min<std::size_t>(mapPathInput.size(), mapPathBuffer.size() - 1), mapPathBuffer.begin());
    loadAndRefresh(path);

    std::unique_ptr<TrafficSimulator> simulator = resetSimulation();
    StatsPanel statsPanel;

    sf::Clock clock;

    while (window.isOpen()) {
        const float dt = clock.restart().asSeconds();

        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(event);
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
        }

        window.setView(window.getDefaultView());
        if (simulator && simulator->getStatisticsManager()) {
            statsPanel.update(simulator->getStatisticsManager()->getSummary());
            statsPanel.draw(window);
        }

        ImGui::SetNextWindowPos(ImVec2(20.0f, static_cast<float>(window.getSize().y) - 140.0f), ImGuiCond_Always);
        if (debugBarCollapsed) {
            ImGui::SetNextWindowSize(ImVec2(220.0f, 36.0f), ImGuiCond_Always);
        } else {
            ImGui::SetNextWindowSize(ImVec2(static_cast<float>(window.getSize().x - 40), 0.0f), ImGuiCond_Always);
        }
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f, 0.07f, 0.10f, 0.90f));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.30f, 0.35f, 0.50f, 0.85f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, debugBarCollapsed ? ImVec2(8.0f, 6.0f) : ImVec2(12.0f, 8.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, debugBarCollapsed ? ImVec2(6.0f, 4.0f) : ImVec2(8.0f, 5.0f));
        ImGui::Begin("##debug_bar", &showDebugBar,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoScrollbar | (debugBarCollapsed ? 0 : ImGuiWindowFlags_AlwaysAutoResize));
        ImGui::TextColored(ImVec4(0.90f, 0.92f, 0.98f, 1.0f), "Debug Console");
        ImGui::SameLine();
        if (ImGui::Button(debugBarCollapsed ? "Expand" : "Hide")) {
            debugBarCollapsed = !debugBarCollapsed;
        }
        if (!debugBarCollapsed) {
            ImGui::Separator();

            std::copy_n(mapPathInput.begin(), std::min<std::size_t>(mapPathInput.size(), mapPathBuffer.size() - 1), mapPathBuffer.begin());
            ImGui::InputText("Map path", mapPathBuffer.data(), mapPathBuffer.size());
            mapPathInput = mapPathBuffer.data();
            ImGui::SameLine();
            if (ImGui::Button("Load map")) {
                std::string chosenPath;
                if (openMapFileDialog(chosenPath)) {
                    mapPathInput = chosenPath;
                    std::copy_n(mapPathInput.begin(), std::min<std::size_t>(mapPathInput.size(), mapPathBuffer.size() - 1), mapPathBuffer.begin());
                    loadAndRefresh(mapPathInput);
                    simulator = resetSimulation();
                } else {
                    loadAndRefresh(mapPathInput);
                    simulator = resetSimulation();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Demo map")) {
                loadAndRefresh("");
                simulator = resetSimulation();
            }
            ImGui::SameLine();
            if (ImGui::Button(simulator && simulator->isPaused() ? "Resume" : "Pause")) {
                if (simulator) {
                    if (simulator->isPaused()) simulator->resume();
                    else simulator->pause();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset view")) {
                zoomFactor = 1.0f;
                view = window.getDefaultView();
                clampViewToMap();
            }
            ImGui::SameLine();
            if (ImGui::Button(heatMapEnabled ? "Heat: ON" : "Heat: OFF")) {
                heatMapEnabled = !heatMapEnabled;
                visualization.setHeatMapEnabled(heatMapEnabled);
            }

            ImGui::TextWrapped("Status: %s", usingDemoMap ? "Demo map is active." : (mapPathInput.empty() ? "No map selected." : mapPathInput.c_str()));
            ImGui::Text("Intersections: %zu | Roads: %zu | Vehicles: %zu", graph.getAllIntersections().size(), graph.getAllRoads().size(), simulator ? simulator->getVehicles().size() : 0u);

            if (!error.empty()) {
                ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f), "Warning: %s", error.c_str());
            }
            if (graph.getAllIntersections().empty() || graph.getAllRoads().empty()) {
                ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f), "The current scene is missing intersections or roads.");
            }
            if (simulator && simulator->getVehicles().empty()) {
                ImGui::TextColored(ImVec4(0.90f, 0.90f, 0.65f, 1.0f), "No vehicles are currently active.");
            }
        }
        ImGui::End();
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(2);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
    return 0;
}
