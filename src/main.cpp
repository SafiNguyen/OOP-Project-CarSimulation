#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <filesystem>

#include "mapload.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
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

sf::Vector2f samplePath(const std::vector<sf::Vector2f>& points,
                        const std::vector<float>& cumulativeLengths,
                        float distance) {
    if (points.empty()) {
        return {0.0f, 0.0f};
    }

    if (points.size() == 1 || cumulativeLengths.empty()) {
        return points.front();
    }

    const float total = cumulativeLengths.back();
    if (total <= 0.01f) {
        return points.front();
    }

    distance = std::fmod(distance, total);
    if (distance < 0.0f) {
        distance += total;
    }

    std::size_t segmentIndex = 0;
    while (segmentIndex < cumulativeLengths.size() && cumulativeLengths[segmentIndex] < distance) {
        ++segmentIndex;
    }

    if (segmentIndex == 0) {
        const float segmentLength = cumulativeLengths[0];
        const float t = (segmentLength > 0.01f) ? (distance / segmentLength) : 0.0f;
        return points[0] + (points[1] - points[0]) * t;
    }

    const float segmentStart = cumulativeLengths[segmentIndex - 1];
    const float segmentEnd = cumulativeLengths[segmentIndex];
    const float segmentLength = segmentEnd - segmentStart;
    const float t = (segmentLength > 0.01f) ? ((distance - segmentStart) / segmentLength) : 0.0f;
    return points[segmentIndex] + (points[segmentIndex + 1] - points[segmentIndex]) * t;
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

    Graph graph;
    std::string error;

    while (!path.empty()) {
        if (MapLoad::loadGraphFromJsonFile(path, graph, &error)) {
            std::cout << "Loaded map: " << path << std::endl;
            break;
        }
        std::cerr << "Failed to load map '" << path << "': " << error << std::endl;
        std::cout << "Enter another path (leave blank to skip): ";
        std::getline(std::cin, path);
    }

    if (graph.getAllIntersections().empty() || graph.getAllRoads().empty()) {
        populateDemoGraph(graph);
        std::cout << "Using built-in demo map for visual testing." << std::endl;
    }

    sf::RenderWindow window(sf::VideoMode(windowW, windowH), "Urban Traffic Simulator - Map Test");
    window.setFramerateLimit(60);

    VisualizationEngine visualization({windowW, windowH});
    visualization.prepare(graph);

    sf::View view = window.getDefaultView();
    float zoomFactor = 1.0f;
    bool isDragging = false;
    sf::Vector2i lastMousePixel;

    const auto& routePoints = visualization.getRoutePoints();

    float mapMinX = routePoints.front().x;
    float mapMinY = routePoints.front().y;
    float mapMaxX = routePoints.front().x;
    float mapMaxY = routePoints.front().y;
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

    clampViewToMap();

    std::vector<float> cumulativeLengths;
    cumulativeLengths.reserve(routePoints.size() - 1);
    float totalLength = 0.0f;
    for (std::size_t i = 1; i < routePoints.size(); ++i) {
        totalLength += std::sqrt(std::pow(routePoints[i].x - routePoints[i - 1].x, 2.0f) +
                                  std::pow(routePoints[i].y - routePoints[i - 1].y, 2.0f));
        cumulativeLengths.push_back(totalLength);
    }

    sf::Clock clock;
    float carDistance = 0.0f;
    const float carSpeed = 110.0f;

    while (window.isOpen()) {
        const float dt = clock.restart().asSeconds();
        carDistance += dt * carSpeed;

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) window.close();
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
                if (event.type == sf::Event::KeyPressed && (event.key.code == sf::Keyboard::Subtract || event.key.code == sf::Keyboard::Hyphen)) {
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

        window.setView(view);
        window.clear(sf::Color(30, 30, 30));
        visualization.drawGraph(window, graph);

        if (routePoints.size() >= 2 && totalLength > 0.0f) {
            sf::Vector2f carPos = samplePath(routePoints, cumulativeLengths, carDistance);

            sf::CircleShape glow(6.0f);
            glow.setOrigin(6.0f, 6.0f);
            glow.setPosition(carPos);
            glow.setFillColor(sf::Color(255, 190, 80, 45));
            window.draw(glow);

            sf::RectangleShape car(sf::Vector2f(6.0f, 6.0f));
            car.setOrigin(3.0f, 3.0f);
            car.setPosition(carPos);
            car.setFillColor(sf::Color(255, 245, 90));
            car.setOutlineThickness(1.5f);
            car.setOutlineColor(sf::Color(20, 20, 20));
            window.draw(car);
        }

        window.display();
    }

    return 0;
}
