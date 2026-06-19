#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <algorithm>

#include "mapload.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"

namespace {

sf::Color mixColor(const sf::Color& a, const sf::Color& b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    auto mix = [t](sf::Uint8 x, sf::Uint8 y) -> sf::Uint8 {
        return static_cast<sf::Uint8>(x + (y - x) * t);
    };

    return sf::Color(mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b), mix(a.a, b.a));
}

sf::Color roadHeatColor(const Road* road) {
    if (road == nullptr) {
        return sf::Color(120, 120, 120);
    }

    if (road->isBlocked()) {
        return sf::Color(180, 40, 40);
    }

    const double congestion = std::max(1.0, road->getCongestionLevel());
    const float normalized = static_cast<float>(std::clamp((congestion - 1.0) / 4.0, 0.0, 1.0));

    const sf::Color green(45, 190, 90);
    const sf::Color yellow(245, 190, 45);
    const sf::Color red(220, 55, 55);

    if (normalized < 0.5f) {
        return mixColor(green, yellow, normalized * 2.0f);
    }

    return mixColor(yellow, red, (normalized - 0.5f) * 2.0f);
}

double visualCongestionForRoad(const Road* road) {
    if (road == nullptr) {
        return 1.0;
    }

    if (road->getCongestionLevel() > 1.01) {
        return road->getCongestionLevel();
    }

    const int seed = road->getId() * 97 + static_cast<int>(road->getDistance() * 13.0);
    const int clamped = std::abs(seed % 400);
    return 1.0 + static_cast<double>(clamped) / 100.0;
}

float distanceBetween(const sf::Vector2f& a, const sf::Vector2f& b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

void drawRoadStrip(sf::RenderWindow& window,
                   const sf::Vector2f& a,
                   const sf::Vector2f& b,
                   const sf::Color& color,
                   float thickness) {
    const float len = distanceBetween(a, b);
    if (len <= 0.01f) {
        return;
    }

    sf::RectangleShape strip({len, thickness});
    strip.setOrigin(0.0f, thickness * 0.5f);
    strip.setPosition(a);
    strip.setRotation(std::atan2(b.y - a.y, b.x - a.x) * 180.0f / 3.14159265f);
    strip.setFillColor(color);
    window.draw(strip);
}

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

int main(int argc, char** argv)
{
    const unsigned int windowW = 800;
    const unsigned int windowH = 600;
    std::string path;
    if (argc > 1) {
        path = argv[1];
    } else {
        std::cout << "Enter path to JSON map (leave blank to skip): ";
        std::getline(std::cin, path);
    }

    Graph graph;
    std::string error;

    // Allow retrying until user provides empty input or loading succeeds
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

    sf::View view = window.getDefaultView();
    float zoomFactor = 1.0f;
    bool isDragging = false;
    sf::Vector2i lastMousePixel;

    // Prepare drawable data
    auto intersections = graph.getAllIntersections();
    auto roads = graph.getAllRoads();

    std::sort(intersections.begin(), intersections.end(), [](Intersection* lhs, Intersection* rhs) {
        return lhs->getId() < rhs->getId();
    });

    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    if (!intersections.empty()) {
        minX = maxX = intersections[0]->getX();
        minY = maxY = intersections[0]->getY();
        for (auto* it : intersections) {
            double x = it->getX();
            double y = it->getY();
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
        }
    }

    const float margin = 20.0f;
    double rangeX = (maxX - minX);
    double rangeY = (maxY - minY);
    double scaleX = (rangeX > 0.0) ? (windowW - 2*margin) / rangeX : 1.0;
    double scaleY = (rangeY > 0.0) ? (windowH - 2*margin) / rangeY : 1.0;
    double scale = std::min(scaleX, scaleY);

    auto worldToScreen = [&](double x, double y) -> sf::Vector2f {
        float sx = static_cast<float>(margin + (x - minX) * scale);
        // invert Y so larger world Y is up on screen
        float sy = static_cast<float>(windowH - margin - (y - minY) * scale);
        return { sx, sy };
    };

    std::vector<sf::Vector2f> routePoints;
    routePoints.reserve(intersections.size());
    for (auto* intersection : intersections) {
        routePoints.push_back(worldToScreen(intersection->getX(), intersection->getY()));
    }

    if (routePoints.size() < 2) {
        routePoints.push_back({windowW * 0.8f, windowH * 0.2f});
        routePoints.push_back({windowW * 0.2f, windowH * 0.8f});
    }

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
        totalLength += distanceBetween(routePoints[i - 1], routePoints[i]);
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

        window.setView(view);
        window.clear(sf::Color(30,30,30));

        // Draw roads
        for (auto* road : roads) {
            Intersection* s = road->getStart();
            Intersection* e = road->getEnd();
            if (!s || !e) continue;
            sf::Vector2f a = worldToScreen(s->getX(), s->getY());
            sf::Vector2f b = worldToScreen(e->getX(), e->getY());
            drawRoadStrip(window, a, b, sf::Color(10, 10, 10, 220), road->isBlocked() ? 15.0f : 12.0f);
            drawRoadStrip(window, a, b, roadHeatColor(road), road->isBlocked() ? 11.0f : 8.0f);
        }

        for (std::size_t i = 1; i < routePoints.size(); ++i) {
            drawRoadStrip(window, routePoints[i - 1], routePoints[i], sf::Color(80, 220, 255, 180), 4.0f);
        }

        sf::RectangleShape border(sf::Vector2f(mapMaxX - mapMinX, mapMaxY - mapMinY));
        border.setPosition(mapMinX, mapMinY);
        border.setFillColor(sf::Color::Transparent);
        border.setOutlineThickness(2.0f);
        border.setOutlineColor(sf::Color(235, 235, 235, 120));
        window.draw(border);

        // Draw intersections
        const float radius = 5.5f;
        for (auto* it : intersections) {
            sf::Vector2f p = worldToScreen(it->getX(), it->getY());
            sf::CircleShape circle(radius);
            circle.setOrigin(radius, radius);
            circle.setPosition(p);
            circle.setFillColor(sf::Color(235, 235, 235));
            circle.setOutlineThickness(1.5f);
            circle.setOutlineColor(sf::Color(20, 20, 20));
            window.draw(circle);
        }

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
