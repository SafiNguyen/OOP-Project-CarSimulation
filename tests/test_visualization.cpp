#include <cassert>
#include <filesystem>
#include <iostream>
#include <vector>

#include "Mapload.h"
#include "visualization/VisualizationEngine.h"
#include "visualization/SimulatorFactory.h"
#include "algorithm/DijkstraStrategy.h"
#include "model/Bus.h"
#include "model/BusStop.h"
#include "model/Car.h"
#include "model/EmergencyVehicle.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Motorbike.h"
#include "model/Road.h"
#include "simulation/TrafficSimulator.h"

namespace {

int countColorComponents(const sf::Image& image,
                         const sf::Color& color,
                         int minimumPixels) {
    const sf::Vector2u size = image.getSize();
    std::vector<unsigned char> visited(
        static_cast<std::size_t>(size.x) * size.y, 0u);
    std::vector<sf::Vector2u> pending;
    int components = 0;

    const auto indexOf = [size](unsigned int x, unsigned int y) {
        return static_cast<std::size_t>(y) * size.x + x;
    };

    for (unsigned int y = 0; y < size.y; ++y) {
        for (unsigned int x = 0; x < size.x; ++x) {
            const std::size_t startIndex = indexOf(x, y);
            if (visited[startIndex] || image.getPixel(x, y) != color) {
                continue;
            }

            int pixels = 0;
            pending.clear();
            pending.push_back({x, y});
            visited[startIndex] = 1u;
            while (!pending.empty()) {
                const sf::Vector2u current = pending.back();
                pending.pop_back();
                ++pixels;

                const int neighbors[4][2] = {
                    {-1, 0}, {1, 0}, {0, -1}, {0, 1}
                };
                for (const auto& neighbor : neighbors) {
                    const int nx = static_cast<int>(current.x) + neighbor[0];
                    const int ny = static_cast<int>(current.y) + neighbor[1];
                    if (nx < 0 || ny < 0 ||
                        nx >= static_cast<int>(size.x) ||
                        ny >= static_cast<int>(size.y)) {
                        continue;
                    }

                    const auto ux = static_cast<unsigned int>(nx);
                    const auto uy = static_cast<unsigned int>(ny);
                    const std::size_t neighborIndex = indexOf(ux, uy);
                    if (!visited[neighborIndex] &&
                        image.getPixel(ux, uy) == color) {
                        visited[neighborIndex] = 1u;
                        pending.push_back({ux, uy});
                    }
                }
            }

            if (pixels >= minimumPixels) {
                ++components;
            }
        }
    }
    return components;
}

} // namespace

int main() {
    Intersection start(1, 0.0, 0.0);
    Intersection end(2, 10.0, 0.0);
    Road road(101, "Test", &start, &end, 10.0, 50.0, 1.0);
    VisualizationEngine engine;

    sf::Color normal = engine.colorForRoad(&road);
    assert(normal.r > 20 && normal.g > 100 && normal.b < 120);

    Road blockedRoad(102, "Test", &start, &end, 10.0, 50.0, 1.0);
    blockedRoad.blockRoad();
    sf::Color blocked = engine.colorForRoad(&blockedRoad);
    assert(blocked == sf::Color(180, 40, 40));

    Road congestedRoad(103, "Test", &start, &end, 10.0, 50.0, 3.0);
    sf::Color congested = engine.colorForRoad(&congestedRoad);
    assert(congested.r > congested.g && congested.r > congested.b);

    engine.setHeatMapEnabled(false);
    assert(!engine.isHeatMapEnabled());
    engine.setHeatMapEnabled(true);
    assert(engine.isHeatMapEnabled());

    Graph graph;
    graph.addIntersection(new Intersection(10, 0.0, 0.0));
    graph.addIntersection(new Intersection(11, 100.0, 0.0));
    auto* busRoad = new Road(
        201, "Bus Road", graph.getIntersection(10), graph.getIntersection(11),
        100.0, 50.0, 1.0, 2);
    graph.addRoad(busRoad);
    assert(busRoad->addBusStop(std::make_unique<BusStop>(
        501, "Rendered Stop", busRoad, 50.0, 1, 15.0)));

    sf::RenderTexture target;
    assert(target.create(800, 600));
    engine.prepare(graph);
    target.clear(sf::Color::Black);
    engine.drawGraph(target, graph);
    target.display();

    const sf::Image rendered = target.getTexture().copyToImage();
    bool foundBusStopBlue = false;
    for (unsigned int y = 0; y < rendered.getSize().y && !foundBusStopBlue; ++y) {
        for (unsigned int x = 0; x < rendered.getSize().x; ++x) {
            const sf::Color pixel = rendered.getPixel(x, y);
            if (pixel.r < 80 && pixel.g > 110 && pixel.b > 180) {
                foundBusStopBlue = true;
                break;
            }
        }
    }
    assert(foundBusStopBlue);

    Graph spawnGraph;
    spawnGraph.addIntersection(new Intersection(20, 0.0, 0.0));
    spawnGraph.addIntersection(new Intersection(21, 100.0, 0.0));
    spawnGraph.addRoad(new Road(
        301, "Outbound", spawnGraph.getIntersection(20),
        spawnGraph.getIntersection(21), 100.0, 50.0));
    spawnGraph.addRoad(new Road(
        302, "Inbound", spawnGraph.getIntersection(21),
        spawnGraph.getIntersection(20), 100.0, 50.0));

    DijkstraStrategy strategy;
    auto simulator = createDemoSimulator(spawnGraph, &strategy);
    int carCount = 0;
    int motorbikeCount = 0;
    int busCount = 0;
    int emergencyCount = 0;
    for (Vehicle* vehicle : simulator->getVehicles()) {
        if (dynamic_cast<Bus*>(vehicle) != nullptr) {
            ++busCount;
        } else if (dynamic_cast<Motorbike*>(vehicle) != nullptr) {
            ++motorbikeCount;
        } else if (dynamic_cast<EmergencyVehicle*>(vehicle) != nullptr) {
            ++emergencyCount;
        } else if (dynamic_cast<Car*>(vehicle) != nullptr) {
            ++carCount;
        }
    }

    const int spawnedCount =
        carCount + motorbikeCount + busCount + emergencyCount;
    assert(spawnedCount == 1000);
    assert(busCount >= 30 && busCount <= 70);
    assert(carCount > busCount);
    assert(motorbikeCount > busCount);

    Graph map4Graph;
    std::string map4Error;
    std::string map4Path = "map4.json";
    if (!std::filesystem::exists(map4Path)) {
        map4Path = "../map4.json";
    }
    assert(MapLoad::loadGraphFromJsonFile(map4Path, map4Graph, &map4Error));

    VisualizationEngine map4Engine({800u, 600u});
    map4Engine.prepare(map4Graph);
    sf::RenderTexture map4Target;
    assert(map4Target.create(800u, 600u));
    map4Target.clear(sf::Color(30, 30, 30));
    map4Engine.drawGraph(map4Target, map4Graph);
    map4Target.display();
    const sf::Image map4Image = map4Target.getTexture().copyToImage();
    assert(countColorComponents(
        map4Image, sf::Color(35, 145, 230), 20) >= 4);

    std::cout << "Visualization tests passed" << std::endl;
    return 0;
}
