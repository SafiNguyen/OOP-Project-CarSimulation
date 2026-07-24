#include <cassert>
#include <iostream>

#include "visualization/VisualizationEngine.h"
#include "model/BusStop.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"

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

    std::cout << "Visualization tests passed" << std::endl;
    return 0;
}
