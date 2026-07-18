#include <cassert>
#include <iostream>

#include "visualization/VisualizationEngine.h"
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

    std::cout << "Visualization tests passed" << std::endl;
    return 0;
}
