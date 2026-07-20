// TEMPORARY: smoke test for the new "no ugly borders at intersections"
// rendering. Renders a tiny graph with crossing roads to a PNG and exits.
// Remove before committing.
#include <SFML/Graphics.hpp>

#include <cstdlib>
#include <iostream>
#include <vector>

#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "visualization/VisualizationEngine.h"

int main() {
    Intersection top(1, 0.0, 50.0);
    Intersection bottom(2, 0.0, -50.0);
    Intersection left(3, -50.0, 0.0);
    Intersection right(4, 50.0, 0.0);
    Intersection center(5, 0.0, 0.0);

    Graph g;
    g.addIntersection(&top);
    g.addIntersection(&bottom);
    g.addIntersection(&left);
    g.addIntersection(&right);
    g.addIntersection(&center);

    auto* r1 = new Road(101, "Vertical", &top, &center, 50.0, 20.0, 1.0, 2);
    auto* r2 = new Road(102, "Vertical", &center, &bottom, 50.0, 20.0, 1.0, 2);
    auto* r3 = new Road(103, "Horizontal", &left, &center, 50.0, 20.0, 1.0, 2);
    auto* r4 = new Road(104, "Horizontal", &center, &right, 50.0, 20.0, 1.0, 2);
    g.addRoad(r1);
    g.addRoad(r2);
    g.addRoad(r3);
    g.addRoad(r4);

    const unsigned int W = 300u;
    const unsigned int H = 300u;
    VisualizationEngine engine({W, H}, 12.0f);
    engine.prepare(g);
    engine.setHeatMapEnabled(false);

    sf::RenderTexture tex;
    if (!tex.create(W, H)) {
        std::cerr << "Failed to create render texture\n";
        return 1;
    }
    tex.clear(sf::Color(60, 60, 60));
    engine.drawGraph(tex, g);
    tex.display();

    const sf::Image img = tex.getTexture().copyToImage();
    const std::string out = "crossing_test.png";
    if (!img.saveToFile(out)) {
        std::cerr << "Failed to save image\n";
        return 1;
    }
    std::cout << "Wrote " << out << "\n";
    return 0;
}
