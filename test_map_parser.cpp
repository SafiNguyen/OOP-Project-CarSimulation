#include <iostream>
#include <string>
#include "model/Graph.h"
#include "mapload.h"

int main() {
    Graph graph;
    std::string err;
    if (!MapLoad::loadGraphFromJsonFile("map4.json", graph, &err)) {
        std::cerr << "FAILED: " << err << std::endl;
        return 1;
    }
    std::cout << "SUCCESS! Nodes: " << graph.getAllIntersections().size() << std::endl;
    return 0;
}
