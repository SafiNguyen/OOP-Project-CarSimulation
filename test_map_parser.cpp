#include <iostream>
#include <string>
#include "model/Graph.h"
#include "model/BusStop.h"
#include "Mapload.h"

int main() {
    Graph graph;
    std::string err;
    if (!MapLoad::loadGraphFromJsonFile("map4.json", graph, &err)) {
        std::cerr << "FAILED: " << err << std::endl;
        return 1;
    }
    Road* forward = graph.getRoad(100);
    Road* reverse = graph.getRoad(-100);
    Road* oneDirectionOnly = graph.getRoad(-104);
    if (forward == nullptr || reverse == nullptr || oneDirectionOnly == nullptr ||
        forward->findBusStopById(501) == nullptr ||
        forward->findBusStopById(502) == nullptr ||
        reverse->findBusStopById(504) == nullptr ||
        !oneDirectionOnly->getBusStops().empty()) {
        std::cerr << "FAILED: map4 bus stops were not attached to the expected directed roads."
                  << std::endl;
        return 1;
    }

    std::cout << "SUCCESS! Nodes: " << graph.getAllIntersections().size()
              << ", demo bus stops verified." << std::endl;
    return 0;
}
