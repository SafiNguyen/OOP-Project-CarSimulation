#include <iostream>
#include <string>
#include "Graph.h"
#include "BusStop.h"
#include "Mapload.h"

int main() {
    Graph graph;
    std::string err;
    if (!MapLoad::loadGraphFromJsonFile("map4.json", graph, &err)) {
        std::cerr << "FAILED: " << err << std::endl;
        return 1;
    }
    const struct ExpectedStop {
        int stopId;
        int roadId;
    } expectedStops[] = {
        {501, 101},
        {502, 103},
        {503, 106},
        {504, -100},
        {505, -104},
        {506, 109},
        {507, -108},
        {508, 114},
        {509, -113}
    };

    for (const ExpectedStop& expected : expectedStops) {
        Road* road = graph.getRoad(expected.roadId);
        const BusStop* stop =
            road != nullptr
                ? road->findBusStopById(expected.stopId)
                : nullptr;
        if (road == nullptr || stop == nullptr ||
            stop->getRoad() != road ||
            !road->isCurbLane(stop->getLaneIndex())) {
            std::cerr << "FAILED: map4 bus stops were not attached to the expected directed roads."
                      << std::endl;
            return 1;
        }
    }

    std::cout << "SUCCESS! Nodes: " << graph.getAllIntersections().size()
              << ", demo bus stops verified." << std::endl;
    return 0;
}
