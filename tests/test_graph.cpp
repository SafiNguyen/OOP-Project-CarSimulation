// ============================================================================
// test_graph.cpp
// ----------------------------------------------------------------------------
// Standalone unit tests for the Graph, Road, and Intersection models.
// Uses the custom TestFramework to verify object connections, memory management
// (cascaded deletions), and physical calculations like travel time,
// congestion, and bus stop positioning.
// ============================================================================

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <filesystem>

#include "../tests/TestFramework.h"
#include "../src/model/Graph.h"
#include "../src/model/Intersection.h"
#include "../src/model/Road.h"
#include "../src/model/BusStop.h"
#include "../src/Mapload.h"

using TestFramework::reportResult;
using TestFramework::nearlyEqual;
using TestFramework::printSummary;

namespace {

std::string mapJsonWithBusStops(const std::string& busStopsJson) {
    return std::string(R"JSON({
        "intersections": [
            {"id": 1, "x": 0.0, "y": 0.0},
            {"id": 2, "x": 100.0, "y": 0.0}
        ],
        "roads": [
            {"id": 12, "start": 1, "end": 2, "distance": 5.0,
             "speedLimit": 40.0, "lanes": 2, "twoWay": true}
        ],
        "busStops": )JSON") + busStopsJson + "\n}";
}

bool loadMap(const std::string& json, Graph& graph, std::string& error) {
    error.clear();
    return MapLoad::loadGraphFromJsonString(json, graph, &error);
}

} // namespace

// ----------------------------------------------------------------------------
// Test Cases for Road
// ----------------------------------------------------------------------------

void test_Road_TravelCost_And_Congestion() {
    std::string testName = "Road: Travel time adapts to congestion and respects blocked status";
    
    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    // Quãng đường 100m, tốc độ tối đa 50m/s, kẹt xe mức 2.0
    Road road(101, "Test", &i1, &i2, 100.0, 50.0, 2.0); 

    // Tốc độ thực tế = 50 / 2.0 = 25m/s. Thời gian = 100 / 25 = 4.0s
    bool passNormal = nearlyEqual(road.getTravelTime(), 4.0);

    // Bị tai nạn chặn đường
    road.blockRoad();
    bool passBlocked = std::isinf(road.getTravelTime());

    // Hết kẹt xe (mức 1.0)
    road.unblockRoad();
    road.updateCongestionLevel(1.0);
    bool passClear = nearlyEqual(road.getTravelTime(), 2.0); // 100 / 50 = 2.0s

    bool passed = passNormal && passBlocked && passClear;

    std::ostringstream d;
    d << "  Expected: time=4.0 (congestion 2.0), time=inf (blocked), time=2.0 (clear)\n";
    d << "  Actual:   time1=" << (100.0 / (50.0/2.0)) << " blocked=" << std::isinf(road.getTravelTime()) << "\n";
    reportResult(testName, passed, d.str());
}

void test_Road_BusStopLogic() {
    std::string testName = "Road: Bus stop logic (sorting, epsilon duplication, boundaries)";
    
    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 50.0);

    // Cố tình add lộn xộn và add trùng lặp để test độ "lì" của logic
    road.addBusStop(50.0);
    road.addBusStop(20.0);
    road.addBusStop(20.005); // Sẽ bị loại vì cách nhau < 0.01m
    road.addBusStop(0.0);    // Sẽ bị loại vì trùng ngã tư đầu
    road.addBusStop(100.0);  // Sẽ bị loại vì trùng ngã tư cuối

    const auto& stops = road.getBusStopPositions();
    bool passCount = (stops.size() == 2);
    bool passOrder = passCount && nearlyEqual(stops[0], 20.0) && nearlyEqual(stops[1], 50.0);

    // Test hàm dò trạm tiếp theo
    bool passNext1 = nearlyEqual(road.getNextBusStop(0.0, 30.0), 20.0);
    bool passNext2 = nearlyEqual(road.getNextBusStop(20.0, 60.0), 50.0);
    bool passNext3 = nearlyEqual(road.getNextBusStop(50.0, 100.0), -1.0);

    bool passed = passCount && passOrder && passNext1 && passNext2 && passNext3;

    std::ostringstream d;
    d << "  Expected: 2 stops [20.0, 50.0], next stops correctly identified\n";
    d << "  Actual:   count=" << stops.size() << " next1=" << road.getNextBusStop(0.0, 30.0) << "\n";
    reportResult(testName, passed, d.str());
}

void test_Road_BusStopModelLogic() {
    std::string testName = "Road: BusStop models retain identity and are ordered by position";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(12, "Test", &i1, &i2, 100.0, 50.0, 1.0, 2);

    bool addedSecond = road.addBusStop(std::make_unique<BusStop>(
        502, "Second", &road, 70.0, 1, 8.0));
    bool addedFirst = road.addBusStop(std::make_unique<BusStop>(
        501, "First", &road, 25.0, 0, 4.0));

    const auto& stops = road.getBusStops();
    const BusStop* next = road.getNextBusStopInfo(25.0, 100.0);
    const bool passed = addedSecond && addedFirst &&
                        stops.size() == 2 &&
                        stops[0]->getId() == 501 &&
                        stops[1]->getId() == 502 &&
                        road.findBusStopById(502) == stops[1].get() &&
                        next == stops[1].get() &&
                        nearlyEqual(stops[0]->getPositionRatio(), 0.25);

    std::ostringstream d;
    d << "  Expected: IDs [501, 502], lookup 502 succeeds, next after 25 is 502\n";
    d << "  Actual:   count=" << stops.size()
      << " next=" << (next ? next->getId() : -1) << "\n";
    reportResult(testName, passed, d.str());
}

void test_Road_CurbLaneConventionForBothDirections() {
    std::string testName = "Road: curb lane is the final model lane in either direction";

    Intersection west(1, 0.0, 0.0);
    Intersection east(2, 100.0, 0.0);
    Road forward(12, "Forward", &west, &east, 100.0, 40.0, 1.0, 2);
    Road reverse(-12, "Reverse", &east, &west, 100.0, 40.0, 1.0, 2);

    const bool passed =
        forward.getCurbLaneIndex() == 1 &&
        reverse.getCurbLaneIndex() == 1 &&
        forward.isCurbLane(1) &&
        reverse.isCurbLane(1) &&
        !forward.isCurbLane(0) &&
        !reverse.isCurbLane(0) &&
        !forward.isCurbLane(2);

    std::ostringstream d;
    d << "  Expected: lane 1 is curb-side for forward and reverse Roads\n";
    d << "  Actual: forward=" << forward.getCurbLaneIndex()
      << " reverse=" << reverse.getCurbLaneIndex() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Map4_RoadsideBusStopsUseDirectionalCurbLane() {
    std::string testName = "MapLoad: map4 roadside stops use each directional Road's curb lane";

    std::string mapPath = "map4.json";
    if (!std::filesystem::exists(mapPath)) {
        mapPath = "../map4.json";
    }

    Graph graph;
    std::string error;
    const bool loaded = MapLoad::loadGraphFromJsonFile(mapPath, graph, &error);
    const struct ExpectedStop {
        int stopId;
        int roadId;
    } expectedStops[] = {
        {501, 100},
        {502, 100},
        {503, 104},
        {504, -100}
    };

    bool stopsValid = loaded;
    for (const ExpectedStop& expected : expectedStops) {
        Road* road = graph.getRoad(expected.roadId);
        const BusStop* stop =
            road != nullptr ? road->findBusStopById(expected.stopId) : nullptr;
        stopsValid = stopsValid &&
                     road != nullptr &&
                     stop != nullptr &&
                     stop->getRoad() == road &&
                     road->isCurbLane(stop->getLaneIndex());
    }

    std::ostringstream d;
    d << "  Expected: stops 501-504 belong to their directional Road and curb lane\n";
    d << "  Load error: " << error << "\n";
    reportResult(testName, stopsValid, d.str());
}

void test_MapLoad_ValidAndOptionalBusStops() {
    std::string testName = "MapLoad: valid bus stops load and legacy maps remain compatible";
    Graph withStops;
    Graph legacy;
    std::string error;

    const bool loadedStops = loadMap(mapJsonWithBusStops(R"JSON([
        {"id": 501, "name": "Central Market", "roadId": 12,
         "positionRatio": 0.25, "lane": 1, "dwellTime": 7.5},
        {"id": 502, "name": "Return Stop", "roadId": -12,
         "positionRatio": 0.6, "lane": 0}
    ])JSON"), withStops, error);

    const std::string legacyJson = R"JSON({
        "intersections": [{"id": 1}, {"id": 2}],
        "roads": [{"id": 12, "start": 1, "end": 2,
                   "distance": 5.0, "speedLimit": 40.0}]
    })JSON";
    std::string legacyError;
    const bool loadedLegacy = loadMap(legacyJson, legacy, legacyError);

    Road* forward = withStops.getRoad(12);
    Road* reverse = withStops.getRoad(-12);
    const BusStop* stop = forward ? forward->findBusStopById(501) : nullptr;
    const bool passed = loadedStops && loadedLegacy &&
                        forward != nullptr && reverse != nullptr &&
                        forward->getBusStops().size() == 1 &&
                        reverse->getBusStops().size() == 1 &&
                        stop != nullptr &&
                        stop->getName() == "Central Market" &&
                        stop->getLaneIndex() == 1 &&
                        nearlyEqual(stop->getPositionOnRoad(), forward->getDistance() * 0.25) &&
                        nearlyEqual(stop->getDwellTime(), 7.5) &&
                        legacy.getRoad(12)->getBusStops().empty();

    std::ostringstream d;
    d << "  Expected: direction-specific stops loaded; old JSON without busStops succeeds\n";
    d << "  Errors: stops='" << error << "' legacy='" << legacyError << "'\n";
    reportResult(testName, passed, d.str());
}

void test_MapLoad_RejectsInvalidBusStopContainerAndFields() {
    std::string testName = "MapLoad: invalid bus stop container and field types are rejected";
    const std::vector<std::string> invalidSections = {
        "{}",
        "[42]",
        R"JSON([{"roadId":12,"positionRatio":0.5}])JSON",
        R"JSON([{"id":501,"positionRatio":0.5}])JSON",
        R"JSON([{"id":501,"roadId":12}])JSON",
        R"JSON([{"id":501,"roadId":12,"positionRatio":0.5,"name":4}])JSON"
    };

    bool passed = true;
    std::string lastError;
    for (const auto& section : invalidSections) {
        Graph graph;
        if (loadMap(mapJsonWithBusStops(section), graph, lastError) ||
            lastError.find("Bus stop") == std::string::npos &&
            lastError.find("busStops") == std::string::npos) {
            passed = false;
            break;
        }
    }

    std::ostringstream d;
    d << "  Expected: every malformed section fails with bus-stop context\n";
    d << "  Last error: " << lastError << "\n";
    reportResult(testName, passed, d.str());
}

void test_MapLoad_RejectsInvalidBusStopValues() {
    std::string testName = "MapLoad: invalid road, ratio, lane, dwell, duplicate ID and spacing are rejected";
    const std::vector<std::string> invalidSections = {
        R"JSON([{"id":501,"roadId":999,"positionRatio":0.5}])JSON",
        R"JSON([{"id":501,"roadId":12,"positionRatio":0.0}])JSON",
        R"JSON([{"id":501,"roadId":12,"positionRatio":1.0}])JSON",
        R"JSON([{"id":501,"roadId":12,"positionRatio":-0.1}])JSON",
        R"JSON([{"id":501,"roadId":12,"positionRatio":1.1}])JSON",
        R"JSON([{"id":501,"roadId":12,"positionRatio":0.5,"lane":2}])JSON",
        R"JSON([{"id":501,"roadId":12,"positionRatio":0.5,"dwellTime":-1.0}])JSON",
        R"JSON([
            {"id":501,"roadId":12,"positionRatio":0.2},
            {"id":501,"roadId":-12,"positionRatio":0.8}
        ])JSON",
        R"JSON([
            {"id":501,"roadId":12,"positionRatio":0.50000},
            {"id":502,"roadId":12,"positionRatio":0.50001}
        ])JSON"
    };

    bool passed = true;
    std::string lastError;
    for (const auto& section : invalidSections) {
        Graph graph;
        if (loadMap(mapJsonWithBusStops(section), graph, lastError)) {
            passed = false;
            break;
        }
    }

    std::ostringstream d;
    d << "  Expected: every invalid value is rejected\n";
    d << "  Last error: " << lastError << "\n";
    reportResult(testName, passed, d.str());
}

// ----------------------------------------------------------------------------
// Test Cases for Graph
// ----------------------------------------------------------------------------

void test_Graph_AddAndRetrieve() {
    std::string testName = "Graph: Successfully adds and retrieves Intersections and Roads";
    Graph g;
    
    g.addIntersection(new Intersection(1, 10.0, 10.0));
    g.addIntersection(new Intersection(2, 20.0, 20.0));
    
    bool passNodes = (g.getIntersection(1) != nullptr) && (g.getIntersection(2) != nullptr);

    g.addRoad(new Road(101, "Test Road", g.getIntersection(1), g.getIntersection(2), 14.14, 50.0));
    
    bool passRoads = (g.getRoad(101) != nullptr);
    bool passTopology = g.getIntersection(1)->getOutgoingRoads().size() == 1 &&
                        g.getIntersection(2)->getIncomingRoads().size() == 1;

    bool passed = passNodes && passRoads && passTopology;

    std::ostringstream d;
    d << "  Expected: Nodes and Roads accessible, topologies correctly linked\n";
    d << "  Actual:   nodesOK=" << passNodes << " roadsOK=" << passRoads << " topoOK=" << passTopology << "\n";
    reportResult(testName, passed, d.str());
}

void test_Graph_CascadingRemoval() {
    std::string testName = "Graph: Removing an intersection cascades and removes connected roads";
    Graph g;
    
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 10.0, 0.0));
    g.addIntersection(new Intersection(3, 20.0, 0.0));

    g.addRoad(new Road(101, "Test Road", g.getIntersection(1), g.getIntersection(2), 10.0, 50.0));
    g.addRoad(new Road(102, "Test Road", g.getIntersection(2), g.getIntersection(3), 10.0, 50.0));

    // Xóa nút 2 ở giữa -> Đường 101 và 102 phải bay màu theo
    g.removeIntersection(2);

    bool passNodeRemoved = (g.getIntersection(2) == nullptr);
    bool passRoad101Removed = (g.getRoad(101) == nullptr);
    bool passRoad102Removed = (g.getRoad(102) == nullptr);
    
    // Nút 1 và 3 vẫn phải tồn tại, nhưng danh sách đường kết nối phải trống
    bool passTopologyCleaned = g.getIntersection(1) != nullptr && g.getIntersection(1)->getOutgoingRoads().empty() &&
                               g.getIntersection(3) != nullptr && g.getIntersection(3)->getIncomingRoads().empty();

    bool passed = passNodeRemoved && passRoad101Removed && passRoad102Removed && passTopologyCleaned;

    std::ostringstream d;
    d << "  Expected: Node 2 gone, Roads 101 & 102 gone, Nodes 1 & 3 have 0 connections\n";
    d << "  Actual:   Road 101 exists? " << (g.getRoad(101) != nullptr) << "\n";
    reportResult(testName, passed, d.str());
}

// ----------------------------------------------------------------------------
// main
// ----------------------------------------------------------------------------

int main() {
    std::cout << "============================================================\n";
    std::cout << " GRAPH & ROAD MODEL UNIT TESTS\n";
    std::cout << "============================================================\n\n";

    test_Road_TravelCost_And_Congestion();
    test_Road_BusStopLogic();
    test_Road_BusStopModelLogic();
    test_Road_CurbLaneConventionForBothDirections();
    test_Map4_RoadsideBusStopsUseDirectionalCurbLane();
    test_MapLoad_ValidAndOptionalBusStops();
    test_MapLoad_RejectsInvalidBusStopContainerAndFields();
    test_MapLoad_RejectsInvalidBusStopValues();
    
    test_Graph_AddAndRetrieve();
    test_Graph_CascadingRemoval();

    printSummary();

    return (TestFramework::g_failed == 0) ? 0 : 1;
}
