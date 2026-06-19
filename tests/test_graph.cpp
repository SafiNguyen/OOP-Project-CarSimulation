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

#include "../tests/TestFramework.h"
#include "../src/model/Graph.h"
#include "../src/model/Intersection.h"
#include "../src/model/Road.h"

using TestFramework::reportResult;
using TestFramework::nearlyEqual;
using TestFramework::printSummary;

// ----------------------------------------------------------------------------
// Test Cases for Road
// ----------------------------------------------------------------------------

void test_Road_TravelCost_And_Congestion() {
    std::string testName = "Road: Travel time adapts to congestion and respects blocked status";
    
    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    // Quãng đường 100m, tốc độ tối đa 50m/s, kẹt xe mức 2.0
    Road road(101, &i1, &i2, 100.0, 50.0, 2.0); 

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
    Road road(101, &i1, &i2, 100.0, 50.0);

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

// ----------------------------------------------------------------------------
// Test Cases for Graph
// ----------------------------------------------------------------------------

void test_Graph_AddAndRetrieve() {
    std::string testName = "Graph: Successfully adds and retrieves Intersections and Roads";
    Graph g;
    
    g.addIntersection(new Intersection(1, 10.0, 10.0));
    g.addIntersection(new Intersection(2, 20.0, 20.0));
    
    bool passNodes = (g.getIntersection(1) != nullptr) && (g.getIntersection(2) != nullptr);

    g.addRoad(new Road(101, g.getIntersection(1), g.getIntersection(2), 14.14, 50.0));
    
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

    g.addRoad(new Road(101, g.getIntersection(1), g.getIntersection(2), 10.0, 50.0));
    g.addRoad(new Road(102, g.getIntersection(2), g.getIntersection(3), 10.0, 50.0));

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
    
    test_Graph_AddAndRetrieve();
    test_Graph_CascadingRemoval();

    printSummary();

    return (TestFramework::g_failed == 0) ? 0 : 1;
}