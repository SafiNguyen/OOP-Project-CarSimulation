// ============================================================================
// test_pathfinding.cpp
// ----------------------------------------------------------------------------
// Standalone unit tests for BFSStrategy, DijkstraStrategy and AStarStrategy.
// No external testing framework is used: each test builds its own small
// Graph (the "input"), runs the strategy under test, prints the input
// parameters and the resulting output to the console, and compares the
// output against a hand-computed expected result.
//
// Build (from project root, after copying these files into your real
// src/ tree):
//   g++ -std=c++17 -Isrc tests/test_pathfinding.cpp \
//       src/model/Graph.cpp src/model/Intersection.cpp src/model/Road.cpp \
//       src/algorithm/BFSStrategy.cpp src/algorithm/DijkstraStrategy.cpp \
//       src/algorithm/AStarStrategy.cpp -o test_pathfinding
//   ./test_pathfinding
// ============================================================================

#include <iostream>
#include <sstream>
#include <memory>

#include "../tests/TestFramework.h"
#include "../src/model/Graph.h"
#include "../src/model/Intersection.h"
#include "../src/model/Road.h"
#include "../src/algorithm/PathFindingStrategy.h"
#include "../src/algorithm/BFSStrategy.h"
#include "../src/algorithm/DijkstraStrategy.h"
#include "../src/algorithm/AStarStrategy.h"

using TestFramework::reportResult;
using TestFramework::nearlyEqual;
using TestFramework::printSummary;

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

std::string pathToString(const std::vector<int>& path) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < path.size(); ++i) {
        oss << path[i];
        if (i + 1 < path.size()) oss << " -> ";
    }
    oss << "]";
    return oss.str();
}

bool pathEquals(const std::vector<int>& a, const std::vector<int>& b) {
    return a == b;
}

// Builds the simple square map described in map.json:
//
//   4 ---- 3
//   |      |
//   1 ---- 2 ---- 5
//
// Roads: 1->2->3->4->1 form a square (distance 100, speed 50).
// 2->5 (dist 111.8, speed 40, congestion 1.5), 5->3 (dist 111.8, speed 40, BLOCKED).
Graph buildSquareGraphWithBranch() {
    Graph g;
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 100.0, 0.0));
    g.addIntersection(new Intersection(3, 100.0, 100.0));
    g.addIntersection(new Intersection(4, 0.0, 100.0));
    g.addIntersection(new Intersection(5, 50.0, 150.0));

    g.addRoad(new Road(101, g.getIntersection(1), g.getIntersection(2), 100.0, 50.0));
    g.addRoad(new Road(102, g.getIntersection(2), g.getIntersection(3), 100.0, 50.0, 1.2));
    g.addRoad(new Road(103, g.getIntersection(3), g.getIntersection(4), 100.0, 50.0));
    g.addRoad(new Road(104, g.getIntersection(4), g.getIntersection(1), 100.0, 50.0));
    g.addRoad(new Road(105, g.getIntersection(2), g.getIntersection(5), 111.8, 40.0, 1.5));
    Road* r106 = new Road(106, g.getIntersection(5), g.getIntersection(3), 111.8, 40.0);
    r106->blockRoad();
    g.addRoad(r106);

    return g;
}

// A plain 4-node line graph: 1 -> 2 -> 3 -> 4, all equal weight.
// Used for simple, easy-to-verify-by-hand BFS/Dijkstra/A* checks.
Graph buildLineGraph() {
    Graph g;
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 10.0, 0.0));
    g.addIntersection(new Intersection(3, 20.0, 0.0));
    g.addIntersection(new Intersection(4, 30.0, 0.0));

    g.addRoad(new Road(1, g.getIntersection(1), g.getIntersection(2), 10.0, 10.0));
    g.addRoad(new Road(2, g.getIntersection(2), g.getIntersection(3), 10.0, 10.0));
    g.addRoad(new Road(3, g.getIntersection(3), g.getIntersection(4), 10.0, 10.0));

    return g;
}

// Diamond graph used to prove that Dijkstra/A* pick the LOWER-COST path
// even when it has MORE hops than a shorter-hop, more congested path:
//
//        2
//      /   \
//   1         4
//      \   /
//        3
//
// 1->2 short but very congested (high travel cost).
// 1->3->4 longer distance but free-flowing (lower travel cost).
Graph buildCongestionDiamond() {
    Graph g;
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 50.0, 50.0));
    g.addIntersection(new Intersection(3, 50.0, -50.0));
    g.addIntersection(new Intersection(4, 100.0, 0.0));

    // 1 -> 2 -> 4: very short distance (10 + 10 = 20) but congestionLevel 10
    // travel cost = distance / (speed / congestion) = 10 / (10/10) = 10 each => 20 total
    g.addRoad(new Road(1, g.getIntersection(1), g.getIntersection(2), 10.0, 10.0, 10.0));
    g.addRoad(new Road(2, g.getIntersection(2), g.getIntersection(4), 10.0, 10.0, 10.0));

    // 1 -> 3 -> 4: longer distance (40 + 40 = 80) but free-flowing (congestion 1)
    // travel cost = 40 / (40/1) = 1 each => 2 total. Much cheaper!
    g.addRoad(new Road(3, g.getIntersection(1), g.getIntersection(3), 40.0, 40.0, 1.0));
    g.addRoad(new Road(4, g.getIntersection(3), g.getIntersection(4), 40.0, 40.0, 1.0));

    return g;
}

// ----------------------------------------------------------------------------
// Test cases
// ----------------------------------------------------------------------------

void test_BFS_LineGraph_FindsShortestHopPath() {
    std::string testName = "BFS: simple line graph 1->2->3->4";
    Graph g = buildLineGraph();
    BFSStrategy bfs;

    int startId = 1, goalId = 4;
    PathResult res = bfs.findPath(g, startId, goalId);

    std::vector<int> expectedPath = {1, 2, 3, 4};
    double expectedHops = 3.0;

    bool passed = res.found &&
                  pathEquals(res.intersectionPath, expectedPath) &&
                  nearlyEqual(res.totalCost, expectedHops);

    std::ostringstream d;
    d << "  Input:    graph=line(1-2-3-4), start=" << startId << ", goal=" << goalId << "\n";
    d << "  Expected: found=true, path=" << pathToString(expectedPath)
      << ", cost(hops)=" << expectedHops << "\n";
    d << "  Actual:   found=" << res.found << ", path=" << pathToString(res.intersectionPath)
      << ", cost(hops)=" << res.totalCost << ", nodesExplored=" << res.nodesExplored << "\n";
    reportResult(testName, passed, d.str());
}

void test_BFS_IgnoresBlockedRoad_TakesDetour() {
    std::string testName = "BFS: blocked road forces detour (square+branch map)";
    Graph g = buildSquareGraphWithBranch();
    BFSStrategy bfs;

    // Road 5->3 (106) is blocked. Going 1->2->5 is a dead end since 5->3 is blocked.
    // The only way from 1 to 3 is via 1->2->3 (2 hops).
    int startId = 1, goalId = 3;
    PathResult res = bfs.findPath(g, startId, goalId);

    std::vector<int> expectedPath = {1, 2, 3};
    double expectedHops = 2.0;

    bool passed = res.found &&
                  pathEquals(res.intersectionPath, expectedPath) &&
                  nearlyEqual(res.totalCost, expectedHops);

    std::ostringstream d;
    d << "  Input:    graph=square+branch (road 106 [5->3] BLOCKED), start=" << startId
      << ", goal=" << goalId << "\n";
    d << "  Expected: found=true, path=" << pathToString(expectedPath)
      << ", cost(hops)=" << expectedHops << " (BFS must avoid the blocked dead-end via node 5)\n";
    d << "  Actual:   found=" << res.found << ", path=" << pathToString(res.intersectionPath)
      << ", cost(hops)=" << res.totalCost << ", nodesExplored=" << res.nodesExplored << "\n";
    reportResult(testName, passed, d.str());
}

void test_BFS_StartEqualsGoal() {
    std::string testName = "BFS: start == goal returns trivial single-node path";
    Graph g = buildLineGraph();
    BFSStrategy bfs;

    int startId = 2, goalId = 2;
    PathResult res = bfs.findPath(g, startId, goalId);

    bool passed = res.found && res.intersectionPath.size() == 1 &&
                  res.intersectionPath[0] == 2 && nearlyEqual(res.totalCost, 0.0);

    std::ostringstream d;
    d << "  Input:    graph=line(1-2-3-4), start=" << startId << ", goal=" << goalId << "\n";
    d << "  Expected: found=true, path=[2], cost=0\n";
    d << "  Actual:   found=" << res.found << ", path=" << pathToString(res.intersectionPath)
      << ", cost=" << res.totalCost << "\n";
    reportResult(testName, passed, d.str());
}

void test_BFS_UnreachableGoal_ReturnsNotFound() {
    std::string testName = "BFS: disconnected node is correctly reported unreachable";
    Graph g = buildLineGraph();
    // Add an isolated node with no roads connecting it to the rest of the graph.
    g.addIntersection(new Intersection(99, 999.0, 999.0));
    BFSStrategy bfs;

    int startId = 1, goalId = 99;
    PathResult res = bfs.findPath(g, startId, goalId);

    bool passed = (res.found == false);

    std::ostringstream d;
    d << "  Input:    graph=line(1-2-3-4)+isolated node 99, start=" << startId
      << ", goal=" << goalId << "\n";
    d << "  Expected: found=false (no road reaches node 99)\n";
    d << "  Actual:   found=" << res.found << "\n";
    reportResult(testName, passed, d.str());
}

void test_BFS_InvalidNode_ReturnsNotFound() {
    std::string testName = "BFS: nonexistent intersection id is handled gracefully";
    Graph g = buildLineGraph();
    BFSStrategy bfs;

    int startId = 1, goalId = 12345; // does not exist in the graph
    PathResult res = bfs.findPath(g, startId, goalId);

    bool passed = (res.found == false);

    std::ostringstream d;
    d << "  Input:    graph=line(1-2-3-4), start=" << startId << ", goal=" << goalId
      << " (does not exist)\n";
    d << "  Expected: found=false (no crash, no exception)\n";
    d << "  Actual:   found=" << res.found << "\n";
    reportResult(testName, passed, d.str());
}

void test_Dijkstra_LineGraph_MatchesBFS_OnUniformWeights() {
    std::string testName = "Dijkstra: line graph with uniform edge cost matches BFS path";
    Graph g = buildLineGraph();
    DijkstraStrategy dijkstra;

    int startId = 1, goalId = 4;
    PathResult res = dijkstra.findPath(g, startId, goalId);

    // Each road: distance 10, speed 10, congestion 1 => travelCost = 10/(10/1) = 1.0
    // Total path 1->2->3->4 = 3 roads => totalCost should be 3.0
    std::vector<int> expectedPath = {1, 2, 3, 4};
    double expectedCost = 3.0;

    bool passed = res.found &&
                  pathEquals(res.intersectionPath, expectedPath) &&
                  nearlyEqual(res.totalCost, expectedCost);

    std::ostringstream d;
    d << "  Input:    graph=line(1-2-3-4), all roads dist=10 speed=10 congestion=1, start="
      << startId << ", goal=" << goalId << "\n";
    d << "  Expected: found=true, path=" << pathToString(expectedPath)
      << ", totalCost=" << expectedCost << "\n";
    d << "  Actual:   found=" << res.found << ", path=" << pathToString(res.intersectionPath)
      << ", totalCost=" << res.totalCost << ", nodesExplored=" << res.nodesExplored << "\n";
    reportResult(testName, passed, d.str());
}

void test_Dijkstra_AvoidsCongestion_PrefersLongerFasterRoute() {
    std::string testName = "Dijkstra: prefers longer-but-faster route over short-but-congested route";
    Graph g = buildCongestionDiamond();
    DijkstraStrategy dijkstra;

    int startId = 1, goalId = 4;
    PathResult res = dijkstra.findPath(g, startId, goalId);

    // Expected: path via node 3 (cost 2.0), NOT via node 2 (cost 20.0)
    std::vector<int> expectedPath = {1, 3, 4};
    double expectedCost = 2.0;

    bool passed = res.found &&
                  pathEquals(res.intersectionPath, expectedPath) &&
                  nearlyEqual(res.totalCost, expectedCost);

    std::ostringstream d;
    d << "  Input:    diamond graph; 1->2->4 short but congestion=10 (cost 20); "
         "1->3->4 longer but congestion=1 (cost 2); start=" << startId << ", goal=" << goalId << "\n";
    d << "  Expected: found=true, path=" << pathToString(expectedPath)
      << ", totalCost=" << expectedCost << " (avoids the congested shortcut)\n";
    d << "  Actual:   found=" << res.found << ", path=" << pathToString(res.intersectionPath)
      << ", totalCost=" << res.totalCost << ", nodesExplored=" << res.nodesExplored << "\n";
    reportResult(testName, passed, d.str());
}

void test_Dijkstra_BlockedRoad_RoutesAround() {
    std::string testName = "Dijkstra: blocked road is treated as infinite cost and avoided";
    Graph g = buildSquareGraphWithBranch();
    DijkstraStrategy dijkstra;

    // Path 2->5->3 has road 106 (5->3) blocked, so even though 2->5->3
    // might look attractive by distance, Dijkstra must route 2->3 directly.
    int startId = 2, goalId = 3;
    PathResult res = dijkstra.findPath(g, startId, goalId);

    std::vector<int> expectedPath = {2, 3}; // direct road 102, cost = 100/(50/1.2) = 2.4
    double expectedCost = 100.0 / (50.0 / 1.2);

    bool passed = res.found &&
                  pathEquals(res.intersectionPath, expectedPath) &&
                  nearlyEqual(res.totalCost, expectedCost, 1e-3);

    std::ostringstream d;
    d << "  Input:    square+branch map, road 106 [5->3] BLOCKED, start=" << startId
      << ", goal=" << goalId << "\n";
    d << "  Expected: found=true, path=" << pathToString(expectedPath)
      << ", totalCost~=" << expectedCost << " (direct road, since 5->3 is blocked)\n";
    d << "  Actual:   found=" << res.found << ", path=" << pathToString(res.intersectionPath)
      << ", totalCost=" << res.totalCost << ", nodesExplored=" << res.nodesExplored << "\n";
    reportResult(testName, passed, d.str());
}

void test_Dijkstra_UnreachableGoal_ReturnsNotFound() {
    std::string testName = "Dijkstra: fully blocked-off destination correctly reports not found";
    Graph g = buildLineGraph();
    // Block the only road leading out of node 1.
    g.getRoad(1)->blockRoad();
    DijkstraStrategy dijkstra;

    int startId = 1, goalId = 4;
    PathResult res = dijkstra.findPath(g, startId, goalId);

    bool passed = (res.found == false);

    std::ostringstream d;
    d << "  Input:    graph=line(1-2-3-4), road 1 [1->2] BLOCKED (only exit from node 1), start="
      << startId << ", goal=" << goalId << "\n";
    d << "  Expected: found=false (node 1 is completely cut off)\n";
    d << "  Actual:   found=" << res.found << "\n";
    reportResult(testName, passed, d.str());
}

void test_AStar_LineGraph_MatchesDijkstraCost() {
    std::string testName = "A*: line graph optimal cost matches Dijkstra's optimal cost";
    Graph g = buildLineGraph();
    DijkstraStrategy dijkstra;
    AStarStrategy astar;

    int startId = 1, goalId = 4;
    PathResult dijkstraRes = dijkstra.findPath(g, startId, goalId);
    PathResult astarRes = astar.findPath(g, startId, goalId);

    bool passed = astarRes.found && dijkstraRes.found &&
                  pathEquals(astarRes.intersectionPath, dijkstraRes.intersectionPath) &&
                  nearlyEqual(astarRes.totalCost, dijkstraRes.totalCost);

    std::ostringstream d;
    d << "  Input:    graph=line(1-2-3-4), start=" << startId << ", goal=" << goalId << "\n";
    d << "  Expected: A* path/cost == Dijkstra path/cost (A* must stay optimal)\n";
    d << "  Actual:   Dijkstra path=" << pathToString(dijkstraRes.intersectionPath)
      << " cost=" << dijkstraRes.totalCost << " nodesExplored=" << dijkstraRes.nodesExplored << "\n";
    d << "            A*       path=" << pathToString(astarRes.intersectionPath)
      << " cost=" << astarRes.totalCost << " nodesExplored=" << astarRes.nodesExplored << "\n";
    reportResult(testName, passed, d.str());
}

void test_AStar_AvoidsCongestion_SameAsDijkstra() {
    std::string testName = "A*: also prefers longer-but-faster route (stays optimal under congestion)";
    Graph g = buildCongestionDiamond();
    AStarStrategy astar;

    int startId = 1, goalId = 4;
    PathResult res = astar.findPath(g, startId, goalId);

    std::vector<int> expectedPath = {1, 3, 4};
    double expectedCost = 2.0;

    bool passed = res.found &&
                  pathEquals(res.intersectionPath, expectedPath) &&
                  nearlyEqual(res.totalCost, expectedCost);

    std::ostringstream d;
    d << "  Input:    diamond graph (same as Dijkstra congestion test), start=" << startId
      << ", goal=" << goalId << "\n";
    d << "  Expected: found=true, path=" << pathToString(expectedPath)
      << ", totalCost=" << expectedCost << "\n";
    d << "  Actual:   found=" << res.found << ", path=" << pathToString(res.intersectionPath)
      << ", totalCost=" << res.totalCost << ", nodesExplored=" << res.nodesExplored << "\n";
    reportResult(testName, passed, d.str());
}

void test_AStar_ExploresFewerOrEqualNodesThanDijkstra_OnLargerMap() {
    std::string testName = "A*: explores fewer-or-equal nodes than Dijkstra on a larger map "
                            "(demonstrates speed optimization via heuristic)";
    // Build a bigger grid (5x5) so the heuristic has room to prune the search.
    Graph g;
    const int N = 5;
    for (int row = 0; row < N; ++row) {
        for (int col = 0; col < N; ++col) {
            int id = row * N + col + 1;
            g.addIntersection(new Intersection(id, col * 50.0, row * 50.0));
        }
    }
    int roadId = 1;
    for (int row = 0; row < N; ++row) {
        for (int col = 0; col < N; ++col) {
            int id = row * N + col + 1;
            if (col + 1 < N) {
                int rightId = row * N + (col + 1) + 1;
                g.addRoad(new Road(roadId++, g.getIntersection(id), g.getIntersection(rightId), 50.0, 50.0));
                g.addRoad(new Road(roadId++, g.getIntersection(rightId), g.getIntersection(id), 50.0, 50.0));
            }
            if (row + 1 < N) {
                int downId = (row + 1) * N + col + 1;
                g.addRoad(new Road(roadId++, g.getIntersection(id), g.getIntersection(downId), 50.0, 50.0));
                g.addRoad(new Road(roadId++, g.getIntersection(downId), g.getIntersection(id), 50.0, 50.0));
            }
        }
    }

    DijkstraStrategy dijkstra;
    AStarStrategy astar;

    int startId = 1;          // top-left corner
    int goalId = N * N;        // bottom-right corner

    PathResult dijkstraRes = dijkstra.findPath(g, startId, goalId);
    PathResult astarRes = astar.findPath(g, startId, goalId);

    bool passed = astarRes.found && dijkstraRes.found &&
                  nearlyEqual(astarRes.totalCost, dijkstraRes.totalCost) &&
                  astarRes.nodesExplored <= dijkstraRes.nodesExplored;

    std::ostringstream d;
    d << "  Input:    5x5 grid graph (25 nodes, uniform 50.0 cost edges both directions), start="
      << startId << ", goal=" << goalId << "\n";
    d << "  Expected: same optimal cost as Dijkstra, with nodesExplored(A*) <= nodesExplored(Dijkstra)\n";
    d << "  Actual:   Dijkstra cost=" << dijkstraRes.totalCost
      << " nodesExplored=" << dijkstraRes.nodesExplored << "\n";
    d << "            A*       cost=" << astarRes.totalCost
      << " nodesExplored=" << astarRes.nodesExplored << "\n";
    reportResult(testName, passed, d.str());
}

void test_AStar_UnreachableGoal_ReturnsNotFound() {
    std::string testName = "A*: unreachable goal (blocked exit) correctly reports not found";
    Graph g = buildLineGraph();
    g.getRoad(1)->blockRoad(); // block the only road out of node 1
    AStarStrategy astar;

    int startId = 1, goalId = 4;
    PathResult res = astar.findPath(g, startId, goalId);

    bool passed = (res.found == false);

    std::ostringstream d;
    d << "  Input:    graph=line(1-2-3-4), road 1 [1->2] BLOCKED, start=" << startId
      << ", goal=" << goalId << "\n";
    d << "  Expected: found=false\n";
    d << "  Actual:   found=" << res.found << "\n";
    reportResult(testName, passed, d.str());
}

void test_Polymorphism_StrategyPattern_RuntimeSwitch() {
    std::string testName = "Strategy Pattern: algorithms are interchangeable via base pointer (polymorphism)";
    Graph g = buildSquareGraphWithBranch();

    std::vector<std::unique_ptr<PathFindingStrategy>> strategies;
    strategies.push_back(std::make_unique<BFSStrategy>());
    strategies.push_back(std::make_unique<DijkstraStrategy>());
    strategies.push_back(std::make_unique<AStarStrategy>());

    int startId = 1, goalId = 3;
    bool allFound = true;

    std::ostringstream d;
    d << "  Input:    square+branch map, start=" << startId << ", goal=" << goalId
      << ", iterating over std::vector<unique_ptr<PathFindingStrategy>>\n";
    d << "  Expected: every strategy (BFS/Dijkstra/A*) finds *a* valid path through the same interface\n";
    for (auto& strategy : strategies) {
        PathResult res = strategy->findPath(g, startId, goalId);
        d << "  Actual:   " << strategy->name() << " -> found=" << res.found
          << ", path=" << pathToString(res.intersectionPath) << ", cost=" << res.totalCost << "\n";
        allFound = allFound && res.found;
    }

    reportResult(testName, allFound, d.str());
}

// ----------------------------------------------------------------------------
// main
// ----------------------------------------------------------------------------

int main() {
    std::cout << "============================================================\n";
    std::cout << " PATHFINDING ALGORITHM UNIT TESTS (BFS / Dijkstra / A*)\n";
    std::cout << "============================================================\n\n";

    test_BFS_LineGraph_FindsShortestHopPath();
    test_BFS_IgnoresBlockedRoad_TakesDetour();
    test_BFS_StartEqualsGoal();
    test_BFS_UnreachableGoal_ReturnsNotFound();
    test_BFS_InvalidNode_ReturnsNotFound();

    test_Dijkstra_LineGraph_MatchesBFS_OnUniformWeights();
    test_Dijkstra_AvoidsCongestion_PrefersLongerFasterRoute();
    test_Dijkstra_BlockedRoad_RoutesAround();
    test_Dijkstra_UnreachableGoal_ReturnsNotFound();

    test_AStar_LineGraph_MatchesDijkstraCost();
    test_AStar_AvoidsCongestion_SameAsDijkstra();
    test_AStar_ExploresFewerOrEqualNodesThanDijkstra_OnLargerMap();
    test_AStar_UnreachableGoal_ReturnsNotFound();

    test_Polymorphism_StrategyPattern_RuntimeSwitch();

    printSummary();

    return (TestFramework::g_failed == 0) ? 0 : 1;
}
