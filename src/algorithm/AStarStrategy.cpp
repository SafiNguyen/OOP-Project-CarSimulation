#include "AStarStrategy.h"

#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cmath>

#include "../model/Graph.h"
#include "../model/Intersection.h"
#include "../model/Road.h"

namespace {
// Min-heap entry: (estimatedTotalCost = g + h, intersectionId)
using PQEntry = std::pair<double, int>;
struct ComparePQ {
    bool operator()(const PQEntry& a, const PQEntry& b) const {
        return a.first > b.first;
    }
};

double euclidean(const Intersection* a, const Intersection* b) {
    double dx = a->getX() - b->getX();
    double dy = a->getY() - b->getY();
    return std::sqrt(dx * dx + dy * dy);
}

// Finds the highest speed limit present anywhere in the graph so the
// heuristic stays admissible (never overestimates true travel cost).
double findMaxSpeedLimit(const Graph& graph) {
    double maxSpeed = 1.0; // avoid division by zero on empty graphs
    for (Road* road : graph.getAllRoads()) {
        if (road != nullptr && road->getSpeedLimit() > maxSpeed) {
            maxSpeed = road->getSpeedLimit();
        }
    }
    return maxSpeed;
}
}

PathResult AStarStrategy::findPath(const Graph& graph, int startId, int goalId) const {
    PathResult result;

    Intersection* startNode = graph.getIntersection(startId);
    Intersection* goalNode = graph.getIntersection(goalId);
    if (startNode == nullptr || goalNode == nullptr) {
        return result;
    }

    if (startId == goalId) {
        result.found = true;
        result.intersectionPath = { startId };
        result.totalCost = 0.0;
        return result;
    }

    const double INF = std::numeric_limits<double>::infinity();
    const double maxSpeed = findMaxSpeedLimit(graph);

    auto heuristic = [&](int id) -> double {
        Intersection* node = graph.getIntersection(id);
        if (node == nullptr) return 0.0;
        return euclidean(node, goalNode) / maxSpeed;
    };

    std::unordered_map<int, double> gScore; // best known cost from start
    std::unordered_map<int, int> cameFromNode;
    std::unordered_map<int, Road*> cameFromRoad;
    std::unordered_map<int, bool> finalized;

    for (Intersection* node : graph.getAllIntersections()) {
        gScore[node->getId()] = INF;
    }
    gScore[startId] = 0.0;

    std::priority_queue<PQEntry, std::vector<PQEntry>, ComparePQ> openSet;
    openSet.push({ heuristic(startId), startId });

    bool reached = false;

    while (!openSet.empty()) {
        auto [estimatedCost, currentId] = openSet.top();
        openSet.pop();

        if (finalized[currentId]) continue;
        finalized[currentId] = true;
        result.nodesExplored++;

        if (currentId == goalId) {
            reached = true;
            break;
        }

        Intersection* current = graph.getIntersection(currentId);
        if (current == nullptr) continue;

        for (Road* road : current->getOutgoingRoads()) {
            if (road == nullptr) continue;

            double edgeCost = road->getTravelCost();
            if (edgeCost == INF) continue; // blocked road

            int neighborId = road->getEnd()->getId();
            if (finalized[neighborId]) continue;

            double candidateG = gScore[currentId] + edgeCost;
            if (candidateG < gScore[neighborId]) {
                gScore[neighborId] = candidateG;
                cameFromNode[neighborId] = currentId;
                cameFromRoad[neighborId] = road;
                openSet.push({ candidateG + heuristic(neighborId), neighborId });
            }
        }
    }

    if (!reached || gScore[goalId] == INF) {
        return result;
    }

    std::vector<int> path;
    std::vector<Road*> roads;
    int cursor = goalId;
    while (cursor != startId) {
        path.push_back(cursor);
        roads.push_back(cameFromRoad[cursor]);
        cursor = cameFromNode[cursor];
    }
    path.push_back(startId);

    std::reverse(path.begin(), path.end());
    std::reverse(roads.begin(), roads.end());

    result.found = true;
    result.intersectionPath = path;
    result.roadPath = roads;
    result.totalCost = gScore[goalId];
    return result;
}
