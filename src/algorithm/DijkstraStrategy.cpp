#include "DijkstraStrategy.h"

#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>

#include "../model/Graph.h"
#include "../model/Intersection.h"
#include "../model/Road.h"

namespace {
// Min-heap entry: (costSoFar, intersectionId)
using PQEntry = std::pair<double, int>;
struct ComparePQ {
    bool operator()(const PQEntry& a, const PQEntry& b) const {
        return a.first > b.first; // smaller cost = higher priority
    }
};
}

PathResult DijkstraStrategy::findPath(const Graph& graph, int startId, int goalId) const {
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

    std::unordered_map<int, double> bestCost;
    std::unordered_map<int, int> cameFromNode;
    std::unordered_map<int, Road*> cameFromRoad;
    std::unordered_map<int, bool> finalized;

    for (Intersection* node : graph.getAllIntersections()) {
        bestCost[node->getId()] = INF;
    }
    bestCost[startId] = 0.0;

    std::priority_queue<PQEntry, std::vector<PQEntry>, ComparePQ> pq;
    pq.push({ 0.0, startId });

    bool reached = false;

    while (!pq.empty()) {
        auto [costSoFar, currentId] = pq.top();
        pq.pop();

        if (finalized[currentId]) continue; // stale entry, already processed with a better cost
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

            double edgeCost = road->getTravelCost(); // infinity if blocked
            if (edgeCost == INF) continue;            // skip impassable roads

            int neighborId = road->getEnd()->getId();
            if (finalized[neighborId]) continue;

            double candidate = costSoFar + edgeCost;
            if (candidate < bestCost[neighborId]) {
                bestCost[neighborId] = candidate;
                cameFromNode[neighborId] = currentId;
                cameFromRoad[neighborId] = road;
                pq.push({ candidate, neighborId });
            }
        }
    }

    if (!reached || bestCost[goalId] == INF) {
        return result; // unreachable (e.g. blocked off completely)
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
    result.totalCost = bestCost[goalId];
    return result;
}
