#include "BFSStrategy.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include "../model/Graph.h"
#include "../model/Intersection.h"
#include "../model/Road.h"

PathResult BFSStrategy::findPath(const Graph& graph, int startId, int goalId) const {
    PathResult result;

    Intersection* startNode = graph.getIntersection(startId);
    Intersection* goalNode = graph.getIntersection(goalId);
    if (startNode == nullptr || goalNode == nullptr) {
        return result; // found == false, invalid endpoints
    }

    if (startId == goalId) {
        result.found = true;
        result.intersectionPath = { startId };
        result.totalCost = 0.0;
        return result;
    }

    std::queue<int> frontier;
    std::unordered_set<int> visited;
    std::unordered_map<int, int> cameFromNode;   // child -> parent intersection id
    std::unordered_map<int, Road*> cameFromRoad; // child -> road used to reach it

    frontier.push(startId);
    visited.insert(startId);

    bool reached = false;
    while (!frontier.empty() && !reached) {
        int currentId = frontier.front();
        frontier.pop();
        result.nodesExplored++;

        Intersection* current = graph.getIntersection(currentId);
        if (current == nullptr) continue;

        for (Road* road : current->getOutgoingRoads()) {
            if (road == nullptr || road->isBlocked()) continue; // skip impassable roads

            int neighborId = road->getEnd()->getId();
            if (visited.find(neighborId) != visited.end()) continue;

            visited.insert(neighborId);
            cameFromNode[neighborId] = currentId;
            cameFromRoad[neighborId] = road;

            if (neighborId == goalId) {
                reached = true;
                break;
            }
            frontier.push(neighborId);
        }
    }

    if (!reached) {
        return result; // no path exists
    }

    // Reconstruct path by walking back from goal to start
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
    result.totalCost = static_cast<double>(roads.size()); // number of hops
    return result;
}
