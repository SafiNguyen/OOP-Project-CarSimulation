#include "Graph.h"
#include <cmath>
#include <limits>

Graph::Graph() {
}

Graph::~Graph() {
    clearGraph();
}

Graph::Graph(Graph&& other) noexcept
    : intersections(std::move(other.intersections)),
      roads(std::move(other.roads)) {
    other.intersections.clear();
    other.roads.clear();
}

Graph& Graph::operator=(Graph&& other) noexcept {
    if (this != &other) {
        clearGraph();
        
        intersections = std::move(other.intersections);
        roads = std::move(other.roads);
        
        other.intersections.clear();
        other.roads.clear();
    }
    return *this;
}

// --- Utility ---
void Graph::clearGraph() {
    // Remove all roads via removeRoad() so intersections are updated safely
    std::vector<int> roadIds;
    roadIds.reserve(roads.size());
    for (auto &p : roads) {
        roadIds.push_back(p.first);
    }
    for (int id : roadIds) {
        removeRoad(id);
    }

    // Now delete all intersections
    for (auto &p : intersections) {
        delete p.second;
    }
    intersections.clear();
}

// --- Intersection Operations ---
void Graph::addIntersection(Intersection* intersection) {
    if (intersection == nullptr) return;

    int id = intersection->getId();
    auto it = intersections.find(id);
    if (it != intersections.end()) {
        delete intersection;
        return;
    }

    intersections[id] = intersection;
}

void Graph::removeIntersection(int id) {
    auto it = intersections.find(id);
    if (it != intersections.end()) {
        Intersection* intersection = it->second;
        
// Remove all roads connected to this intersection
        std::vector<int> roadsToRemove;
        for (auto& road_pair : roads) {
            Road* road = road_pair.second;
            if (road->getStart()->getId() == id || road->getEnd()->getId() == id) {
                roadsToRemove.push_back(road->getId());
            }
        }
        
// Remove the roads
        for (int roadId : roadsToRemove) {
            removeRoad(roadId);
        }
        
// Remove the intersection
        intersections.erase(it);
        delete intersection;
    }
}

Intersection* Graph::getIntersection(int id) const {
    auto it = intersections.find(id);
    if (it != intersections.end()) {
        return it->second;
    }
    return nullptr;
}

// --- Road Operations ---
void Graph::addRoad(Road* road) {
    if (road != nullptr) {
// Get the start and end intersections
        Intersection* start = road->getStart();
        Intersection* end = road->getEnd();
        
        if (start == nullptr || end == nullptr) {
            delete road;
            return;
        }

        int id = road->getId();
        if (roads.find(id) != roads.end()) {
            delete road;
            return;
        }

        // Ensure the referenced intersections belong to this graph (by id)
        auto sit = intersections.find(start->getId());
        auto eit = intersections.find(end->getId());
        if (sit == intersections.end() || eit == intersections.end()) {
            // One of the endpoints is not in this graph; reject the road
            delete road;
            return;
        }

        // Add road to the roads map
        roads[id] = road;

        // Update the intersections with incoming and outgoing roads using the stored intersection objects
        sit->second->addOutgoingRoad(road);
        eit->second->addIncomingRoad(road);
    }
    }


void Graph::removeRoad(int id) {
    auto it = roads.find(id);
    if (it != roads.end()) {
        Road* road = it->second;
        
// Get the start and end intersections
        Intersection* start = road->getStart();
        Intersection* end = road->getEnd();
        
        if (start != nullptr && end != nullptr) {
// Remove road from the intersections
            start->removeOutgoingRoad(road);
            end->removeIncomingRoad(road);
        }
        
// Remove from roads map and delete
        roads.erase(it);
        delete road;
    }
}

Road* Graph::getRoad(int id) const {
    auto it = roads.find(id);
    if (it != roads.end()) {
        return it->second;
    }
    return nullptr;
}

std::vector<Intersection*> Graph::getAllIntersections() const {
    std::vector<Intersection*> result;
    result.reserve(intersections.size());
    for (const auto &p : intersections) result.push_back(p.second);
    return result;
}

std::vector<Road*> Graph::getAllRoads() const {
    std::vector<Road*> result;
    result.reserve(roads.size());
    for (const auto &p : roads) result.push_back(p.second);
    return result;
}

// --- Graph Functions ---
void Graph::updateRoadCondition(int roadId, double congestionLevel, bool blocked) {
    Road* road = getRoad(roadId);
    if (road != nullptr) {
        road->updateCongestionLevel(congestionLevel);
        if (blocked) {
            road->blockRoad();
        } else {
            road->unblockRoad();
        }
    }
}

std::vector<Road*> Graph::getConnectedRoads(int intersectionId) const {
    std::vector<Road*> connectedRoads;
    Intersection* intersection = getIntersection(intersectionId);
    
    if (intersection != nullptr) {
// Get all incoming roads
        std::vector<Road*> incoming = intersection->getIncomingRoads();
        connectedRoads.insert(connectedRoads.end(), incoming.begin(), incoming.end());
        
// Get all outgoing roads
        std::vector<Road*> outgoing = intersection->getOutgoingRoads();
        connectedRoads.insert(connectedRoads.end(), outgoing.begin(), outgoing.end());
    }
    
    return connectedRoads;
}

std::vector<Road*> Graph::getNeighbors(int intersectionId) const {
    std::vector<Road*> neighbors;
    Intersection* intersection = getIntersection(intersectionId);
    
    if (intersection != nullptr) {
// Get all outgoing roads (neighbors)
        std::vector<Road*> outgoing = intersection->getOutgoingRoads();
        neighbors.insert(neighbors.end(), outgoing.begin(), outgoing.end());
    }

    return neighbors;
}

double Graph::calculateDistance(int startId, int destId) const {
    Intersection* start = getIntersection(startId);
    Intersection* dest = getIntersection(destId);
    
    if (start == nullptr || dest == nullptr) {
        return std::numeric_limits<double>::infinity();
    }
    
    double dx = dest->getX() - start->getX();
    double dy = dest->getY() - start->getY();
    return std::sqrt(dx * dx + dy * dy);
}