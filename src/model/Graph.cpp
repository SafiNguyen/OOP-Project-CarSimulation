#include "Graph.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <utility>

#include "BusService.h"
#include "SpawnPoint.h"

Graph::Graph() {
}

Graph::~Graph() {
    clearGraph();
}

Graph::Graph(Graph&& other) noexcept
    : intersections(std::move(other.intersections)),
      roads(std::move(other.roads)),
      pois(std::move(other.pois)),
      busStations(std::move(other.busStations)),
      busServices(std::move(other.busServices)) {
    other.intersections.clear();
    other.roads.clear();
    other.pois.clear();
    other.busStations.clear();
    other.busServices.clear();
}

Graph& Graph::operator=(Graph&& other) noexcept {
    if (this != &other) {
        clearGraph();
        
        intersections = std::move(other.intersections);
        roads = std::move(other.roads);
        pois = std::move(other.pois);
        busStations = std::move(other.busStations);
        busServices = std::move(other.busServices);
        
        other.intersections.clear();
        other.roads.clear();
        other.pois.clear();
        other.busStations.clear();
        other.busServices.clear();
    }
    return *this;
}

// --- Utility ---
void Graph::clearGraph() {
    busServices.clear();
    busStations.clear();

    // Remove all roads via removeRoad() so intersections are updated safely
    std::vector<int> roadIds;
    roadIds.reserve(roads.size());
    for (auto &p : roads) {
        roadIds.push_back(p.first);
    }
    for (int id : roadIds) {
        removeRoad(id);
    }

    // Delete all POIs
    for (auto* poi : pois) {
        delete poi;
    }
    pois.clear();

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

        // Pair the two directional objects by topology once when the graph
        // changes. Geometry/render code can then query the counterpart in
        // O(1), without relying on road-id signs or scanning every frame.
        for (const auto& entry : roads) {
            Road* candidate = entry.second;
            if (candidate == nullptr || candidate == road) {
                continue;
            }
            if (candidate->getStart() == end &&
                candidate->getEnd() == start) {
                road->setReverseRoad(candidate);
                candidate->setReverseRoad(road);
                break;
            }
        }

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
        if (road->getReverseRoad() != nullptr) {
            road->getReverseRoad()->setReverseRoad(nullptr);
            road->setReverseRoad(nullptr);
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
void Graph::updateRoadCondition(int roadId, double congestionLevel, bool blocked, int laneIndex) {
    Road* road = getRoad(roadId);
    if (road != nullptr) {
        road->updateCongestionLevel(congestionLevel);
        if (laneIndex == -1) {
            if (blocked) {
                road->blockRoad();
            } else {
                road->unblockRoad();
            }
        } else {
            if (blocked) {
                road->blockLane(laneIndex);
            } else {
                road->unblockLane(laneIndex);
            }
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

// --- POI Operations ---
void Graph::addPOI(PointOfInterest* poi) {
    if (poi) {
        pois.push_back(poi);
    }
}

const std::vector<PointOfInterest*>& Graph::getAllPOIs() const {
    return pois;
}

PointOfInterest* Graph::getPOI(int id) const {
    for (auto* poi : pois) {
        if (poi->getId() == id) return poi;
    }
    return nullptr;
}

PointOfInterest* Graph::getPointOfInterest(int id) const {
    if (PointOfInterest* poi = getPOI(id)) {
        return poi;
    }
    return getBusStation(id);
}

const BusStop* Graph::getBusStop(int id) const {
    for (Road* road : getAllRoads()) {
        if (road != nullptr) {
            if (const BusStop* stop = road->findBusStopById(id)) {
                return stop;
            }
        }
    }
    return nullptr;
}

std::vector<PointOfInterest*> Graph::getSpawnPoints() const {
    std::vector<PointOfInterest*> result;
    for (auto* poi : pois) {
        if (poi->isSpawnPoint()) result.push_back(poi);
    }
    return result;
}

std::vector<PointOfInterest*> Graph::getDestinations() const {
    std::vector<PointOfInterest*> result;
    for (auto* poi : pois) {
        if (poi->isDestination()) result.push_back(poi);
    }
    return result;
}

void Graph::bindPOIsToRoads() {
    for (auto* poi : pois) {
        if (!poi) continue;
        if (poi->hasExplicitRoadAccess()) {
            continue;
        }
        
        Road* bestRoad = nullptr;
        double minSqDist = std::numeric_limits<double>::infinity();
        double bestOffset = 0.0;
        
        double px = poi->getX();
        double py = poi->getY();
        
        for (const auto& pair : roads) {
            Road* road = pair.second;
            if (!road || !road->getStart() || !road->getEnd()) continue;
            
            double sx = road->getStart()->getX();
            double sy = road->getStart()->getY();
            double ex = road->getEnd()->getX();
            double ey = road->getEnd()->getY();
            
            double dx = ex - sx;
            double dy = ey - sy;
            double lenSq = dx * dx + dy * dy;
            
            double t = 0.0;
            if (lenSq > 0.0001) {
                t = ((px - sx) * dx + (py - sy) * dy) / lenSq;
                t = std::max(0.0, std::min(1.0, t));
            }
            
            double projX = sx + t * dx;
            double projY = sy + t * dy;
            
            double distSq = (px - projX) * (px - projX) + (py - projY) * (py - projY);
            const bool closer =
                distSq < minSqDist - 1e-9;
            const bool tied =
                std::fabs(distSq - minSqDist) <= 1e-9;
            const auto directionRank =
                [](const Road* candidate) {
                    return std::make_pair(
                        candidate->getId() < 0 ? 1 : 0,
                        std::abs(candidate->getId()));
                };
            if (closer ||
                (tied &&
                 (bestRoad == nullptr ||
                  directionRank(road) <
                      directionRank(bestRoad)))) {
                minSqDist = distSq;
                bestRoad = road;
                bestOffset = t * std::sqrt(lenSq);
            }
        }
        
        if (bestRoad) {
            poi->setConnectedRoad(bestRoad);
            poi->setProgressOffset(bestOffset);
        }
    }
}

bool Graph::addBusStation(
    std::unique_ptr<BusStation> station) {
    if (station == nullptr ||
        !station->isConfiguredTransitStation() ||
        getBusStation(station->getId()) != nullptr ||
        getBusStationByCode(station->getCode()) != nullptr) {
        return false;
    }
    busStations.push_back(std::move(station));
    return true;
}

BusStation* Graph::getBusStation(int id) const {
    const auto found = std::find_if(
        busStations.begin(),
        busStations.end(),
        [id](const std::unique_ptr<BusStation>& station) {
            return station != nullptr &&
                   station->getId() == id;
        });
    return found != busStations.end()
        ? found->get()
        : nullptr;
}

BusStation* Graph::getBusStationByCode(
    const std::string& code) const {
    const auto found = std::find_if(
        busStations.begin(),
        busStations.end(),
        [&code](const std::unique_ptr<BusStation>& station) {
            return station != nullptr &&
                   station->getCode() == code;
        });
    return found != busStations.end()
        ? found->get()
        : nullptr;
}

std::vector<BusStation*> Graph::getAllBusStations() const {
    std::vector<BusStation*> result;
    result.reserve(busStations.size());
    for (const auto& station : busStations) {
        if (station != nullptr) {
            result.push_back(station.get());
        }
    }
    return result;
}

bool Graph::addBusService(
    std::unique_ptr<BusService> service) {
    if (service == nullptr ||
        getBusService(service->getId()) != nullptr ||
        getBusServiceByCode(service->getCode()) != nullptr) {
        return false;
    }
    busServices.push_back(std::move(service));
    return true;
}

BusService* Graph::getBusService(int id) const {
    const auto found = std::find_if(
        busServices.begin(),
        busServices.end(),
        [id](const std::unique_ptr<BusService>& service) {
            return service != nullptr &&
                   service->getId() == id;
        });
    return found != busServices.end()
        ? found->get()
        : nullptr;
}

BusService* Graph::getBusServiceByCode(
    const std::string& code) const {
    const auto found = std::find_if(
        busServices.begin(),
        busServices.end(),
        [&code](const std::unique_ptr<BusService>& service) {
            return service != nullptr &&
                   service->getCode() == code;
        });
    return found != busServices.end()
        ? found->get()
        : nullptr;
}

std::vector<BusService*> Graph::getAllBusServices() const {
    std::vector<BusService*> result;
    result.reserve(busServices.size());
    for (const auto& service : busServices) {
        if (service != nullptr) {
            result.push_back(service.get());
        }
    }
    return result;
}
