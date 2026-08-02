#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>
#include <memory>
#include "Intersection.h"
#include "Road.h"
#include "PointOfInterest.h"

class BusService;
class BusStation;
class BusStop;

class Graph {
private:
    std::unordered_map<int, Intersection*> intersections;
    std::unordered_map<int, Road*> roads;
    std::vector<PointOfInterest*> pois;
    std::vector<std::unique_ptr<BusStation>> busStations;
    std::vector<std::unique_ptr<BusService>> busServices;

public:
    Graph();
    ~Graph();

    Graph(const Graph&) = delete;
    Graph& operator=(const Graph&) = delete;

    Graph(Graph&& other) noexcept;
    Graph& operator=(Graph&& other) noexcept;

    // --- Intersection Operations ---
    void addIntersection(Intersection* intersection);
    void removeIntersection(int id);
    Intersection* getIntersection(int id) const;
    
    // --- Road Operations ---
    void addRoad(Road* road);
    void removeRoad(int id);
    Road* getRoad(int id) const;
    
    // --- Graph Functions ---
    void updateRoadCondition(int roadId, double congestionLevel, bool blocked, int laneIndex = -1);
    std::vector<Road*> getConnectedRoads(int intersectionId) const;

    // Accessors for visualization/testing
    std::vector<Intersection*> getAllIntersections() const;
    std::vector<Road*> getAllRoads() const;
    std::vector<Road*> getNeighbors(int intersectionId) const;

    double calculateDistance(int startId, int destId) const; // for A* heuristic

    // --- Utility ---
    void clearGraph();

    // --- POI Operations ---
    void addPOI(PointOfInterest* poi);
    const std::vector<PointOfInterest*>& getAllPOIs() const;
    PointOfInterest* getPOI(int id) const;
    PointOfInterest* getPointOfInterest(int id) const;
    const BusStop* getBusStop(int id) const;
    std::vector<PointOfInterest*> getSpawnPoints() const;
    std::vector<PointOfInterest*> getDestinations() const;
    
    void bindPOIsToRoads();

    // --- Public transit metadata ---
    bool addBusStation(std::unique_ptr<BusStation> station);
    BusStation* getBusStation(int id) const;
    BusStation* getBusStationByCode(const std::string& code) const;
    std::vector<BusStation*> getAllBusStations() const;

    bool addBusService(std::unique_ptr<BusService> service);
    BusService* getBusService(int id) const;
    BusService* getBusServiceByCode(const std::string& code) const;
    std::vector<BusService*> getAllBusServices() const;
};

#endif
