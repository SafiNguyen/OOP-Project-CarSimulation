#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

class Road;  
class TrafficLight;

// attributes
class Intersection {
private:
    int id;
    double x;
    double y;
    
    std::vector<Road*> incomingRoads; 
    std::vector<Road*> outgoingRoads;
    // key = id cua incoming road; moi road vao co dung 1 den rieng
    std::unordered_map<int, std::unique_ptr<TrafficLight>> trafficLights;

public:
    Intersection(int id, double x = 0.0, double y = 0.0);

    Intersection(const Intersection&) = delete;
    Intersection& operator=(const Intersection&) = delete;
    
    int getId() const;
    double getX() const;
    double getY() const;
    
    const std::vector<Road*>& getIncomingRoads() const; 
    const std::vector<Road*>& getOutgoingRoads() const;

    //methods
    void addIncomingRoad(Road* road);
    void addOutgoingRoad(Road* road);
    void removeIncomingRoad(Road* road);
    void removeOutgoingRoad(Road* road);
    // Traffic light management 
    void registerIncomingLight(Road* road);
    TrafficLight* getLightForIncomingRoad(int roadId) const;
    TrafficLight* getLightForIncomingRoad(const Road* road) const;
    // Goi moi tick tu TrafficSimulator::update(dt)
    void updateTrafficLights(double dt);
    std::string toString() const;  

    ~Intersection(); 
};

#endif