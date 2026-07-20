#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>

class Road;  
class TrafficLight;


enum class IntersectionType {
    PASS_THROUGH, // 0-2 incoming roads: nothing to arbitrate between
    THREE_WAY,    // nga ba
    FOUR_WAY,     // nga tu
    COMPLEX       // 5+ incoming roads
};

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

    
    std::vector<std::vector<Road*>> phaseGroups;
    std::size_t activePhaseGroup;
    double phaseElapsedTime;

    void rebuildPhaseGroups();

    // --- Intersection-box reservation (prevents multiple vehicles from
    // different roads overlapping inside the junction at the same time,
    // independent of the traffic-light phase groups above) ---
    int capacity_ = 1;                  // max vehicles allowed inside the box at once
    std::unordered_set<int> occupants_; // ids of vehicles currently holding a slot

public:
    Intersection(int id, double x = 0.0, double y = 0.0);

    Intersection(const Intersection&) = delete;
    Intersection& operator=(const Intersection&) = delete;
    
    int getId() const;
    double getX() const;
    double getY() const;
    
    const std::vector<Road*>& getIncomingRoads() const; 
    const std::vector<Road*>& getOutgoingRoads() const;

    //intersection shape, derived from the number of incoming
    // approaches (nga ba / nga tu / etc).
    IntersectionType getIntersectionType() const;
    std::string getIntersectionTypeLabel() const;

    virtual bool isRoundabout() const { return false; }

    //methods
    void addIncomingRoad(Road* road);
    void addOutgoingRoad(Road* road);
    void removeIncomingRoad(Road* road);
    void removeOutgoingRoad(Road* road);
    // Traffic light management 
    void registerIncomingLight(Road* road);
    void unregisterIncomingLight(Road* road);
    bool hasTrafficLights() const;
    TrafficLight* getLightForIncomingRoad(int roadId) const;
    TrafficLight* getLightForIncomingRoad(const Road* road) const;
    // Goi moi tick tu TrafficSimulator::update(dt)
    void updateTrafficLights(double dt);
    bool mustStopForRoad(const Road* road) const;

    // Number of independent signal phases this intersection cycles
    // through (e.g. 2 for a typical nga tu with opposing through-roads
    // paired up, 1 if there is nothing to arbitrate).
    std::size_t getPhaseGroupCount() const { return phaseGroups.size(); }
    // True if `a` and `b` are allowed to move at the same time (either
    // they are the same phase group, or one/both have no light at all).
    bool areRoadsInSamePhase(const Road* a, const Road* b) const;

    // --- Intersection-box reservation ---
    // Attempts to claim one of the `capacity_` slots for `vehicleId`. Returns
    // true if the vehicle now holds a slot (either newly granted, or it
    // already held one - calling this again is safe/idempotent). Returns
    // false if the box is full and the vehicle must keep waiting at the
    // stop line.
    bool tryEnter(int vehicleId);
    // Releases the slot held by `vehicleId`, if any. Safe to call even if
    // the vehicle never held a slot.
    void exit(int vehicleId);
    // True if every slot is currently occupied.
    bool isFull() const;
    void setCapacity(int cap);
    int getCapacity() const { return capacity_; }

    std::string toString() const;  

    ~Intersection(); 
};

#endif