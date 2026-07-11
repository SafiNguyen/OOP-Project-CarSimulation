#include "Intersection.h"
#include "Road.h" 
#include <algorithm>

Intersection::Intersection(int id, double x, double y) {
    this->id = id;
    this->x = x;
    this->y = y;
}
//getter 
int Intersection::getId() const {
    return id;
}

double Intersection::getX() const {
    return x;
}

double Intersection::getY() const {
    return y;
}

const std::vector<Road*>& Intersection::getIncomingRoads() const {
    return incomingRoads;
}

const std::vector<Road*>& Intersection::getOutgoingRoads() const {
    return outgoingRoads;
}
//method: add road 
void Intersection::addIncomingRoad(Road* road) {
    if (road!= nullptr) {
        incomingRoads.push_back(road);
    }
}

void Intersection::addOutgoingRoad(Road* road) {
    if (road != nullptr) {
        outgoingRoads.push_back(road);
    }
}

//method: remove road
void Intersection::removeIncomingRoad(Road* road) {
    incomingRoads.erase(std::remove(incomingRoads.begin(),
                        incomingRoads.end(), road), incomingRoads.end());
}

void Intersection::removeOutgoingRoad(Road* road) {
    outgoingRoads.erase(std::remove(outgoingRoads.begin(),
                        outgoingRoads.end(), road), outgoingRoads.end());
}

//utility method
std::string Intersection::toString() const {
    return "Intersection[ID: " + std::to_string(id) + 
           ", X: " + std::to_string(x) + 
           ", Y: " + std::to_string(y) + 
           ", Incoming: " + std::to_string(incomingRoads.size()) + 
           ", Outgoing: " + std::to_string(outgoingRoads.size()) + "]";
}

Intersection::~Intersection() {
}