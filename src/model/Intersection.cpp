#include "Intersection.h"
#include "Road.h" 
#include "TrafficLight.h"
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
        registerIncomingLight(road);
    }
}

void Intersection::addOutgoingRoad(Road* road) {
    if (road != nullptr) {
        outgoingRoads.push_back(road);
    }
}

//method: remove road
void Intersection::removeIncomingRoad(Road* road) {
    if (road == nullptr) return;
    incomingRoads.erase(std::remove(incomingRoads.begin(),
                        incomingRoads.end(), road), incomingRoads.end());
    trafficLights.erase(road->getId());
}

void Intersection::removeOutgoingRoad(Road* road) {
    outgoingRoads.erase(std::remove(outgoingRoads.begin(),
                        outgoingRoads.end(), road), outgoingRoads.end());
}

//Traffic light management 
 
void Intersection::registerIncomingLight(Road* road) {
    if (road == nullptr) return;
 
    int roadId = road->getId();
    if (trafficLights.find(roadId) != trafficLights.end()) {
        return; 
    }
 
    size_t countSoFar = trafficLights.size();
    LightState initialState = (countSoFar % 2 == 0) ? LightState::GREEN : LightState::RED;
 
    trafficLights[roadId] = std::make_unique<TrafficLight>(
        roadId, /*green=*/30.0, /*yellow=*/3.0, /*red=*/25.0, initialState);
}
 
TrafficLight* Intersection::getLightForIncomingRoad(int roadId) const {
    auto it = trafficLights.find(roadId);
    if (it != trafficLights.end()) {
        return it->second.get();
    }
    return nullptr;
}
 
TrafficLight* Intersection::getLightForIncomingRoad(const Road* road) const {
    if (road == nullptr) return nullptr;
    return getLightForIncomingRoad(road->getId());
}
 
void Intersection::updateTrafficLights(double dt) {
    for (auto& pair : trafficLights) {
        pair.second->update(dt);
    }
}

bool Intersection::mustStopForRoad(const Road *road) const{
    TrafficLight* light = getLightForIncomingRoad(road);
    //không có đèn -> mặc định ko bắt dừng
    return (light != nullptr) && light->mustStop();
}

//utility method
std::string Intersection::toString() const {
    return "Intersection[ID: " + std::to_string(id) + 
           ", X: " + std::to_string(x) + 
           ", Y: " + std::to_string(y) + 
           ", Incoming: " + std::to_string(incomingRoads.size()) + 
           ", Outgoing: " + std::to_string(outgoingRoads.size()) +
           ", Lights: " + std::to_string(trafficLights.size()) + "]";
}

Intersection::~Intersection() {
}