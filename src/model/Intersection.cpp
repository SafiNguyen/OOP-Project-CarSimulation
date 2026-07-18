#include "Intersection.h"
#include "Road.h" 
#include "TrafficLight.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr double PI = 3.14159265358979323846;
// How far from exactly opposite (180 degrees) two approaches may be and
// still be considered "the same through movement" for phase grouping.
// Generous enough to tolerate maps whose roads aren't perfectly aligned.
constexpr double OPPOSITE_TOLERANCE_RAD = PI / 4.0; // 45 degrees
}

Intersection::Intersection(int id, double x, double y)
    : activePhaseGroup(0), phaseElapsedTime(0.0) {
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

IntersectionType Intersection::getIntersectionType() const {
    const size_t approaches = incomingRoads.size();
    if (approaches <= 2) return IntersectionType::PASS_THROUGH;
    if (approaches == 3) return IntersectionType::THREE_WAY;
    if (approaches == 4) return IntersectionType::FOUR_WAY;
    return IntersectionType::COMPLEX;
}

std::string Intersection::getIntersectionTypeLabel() const {
    switch (getIntersectionType()) {
        case IntersectionType::PASS_THROUGH: return "Pass-through";
        case IntersectionType::THREE_WAY:    return "3-way (nga ba)";
        case IntersectionType::FOUR_WAY:     return "4-way (nga tu)";
        case IntersectionType::COMPLEX:      return "Complex junction";
    }
    return "Unknown";
}

//method: add road 
void Intersection::addIncomingRoad(Road* road) {
    if (road!= nullptr) {
        incomingRoads.push_back(road);
        registerIncomingLight(road);
        rebuildPhaseGroups();
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
    rebuildPhaseGroups();
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

    // Every light starts RED. Which road(s) actually get GREEN first is
    // decided by rebuildPhaseGroups()/updateTrafficLights() based on real
    // intersection geometry, not by an arbitrary insertion-order parity
    // (the old behaviour, which could put two crossing approaches on
    // GREEN at once).
    trafficLights[roadId] = std::make_unique<TrafficLight>(
        roadId, /*green=*/30.0, /*yellow=*/3.0, /*red=*/25.0, LightState::RED);
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


void Intersection::rebuildPhaseGroups() {
    phaseGroups.clear();
    activePhaseGroup = 0;
    phaseElapsedTime = 0.0;

    const size_t n = incomingRoads.size();
    std::vector<bool> assigned(n, false);

    auto approachAngle = [this](Road* road) -> double {
        // Direction from which traffic arrives, i.e. vector from this
        // intersection back towards where the road started.
        const Intersection* from = road->getStart();
        const double dx = from->getX() - x;
        const double dy = from->getY() - y;
        return std::atan2(dy, dx);
    };

    for (size_t i = 0; i < n; ++i) {
        if (assigned[i]) continue;
        std::vector<Road*> group;
        group.push_back(incomingRoads[i]);
        assigned[i] = true;

        const double angleI = approachAngle(incomingRoads[i]);

        for (size_t j = i + 1; j < n; ++j) {
            if (assigned[j]) continue;
            const double angleJ = approachAngle(incomingRoads[j]);

            double diff = std::fabs(angleI - angleJ);
            if (diff > PI) diff = 2.0 * PI - diff;
            const double distanceFromOpposite = std::fabs(diff - PI);

            if (distanceFromOpposite <= OPPOSITE_TOLERANCE_RAD) {
                group.push_back(incomingRoads[j]);
                assigned[j] = true;
                break; // each approach pairs with at most one opposite partner
            }
        }

        phaseGroups.push_back(std::move(group));
    }
}

bool Intersection::areRoadsInSamePhase(const Road* a, const Road* b) const {
    if (a == nullptr || b == nullptr) return true;
    if (a == b) return true;

    for (const auto& group : phaseGroups) {
        bool hasA = std::find(group.begin(), group.end(), a) != group.end();
        bool hasB = std::find(group.begin(), group.end(), b) != group.end();
        if (hasA && hasB) return true;
        if (hasA || hasB) return false; // found one but not the other -> different groups
    }
    return false; // neither found (shouldn't normally happen)
}

void Intersection::updateTrafficLights(double dt) {
    if (phaseGroups.empty() || dt <= 0.0) {
        return;
    }

    static constexpr double GREEN_DURATION = 30.0;
    static constexpr double YELLOW_DURATION = 3.0;
    static constexpr double CYCLE_DURATION = GREEN_DURATION + YELLOW_DURATION;

    phaseElapsedTime += dt;
    while (phaseElapsedTime >= CYCLE_DURATION) {
        phaseElapsedTime -= CYCLE_DURATION;
        activePhaseGroup = (activePhaseGroup + 1) % phaseGroups.size();
    }

    const LightState activeState =
        (phaseElapsedTime < GREEN_DURATION) ? LightState::GREEN : LightState::YELLOW;


    for (size_t g = 0; g < phaseGroups.size(); ++g) {
        const LightState desired = (g == activePhaseGroup) ? activeState : LightState::RED;
        for (Road* road : phaseGroups[g]) {
            TrafficLight* light = getLightForIncomingRoad(road);
            if (light != nullptr && light->getState() != desired) {
                light->forceState(desired);
            }
        }
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
           ", Type: " + getIntersectionTypeLabel() +
           ", Incoming: " + std::to_string(incomingRoads.size()) + 
           ", Outgoing: " + std::to_string(outgoingRoads.size()) +
           ", Lights: " + std::to_string(trafficLights.size()) +
           ", PhaseGroups: " + std::to_string(phaseGroups.size()) + "]";
}

Intersection::~Intersection() {
}