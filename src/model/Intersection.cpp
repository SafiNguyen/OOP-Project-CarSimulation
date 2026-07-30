#include "Intersection.h"
#include "Road.h" 
#include "TrafficLight.h"
#include "LaneMapping.h"
#include "MotionPath.h"
#include "RoadGeometry.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr double PI = 3.14159265358979323846;
// How far from exactly opposite (180 degrees) two approaches may be and
// still be considered "the same through movement" for phase grouping.
// Kept tight on purpose: two approaches must be genuinely a straight-through
// pair (e.g. North vs South) to safely share a green phase. A wider
// tolerance (the old value was 45 degrees) can merge two approaches that
// are only ~135 degrees apart into a single "safe" group - i.e. two roads
// that actually cross paths inside the intersection end up green at the
// same time. Worse, if an intersection only has two incoming roads and
// they get merged this way, rebuildPhaseGroups() produces a single phase
// group, so that intersection's light never has anything to hand off to
// and stays GREEN/YELLOW forever - it can never turn RED, so vehicles on
// either approach never get stopped at all.
constexpr double OPPOSITE_TOLERANCE_RAD = PI / 9.0; // 20 degrees

struct OrientedVehicleBounds {
    Vec2 centreMetres;
    Vec2 forward;
    Vec2 side;
    double halfLengthMetres = 0.0;
    double halfWidthMetres = 0.0;
};

OrientedVehicleBounds makeVehicleBounds(
    const Pose2D& pose,
    double metresPerWorldUnit,
    double lengthMetres,
    double widthMetres,
    double clearanceMetres) {
    const Vec2 forward{
        std::cos(pose.headingRadians),
        std::sin(pose.headingRadians)
    };
    const double padding =
        std::max(0.0, clearanceMetres) * 0.5;
    return {
        pose.position * metresPerWorldUnit,
        forward,
        rightNormal(forward),
        std::max(0.1, lengthMetres) * 0.5 + padding,
        std::max(0.1, widthMetres) * 0.5 + padding
    };
}

bool boundsOverlap(const OrientedVehicleBounds& first,
                   const OrientedVehicleBounds& second) {
    const Vec2 delta =
        second.centreMetres - first.centreMetres;
    const Vec2 axes[] = {
        first.forward,
        first.side,
        second.forward,
        second.side
    };
    for (const Vec2& axis : axes) {
        const double firstRadius =
            first.halfLengthMetres *
                std::fabs(dot(first.forward, axis)) +
            first.halfWidthMetres *
                std::fabs(dot(first.side, axis));
        const double secondRadius =
            second.halfLengthMetres *
                std::fabs(dot(second.forward, axis)) +
            second.halfWidthMetres *
                std::fabs(dot(second.side, axis));
        if (std::fabs(dot(delta, axis)) + 1e-9 >=
            firstRadius + secondRadius) {
            return false;
        }
    }
    return true;
}
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
        invalidateConnectorCache();
        // Traffic lights are NOT auto-registered. Use registerIncomingLight()
        // explicitly or through the debug console to add lights.
    }
}

void Intersection::addOutgoingRoad(Road* road) {
    if (road != nullptr) {
        outgoingRoads.push_back(road);
        invalidateConnectorCache();
    }
}

//method: remove road
void Intersection::removeIncomingRoad(Road* road) {
    if (road == nullptr) return;
    incomingRoads.erase(std::remove(incomingRoads.begin(),
                        incomingRoads.end(), road), incomingRoads.end());
    trafficLights.erase(road->getId());
    rebuildPhaseGroups();
    invalidateConnectorCache();
}

void Intersection::removeOutgoingRoad(Road* road) {
    outgoingRoads.erase(std::remove(outgoingRoads.begin(),
                        outgoingRoads.end(), road), outgoingRoads.end());
    invalidateConnectorCache();
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

    rebuildPhaseGroups();
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

void Intersection::unregisterIncomingLight(Road* road) {
    if (road == nullptr) return;
    trafficLights.erase(road->getId());
    rebuildPhaseGroups();
}

bool Intersection::hasTrafficLights() const {
    return !trafficLights.empty();
}


void Intersection::rebuildPhaseGroups() {
    phaseGroups.clear();
    activePhaseGroup = 0;
    phaseElapsedTime = 0.0;

    std::vector<Road*> activeIncomingRoads;
    for (Road* road : incomingRoads) {
        if (getLightForIncomingRoad(road) != nullptr) {
            activeIncomingRoads.push_back(road);
        }
    }

    const size_t n = activeIncomingRoads.size();
    if (n == 0) return;

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
        group.push_back(activeIncomingRoads[i]);
        assigned[i] = true;

        const double angleI = approachAngle(activeIncomingRoads[i]);

        for (size_t j = i + 1; j < n; ++j) {
            if (assigned[j]) continue;
            const double angleJ = approachAngle(activeIncomingRoads[j]);

            double diff = std::fabs(angleI - angleJ);
            if (diff > PI) diff = 2.0 * PI - diff;
            const double distanceFromOpposite = std::fabs(diff - PI);

            if (distanceFromOpposite <= OPPOSITE_TOLERANCE_RAD) {
                group.push_back(activeIncomingRoads[j]);
                assigned[j] = true;
                break; // each approach pairs with at most one opposite partner
            }
        }

        phaseGroups.push_back(std::move(group));
    }

    // Immediately set active group's lights to GREEN so adding a light starts it active right away
    if (!phaseGroups.empty()) {
        for (size_t g = 0; g < phaseGroups.size(); ++g) {
            const LightState desired = (g == 0) ? LightState::GREEN : LightState::RED;
            for (Road* road : phaseGroups[g]) {
                TrafficLight* light = getLightForIncomingRoad(road);
                if (light != nullptr) {
                    light->forceState(desired);
                }
            }
        }
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
    return true;
}

void Intersection::updateTrafficLights(double dt) {
    if (phaseGroups.empty() || dt <= 0.0) {
        return;
    }

    if (preemptionTimer_ > 0.0) {
        preemptionTimer_ -= dt;
        for (const auto& group : phaseGroups) {
            const bool isPreemptedGroup =
                std::find(group.begin(), group.end(), preemptedRoad_) != group.end();
            const LightState desired = isPreemptedGroup ? LightState::GREEN : LightState::RED;
            for (Road* road : group) {
                TrafficLight* light = getLightForIncomingRoad(road);
                if (light != nullptr && light->getState() != desired) {
                    light->forceState(desired);
                }
            }
        }
        if (preemptionTimer_ <= 0.0) {
            preemptionTimer_ = 0.0;
            preemptedRoad_ = nullptr;
            phaseElapsedTime = 0.0;
        }
        return; 
    }

    static constexpr double GREEN_DURATION = 30.0;
    static constexpr double YELLOW_DURATION = 3.0;
    // Clearance gap between one phase group's YELLOW ending and the next
    // group's GREEN starting. Without this, the previously-active approach
    // turns RED on the exact same tick the next approach turns GREEN, i.e.
    // zero time to clear the intersection before conflicting traffic gets a
    // green light. Only applies when there is more than one phase group -
    // a single-group (pass-through) intersection has no conflicting traffic
    // to protect against and doesn't need to pause.
    const double allRedDuration = (phaseGroups.size() > 1) ? 2.0 : 0.0;
    const double cycleDuration = GREEN_DURATION + YELLOW_DURATION + allRedDuration;

    phaseElapsedTime += dt;
    while (phaseElapsedTime >= cycleDuration) {
        phaseElapsedTime -= cycleDuration;
        activePhaseGroup = (activePhaseGroup + 1) % phaseGroups.size();
    }

    LightState activeState;
    if (phaseElapsedTime < GREEN_DURATION) {
        activeState = LightState::GREEN;
    } else if (phaseElapsedTime < GREEN_DURATION + YELLOW_DURATION) {
        activeState = LightState::YELLOW;
    } else {
        // All-red clearance window: even the "active" group shows RED here.
        activeState = LightState::RED;
    }


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

void Intersection::requestEmergencyPreemption(const Road* incomingRoad, double holdDuration) {
    if (incomingRoad == nullptr || holdDuration <= 0.0) return;
    preemptedRoad_ = incomingRoad;
    preemptionTimer_ = holdDuration; 
}

bool Intersection::mustStopForRoad(const Road *road) const{
    TrafficLight* light = getLightForIncomingRoad(road);
    //không có đèn -> mặc định ko bắt dừng
    return (light != nullptr) && light->mustStop();
}

std::shared_ptr<const JunctionConnector> Intersection::createConnector(
    const Road& incoming,
    int incomingLane,
    const Road& outgoing,
    int outgoingLane) const {
    const Vec2 start =
        RoadGeometry::laneEndpoint(incoming, incomingLane, false);
    const Vec2 finish =
        RoadGeometry::laneEndpoint(outgoing, outgoingLane, true);
    const Vec2 startTangent = RoadGeometry::roadDirection(incoming);
    const Vec2 endTangent = RoadGeometry::roadDirection(outgoing);
    const MovementType movement =
        TurnLanePolicy::classify(incoming, outgoing);
    const double chord = distance(start, finish);
    const double junctionRadiusWorld =
        RoadGeometry::junctionBoundaryRadiusWorld(*this);

    double movementFactor = 0.75;
    switch (movement) {
        case MovementType::Straight: movementFactor = 0.45; break;
        case MovementType::Right: movementFactor = 0.60; break;
        case MovementType::Left: movementFactor = 0.85; break;
        case MovementType::UTurn: movementFactor = 1.15; break;
    }
    const double scale =
        RoadGeometry::metresPerWorldUnit(*this);
    const double controlDistance = std::max({
        chord * 0.30,
        junctionRadiusWorld * movementFactor,
        junctionRadiusWorld > 1e-9
            ? RoadGeometry::LANE_WIDTH_METRES / scale
            : 0.0
    });

    auto path = std::make_shared<BezierJunctionPath>(
        start,
        start + startTangent * controlDistance,
        finish - endTangent * controlDistance,
        finish,
        scale);
    const ConnectorKey key{
        incoming.getId(), incomingLane,
        outgoing.getId(), outgoingLane
    };
    return std::make_shared<JunctionConnector>(
        key,
        &incoming,
        &outgoing,
        movement,
        startTangent,
        endTangent,
        RoadGeometry::metresPerWorldUnit(outgoing),
        std::move(path));
}

std::shared_ptr<const JunctionConnector> Intersection::getConnector(
    const Road* incoming,
    int incomingLane,
    const Road* outgoing,
    int outgoingLane) const {
    if (incoming == nullptr || outgoing == nullptr ||
        incomingLane < 0 || incomingLane >= incoming->getLaneCount() ||
        outgoingLane < 0 || outgoingLane >= outgoing->getLaneCount() ||
        incoming->getEnd() != this || outgoing->getStart() != this) {
        return nullptr;
    }

    const ConnectorKey key{
        incoming->getId(), incomingLane,
        outgoing->getId(), outgoingLane
    };
    const auto found = connectorCache_.find(key);
    if (found != connectorCache_.end()) {
        return found->second;
    }

    auto connector = createConnector(
        *incoming, incomingLane, *outgoing, outgoingLane);
    if (connector != nullptr) {
        connectorCache_.emplace(key, connector);
    }
    return connector;
}

void Intersection::invalidateConnectorCache() {
    connectorCache_.clear();
}

// --- Intersection-box reservation ---
// Deliberately independent from the traffic-light phase logic above: a
// green light only means "your approach's turn according to the signal
// cycle", it says nothing about whether the physical box in the middle of
// the junction is currently occupied by a vehicle arriving from another
// approach. This is the missing piece that stops vehicles from different
// roads/lanes rendering on top of each other inside the junction.

bool Intersection::canEnter(int vehicleId, const Road* fromRoad) const {
    if (occupants_.count(vehicleId) > 0) {
        return true; // dang giu cho roi
    }
    for (const auto& occupant : occupants_) {
        if (!areRoadsInSamePhase(
                fromRoad, occupant.second.fromRoad)) {
            return false; 
        }
    }
    return static_cast<int>(occupants_.size()) < capacity_;
}

bool Intersection::canEnterMovement(
    int vehicleId,
    const std::shared_ptr<const JunctionConnector>& connector,
    double requiredGapMetres,
    double vehicleLengthMetres,
    double vehicleWidthMetres) const {
    if (connector == nullptr || !canEnter(vehicleId, connector->getIncomingRoad())) {
        return false;
    }

    const double metricScale = RoadGeometry::metresPerWorldUnit(*this);
    const double step = std::max(0.5, std::min(vehicleLengthMetres, vehicleWidthMetres) * 0.5);
    const double length1 = connector->getLength();
    
    // Quick out for empty intersection
    if (occupants_.empty() || (occupants_.size() == 1 && occupants_.count(vehicleId) > 0)) {
        return true;
    }

    for (double p1 = 0.0; p1 <= length1; p1 += step) {
        const OrientedVehicleBounds candidate = makeVehicleBounds(
            connector->sampleByDistance(p1), metricScale, vehicleLengthMetres, vehicleWidthMetres, 0.5);

        for (const auto& entry : occupants_) {
            if (entry.first == vehicleId) continue;
            const Reservation& res = entry.second;
            if (res.connector == nullptr) continue;

            const double length2 = res.connector->getLength();
            for (double p2 = res.progressMetres; p2 <= length2; p2 += step) {
                const OrientedVehicleBounds other = makeVehicleBounds(
                    res.connector->sampleByDistance(p2), metricScale, res.vehicleLengthMetres, res.vehicleWidthMetres, 0.5);
                
                if (boundsOverlap(candidate, other)) {
                    return false; // Paths overlap, must yield
                }
            }
        }
    }
    return true;
}

bool Intersection::tryEnter(int vehicleId, const Road* fromRoad) {
    if (occupants_.count(vehicleId) > 0) {
        return true; // already holding a slot, nothing to do
    }
    if (!canEnter(vehicleId, fromRoad)) {
        return false;
    }
    occupants_.emplace(
        vehicleId,
        Reservation{fromRoad, nullptr, 0.0, 4.5, 1.8});
    return true;
}

bool Intersection::tryEnterMovement(
    int vehicleId,
    const std::shared_ptr<const JunctionConnector>& connector,
    double requiredGapMetres,
    double vehicleLengthMetres,
    double vehicleWidthMetres) {
    auto existing = occupants_.find(vehicleId);
    if (existing != occupants_.end()) {
        if (connector != nullptr) {
            existing->second.connector = connector;
        }
        existing->second.vehicleLengthMetres =
            std::max(0.1, vehicleLengthMetres);
        existing->second.vehicleWidthMetres =
            std::max(0.1, vehicleWidthMetres);
        return true;
    }
    if (!canEnterMovement(
            vehicleId,
            connector,
            requiredGapMetres,
            vehicleLengthMetres,
            vehicleWidthMetres)) {
        return false;
    }
    occupants_.emplace(
        vehicleId,
        Reservation{
            connector->getIncomingRoad(),
            connector,
            0.0,
            std::max(0.1, vehicleLengthMetres),
            std::max(0.1, vehicleWidthMetres)
        });
    return true;
}

void Intersection::updateReservationProgress(
    int vehicleId,
    double progressMetres) {
    auto found = occupants_.find(vehicleId);
    if (found != occupants_.end()) {
        found->second.progressMetres =
            std::max(0.0, progressMetres);
    }
}

double Intersection::limitTraversalAdvance(
    int vehicleId,
    const std::shared_ptr<const JunctionConnector>& connector,
    double currentProgressMetres,
    double desiredAdvanceMetres,
    double vehicleLengthMetres,
    double vehicleWidthMetres,
    double clearanceMetres) const {
    if (connector == nullptr ||
        desiredAdvanceMetres <= 0.0 ||
        occupants_.size() <= 1) {
        return std::max(0.0, desiredAdvanceMetres);
    }

    const double metricScale =
        RoadGeometry::metresPerWorldUnit(*this);
    const auto isSafeAt = [&](double progressMetres) {
        const OrientedVehicleBounds candidate =
            makeVehicleBounds(
                connector->sampleByDistance(progressMetres),
                metricScale,
                vehicleLengthMetres,
                vehicleWidthMetres,
                clearanceMetres);
        for (const auto& entry : occupants_) {
            if (entry.first == vehicleId) continue;
            const Reservation& reservation = entry.second;
            if (reservation.connector == nullptr) {
                return false;
            }
            const OrientedVehicleBounds other =
                makeVehicleBounds(
                    reservation.connector->sampleByDistance(
                        reservation.progressMetres),
                    metricScale,
                    reservation.vehicleLengthMetres,
                    reservation.vehicleWidthMetres,
                    clearanceMetres);
            if (boundsOverlap(candidate, other)) {
                return false;
            }
        }
        return true;
    };

    if (!isSafeAt(currentProgressMetres)) {
        return 0.0;
    }

    // Sweep in short increments so a large caller step cannot tunnel through
    // another rectangle merely because both end positions are collision-free.
    const double sweepStepMetres = std::max(
        0.25,
        std::min(
            std::max(0.1, vehicleLengthMetres),
            std::max(0.1, vehicleWidthMetres)) * 0.25);
    const int sweepSteps = std::max(
        1,
        static_cast<int>(std::ceil(
            desiredAdvanceMetres / sweepStepMetres)));
    double safeAdvance = 0.0;
    for (int step = 1; step <= sweepSteps; ++step) {
        const double candidateAdvance =
            desiredAdvanceMetres *
            static_cast<double>(step) /
            static_cast<double>(sweepSteps);
        if (isSafeAt(
                currentProgressMetres + candidateAdvance)) {
            safeAdvance = candidateAdvance;
            continue;
        }

        double unsafeAdvance = candidateAdvance;
        for (int iteration = 0; iteration < 14; ++iteration) {
            const double candidate =
                (safeAdvance + unsafeAdvance) * 0.5;
            if (isSafeAt(
                    currentProgressMetres + candidate)) {
                safeAdvance = candidate;
            } else {
                unsafeAdvance = candidate;
            }
        }
        return safeAdvance;
    }
    return desiredAdvanceMetres;
}

double Intersection::constrainIncomingStopPosition(
    const Road* incomingRoad,
    int incomingLane,
    double nominalStopPositionMetres,
    double waitingVehicleLengthMetres,
    double requiredClearanceMetres) const {
    if (incomingRoad == nullptr || incomingLane < 0) {
        return std::max(0.0, nominalStopPositionMetres);
    }

    double stopPosition =
        std::max(0.0, nominalStopPositionMetres);
    const double waitingHalfLength =
        std::max(0.1, waitingVehicleLengthMetres) * 0.5;
    const double clearance =
        std::max(0.0, requiredClearanceMetres);
    for (const auto& entry : occupants_) {
        const Reservation& reservation = entry.second;
        if (reservation.connector == nullptr ||
            reservation.connector->getIncomingRoad() != incomingRoad ||
            reservation.connector->getIncomingLane() != incomingLane) {
            continue;
        }

        // Connector progress is measured from the incoming lane endpoint.
        // Until the reserved vehicle's rear bumper clears that endpoint, its
        // body still occupies part of the approach lane even though it has
        // already been removed from Lane::vehicles.
        const double reservedRearFromBoundary =
            reservation.progressMetres -
            reservation.vehicleLengthMetres * 0.5;
        const double safeFollowerCentre =
            incomingRoad->getDistance() +
            reservedRearFromBoundary -
            clearance -
            waitingHalfLength;
        stopPosition = std::min(
            stopPosition,
            std::max(0.0, safeFollowerCentre));
    }
    return stopPosition;
}

bool Intersection::isOutgoingLaneReserved(
    const Road* outgoingRoad,
    int outgoingLane) const {
    if (outgoingRoad == nullptr || outgoingLane < 0) {
        return false;
    }
    for (const auto& entry : occupants_) {
        const auto& connector = entry.second.connector;
        if (connector != nullptr &&
            connector->getOutgoingRoad() == outgoingRoad &&
            connector->getOutgoingLane() == outgoingLane) {
            return true;
        }
    }
    return false;
}

void Intersection::exit(int vehicleId) {
    occupants_.erase(vehicleId);
}

bool Intersection::isFull() const {
    return static_cast<int>(occupants_.size()) >= capacity_;
}

void Intersection::setCapacity(int cap) {
    capacity_ = std::max(1, cap);
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
