#include "VehicleSprite.h"
#include "../model/Road.h"
#include "../model/Intersection.h"
#include "../model/Car.h"
#include "../model/Bus.h"
#include "../model/Motorbike.h"
#include "../model/EmergencyVehicle.h"
#include <algorithm>
#include <cmath>

VehicleSprite::VehicleSprite(Vehicle* v, const VisualizationEngine* eng)
    : vehicle(v), engine(eng) {
    if (dynamic_cast<Bus*>(vehicle)) {
        vehicleType = Type::BUS;
    } else if (dynamic_cast<Motorbike*>(vehicle)) {
        vehicleType = Type::MOTORBIKE;
    } else if (dynamic_cast<EmergencyVehicle*>(vehicle)) {
        vehicleType = Type::EMERGENCY;
    } else {
        vehicleType = Type::CAR;
    }

    setupShape();
}

void VehicleSprite::setupShape() {
    switch (vehicleType) {
    case Type::CAR:
        shape.setSize({12.0f, 6.0f});
        shape.setFillColor(sf::Color(50, 150, 255));
        shape.setOrigin(6.0f, 3.0f);
        break;
    case Type::BUS:
        shape.setSize({20.0f, 8.0f});
        shape.setFillColor(sf::Color(50, 200, 50));
        shape.setOrigin(10.0f, 4.0f);
        break;
    case Type::EMERGENCY:
        shape.setSize({14.0f, 7.0f});
        shape.setFillColor(sf::Color::Red);
        shape.setOrigin(7.0f, 3.5f);
        break;
    case Type::MOTORBIKE:
        bikeShape.setRadius(3.0f);
        bikeShape.setFillColor(sf::Color(255, 200, 0));
        bikeShape.setOrigin(3.0f, 3.0f);
        break;
    }
}

sf::Vector2f VehicleSprite::pointOnRoad(const Road* road, double ratio) const {
    if (road == nullptr || engine == nullptr) {
        return {};
    }

    const Intersection* start = road->getStart();
    const Intersection* end = road->getEnd();
    if (start == nullptr || end == nullptr) {
        return {};
    }

    const double worldX = start->getX() + ratio * (end->getX() - start->getX());
    const double worldY = start->getY() + ratio * (end->getY() - start->getY());
    return engine->worldToScreen(worldX, worldY);
}

sf::Vector2f VehicleSprite::laneOffset(const Road* road, const sf::Vector2f& basePosition) const {
    if (road == nullptr || engine == nullptr) {
        return basePosition;
    }

    const Intersection* start = road->getStart();
    const Intersection* end = road->getEnd();
    if (start == nullptr || end == nullptr) {
        return basePosition;
    }

    const sf::Vector2f a = engine->worldToScreen(start->getX(), start->getY());
    const sf::Vector2f b = engine->worldToScreen(end->getX(), end->getY());
    const sf::Vector2f dir = b - a;
    const float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (length <= 0.01f) {
        return basePosition;
    }

    const sf::Vector2f norm(-dir.y / length, dir.x / length);
    const float laneWidth = 10.0f;
    const int laneCount = road->getLaneCount();
    const int laneIndex = vehicle != nullptr ? vehicle->getCurrentLaneIndex() : 0;

    bool hasReverse = false;
    for (Road* r : end->getOutgoingRoads()) {
        if (r->getEnd() == start) {
            hasReverse = true;
            break;
        }
    }

    if (hasReverse) {
        // rd.offsetAmount in VisualizationEngine: totalWidth * 0.5f + 1.0f
        // then the lane is offset by (-totalWidth * 0.5f + laneIndex * 10.0f + 5.0f)
        // total offset from median = (totalWidth * 0.5f + 1.0f) + (-totalWidth * 0.5f + laneIndex * 10.0f + 5.0f)
        //                          = 1.0f + laneIndex * 10.0f + 5.0f = 6.0f + laneIndex * 10.0f
        const float totalOffset = 6.0f + laneIndex * laneWidth;
        return basePosition + norm * totalOffset;
    }

    const float totalWidth = laneCount * laneWidth;
    const float laneOffset = -totalWidth * 0.5f + laneIndex * laneWidth + laneWidth * 0.5f;
    return basePosition + norm * laneOffset;
}

sf::Vector2f VehicleSprite::resolvePosition() const {
    if (!vehicle || vehicle->getCurrentRoad() == nullptr) {
        return {};
    }

    Road* currentRoad = vehicle->getCurrentRoad();
    Road* nextRoad = vehicle->getNextRoad();
    const double ratio = vehicle->getProgressRatio();
    const sf::Vector2f currentPos = pointOnRoad(currentRoad, ratio);

    float turnFactor = 0.0f;
    if (nextRoad != nullptr && nextRoad->getStart() == currentRoad->getEnd()) {
        const double turnDistance = 20.0;
        const double distanceRemaining = currentRoad->getDistance() * (1.0 - ratio);
        if (distanceRemaining < turnDistance) {
            turnFactor = static_cast<float>(1.0 - (distanceRemaining / turnDistance));
            turnFactor = std::clamp(turnFactor, 0.0f, 1.0f);
        }
        if (vehicle != nullptr && vehicle->isAwaitingIntersectionTransition()) {
            turnFactor = std::max(turnFactor, static_cast<float>(vehicle->getIntersectionTransitionProgress()));
        }
        
        if (turnFactor > 0.0f) {
            // Apply smoothing curve to turnFactor so it feels like a real steering curve
            float t = turnFactor * turnFactor * (3.0f - 2.0f * turnFactor);
            
            sf::Vector2f offset1 = laneOffset(currentRoad, currentPos) - currentPos;
            sf::Vector2f offset2 = laneOffset(nextRoad, currentPos) - currentPos;
            sf::Vector2f blendedOffset = (1.0f - t) * offset1 + t * offset2;
            
            return currentPos + blendedOffset;
        }
    }

    return laneOffset(currentRoad, currentPos);
}

float VehicleSprite::resolveAngle() const {
    if (!vehicle || vehicle->getCurrentRoad() == nullptr) {
        return 0.0f;
    }

    Road* currentRoad = vehicle->getCurrentRoad();
    Road* nextRoad = vehicle->getNextRoad();
    const double ratio = vehicle->getProgressRatio();
    float turnFactor = 0.0f;
    if (nextRoad != nullptr && nextRoad->getStart() == currentRoad->getEnd()) {
        const double turnDistance = 20.0;
        const double distanceRemaining = currentRoad->getDistance() * (1.0 - ratio);
        if (distanceRemaining < turnDistance) {
            turnFactor = static_cast<float>(1.0 - (distanceRemaining / turnDistance));
            turnFactor = std::clamp(turnFactor, 0.0f, 1.0f);
        }
    }

    if (vehicle != nullptr && vehicle->isAwaitingIntersectionTransition()) {
        const float transitionProgress = std::clamp(static_cast<float>(vehicle->getIntersectionTransitionProgress()), 0.0f, 1.0f);
        turnFactor = std::max(turnFactor, transitionProgress);
    }

    const double currentAngle = std::atan2(currentRoad->getEnd()->getY() - currentRoad->getStart()->getY(),
                                          currentRoad->getEnd()->getX() - currentRoad->getStart()->getX());
    if (turnFactor <= 0.0f || nextRoad == nullptr) {
        return static_cast<float>(-currentAngle * 180.0f / 3.14159265358979323846);
    }

    const double nextAngle = std::atan2(nextRoad->getEnd()->getY() - nextRoad->getStart()->getY(),
                                       nextRoad->getEnd()->getX() - nextRoad->getStart()->getX());
    
    double diff = nextAngle - currentAngle;
    while (diff < -3.14159265358979323846) diff += 2.0 * 3.14159265358979323846;
    while (diff > 3.14159265358979323846) diff -= 2.0 * 3.14159265358979323846;
    
    const double blended = currentAngle + diff * turnFactor;
    return static_cast<float>(-blended * 180.0f / 3.14159265358979323846);
}

void VehicleSprite::update(float dt) {
    if (!vehicle || vehicle->getCurrentRoad() == nullptr) {
        return;
    }

    if (vehicleType == Type::EMERGENCY) {
        flashTimer += dt;
        if (flashTimer > 0.2f) {
            shape.setFillColor(shape.getFillColor() == sf::Color::Red ? sf::Color::Blue : sf::Color::Red);
            flashTimer = 0.0f;
        }
    }
}

void VehicleSprite::draw(sf::RenderTarget& target) const {
    if (!vehicle || vehicle->getCurrentRoad() == nullptr) {
        return;
    }

    const sf::Vector2f screenPos = resolvePosition();
    const float angle = resolveAngle();

    if (vehicleType == Type::MOTORBIKE) {
        sf::CircleShape tempBike = bikeShape;
        tempBike.setPosition(screenPos);
        target.draw(tempBike);
    } else {
        sf::RectangleShape tempShape = shape;
        tempShape.setPosition(screenPos);
        tempShape.setRotation(angle);
        target.draw(tempShape);
    }
}

void VehicleSprite::drawAt(sf::RenderTarget& target, const sf::Vector2f& position, float angle) const {
    if (vehicleType == Type::MOTORBIKE) {
        sf::CircleShape tempBike = bikeShape;
        tempBike.setPosition(position);
        target.draw(tempBike);
    } else {
        sf::RectangleShape tempShape = shape;
        tempShape.setPosition(position);
        tempShape.setRotation(angle);
        target.draw(tempShape);
    }
}