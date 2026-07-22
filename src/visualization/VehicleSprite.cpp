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

float scale  = 0.55;

void VehicleSprite::setupShape() {
    switch (vehicleType) {
    case Type::CAR:
        shape.setSize({12.0f * scale, 6.0f * scale});
        shape.setFillColor(sf::Color(50, 150, 255));
        shape.setOrigin(6.0f * scale, 3.0f * scale);
        shape.setOutlineThickness(1.0f * scale);
        shape.setOutlineColor(sf::Color::Black);
        break;
    case Type::BUS:
        shape.setSize({20.0f* scale, 8.0f* scale});
        shape.setFillColor(sf::Color(200, 162, 50));
        shape.setOrigin(10.0f* scale, 4.0f* scale);
        shape.setOutlineThickness(1.0f * scale);
        shape.setOutlineColor(sf::Color::Black);
        break;
    case Type::EMERGENCY:
        shape.setSize({14.0f* scale, 7.0f * scale});
        shape.setFillColor(sf::Color::Red);
        shape.setOrigin(7.0f* scale, 3.5f* scale);
        shape.setOutlineThickness(1.0f * scale);
        shape.setOutlineColor(sf::Color::Black);
        break;
    case Type::MOTORBIKE:
        bikeShape.setRadius(3.0f* scale);
        bikeShape.setFillColor(sf::Color(255, 200, 0));
        bikeShape.setOrigin(3.0f* scale, 3.0f* scale);
        bikeShape.setOutlineThickness(1.0f * scale);
        bikeShape.setOutlineColor(sf::Color::Black);
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

    const sf::Vector2f edgeStart = engine->getRoadCenterlineEntryPoint(road, start);
    const sf::Vector2f edgeEnd = engine->getRoadCenterlineEntryPoint(road, end);
    const double clampedRatio = std::clamp(ratio, 0.0, 1.0);
    return edgeStart + static_cast<float>(clampedRatio) * (edgeEnd - edgeStart);
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
    const double ratio = vehicle->getProgressRatio();
    const sf::Vector2f currentLanePos = laneOffset(currentRoad, pointOnRoad(currentRoad, ratio));

    Road* prevRoad = vehicle->getPreviousRoad();
    if (prevRoad != nullptr) {
        const double progressOnRoad = vehicle->getProgressOnRoad();
        const double roadLength = currentRoad->getDistance();
        const double transitionDist = std::min(15.0, roadLength * 0.5);
        if (transitionDist > 0.0 && progressOnRoad < transitionDist) {
            float rawT = std::clamp(static_cast<float>(progressOnRoad / transitionDist), 0.0f, 1.0f);
            float t = rawT * rawT * (3.0f - 2.0f * rawT);
            const sf::Vector2f prevLanePos = laneOffset(prevRoad, pointOnRoad(prevRoad, 1.0));
            return (1.0f - t) * prevLanePos + t * currentLanePos;
        }
    }

    return currentLanePos;
}

float VehicleSprite::resolveAngle() const {
    if (!vehicle || vehicle->getCurrentRoad() == nullptr) {
        return 0.0f;
    }

    auto roadAngle = [](const Road* r) {
        return std::atan2(r->getEnd()->getY() - r->getStart()->getY(),
                           r->getEnd()->getX() - r->getStart()->getX());
    };
    auto blendAngle = [](double from, double to, float t) {
        double diff = to - from;
        while (diff < -3.14159265358979323846) diff += 2.0 * 3.14159265358979323846;
        while (diff > 3.14159265358979323846) diff -= 2.0 * 3.14159265358979323846;
        return from + diff * t;
    };

    Road* currentRoad = vehicle->getCurrentRoad();
    const double currentAngle = roadAngle(currentRoad);

    Road* prevRoad = vehicle->getPreviousRoad();
    if (prevRoad != nullptr) {
        const double progressOnRoad = vehicle->getProgressOnRoad();
        const double roadLength = currentRoad->getDistance();
        const double transitionDist = std::min(15.0, roadLength * 0.5);
        if (transitionDist > 0.0 && progressOnRoad < transitionDist) {
            float rawT = std::clamp(static_cast<float>(progressOnRoad / transitionDist), 0.0f, 1.0f);
            float t = rawT * rawT * (3.0f - 2.0f * rawT);
            const double blended = blendAngle(roadAngle(prevRoad), currentAngle, t);
            return static_cast<float>(-blended * 180.0 / 3.14159265358979323846);
        }
    }

    return static_cast<float>(-currentAngle * 180.0 / 3.14159265358979323846);
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