#include "VehicleSprite.h"

#include <cmath>

#include "VehicleRenderGeometry.h"

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr float OUTLINE_PIXELS = 0.55f;
}

VehicleSprite::VehicleSprite(
    Vehicle* vehicleValue,
    const VisualizationEngine* engineValue)
    : vehicle(vehicleValue),
      engine(engineValue),
      vehicleType(Type::CAR) {
    switch (vehicle != nullptr
                ? vehicle->getVehicleKind()
                : VehicleKind::Car) {
        case VehicleKind::Bus:
            vehicleType = Type::BUS;
            break;
        case VehicleKind::Motorbike:
            vehicleType = Type::MOTORBIKE;
            break;
        case VehicleKind::Emergency:
            vehicleType = Type::EMERGENCY;
            break;
        case VehicleKind::Car:
        default:
            vehicleType = Type::CAR;
            break;
    }
    setupShape();
}

void VehicleSprite::setupShape() {
    const VehicleScreenSize screenSize =
        vehicle != nullptr && engine != nullptr
            ? getVehicleScreenSize(*vehicle, *engine)
            : VehicleScreenSize{6.0f, 3.0f};
    const sf::Vector2f size{
        screenSize.lengthPixels,
        screenSize.widthPixels
    };

    switch (vehicleType) {
        case Type::CAR:
            shape.setSize(size);
            shape.setFillColor(sf::Color(50, 150, 255));
            shape.setOrigin(size.x * 0.5f, size.y * 0.5f);
            break;
        case Type::BUS:
            shape.setSize(size);
            shape.setFillColor(sf::Color(200, 162, 50));
            shape.setOrigin(size.x * 0.5f, size.y * 0.5f);
            break;
        case Type::EMERGENCY:
            shape.setSize(size);
            shape.setFillColor(sf::Color::Red);
            shape.setOrigin(size.x * 0.5f, size.y * 0.5f);
            break;
        case Type::MOTORBIKE:
            // A slim rectangle keeps the existing compact style while making
            // the path-derived heading visible.
            bikeShape.setSize(size);
            bikeShape.setFillColor(sf::Color(255, 200, 0));
            bikeShape.setOrigin(size.x * 0.5f, size.y * 0.5f);
            break;
    }

    sf::RectangleShape& configured =
        vehicleType == Type::MOTORBIKE ? bikeShape : shape;
    configured.setOutlineThickness(OUTLINE_PIXELS);
    configured.setOutlineColor(sf::Color::Black);
}

sf::Vector2f VehicleSprite::resolvePosition() const {
    if (vehicle == nullptr || engine == nullptr ||
        vehicle->getCurrentRoad() == nullptr) {
        return {};
    }
    const Pose2D pose = vehicle->getPose();
    return engine->worldToScreen(
        pose.position.x, pose.position.y);
}

float VehicleSprite::resolveAngle() const {
    if (vehicle == nullptr ||
        vehicle->getCurrentRoad() == nullptr) {
        return 0.0f;
    }
    return static_cast<float>(
        -vehicle->getPose().headingRadians * 180.0 / PI);
}

void VehicleSprite::update(float dt) {
    if (vehicle == nullptr ||
        vehicle->getCurrentRoad() == nullptr) {
        return;
    }
    if (vehicleType == Type::EMERGENCY) {
        flashTimer += dt;
        if (flashTimer > 0.2f) {
            shape.setFillColor(
                shape.getFillColor() == sf::Color::Red
                    ? sf::Color::Blue
                    : sf::Color::Red);
            flashTimer = 0.0f;
        }
    }
}

void VehicleSprite::draw(sf::RenderTarget& target) const {
    if (vehicle == nullptr ||
        vehicle->getCurrentRoad() == nullptr) {
        return;
    }

    const sf::Vector2f screenPosition = resolvePosition();
    const float angle = resolveAngle();
    if (vehicleType == Type::MOTORBIKE) {
        sf::RectangleShape drawable = bikeShape;
        drawable.setPosition(screenPosition);
        drawable.setRotation(angle);
        target.draw(drawable);
    } else {
        sf::RectangleShape drawable = shape;
        drawable.setPosition(screenPosition);
        drawable.setRotation(angle);
        target.draw(drawable);
    }
}

void VehicleSprite::drawAt(
    sf::RenderTarget& target,
    const sf::Vector2f& position,
    float angle) const {
    if (vehicleType == Type::MOTORBIKE) {
        sf::RectangleShape drawable = bikeShape;
        drawable.setPosition(position);
        drawable.setRotation(angle);
        target.draw(drawable);
    } else {
        sf::RectangleShape drawable = shape;
        drawable.setPosition(position);
        drawable.setRotation(angle);
        target.draw(drawable);
    }
}
