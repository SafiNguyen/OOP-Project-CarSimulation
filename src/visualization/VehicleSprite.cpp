#include "VehicleSprite.h"

#include <algorithm>
#include <cmath>

#include "VehicleAssets.h"
#include "VehicleRenderGeometry.h"

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr float OUTLINE_PIXELS = 0.55f;
const sf::Color TURN_SIGNAL_AMBER(255, 165, 0);
constexpr float SIGNAL_CORNER_OFFSET_PIXELS = 0.2f;
constexpr float MIN_SIGNAL_RADIUS_PIXELS = 1.45f;
constexpr float MAX_SIGNAL_RADIUS_PIXELS = 2.3f;
constexpr float SIGNAL_HALO_PIXELS = 0.55f;
constexpr float SPRITE_HEADING_OFFSET_DEGREES = 90.0f;
constexpr float MIN_EMERGENCY_LIGHT_RADIUS_PIXELS = 1.1f;
constexpr float MAX_EMERGENCY_LIGHT_RADIUS_PIXELS = 1.8f;
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
            ? getVehicleVisualScreenSize(
                  *vehicle, *engine)
            : VehicleScreenSize{6.0f, 3.0f};
    bodySize = {
        screenSize.lengthPixels,
        screenSize.widthPixels
    };

    switch (vehicleType) {
        case Type::CAR:
            fallbackShape.setFillColor(
                sf::Color(50, 150, 255));
            break;
        case Type::BUS:
            fallbackShape.setFillColor(
                sf::Color(200, 162, 50));
            break;
        case Type::EMERGENCY:
            fallbackShape.setFillColor(sf::Color::Red);
            break;
        case Type::MOTORBIKE:
            fallbackShape.setFillColor(
                sf::Color(255, 200, 0));
            break;
    }

    fallbackShape.setSize(bodySize);
    fallbackShape.setOrigin(
        bodySize.x * 0.5f,
        bodySize.y * 0.5f);
    fallbackShape.setOutlineThickness(OUTLINE_PIXELS);
    fallbackShape.setOutlineColor(sf::Color::Black);

    const VehicleKind kind =
        vehicle != nullptr
            ? vehicle->getVehicleKind()
            : VehicleKind::Car;
    const sf::Texture* texture =
        VehicleAssets::instance().textureFor(kind);
    if (texture == nullptr ||
        texture->getSize().x == 0u ||
        texture->getSize().y == 0u) {
        return;
    }

    textureAvailable = true;
    sprite.setTexture(*texture, true);
    const sf::Vector2u textureSize = texture->getSize();
    sprite.setOrigin(
        static_cast<float>(textureSize.x) * 0.5f,
        static_cast<float>(textureSize.y) * 0.5f);
    // Source art faces upward: image X is vehicle width and image Y is
    // vehicle length. A +90 degree rotation aligns it with the renderer's
    // local +X forward direction.
    sprite.setScale(
        bodySize.y / static_cast<float>(textureSize.x),
        bodySize.x / static_cast<float>(textureSize.y));
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
            emergencyBlue = !emergencyBlue;
            flashTimer = 0.0f;
        }
    }
}

void VehicleSprite::drawBody(
    sf::RenderTarget& target,
    const sf::Vector2f& position,
    float angle) const {
    if (textureAvailable) {
        sf::Sprite drawable = sprite;
        drawable.setPosition(position);
        drawable.setRotation(
            angle + SPRITE_HEADING_OFFSET_DEGREES);
        target.draw(drawable);
        return;
    }

    sf::RectangleShape drawable = fallbackShape;
    drawable.setPosition(position);
    drawable.setRotation(angle);
    target.draw(drawable);
}

void VehicleSprite::drawEmergencyLights(
    sf::RenderTarget& target,
    const sf::Vector2f& position,
    float angle) const {
    if (vehicleType != Type::EMERGENCY) {
        return;
    }

    sf::Transform transform;
    transform.translate(position);
    transform.rotate(angle);
    const float radius = std::clamp(
        bodySize.y * 0.18f,
        MIN_EMERGENCY_LIGHT_RADIUS_PIXELS,
        MAX_EMERGENCY_LIGHT_RADIUS_PIXELS);
    const sf::Vector2f lampPosition =
        transform.transformPoint(
            bodySize.x * 0.22f,
            0.0f);
    sf::CircleShape lamp(radius);
    lamp.setOrigin(radius, radius);
    lamp.setPosition(lampPosition);
    lamp.setFillColor(
        emergencyBlue
            ? sf::Color::Blue
            : sf::Color::Red);
    target.draw(lamp);
}

void VehicleSprite::draw(sf::RenderTarget& target) const {
    if (vehicle == nullptr ||
        vehicle->getCurrentRoad() == nullptr) {
        return;
    }

    const sf::Vector2f screenPosition = resolvePosition();
    const float angle = resolveAngle();
    drawBody(target, screenPosition, angle);
    drawEmergencyLights(
        target, screenPosition, angle);
    drawTurnSignal(
        target, screenPosition, angle, bodySize);
}

void VehicleSprite::drawTurnSignal(
    sf::RenderTarget& target,
    const sf::Vector2f& position,
    float angle,
    const sf::Vector2f& bodySize) const {
    if (vehicle == nullptr ||
        vehicle->getTurnSignal() == TurnSignal::Off ||
        !vehicle->isTurnSignalBlinkOn()) {
        return;
    }

    sf::Transform transform;
    transform.translate(position);
    transform.rotate(angle);
    const float localY =
        vehicle->getTurnSignal() == TurnSignal::Left
            ? -bodySize.y * 0.5f
            : bodySize.y * 0.5f;
    const float radius = std::clamp(
        bodySize.y * 0.24f,
        MIN_SIGNAL_RADIUS_PIXELS,
        MAX_SIGNAL_RADIUS_PIXELS);
    const sf::Vector2f lampPosition =
        transform.transformPoint(
            -bodySize.x * 0.5f -
                radius * SIGNAL_CORNER_OFFSET_PIXELS,
            localY +
                (vehicle->getTurnSignal() ==
                         TurnSignal::Left
                     ? -radius *
                           SIGNAL_CORNER_OFFSET_PIXELS
                     : radius *
                           SIGNAL_CORNER_OFFSET_PIXELS));
    sf::CircleShape halo(
        radius + SIGNAL_HALO_PIXELS);
    halo.setOrigin(
        radius + SIGNAL_HALO_PIXELS,
        radius + SIGNAL_HALO_PIXELS);
    halo.setPosition(lampPosition);
    halo.setFillColor(sf::Color(55, 30, 5));
    target.draw(halo);
    sf::CircleShape lamp(radius);
    lamp.setOrigin(radius, radius);
    lamp.setPosition(lampPosition);
    lamp.setFillColor(TURN_SIGNAL_AMBER);
    target.draw(lamp);
}

void VehicleSprite::drawAt(
    sf::RenderTarget& target,
    const sf::Vector2f& position,
    float angle) const {
    drawBody(target, position, angle);
    drawEmergencyLights(target, position, angle);
    drawTurnSignal(
        target, position, angle, bodySize);
}
