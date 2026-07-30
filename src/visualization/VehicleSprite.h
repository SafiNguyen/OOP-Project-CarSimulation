#ifndef VEHICLE_SPRITE_H
#define VEHICLE_SPRITE_H

#include <SFML/Graphics.hpp>
#include "Vehicle.h"
#include "VisualizationEngine.h"

class VehicleSprite {
private:
    Vehicle* vehicle;
    const VisualizationEngine* engine;
    sf::RectangleShape fallbackShape;
    sf::Sprite sprite;
    sf::Vector2f bodySize;
    bool textureAvailable = false;
    float flashTimer = 0.0f;
    bool emergencyBlue = false;
    enum class Type { CAR, BUS, MOTORBIKE, EMERGENCY };
    Type vehicleType;

    void setupShape();
    sf::Vector2f resolvePosition() const;
    float resolveAngle() const;
    void drawBody(sf::RenderTarget& target,
                  const sf::Vector2f& position,
                  float angle) const;
    void drawEmergencyLights(
        sf::RenderTarget& target,
        const sf::Vector2f& position,
        float angle) const;
    void drawTurnSignal(
        sf::RenderTarget& target,
        const sf::Vector2f& position,
        float angle,
        const sf::Vector2f& bodySize) const;

public:
    VehicleSprite(Vehicle* v, const VisualizationEngine* eng);
    sf::Vector2f getPosition() const { return resolvePosition(); }
    float getAngle() const { return resolveAngle(); }
    bool usesTexture() const { return textureAvailable; }
    void update(float dt);
    void draw(sf::RenderTarget& target) const;
    void drawAt(sf::RenderTarget& target, const sf::Vector2f& position, float angle) const;
};

#endif
