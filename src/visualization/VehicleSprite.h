#ifndef VEHICLE_SPRITE_H
#define VEHICLE_SPRITE_H

#include <SFML/Graphics.hpp>
#include "../model/Vehicle.h"
#include "VisualizationEngine.h"

class VehicleSprite {
private:
    Vehicle* vehicle;
    const VisualizationEngine* engine;
    sf::RectangleShape shape;
    sf::RectangleShape bikeShape;
    float flashTimer = 0.0f;
    enum class Type { CAR, BUS, MOTORBIKE, EMERGENCY };
    Type vehicleType;

    void setupShape();
    sf::Vector2f resolvePosition() const;
    float resolveAngle() const;

public:
    VehicleSprite(Vehicle* v, const VisualizationEngine* eng);
    sf::Vector2f getPosition() const { return resolvePosition(); }
    float getAngle() const { return resolveAngle(); }
    void update(float dt);
    void draw(sf::RenderTarget& target) const;
    void drawAt(sf::RenderTarget& target, const sf::Vector2f& position, float angle) const;
};

#endif
