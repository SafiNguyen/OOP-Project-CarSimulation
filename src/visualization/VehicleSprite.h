#ifndef VEHICLE_SPRITE_H
#define VEHICLE_SPRITE_H

#include <SFML/Graphics.hpp>
#include "../model/Vehicle.h"
#include "VisualizationEngine.h"

class VehicleSprite {
private:
    Vehicle* vehicle;
    const VisualizationEngine* engine;

    // Basic shape
    sf::RectangleShape shape;
    sf::CircleShape bikeShape;

    float flashTimer = 0.0f;

    // Determine the vehicle type based on dynamic_cast
    enum class Type { CAR, BUS, MOTORBIKE, EMERGENCY };
    Type vehicleType;

    void setupShape();

public:
    VehicleSprite(Vehicle* v, const VisualizationEngine* eng);
    
    // Update location and color (e.g., flashing for ambulances)
    void update(float dt);
    
    // Draw a car on the screen
    void draw(sf::RenderTarget& target) const;
    void drawAt(sf::RenderTarget& target, const sf::Vector2f& position, float angle) const;
};

#endif