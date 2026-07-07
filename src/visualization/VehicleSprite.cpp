#include "VehicleSprite.h"
#include "../model/Road.h"
#include "../model/Intersection.h"
#include "../model/Car.h"
#include "../model/Bus.h"
#include "../model/Motorbike.h"
#include "../model/EmergencyVehicle.h"
#include <cmath>


VehicleSprite::VehicleSprite(Vehicle* v, const VisualizationEngine* eng) 
    : vehicle(v), engine(eng) {
    
    // Phân loại xe để vẽ hình tương ứng
    if (dynamic_cast<Bus*>(vehicle)) vehicleType = Type::BUS;
    else if (dynamic_cast<Motorbike*>(vehicle)) vehicleType = Type::MOTORBIKE;
    else if (dynamic_cast<EmergencyVehicle*>(vehicle)) vehicleType = Type::EMERGENCY;
    else vehicleType = Type::CAR;

    setupShape();
}

void VehicleSprite::setupShape() {
    switch (vehicleType) {
        case Type::CAR:
            shape.setSize({12.0f, 6.0f});
            shape.setFillColor(sf::Color(50, 150, 255)); // Màu xanh dương
            shape.setOrigin(6.0f, 3.0f);
            break;
        case Type::BUS:
            shape.setSize({20.0f, 8.0f});
            shape.setFillColor(sf::Color(50, 200, 50)); // Màu xanh lá dài
            shape.setOrigin(10.0f, 4.0f);
            break;
        case Type::EMERGENCY:
            shape.setSize({14.0f, 7.0f});
            shape.setFillColor(sf::Color::Red); // Màu đỏ cứu thương
            shape.setOrigin(7.0f, 3.5f);
            break;
        case Type::MOTORBIKE:
            bikeShape.setRadius(3.0f);
            bikeShape.setFillColor(sf::Color(255, 200, 0)); // Màu vàng tròn nhỏ
            bikeShape.setOrigin(3.0f, 3.0f);
            break;
    }
}

void VehicleSprite::update(float dt) {
    if (!vehicle || vehicle->getCurrentRoad() == nullptr) return;

    if (vehicleType == Type::EMERGENCY) {
       flashTimer = 0.0f;
        flashTimer += dt;
        if (flashTimer > 0.2f) {
            if (shape.getFillColor() == sf::Color::Red)
                shape.setFillColor(sf::Color::Blue);
            else
                shape.setFillColor(sf::Color::Red);
            flashTimer = 0.0f;
        }
    }
}

void VehicleSprite::draw(sf::RenderTarget& target) const {
    if (!vehicle || vehicle->getCurrentRoad() == nullptr) return;

    Road* currentRoad = vehicle->getCurrentRoad();
    Intersection* start = currentRoad->getStart();
    Intersection* end = currentRoad->getEnd();

    float ratio = vehicle->getProgressRatio();
    double worldX = start->getX() + ratio * (end->getX() - start->getX());
    double worldY = start->getY() + ratio * (end->getY() - start->getY());

    sf::Vector2f screenPos = engine->worldToScreen(worldX, worldY);

    float angle = std::atan2(end->getY() - start->getY(), 
                         end->getX() - start->getX()) 
                        * 180.0f / 3.14159265f;
    angle = -angle; 

    // 4. Vẽ xe
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