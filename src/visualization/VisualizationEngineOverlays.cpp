#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>

#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/PointOfInterest.h"
#include "model/Road.h"

void VisualizationEngine::drawTrafficLights(sf::RenderTarget& target, const Graph& graph) const {
    for (auto* intersection : graph.getAllIntersections()) {
        for (const auto* road : intersection->getIncomingRoads()) {
            auto* light = intersection->getLightForIncomingRoad(road);
            if (light == nullptr) {
                continue;
            }

            const sf::Vector2f point = getRoadEntryPoint(const_cast<Road*>(road), intersection);
            const sf::Vector2f intersectionPoint = worldToScreen(intersection->getX(), intersection->getY());
            const sf::Vector2f roadStart = worldToScreen(road->getStart()->getX(), road->getStart()->getY());
            sf::Vector2f approachDirection = intersectionPoint - roadStart;
            const float approachLength = std::sqrt(approachDirection.x * approachDirection.x + approachDirection.y * approachDirection.y);
            if (approachLength > 0.01f) {
                approachDirection /= approachLength;
            } else {
                approachDirection = {0.0f, -1.0f};
            }
            const sf::Vector2f approachNormal = roadNormal(roadStart, intersectionPoint);
            const int laneCount = std::max(1, road->getLaneCount());
            const float laneSpacing = 4.5f;
            const float laneBarLength = std::max(16.0f, static_cast<float>(laneCount - 1) * laneSpacing + 10.0f);
            const sf::Vector2f laneIndicatorCenter = point + approachNormal * 12.0f - approachDirection * 16.0f;
            const float laneIndicatorAngle = std::atan2(approachDirection.y, approachDirection.x) * 180.0f / 3.14159265f;

            sf::RectangleShape laneBar({laneBarLength, 2.8f});
            laneBar.setOrigin(laneBarLength * 0.5f, 1.4f);
            laneBar.setPosition(laneIndicatorCenter);
            laneBar.setRotation(laneIndicatorAngle);
            laneBar.setFillColor(sf::Color(230, 230, 230, 200));
            target.draw(laneBar);

            const float laneStartOffset = -((laneCount - 1) * laneSpacing) * 0.5f;
            for (int laneIndex = 0; laneIndex < laneCount; ++laneIndex) {
                const float laneOffset = laneStartOffset + laneIndex * laneSpacing;
                const sf::Vector2f markerPosition = laneIndicatorCenter + approachDirection * laneOffset;
                sf::RectangleShape laneMarker({2.0f, 8.0f});
                laneMarker.setOrigin(1.0f, 4.0f);
                laneMarker.setPosition(markerPosition);
                laneMarker.setRotation(laneIndicatorAngle + 90.0f);
                laneMarker.setFillColor(sf::Color(40, 40, 40, 220));
                target.draw(laneMarker);
            }

            sf::RectangleShape approachArrow({16.0f, 2.5f});
            approachArrow.setOrigin(0.0f, 1.25f);
            approachArrow.setPosition(point.x + approachDirection.x * 2.0f, point.y + approachDirection.y * 2.0f);
            approachArrow.setRotation(laneIndicatorAngle);
            approachArrow.setFillColor(sf::Color(255, 255, 255, 180));
            target.draw(approachArrow);

            sf::RectangleShape housing({12.0f, 28.0f});
            housing.setOrigin(6.0f, 24.0f);
            housing.setPosition(point);
            housing.setFillColor(sf::Color(25, 25, 25, 235));
            housing.setOutlineThickness(1.0f);
            housing.setOutlineColor(sf::Color(90, 90, 90));
            target.draw(housing);

            sf::RectangleShape pole({3.0f, 14.0f});
            pole.setFillColor(sf::Color(40, 40, 40));
            pole.setPosition(point.x - 1.5f, point.y - 14.0f);
            target.draw(pole);

            const LightState state = light->getState();
            const sf::Color offColor(55, 55, 55);
            const sf::Color red = (state == LightState::RED) ? lightColor(LightState::RED) : offColor;
            const sf::Color yellow = (state == LightState::YELLOW) ? lightColor(LightState::YELLOW) : offColor;
            const sf::Color green = (state == LightState::GREEN) ? lightColor(LightState::GREEN) : offColor;

            auto drawLamp = [&target](const sf::Vector2f& center, const sf::Color& color) {
                sf::CircleShape lamp(3.5f);
                lamp.setOrigin(3.5f, 3.5f);
                lamp.setPosition(center);
                lamp.setFillColor(color);
                lamp.setOutlineThickness(0.8f);
                lamp.setOutlineColor(sf::Color::Black);
                target.draw(lamp);
            };

            drawLamp({point.x, point.y - 19.0f}, red);
            drawLamp({point.x, point.y - 11.0f}, yellow);
            drawLamp({point.x, point.y - 3.0f}, green);
        }
    }
}

void VisualizationEngine::drawIntersectionNode(sf::RenderTarget& target, const Intersection* intersection) const {
    if (intersection == nullptr) {
        return;
    }

    const sf::Vector2f point = worldToScreen(intersection->getX(), intersection->getY());
    if (intersection->isRoundabout()) {
        sf::CircleShape hub(20.0f);
        hub.setOrigin(20.0f, 20.0f);
        hub.setPosition(point);
        hub.setFillColor(sf::Color(100, 150, 100));
        hub.setOutlineThickness(8.0f);
        hub.setOutlineColor(sf::Color(110, 110, 110));
        target.draw(hub);
    } else {
        sf::CircleShape hub(16.0f);
        hub.setOrigin(16.0f, 16.0f);
        hub.setPosition(point);
        hub.setFillColor(sf::Color(85, 85, 85));
        hub.setOutlineThickness(2.0f);
        hub.setOutlineColor(sf::Color(210, 210, 210, 180));
        target.draw(hub);

        sf::CircleShape core(6.0f);
        core.setOrigin(6.0f, 6.0f);
        core.setPosition(point);
        core.setFillColor(sf::Color(230, 230, 230));
        target.draw(core);
    }

    if (spriteTexture_ != nullptr) {
        sf::Sprite sprite(*spriteTexture_, spriteRect_);
        sprite.setPosition(point.x - spriteSize_.x * 0.5f, point.y - spriteSize_.y * 0.5f);
        sprite.setScale(spriteSize_.x / std::max(1.0f, static_cast<float>(spriteRect_.width ? spriteRect_.width : spriteTexture_->getSize().x)),
                        spriteSize_.y / std::max(1.0f, static_cast<float>(spriteRect_.height ? spriteRect_.height : spriteTexture_->getSize().y)));
        target.draw(sprite);
    }
}

sf::Vector2f VisualizationEngine::getRoadEntryPoint(const Road* road, const Intersection* intersection) const {
    if (road == nullptr || intersection == nullptr) {
        return {};
    }

    const sf::Vector2f intersectionPoint = worldToScreen(intersection->getX(), intersection->getY());
    const sf::Vector2f roadStart = worldToScreen(road->getStart()->getX(), road->getStart()->getY());
    sf::Vector2f direction = intersectionPoint - roadStart;
    const float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (length <= 0.01f) {
        return intersectionPoint;
    }
    direction /= length;
    const sf::Vector2f normal = roadNormal(roadStart, intersectionPoint);
    return intersectionPoint - direction * 18.0f + normal * 8.0f;
}

sf::Color VisualizationEngine::lightColor(LightState state) const {
    switch (state) {
    case LightState::GREEN:
        return sf::Color(70, 210, 80);
    case LightState::YELLOW:
        return sf::Color(240, 200, 40);
    case LightState::RED:
        return sf::Color(220, 60, 60);
    }
    return sf::Color(120, 120, 120);
}

void VisualizationEngine::drawPOIs(sf::RenderTarget& target, const Graph& graph) const {
    const auto& pois = graph.getAllPOIs();
    for (const auto* poi : pois) {
        if (!poi) continue;
        sf::Vector2f pos = worldToScreen(poi->getX(), poi->getY());

        sf::Color poiColor(255, 150, 0);
        if (poi->getType() == POIType::PARKING_LOT) poiColor = sf::Color(100, 100, 255);
        else if (poi->getType() == POIType::BUS_STATION) poiColor = sf::Color(50, 200, 50);
        else if (poi->getType() == POIType::HOSPITAL) poiColor = sf::Color(255, 50, 50);
        else if (poi->getType() == POIType::SUPERMARKET) poiColor = sf::Color(200, 200, 50);

        sf::CircleShape circle(5.0f);
        circle.setOrigin(5.0f, 5.0f);
        circle.setPosition(pos);
        circle.setFillColor(poiColor);
        circle.setOutlineThickness(1.0f);
        circle.setOutlineColor(sf::Color::White);
        target.draw(circle);

        if (font_) {
            sf::Text text;
            text.setFont(*font_);
            text.setString(poi->getName());
            text.setCharacterSize(10);
            text.setFillColor(sf::Color::White);
            text.setOutlineColor(sf::Color::Black);
            text.setOutlineThickness(1.0f);
            text.setPosition(pos.x + 8.0f, pos.y - 6.0f);
            target.draw(text);
        }
    }
}

void VisualizationEngine::drawRoadNames(sf::RenderTarget& target, const std::vector<Road*>& roads) const {
    if (!font_) return;

    for (const auto* road : roads) {
        if (!road || road->getName().empty()) continue;
        const Intersection* start = road->getStart();
        const Intersection* end = road->getEnd();
        if (!start || !end) continue;

        sf::Vector2f a = worldToScreen(start->getX(), start->getY());
        sf::Vector2f b = worldToScreen(end->getX(), end->getY());
        sf::Vector2f mid = (a + b) * 0.5f;

        float angle = std::atan2(b.y - a.y, b.x - a.x) * 180.0f / 3.14159265f;
        if (angle > 90.0f || angle < -90.0f) {
            angle += 180.0f;
        }

        sf::Text text;
        text.setFont(*font_);
        text.setString(road->getName());
        text.setCharacterSize(12);
        text.setFillColor(sf::Color::White);
        text.setOutlineColor(sf::Color::Black);
        text.setOutlineThickness(1.0f);

        sf::FloatRect bounds = text.getLocalBounds();
        text.setOrigin(bounds.left + bounds.width * 0.5f, bounds.top + bounds.height * 0.5f);
        text.setPosition(mid);
        text.setRotation(angle);

        target.draw(text);
    }
}