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

            const sf::Vector2f intersectionPoint = worldToScreen(intersection->getX(), intersection->getY());
            const sf::Vector2f roadStart = worldToScreen(road->getStart()->getX(), road->getStart()->getY());

            // Compute approach direction (from road start towards intersection)
            sf::Vector2f approachDir = intersectionPoint - roadStart;
            const float approachLength = std::sqrt(approachDir.x * approachDir.x + approachDir.y * approachDir.y);
            if (approachLength > 0.01f) {
                approachDir /= approachLength;
            } else {
                approachDir = {0.0f, -1.0f};
            }

            // Perpendicular to approach direction
            const sf::Vector2f approachNormal = roadNormal(roadStart, intersectionPoint);

            // Compute road width and offset
            const int laneCount = std::max(1, road->getLaneCount());
            const float laneWidth = 10.0f;
            const float totalWidth = static_cast<float>(laneCount) * laneWidth;

            bool hasReverse = false;
            auto* endIntersection = road->getEnd();
            auto* startIntersection = road->getStart();
            if (endIntersection && startIntersection) {
                for (Road* r : endIntersection->getOutgoingRoads()) {
                    if (r->getEnd() == startIntersection) {
                        hasReverse = true;
                        break;
                    }
                }
            }
            const float offsetAmount = hasReverse ? (totalWidth * 0.5f + 1.0f) : 0.0f;

            // Stop line position: on the road, pulled back from intersection
            const sf::Vector2f stopLineCenter = intersectionPoint - approachDir * 20.0f + approachNormal * offsetAmount;

            const float approachAngle = std::atan2(approachDir.y, approachDir.x) * 180.0f / 3.14159265f;

            // --- Stop line across the road ---
            sf::RectangleShape stopLine({totalWidth, 2.0f});
            stopLine.setOrigin(totalWidth * 0.5f, 1.0f);
            stopLine.setPosition(stopLineCenter);
            stopLine.setRotation(approachAngle + 90.0f);
            stopLine.setFillColor(sf::Color(255, 255, 255, 160));
            target.draw(stopLine);

            // --- Traffic light housing: on the SIDE of the road ---
            // Place it at the road edge, offset outward from the road center
            const sf::Vector2f roadEdgePoint = stopLineCenter + approachNormal * (totalWidth * 0.5f + 4.0f);
            const sf::Vector2f housingDir = approachNormal; // outward from road
            const sf::Vector2f housingCenter = roadEdgePoint + housingDir * 12.0f;

            const float housingAngle = std::atan2(housingDir.y, housingDir.x) * 180.0f / 3.14159265f;

            // Housing background
            sf::RectangleShape housing({28.0f, 12.0f});
            housing.setOrigin(14.0f, 6.0f);
            housing.setPosition(housingCenter);
            housing.setRotation(housingAngle);
            housing.setFillColor(sf::Color(25, 25, 25, 235));
            housing.setOutlineThickness(1.0f);
            housing.setOutlineColor(sf::Color(90, 90, 90));
            target.draw(housing);

            // Pole connecting road edge to housing
            sf::RectangleShape pole({12.0f, 2.5f});
            pole.setOrigin(0.0f, 1.25f);
            pole.setPosition(roadEdgePoint);
            pole.setRotation(housingAngle);
            pole.setFillColor(sf::Color(50, 50, 50));
            target.draw(pole);

            // --- Lamps ---
            const LightState state = light->getState();
            const sf::Color offColor(55, 55, 55);
            const sf::Color redColor = (state == LightState::RED) ? lightColor(LightState::RED) : offColor;
            const sf::Color yellowColor = (state == LightState::YELLOW) ? lightColor(LightState::YELLOW) : offColor;
            const sf::Color greenColor = (state == LightState::GREEN) ? lightColor(LightState::GREEN) : offColor;

            auto drawLamp = [&target](const sf::Vector2f& center, const sf::Color& color) {
                sf::CircleShape lamp(3.5f);
                lamp.setOrigin(3.5f, 3.5f);
                lamp.setPosition(center);
                lamp.setFillColor(color);
                lamp.setOutlineThickness(0.8f);
                lamp.setOutlineColor(sf::Color::Black);
                target.draw(lamp);
            };

            // Lamps along the housing direction: Red -> Yellow -> Green
            drawLamp(housingCenter - housingDir * 8.0f, redColor);
            drawLamp(housingCenter, yellowColor);
            drawLamp(housingCenter + housingDir * 8.0f, greenColor);
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
        // Simple small white dot to mark intersection without cluttering the view
        sf::CircleShape core(5.0f);
        core.setOrigin(5.0f, 5.0f);
        core.setPosition(point);
        core.setFillColor(sf::Color(230, 230, 230, 200));
        core.setOutlineThickness(1.0f);
        core.setOutlineColor(sf::Color(255, 255, 255, 120));
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

    // Compute road offset the same way drawGraph does
    const float laneWidth = 10.0f;
    const int laneCount = std::max(1, road->getLaneCount());
    const float totalWidth = static_cast<float>(laneCount) * laneWidth;

    bool hasReverse = false;
    auto* endIntersection = road->getEnd();
    auto* startIntersection = road->getStart();
    if (endIntersection && startIntersection) {
        for (Road* r : endIntersection->getOutgoingRoads()) {
            if (r->getEnd() == startIntersection) {
                hasReverse = true;
                break;
            }
        }
    }

    const float offsetAmount = hasReverse ? (totalWidth * 0.5f + 1.0f) : 0.0f;

    // Position the entry point at the intersection end of the offset road centerline,
    // pulled back along the direction by a small amount so it sits right at the road edge
    return intersectionPoint - direction * 18.0f + normal * offsetAmount;
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