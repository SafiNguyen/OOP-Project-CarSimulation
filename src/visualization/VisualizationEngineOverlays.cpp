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
            const float boxHalfExtent = getIntersectionBoxHalfExtent(intersection);

            // Stop line position: pulled back to the edge of the intersection box
            // so the road and junction read as one connected shape.
            const sf::Vector2f stopLineCenter = intersectionPoint - approachDir * (boxHalfExtent + 4.0f) + approachNormal * offsetAmount;

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
        const float halfExtent = getIntersectionBoxHalfExtent(intersection);

        sf::RectangleShape core({halfExtent * 2.0f, halfExtent * 2.0f});
        core.setOrigin(halfExtent, halfExtent);
        core.setPosition(point);
        core.setFillColor(getIntersectionBoxColor(intersection));
        core.setOutlineThickness(0.0f);
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
    const Intersection* start = road->getStart();
    const Intersection* end = road->getEnd();
    if (start == nullptr || end == nullptr) {
        return worldToScreen(intersection->getX(), intersection->getY());
    }

    const sf::Vector2f roadStart = worldToScreen(start->getX(), start->getY());
    const sf::Vector2f roadEnd = worldToScreen(end->getX(), end->getY());
    const sf::Vector2f normal = roadNormal(roadStart, roadEnd);

    const float laneWidth = 10.0f;
    const int laneCount = std::max(1, road->getLaneCount());
    const float totalWidth = static_cast<float>(laneCount) * laneWidth;

    bool hasReverse = false;
    if (end && start) {
        for (Road* r : end->getOutgoingRoads()) {
            if (r->getEnd() == start) { hasReverse = true; break; }
        }
    }
    const float offsetAmount = hasReverse ? (totalWidth * 0.5f + 1.0f) : 0.0f;

    return getRoadCenterlineEntryPoint(road, intersection) + normal * offsetAmount;
}


sf::Vector2f VisualizationEngine::getRoadCenterlineEntryPoint(const Road* road, const Intersection* intersection) const {
    if (road == nullptr || intersection == nullptr) {
        return {};
    }
    const Intersection* start = road->getStart();
    const Intersection* end = road->getEnd();
    if (start == nullptr || end == nullptr) {
        return worldToScreen(intersection->getX(), intersection->getY());
    }

    const sf::Vector2f intersectionPoint = worldToScreen(intersection->getX(), intersection->getY());
    const sf::Vector2f roadStart = worldToScreen(start->getX(), start->getY());
    const sf::Vector2f roadEnd = worldToScreen(end->getX(), end->getY());

    sf::Vector2f towardOtherEnd = (intersection == start) ? (roadEnd - roadStart) : (roadStart - roadEnd);
    const float length = std::sqrt(towardOtherEnd.x * towardOtherEnd.x + towardOtherEnd.y * towardOtherEnd.y);
    if (length <= 0.01f) {
        return intersectionPoint;
    }
    towardOtherEnd /= length;

    const float boxHalfExtent = getIntersectionBoxHalfExtent(intersection);
    const float inset = std::min(boxHalfExtent, length * 0.5f);
    return intersectionPoint + towardOtherEnd * inset; // no normal/offsetAmount here
}

float VisualizationEngine::getIntersectionBoxHalfExtent(const Intersection* intersection) const {
    if (intersection == nullptr) {
        return 12.0f;
    }

    if (intersection->isRoundabout()) {
        return 20.0f;
    }

    const auto collectMaxRoadWidth = [](const std::vector<Road*>& roads) {
        float maxWidth = 0.0f;
        for (const auto* road : roads) {
            if (road == nullptr) {
                continue;
            }
            const int laneCount = std::max(1, road->getLaneCount());
            maxWidth = std::max(maxWidth, static_cast<float>(laneCount) * 10.0f);
        }
        return maxWidth;
    };

    const float maxWidth = std::max(collectMaxRoadWidth(intersection->getIncomingRoads()),
                                     collectMaxRoadWidth(intersection->getOutgoingRoads()));
    float halfExtent = std::max(14.0f, maxWidth * 0.5f + 8.0f);

    constexpr float kMaxShareOfRoad = 0.35f;
    float shortestRoadLength = -1.0f;

    const auto trackShortest = [&](const std::vector<Road*>& roads) {
        for (const Road* road : roads) {
            if (road == nullptr || road->getStart() == nullptr || road->getEnd() == nullptr) {
                continue;
            }
            const sf::Vector2f a = worldToScreen(road->getStart()->getX(), road->getStart()->getY());
            const sf::Vector2f b = worldToScreen(road->getEnd()->getX(), road->getEnd()->getY());
            const float len = distanceBetween(a, b);
            if (shortestRoadLength < 0.0f || len < shortestRoadLength) {
                shortestRoadLength = len;
            }
        }
    };
    trackShortest(intersection->getIncomingRoads());
    trackShortest(intersection->getOutgoingRoads());

    if (shortestRoadLength >= 0.0f) {
        halfExtent = std::min(halfExtent, shortestRoadLength * kMaxShareOfRoad);
    }

    return std::max(6.0f, halfExtent); 
}

sf::Color VisualizationEngine::getIntersectionBoxColor(const Intersection* intersection) const {
    const sf::Color fallbackGray(110, 110, 110);
    if (intersection == nullptr) {
        return fallbackGray;
    }

    int r = 0;
    int g = 0;
    int b = 0;
    int count = 0;

    const auto accumulate = [&](const std::vector<Road*>& roads) {
        for (const Road* road : roads) {
            if (road == nullptr || road->isBridge() || road->isTunnel()) {
                continue; // these have their own distinct colors, not congestion-based
            }
            const sf::Color c = heatMapEnabled_ ? colorForRoad(road) : sf::Color(110, 110, 110);
            r += c.r;
            g += c.g;
            b += c.b;
            ++count;
        }
    };
    accumulate(intersection->getIncomingRoads());
    accumulate(intersection->getOutgoingRoads());

    if (count == 0) {
        return fallbackGray;
    }
    return sf::Color(static_cast<sf::Uint8>(r / count),
                      static_cast<sf::Uint8>(g / count),
                      static_cast<sf::Uint8>(b / count));
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