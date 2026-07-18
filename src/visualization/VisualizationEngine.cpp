#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>

#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "model/PointOfInterest.h"

VisualizationEngine::VisualizationEngine(sf::Vector2u windowSize, float margin)
    : windowSize_(windowSize),
      margin_(margin),
      minX_(0.0),
      minY_(0.0),
      maxX_(100.0),
      maxY_(100.0),
      scale_(1.0),
      spriteTexture_(nullptr),
      spriteRect_(),
      spriteSize_(24.0f, 24.0f),
      font_(nullptr),
      heatMapEnabled_(true) {
}

void VisualizationEngine::prepare(const Graph& graph) {
    auto intersections = graph.getAllIntersections();
    std::sort(intersections.begin(), intersections.end(), [](Intersection* lhs, Intersection* rhs) {
        return lhs->getId() < rhs->getId();
    });

    if (!intersections.empty()) {
        minX_ = maxX_ = intersections.front()->getX();
        minY_ = maxY_ = intersections.front()->getY();
        for (auto* intersection : intersections) {
            const double x = intersection->getX();
            const double y = intersection->getY();
            minX_ = std::min(minX_, x);
            maxX_ = std::max(maxX_, x);
            minY_ = std::min(minY_, y);
            maxY_ = std::max(maxY_, y);
        }
    } else {
        minX_ = 0.0;
        maxX_ = 100.0;
        minY_ = 0.0;
        maxY_ = 100.0;
    }

    const double rangeX = std::max(1.0, maxX_ - minX_);
    const double rangeY = std::max(1.0, maxY_ - minY_);
    const double scaleX = (windowSize_.x > 2 * margin_) ? (windowSize_.x - 2 * margin_) / rangeX : 1.0;
    const double scaleY = (windowSize_.y > 2 * margin_) ? (windowSize_.y - 2 * margin_) / rangeY : 1.0;
    scale_ = std::min(scaleX, scaleY);

    routePoints_.clear();
    routePoints_.reserve(intersections.size());
    for (auto* intersection : intersections) {
        routePoints_.push_back(worldToScreen(intersection->getX(), intersection->getY()));
    }

    if (routePoints_.size() < 2) {
        routePoints_.push_back({windowSize_.x * 0.8f, windowSize_.y * 0.2f});
        routePoints_.push_back({windowSize_.x * 0.2f, windowSize_.y * 0.8f});
    }
}

sf::Vector2f VisualizationEngine::worldToScreen(double x, double y) const {
    const float sx = static_cast<float>(margin_ + (x - minX_) * scale_);
    const float sy = static_cast<float>(windowSize_.y - margin_ - (y - minY_) * scale_);
    return {sx, sy};
}

const std::vector<sf::Vector2f>& VisualizationEngine::getRoutePoints() const {
    return routePoints_;
}

sf::Color VisualizationEngine::colorForRoad(const Road* road) const {
    if (road == nullptr) {
        return sf::Color(120, 120, 120);
    }

    if (road->isBlocked()) {
        return sf::Color(180, 40, 40);
    }

    const double congestion = std::max(1.0, road->getCongestionLevel());
    const float normalized = static_cast<float>(std::clamp((congestion - 1.0) / 4.0, 0.0, 1.0));

    const sf::Color green(45, 190, 90);
    const sf::Color yellow(245, 190, 45);
    const sf::Color red(220, 55, 55);

    if (normalized < 0.5f) {
        return mixColor(green, yellow, normalized * 2.0f);
    }

    return mixColor(yellow, red, (normalized - 0.5f) * 2.0f);
}

void VisualizationEngine::drawGraph(sf::RenderTarget& target, const Graph& graph) const {
    auto roads = graph.getAllRoads();
    auto intersections = graph.getAllIntersections();

    std::sort(intersections.begin(), intersections.end(), [](Intersection* lhs, Intersection* rhs) {
        return lhs->getId() < rhs->getId();
    });

    for (auto* road : roads) {
        auto* start = road->getStart();
        auto* end = road->getEnd();
        if (start == nullptr || end == nullptr) {
            continue;
        }

        const sf::Vector2f a = worldToScreen(start->getX(), start->getY());
        const sf::Vector2f b = worldToScreen(end->getX(), end->getY());
        const float laneWidth = 10.0f;
        const int laneCount = road->getLaneCount();
        const float totalWidth = static_cast<float>(laneCount) * laneWidth;
        sf::Vector2f dir = b - a;
        const float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (length <= 0.01f) {
            continue;
        }
        const sf::Vector2f norm = roadNormal(a, b);

        bool hasReverse = false;
        for (Road* r : end->getOutgoingRoads()) {
            if (r->getEnd() == start) {
                hasReverse = true;
                break;
            }
        }

        const float offsetAmount = hasReverse ? (totalWidth * 0.5f + 1.0f) : 0.0f;
        const sf::Vector2f offsetA = a + norm * offsetAmount;
        const sf::Vector2f offsetB = b + norm * offsetAmount;

        sf::Color roadColor;
        if (road->isBridge()) {
            roadColor = sf::Color(100, 149, 237); // CornflowerBlue
            // Draw bridge barriers
            drawRoadStrip(target, offsetA, offsetB, sf::Color(50, 50, 50), totalWidth + 4.0f);
        } else if (road->isTunnel()) {
            roadColor = sf::Color(40, 40, 40); // Dark gray
        } else {
            drawRoadStrip(target, offsetA, offsetB, sf::Color(10, 10, 10, 220), totalWidth + 3.0f);
            roadColor = heatMapEnabled_
                ? colorForRoad(road)
                : (road->isBlocked() ? sf::Color(180, 40, 40) : sf::Color(110, 110, 110));
        }

        drawRoadStrip(target, offsetA, offsetB, roadColor, totalWidth - 1.0f);

        if (road->isTunnel()) {
            // draw dashed borders for tunnel
            const float dashLen = 8.0f;
            const float gapLen = 8.0f;
            const int dashCount = static_cast<int>(length / (dashLen + gapLen));
            const sf::Vector2f dirNorm = dir / length;
            const float halfW = totalWidth * 0.5f;
            for (int j = 0; j < dashCount; ++j) {
                const sf::Vector2f dashStart = offsetA + dirNorm * (j * (dashLen + gapLen));
                const sf::Vector2f dashEnd = dashStart + dirNorm * dashLen;
                drawRoadStrip(target, dashStart + norm * halfW, dashEnd + norm * halfW, sf::Color::Yellow, 1.0f);
                drawRoadStrip(target, dashStart - norm * halfW, dashEnd - norm * halfW, sf::Color::Yellow, 1.0f);
            }
        }

        if (laneCount > 1 && !road->isTunnel()) {
            for (int i = 1; i < laneCount; ++i) {
                const float sepOffset = -totalWidth * 0.5f + i * laneWidth;
                const sf::Vector2f sepA = offsetA + norm * sepOffset;
                const sf::Vector2f sepB = offsetB + norm * sepOffset;
                const float dashLen = 8.0f;
                const float gapLen = 8.0f;
                const int dashCount = static_cast<int>(length / (dashLen + gapLen));
                const sf::Vector2f dirNorm = dir / length;
                for (int j = 0; j < dashCount; ++j) {
                    const sf::Vector2f dashStart = sepA + dirNorm * (j * (dashLen + gapLen));
                    const sf::Vector2f dashEnd = dashStart + dirNorm * dashLen;
                    drawRoadStrip(target, dashStart, dashEnd, sf::Color(220, 220, 220, 200), 1.0f);
                }
            }
        }
    }

    const float borderLeft = std::min_element(routePoints_.begin(), routePoints_.end(), [](const sf::Vector2f& lhs, const sf::Vector2f& rhs) {
        return lhs.x < rhs.x;
    })->x - 20.0f;
    const float borderTop = std::min_element(routePoints_.begin(), routePoints_.end(), [](const sf::Vector2f& lhs, const sf::Vector2f& rhs) {
        return lhs.y < rhs.y;
    })->y - 20.0f;
    const float borderRight = std::max_element(routePoints_.begin(), routePoints_.end(), [](const sf::Vector2f& lhs, const sf::Vector2f& rhs) {
        return lhs.x < rhs.x;
    })->x + 20.0f;
    const float borderBottom = std::max_element(routePoints_.begin(), routePoints_.end(), [](const sf::Vector2f& lhs, const sf::Vector2f& rhs) {
        return lhs.y < rhs.y;
    })->y + 20.0f;

    sf::RectangleShape border({borderRight - borderLeft, borderBottom - borderTop});
    border.setPosition(borderLeft, borderTop);
    border.setFillColor(sf::Color::Transparent);
    border.setOutlineThickness(2.0f);
    border.setOutlineColor(sf::Color(235, 235, 235, 120));
    target.draw(border);

    for (auto* intersection : intersections) {
        drawIntersectionNode(target, intersection);
    }

    drawPOIs(target, graph);
    drawRoadNames(target, roads);
    drawTrafficLights(target, graph);
}

void VisualizationEngine::setFont(const sf::Font& font) {
    font_ = &font;
}

void VisualizationEngine::clearFont() {
    font_ = nullptr;
}

void VisualizationEngine::setSpriteTexture(const sf::Texture& texture,
                                           const sf::IntRect& rect,
                                           const sf::Vector2f& size) {
    spriteTexture_ = &texture;
    spriteRect_ = rect;
    spriteSize_ = size;
}

void VisualizationEngine::clearSpriteTexture() {
    spriteTexture_ = nullptr;
    spriteRect_ = sf::IntRect();
    spriteSize_ = {24.0f, 24.0f};
}

void VisualizationEngine::setHeatMapEnabled(bool enabled) {
    heatMapEnabled_ = enabled;
}

bool VisualizationEngine::isHeatMapEnabled() const {
    return heatMapEnabled_;
}

sf::Color VisualizationEngine::mixColor(const sf::Color& a, const sf::Color& b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    auto mix = [t](sf::Uint8 x, sf::Uint8 y) -> sf::Uint8 {
        return static_cast<sf::Uint8>(x + (y - x) * t);
    };

    return sf::Color(mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b), mix(a.a, b.a));
}

void VisualizationEngine::drawPOIs(sf::RenderTarget& target, const Graph& graph) const {
    const auto& pois = graph.getAllPOIs();
    for (const auto* poi : pois) {
        if (!poi) continue;
        sf::Vector2f pos = worldToScreen(poi->getX(), poi->getY());

        sf::Color poiColor(255, 150, 0); // Default orange
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
        // Keep text upright
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

float VisualizationEngine::distanceBetween(const sf::Vector2f& a, const sf::Vector2f& b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

void VisualizationEngine::drawRoadStrip(sf::RenderTarget& target,
                                        const sf::Vector2f& a,
                                        const sf::Vector2f& b,
                                        const sf::Color& color,
                                        float thickness) const {
    const float len = distanceBetween(a, b);
    if (len <= 0.01f) {
        return;
    }

    sf::RectangleShape strip({len, thickness});
    strip.setOrigin(0.0f, thickness * 0.5f);
    strip.setPosition(a);
    strip.setRotation(std::atan2(b.y - a.y, b.x - a.x) * 180.0f / 3.14159265f);
    strip.setFillColor(color);
    target.draw(strip);
}

void VisualizationEngine::drawTrafficLights(sf::RenderTarget& target, const Graph& graph) const {
    (void)graph;
    for (auto* intersection : graph.getAllIntersections()) {
        for (const auto* road : intersection->getIncomingRoads()) {
            auto* light = intersection->getLightForIncomingRoad(road);
            if (light == nullptr) {
                continue;
            }
            const sf::Vector2f point = getRoadEntryPoint(const_cast<Road*>(road), intersection);
            sf::RectangleShape pole({3.0f, 14.0f});
            pole.setFillColor(sf::Color(40, 40, 40));
            pole.setPosition(point.x - 1.5f, point.y - 14.0f);
            target.draw(pole);

            sf::CircleShape head(5.0f);
            head.setOrigin(5.0f, 5.0f);
            head.setPosition(point);
            head.setFillColor(lightColor(light->getState()));
            head.setOutlineThickness(1.0f);
            head.setOutlineColor(sf::Color::Black);
            target.draw(head);
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
        hub.setFillColor(sf::Color(100, 150, 100)); // green center island
        hub.setOutlineThickness(8.0f);
        hub.setOutlineColor(sf::Color(110, 110, 110)); // road strip around
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

sf::Vector2f VisualizationEngine::roadNormal(const sf::Vector2f& a, const sf::Vector2f& b) const {
    sf::Vector2f dir = b - a;
    const float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (length <= 0.01f) {
        return {0.0f, 0.0f};
    }
    return {-dir.y / length, dir.x / length};
}
