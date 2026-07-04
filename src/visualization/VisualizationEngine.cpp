#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"

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
      spriteSize_(24.0f, 24.0f) {
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

        drawRoadStrip(target, a, b, sf::Color(10, 10, 10, 220), road->isBlocked() ? 15.0f : 12.0f);
        drawRoadStrip(target, a, b, colorForRoad(road), road->isBlocked() ? 11.0f : 8.0f);
    }

    for (std::size_t i = 1; i < routePoints_.size(); ++i) {
        drawRoadStrip(target, routePoints_[i - 1], routePoints_[i], sf::Color(80, 220, 255, 180), 4.0f);
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
        const sf::Vector2f point = worldToScreen(intersection->getX(), intersection->getY());
        sf::CircleShape circle(5.5f);
        circle.setOrigin(5.5f, 5.5f);
        circle.setPosition(point);
        circle.setFillColor(sf::Color(235, 235, 235));
        circle.setOutlineThickness(1.5f);
        circle.setOutlineColor(sf::Color(20, 20, 20));
        target.draw(circle);

        if (spriteTexture_ != nullptr) {
            sf::Sprite sprite(*spriteTexture_, spriteRect_);
            sprite.setPosition(point.x - spriteSize_.x * 0.5f, point.y - spriteSize_.y * 0.5f);
            sprite.setScale(spriteSize_.x / std::max(1.0f, static_cast<float>(spriteRect_.width ? spriteRect_.width : spriteTexture_->getSize().x)),
                            spriteSize_.y / std::max(1.0f, static_cast<float>(spriteRect_.height ? spriteRect_.height : spriteTexture_->getSize().y)));
            target.draw(sprite);
        }
    }
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

sf::Color VisualizationEngine::mixColor(const sf::Color& a, const sf::Color& b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    auto mix = [t](sf::Uint8 x, sf::Uint8 y) -> sf::Uint8 {
        return static_cast<sf::Uint8>(x + (y - x) * t);
    };

    return sf::Color(mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b), mix(a.a, b.a));
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
