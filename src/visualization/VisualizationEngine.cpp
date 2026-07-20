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

void VisualizationEngine::setWindowSize(sf::Vector2u windowSize) {
    if (windowSize.x == 0u || windowSize.y == 0u) {
        return;
    }
    windowSize_ = windowSize;
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
            roadColor = sf::Color(100, 149, 237);
            drawRoadStrip(target, offsetA, offsetB, sf::Color(50, 50, 50), totalWidth + 4.0f);
        } else if (road->isTunnel()) {
            roadColor = sf::Color(40, 40, 40);
        } else {
            drawRoadStrip(target, offsetA, offsetB, sf::Color(10, 10, 10, 220), totalWidth + 3.0f);
            roadColor = heatMapEnabled_
                ? colorForRoad(road)
                : (road->isBlocked() ? sf::Color(180, 40, 40) : sf::Color(110, 110, 110));
        }

        drawRoadStrip(target, offsetA, offsetB, roadColor, totalWidth - 1.0f);

        // Draw lane divider lines for multi-lane roads
        if (laneCount > 1) {
            dir /= length; // normalize direction
            for (int i = 1; i < laneCount; ++i) {
                // Offset from road center to lane boundary
                const float laneBoundaryOffset = -totalWidth * 0.5f + static_cast<float>(i) * laneWidth;
                const sf::Vector2f laneLineA = offsetA + norm * laneBoundaryOffset;
                const sf::Vector2f laneLineB = offsetB + norm * laneBoundaryOffset;

                // Draw dashed line
                const float dashLength = 6.0f;
                const float gapLength = 4.0f;
                const float segmentLength = dashLength + gapLength;
                float traveled = 0.0f;
                while (traveled < length) {
                    const float dashEnd = std::min(traveled + dashLength, length);
                    const sf::Vector2f dashA = laneLineA + dir * traveled;
                    const sf::Vector2f dashB = laneLineA + dir * dashEnd;
                    drawRoadStrip(target, dashA, dashB, sf::Color(255, 255, 255, 100), 0.8f);
                    traveled += segmentLength;
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