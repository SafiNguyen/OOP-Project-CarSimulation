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

    constexpr unsigned int kBorderMaskCellSize = 2; // 2x2 px cells
    const sf::Vector2u targetSize = target.getSize();
    const unsigned int gridW = (targetSize.x + kBorderMaskCellSize - 1u) / kBorderMaskCellSize;
    const unsigned int gridH = (targetSize.y + kBorderMaskCellSize - 1u) / kBorderMaskCellSize;
    std::vector<uint8_t> bodyMask(static_cast<std::size_t>(gridW) * static_cast<std::size_t>(gridH), 0u);

    // Pre-compute per-road geometry so the two passes don't recompute it.
    struct RoadDraw {
        sf::Vector2f offsetA;
        sf::Vector2f offsetB;
        sf::Vector2f norm;
        sf::Vector2f dirUnit;
        float length;
        float totalWidth;
        int laneCount;
        bool isBridge;
        bool isTunnel;
        sf::Color bodyColor;
        bool hasBorder;
        sf::Color borderColor;
        float borderWidth;
    };
    std::vector<RoadDraw> drawList;
    drawList.reserve(roads.size());

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
        const sf::Vector2f offsetA = getRoadEntryPoint(road, start);
        const sf::Vector2f offsetB = getRoadEntryPoint(road, end);
        RoadDraw rd;
        rd.offsetA = offsetA;
        rd.offsetB = offsetB;
        rd.norm = norm;
        rd.length = length;
        rd.totalWidth = totalWidth;
        rd.laneCount = laneCount;
        rd.isBridge = road->isBridge();
        rd.isTunnel = road->isTunnel();
        rd.dirUnit = {dir.x / length, dir.y / length};

        if (rd.isBridge) {
            rd.bodyColor = sf::Color(100, 149, 237);
            rd.hasBorder = true;
            rd.borderColor = sf::Color(50, 50, 50);
            rd.borderWidth = totalWidth + 4.0f;
        } else if (rd.isTunnel) {
            rd.bodyColor = sf::Color(40, 40, 40);
            rd.hasBorder = false;
            rd.borderColor = sf::Color::Transparent;
            rd.borderWidth = 0.0f;
        } else {
            if (heatMapEnabled_) {
                rd.bodyColor = colorForRoad(road);
            } else {
                rd.bodyColor = sf::Color(110, 110, 110);
            }
            rd.hasBorder = true;
            rd.borderColor = sf::Color(10, 10, 10, 220);
            rd.borderWidth = totalWidth + 3.0f;
        }

        drawList.push_back(rd);
    }

    // Pass 1: bodies + lane markers + mask population.
    int roadIndex = 0;
    for (const RoadDraw& rd : drawList) {
        drawRoadStrip(target, rd.offsetA, rd.offsetB, rd.bodyColor, rd.totalWidth - 1.0f);
        rasterizeBodyToMask(rd.offsetA, rd.offsetB, rd.totalWidth - 1.0f,
                            bodyMask, gridW, gridH, kBorderMaskCellSize);

        // Draw individual blocked lanes
        Road* roadObj = roads[roadIndex++];
        for (int i = 0; i < rd.laneCount; ++i) {
            if (roadObj->getLane(i).isBlocked() && !roadObj->isBlocked() && heatMapEnabled_) {
                const float laneBoundaryOffset = -rd.totalWidth * 0.5f + static_cast<float>(i) * 10.0f + 5.0f;
                const sf::Vector2f laneCenterA = rd.offsetA + rd.norm * laneBoundaryOffset;
                const sf::Vector2f laneCenterB = rd.offsetB + rd.norm * laneBoundaryOffset;
                drawRoadStrip(target, laneCenterA, laneCenterB, sf::Color(180, 40, 40), 9.0f);
            }
        }

        // Lane divider lines for multi-lane roads.
        if (rd.laneCount > 1) {
            for (int i = 1; i < rd.laneCount; ++i) {
                const float laneBoundaryOffset = -rd.totalWidth * 0.5f + static_cast<float>(i) * 10.0f;
                const sf::Vector2f laneLineA = rd.offsetA + rd.norm * laneBoundaryOffset;
                const sf::Vector2f laneLineB = rd.offsetB + rd.norm * laneBoundaryOffset;

                const float dashLength = 6.0f;
                const float gapLength = 4.0f;
                const float segmentLength = dashLength + gapLength;
                float traveled = 0.0f;
                while (traveled < rd.length) {
                    const float dashEnd = std::min(traveled + dashLength, rd.length);
                    const sf::Vector2f dashA = laneLineA + rd.dirUnit * traveled;
                    const sf::Vector2f dashB = laneLineA + rd.dirUnit * dashEnd;
                    drawRoadStrip(target, dashA, dashB, sf::Color(255, 255, 255, 100), 0.8f);
                    traveled += segmentLength;
                }
            }
        }
    }

    // Pass 2: borders, skipping any chunk that lands on another road's body.
    for (const RoadDraw& rd : drawList) {
        if (!rd.hasBorder || rd.borderWidth <= 0.0f) {
            continue;
        }
        drawRoadBorderMasked(target, rd.offsetA, rd.offsetB, rd.borderColor, rd.borderWidth,
                             bodyMask, gridW, gridH, kBorderMaskCellSize);
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