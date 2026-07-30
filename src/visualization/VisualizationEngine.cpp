#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "Graph.h"
#include "Intersection.h"
#include "Road.h"
#include "RoadGeometry.h"
#include "PointOfInterest.h"
#include "SpawnPoint.h"

VisualizationEngine::VisualizationEngine(sf::Vector2u windowSize, float margin)
    : windowSize_(windowSize),
      margin_(margin),
      minX_(0.0),
      minY_(0.0),
      maxX_(100.0),
      maxY_(100.0),
      scale_(1.0),
      offsetX_(margin),
      offsetY_(margin),
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
    ++revision_;
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

        const auto includePoint = [this](Vec2 point) {
            minX_ = std::min(minX_, point.x);
            maxX_ = std::max(maxX_, point.x);
            minY_ = std::min(minY_, point.y);
            maxY_ = std::max(maxY_, point.y);
        };
        for (const Road* road : graph.getAllRoads()) {
            if (road == nullptr) continue;
            for (int lane = 0; lane < road->getLaneCount(); ++lane) {
                includePoint(
                    RoadGeometry::laneEndpoint(*road, lane, true));
                includePoint(
                    RoadGeometry::laneEndpoint(*road, lane, false));
            }
            for (bool rightSide : {false, true}) {
                includePoint(
                    RoadGeometry::sampleSidewalk(
                        *road, rightSide, 0.0));
                includePoint(
                    RoadGeometry::sampleSidewalk(
                        *road,
                        rightSide,
                        road->getDistance()));
            }
        }
        for (const Intersection* intersection : intersections) {
            const double radius =
                RoadGeometry::junctionBoundaryRadiusWorld(*intersection);
            includePoint(
                {intersection->getX() - radius,
                 intersection->getY() - radius});
            includePoint(
                {intersection->getX() + radius,
                 intersection->getY() + radius});
        }
        for (const BusStation* station :
             graph.getAllBusStations()) {
            if (station != nullptr) {
                includePoint(
                    {station->getX(), station->getY()});
            }
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
    const double availableWidth =
        std::max(
            0.0,
            static_cast<double>(windowSize_.x) -
                2.0 * margin_);
    const double availableHeight =
        std::max(
            0.0,
            static_cast<double>(windowSize_.y) -
                2.0 * margin_);
    offsetX_ =
        margin_ +
        std::max(
            0.0,
            (availableWidth - rangeX * scale_) * 0.5);
    offsetY_ =
        margin_ +
        std::max(
            0.0,
            (availableHeight - rangeY * scale_) * 0.5);

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

    drawSidewalks(target, graph);
    // Driveways sit above the sidewalk but below the carriageway. Drawing
    // them here lets the road surface cleanly mask their curb connection.
    drawPOIDriveways(target, graph);

    constexpr unsigned int kBorderMaskCellSize = 2; // 2x2 px cells
    const sf::Vector2u targetSize = target.getSize();
    const unsigned int gridW = (targetSize.x + kBorderMaskCellSize - 1u) / kBorderMaskCellSize;
    const unsigned int gridH = (targetSize.y + kBorderMaskCellSize - 1u) / kBorderMaskCellSize;
    std::vector<uint8_t> bodyMask(static_cast<std::size_t>(gridW) * static_cast<std::size_t>(gridH), 0u);

    // Pre-compute per-road geometry so the two passes don't recompute it.
    struct RoadDraw {
        Road* road;
        sf::Vector2f offsetA;
        sf::Vector2f offsetB;
        float laneWidth;
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

        const float laneWidth = getLaneWidthPixels(road);
        const int laneCount = road->getLaneCount();
        const float totalWidth = static_cast<float>(laneCount) * laneWidth;
        const sf::Vector2f offsetA =
            getRoadEntryPoint(road, start);
        const sf::Vector2f offsetB =
            getRoadEntryPoint(road, end);
        const sf::Vector2f dir = offsetB - offsetA;
        const float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (length <= 0.01f) {
            continue;
        }
        RoadDraw rd;
        rd.road = road;
        rd.offsetA = offsetA;
        rd.offsetB = offsetB;
        rd.laneWidth = laneWidth;
        rd.totalWidth = totalWidth;
        rd.laneCount = laneCount;
        rd.isBridge = road->isBridge();
        rd.isTunnel = road->isTunnel();
        // brigde sprite
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
            rd.hasBorder = false;
            rd.borderColor = sf::Color::Transparent;
            rd.borderWidth = 0.0f;
        }

        drawList.push_back(rd);
    }

    // Pass 1: road bodies and blocked-lane fills. Markings are deliberately
    // deferred until every body exists so a later road cannot paint over an
    // earlier road's lane dividers.
    for (const RoadDraw& rd : drawList) {
        drawRoadStrip(
            target,
            rd.offsetA,
            rd.offsetB,
            rd.bodyColor,
            rd.totalWidth);
        rasterizeBodyToMask(rd.offsetA, rd.offsetB, rd.totalWidth,
                            bodyMask, gridW, gridH, kBorderMaskCellSize);

        // Draw individual blocked lanes
        Road* roadObj = rd.road;
        for (int i = 0; i < rd.laneCount; ++i) {
            if (roadObj->getLane(i).isBlocked() && !roadObj->isBlocked() && heatMapEnabled_) {
                const Vec2 laneStart =
                    RoadGeometry::laneEndpoint(*roadObj, i, true);
                const Vec2 laneEnd =
                    RoadGeometry::laneEndpoint(*roadObj, i, false);
                const sf::Vector2f laneCenterA =
                    worldToScreen(laneStart.x, laneStart.y);
                const sf::Vector2f laneCenterB =
                    worldToScreen(laneEnd.x, laneEnd.y);
                drawRoadStrip(
                    target,
                    laneCenterA,
                    laneCenterB,
                    sf::Color(180, 40, 40),
                    std::max(1.0f, rd.laneWidth - 1.0f));
            }
        }
    }

    // Pass 2: lane dividers and carriageway edges.
    for (const RoadDraw& rd : drawList) {
        Road* roadObj = rd.road;
        // Lane divider lines for multi-lane roads.
        if (rd.laneCount > 1) {
            for (int i = 1; i < rd.laneCount; ++i) {
                const Vec2 boundaryStart =
                    RoadGeometry::laneBoundaryEndpoint(
                        *roadObj, i, true);
                Vec2 boundaryEnd =
                    RoadGeometry::laneBoundaryEndpoint(
                        *roadObj, i, false);
                if (roadObj->getEnd() != nullptr &&
                    roadObj->getEnd()->getLightForIncomingRoad(
                        roadObj) != nullptr) {
                    boundaryEnd = lerp(
                        boundaryStart,
                        boundaryEnd,
                        RoadGeometry::stopLineProgressMetres(
                            *roadObj) /
                            roadObj->getDistance());
                }
                const sf::Vector2f laneLineA =
                    worldToScreen(boundaryStart.x, boundaryStart.y);
                const sf::Vector2f laneLineB =
                    worldToScreen(boundaryEnd.x, boundaryEnd.y);
                const sf::Vector2f laneDirection =
                    laneLineB - laneLineA;
                const float laneLineLength =
                    distanceBetween(laneLineA, laneLineB);
                if (laneLineLength <= 0.01f) continue;
                const sf::Vector2f laneDirectionUnit =
                    laneDirection / laneLineLength;

                const float dashLength =
                    std::max(
                        4.0f,
                        metresToScreenPixels(4.0, roadObj));
                const float gapLength =
                    std::max(
                        3.0f,
                        metresToScreenPixels(3.0, roadObj));
                const float segmentLength = dashLength + gapLength;
                float traveled = 0.0f;
                while (traveled < laneLineLength) {
                    const float dashEnd =
                        std::min(
                            traveled + dashLength,
                            laneLineLength);
                    const sf::Vector2f dashA =
                        laneLineA +
                        laneDirectionUnit * traveled;
                    const sf::Vector2f dashB =
                        laneLineA +
                        laneDirectionUnit * dashEnd;
                    drawRoadStrip(
                        target,
                        dashA,
                        dashB,
                        sf::Color(245, 245, 245, 190),
                        std::max(
                            1.25f,
                            metresToScreenPixels(
                                0.12, roadObj)));
                    traveled += segmentLength;
                }
            }
        }

        // Solid white carriageway edges use the same model-space boundary
        // endpoints as lane centres and road surfaces.
        for (bool rightEdge : {false, true}) {
            const Vec2 edgeStart =
                RoadGeometry::roadEdgeEndpoint(
                    *roadObj, rightEdge, true);
            const Vec2 edgeEnd =
                RoadGeometry::roadEdgeEndpoint(
                    *roadObj, rightEdge, false);
            drawRoadStrip(
                target,
                worldToScreen(edgeStart.x, edgeStart.y),
                worldToScreen(edgeEnd.x, edgeEnd.y),
                sf::Color(245, 245, 245, 205),
                std::max(
                    1.5f,
                    metresToScreenPixels(0.12, roadObj)));
        }
    }

    // One continuous yellow centreline per topology-paired physical road.
    // The pair is selected by identity only for de-duplicating the draw;
    // pairing itself is cached by Graph from reversed endpoints.
    std::unordered_set<const Road*> centrelineDrawn;
    for (const RoadDraw& rd : drawList) {
        const Road* reverse = rd.road->getReverseRoad();
        if (reverse == nullptr ||
            centrelineDrawn.count(rd.road) != 0 ||
            centrelineDrawn.count(reverse) != 0) {
            continue;
        }
        centrelineDrawn.insert(rd.road);
        centrelineDrawn.insert(reverse);

        const Vec2 originalCentreStart =
            RoadGeometry::roadReferenceEndpoint(
                *rd.road, true);
        const Vec2 originalCentreEnd =
            RoadGeometry::roadReferenceEndpoint(
                *rd.road, false);
        Vec2 centreStart = originalCentreStart;
        Vec2 centreEnd = originalCentreEnd;
        if (reverse->getEnd() != nullptr &&
            reverse->getEnd()->getLightForIncomingRoad(
                reverse) != nullptr) {
            const double reverseRatio =
                RoadGeometry::stopLineProgressMetres(
                    *reverse) /
                reverse->getDistance();
            centreStart =
                lerp(
                    originalCentreStart,
                    originalCentreEnd,
                    1.0 - reverseRatio);
        }
        if (rd.road->getEnd() != nullptr &&
            rd.road->getEnd()->getLightForIncomingRoad(
                rd.road) != nullptr) {
            const double forwardRatio =
                RoadGeometry::stopLineProgressMetres(
                    *rd.road) /
                rd.road->getDistance();
            centreEnd =
                lerp(
                    originalCentreStart,
                    originalCentreEnd,
                    forwardRatio);
        }
        const Vec2 centreDirection = normalized(
            centreEnd - centreStart,
            RoadGeometry::roadDirection(*rd.road));
        const Vec2 centreNormal =
            rightNormal(centreDirection);
        const double metricScale =
            RoadGeometry::metresPerWorldUnit(*rd.road);
        constexpr double centrelineSeparationMetres = 0.36;
        const Vec2 halfSeparation =
            centreNormal *
            (centrelineSeparationMetres * 0.5 /
             metricScale);
        for (double side : {-1.0, 1.0}) {
            const Vec2 offset =
                halfSeparation * side;
            drawRoadStrip(
                target,
                worldToScreen(
                    centreStart.x + offset.x,
                    centreStart.y + offset.y),
                worldToScreen(
                    centreEnd.x + offset.x,
                    centreEnd.y + offset.y),
                sf::Color(245, 195, 45),
                std::max(
                    1.25f,
                    metresToScreenPixels(
                        0.12, rd.road)));
        }
    }

    // Pass 3: borders, skipping any chunk that lands on another road's body.
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

    drawCrosswalks(target, graph);
    drawBusStops(target, graph);
    drawBusStations(target, graph);
    drawPOIs(target, graph);
    drawRoadNames(target, roads);
    drawTrafficLights(target, graph);
}

void VisualizationEngine::setFont(const sf::Font& font) {
    font_ = &font;
    ++revision_;
}

void VisualizationEngine::clearFont() {
    if (font_ == nullptr) {
        return;
    }
    font_ = nullptr;
    ++revision_;
}

void VisualizationEngine::setSpriteTexture(const sf::Texture& texture,
                                           const sf::IntRect& rect,
                                           const sf::Vector2f& size) {
    spriteTexture_ = &texture;
    spriteRect_ = rect;
    spriteSize_ = size;
    ++revision_;
}

void VisualizationEngine::clearSpriteTexture() {
    if (spriteTexture_ == nullptr) {
        return;
    }
    spriteTexture_ = nullptr;
    spriteRect_ = sf::IntRect();
    spriteSize_ = {24.0f, 24.0f};
    ++revision_;
}

void VisualizationEngine::setHeatMapEnabled(bool enabled) {
    if (heatMapEnabled_ == enabled) {
        return;
    }
    heatMapEnabled_ = enabled;
    ++revision_;
}

bool VisualizationEngine::isHeatMapEnabled() const {
    return heatMapEnabled_;
}

std::uint64_t VisualizationEngine::getRevision() const {
    return revision_;
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

    // Sum active vehicles currently driving/queued on all lanes of this road
    int vehicleCount = 0;
    for (const Lane& lane : road->getLanes()) {
        vehicleCount += lane.getVehicleCount();
    }

    // Physical capacity estimate based on road length & lane count (~6.5m per car slot)
    const double vehicleSpace = 6.5;
    const double maxCapacity = std::max(1.0, (road->getDistance() / vehicleSpace) * std::max(1, road->getLaneCount()));

    // Occupancy ratio (0.0 = no cars, 1.0+ = road at or over capacity)
    const double occupancy = static_cast<double>(vehicleCount) / maxCapacity;

    // Blend static base congestion factor with dynamic vehicle occupancy
    const double baseCongestionFactor = std::max(1.0, road->getCongestionLevel());
    const double effectiveLoad = (baseCongestionFactor > 1.0)
        ? std::max(occupancy, (baseCongestionFactor - 1.0) / 3.0)
        : occupancy;

    const float normalized = static_cast<float>(std::clamp(effectiveLoad, 0.0, 1.0));

    const sf::Color green(45, 190, 90);    // Free flow / 0 cars
    const sf::Color yellow(245, 190, 45);  // Moderate traffic
    const sf::Color red(220, 45, 45);      // Heavy congestion / Too many cars

    if (normalized < 0.5f) {
        return mixColor(green, yellow, normalized * 2.0f);
    }

    return mixColor(yellow, red, (normalized - 0.5f) * 2.0f);
}
