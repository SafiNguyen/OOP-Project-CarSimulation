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

namespace {

// Append a rotated rectangle (a "strip") as a quad to a vertex array.
// Mirrors the geometry produced by drawRoadStrip() so batched rendering
// looks identical to the per-shape path, but issues a single draw call for
// the whole batch instead of one draw call per strip.
void appendStripQuad(std::vector<sf::Vertex>& vertices,
                     const sf::Vector2f& a,
                     const sf::Vector2f& b,
                     const sf::Color& color,
                     float thickness) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len <= 0.01f || thickness <= 0.01f) {
        return;
    }
    const float invLen = 1.0f / len;
    const float dirX = dx * invLen;
    const float dirY = dy * invLen;
    const float halfThick = thickness * 0.5f;
    const sf::Vector2f lateral(-dirY * halfThick, dirX * halfThick);
    const sf::Vector2f longitudinal(dirX * len, dirY * len);
    vertices.emplace_back(a, color);
    vertices.emplace_back(a + longitudinal, color);
    vertices.emplace_back(a + longitudinal + lateral, color);
    vertices.emplace_back(a + lateral, color);
}

} // namespace

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
    if (lodMode_ == LodMode::Auto) {
        lodLevel_ = LodLevel::Low;
    }
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

    // World-coordinate span alone is not a reliable measure of complexity:
    // map4 covers several hundred units but contains only a few dozen roads.
    // Use graph size to decide whether the imported-map scale and density
    // adjustments apply, so small maps retain all functional markers.
    constexpr std::size_t kDenseMapIntersectionThreshold = 500u;
    constexpr std::size_t kDenseMapRoadThreshold = 750u;
    const bool denseMap =
        intersections.size() >= kDenseMapIntersectionThreshold ||
        graph.getAllRoads().size() >= kDenseMapRoadThreshold;

    // Preserve the denser visual weight introduced for large imported maps.
    if (denseMap) {
        scale_ *= 10.0;
    }

    mapDetailFactor_ = 1.0f;
    if (denseMap && scale_ > 0.0 && scale_ < 2.0) {
        mapDetailFactor_ = std::clamp(
            static_cast<float>(scale_ / 2.0),
            0.25f,
            1.0f);
    }
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

std::vector<VisualizationEngine::RoadDraw>
VisualizationEngine::buildRoadDrawList(const Graph& graph) const {
    std::vector<RoadDraw> drawList;
    const auto roads = graph.getAllRoads();
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
            rd.bodyColor = sf::Color(110, 110, 110);
            rd.hasBorder = false;
            rd.borderColor = sf::Color::Transparent;
            rd.borderWidth = 0.0f;
        }

        drawList.push_back(rd);
    }
    return drawList;
}

void VisualizationEngine::drawLaneMarkings(
    sf::RenderTarget& target,
    const std::vector<RoadDraw>& drawList) const {
    // Zoom-aware LOD: when the view is zoomed out far enough that lane
    // dashes would be sub-pixel, skip the whole layer. This is the single
    // most expensive per-frame layer on large maps, and at a small zoom the
    // markings are visually meaningless anyway.
    const float detailScale = getDetailScale(target.getView());
    if (detailScale < 0.6f) {
        return;
    }

    // Batch every lane marking strip into a single vertex array so the
    // whole layer costs one draw call instead of one draw call per dash.
    // The viewport cull rejects roads entirely off-screen, which matters
    // for large maps (thousands of roads) where only a fraction is visible.
    const ViewportBounds viewportBounds(
        target.getView(),
        2.0f);
    std::vector<sf::Vertex> vertices;
    vertices.reserve(drawList.size() * 16u);

    // Lane dividers and carriageway edges.
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
                const float dashThickness =
                    std::max(
                        1.25f,
                        metresToScreenPixels(
                            0.12, roadObj));
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
                    appendStripQuad(
                        vertices,
                        dashA,
                        dashB,
                        sf::Color(245, 245, 245, 190),
                        dashThickness);
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
            const sf::Vector2f edgeA =
                worldToScreen(edgeStart.x, edgeStart.y);
            const sf::Vector2f edgeB =
                worldToScreen(edgeEnd.x, edgeEnd.y);
            if (!viewportBounds.intersectsSegment(
                    edgeA,
                    edgeB,
                    2.0f)) {
                continue;
            }
            appendStripQuad(
                vertices,
                edgeA,
                edgeB,
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
        const float centreThickness =
            std::max(
                1.25f,
                metresToScreenPixels(
                    0.12, rd.road));
        for (double side : {-1.0, 1.0}) {
            const Vec2 offset =
                halfSeparation * side;
            const sf::Vector2f centreA =
                worldToScreen(
                    centreStart.x + offset.x,
                    centreStart.y + offset.y);
            const sf::Vector2f centreB =
                worldToScreen(
                    centreEnd.x + offset.x,
                    centreEnd.y + offset.y);
            if (!viewportBounds.intersectsSegment(
                    centreA,
                    centreB,
                    centreThickness)) {
                continue;
            }
            appendStripQuad(
                vertices,
                centreA,
                centreB,
                sf::Color(245, 195, 45),
                centreThickness);
        }
    }

    if (!vertices.empty()) {
        target.draw(
            vertices.data(),
            vertices.size(),
            sf::Quads);
    }
}

void VisualizationEngine::drawStaticLayer(sf::RenderTarget& target, const Graph& graph) const {
    auto roads = graph.getAllRoads();
    auto intersections = graph.getAllIntersections();

    std::sort(intersections.begin(), intersections.end(), [](Intersection* lhs, Intersection* rhs) {
        return lhs->getId() < rhs->getId();
    });

    drawSidewalks(target, graph);
    drawPOIDriveways(target, graph);

    constexpr unsigned int kBorderMaskCellSize = 2; // 2x2 px cells
    const sf::Vector2u targetSize = target.getSize();
    const unsigned int gridW = (targetSize.x + kBorderMaskCellSize - 1u) / kBorderMaskCellSize;
    const unsigned int gridH = (targetSize.y + kBorderMaskCellSize - 1u) / kBorderMaskCellSize;
    std::vector<uint8_t> bodyMask(static_cast<std::size_t>(gridW) * static_cast<std::size_t>(gridH), 0u);

    const std::vector<RoadDraw> drawList = buildRoadDrawList(graph);

    // Viewport cull for the static body/mask passes. Without this, roads
    // far outside the current view (especially on large maps rendered at
    // the >=500-intersection 10x scale bump) still pay the full cost of
    // rasterizeBodyToMask's per-cell rectangle test over their entire
    // screen-space AABB, which can stall the frame badly enough that the
    // grey body layer effectively never finishes presenting.
    const ViewportBounds staticViewportBounds(target.getView(), 2.0f);

    // Pass 1: neutral road bodies.
    for (const RoadDraw& rd : drawList) {
        if (!staticViewportBounds.intersectsSegment(
                rd.offsetA, rd.offsetB, rd.totalWidth * 0.5f)) {
            continue;
        }
        drawRoadStrip(
            target,
            rd.offsetA,
            rd.offsetB,
            rd.bodyColor,
            rd.totalWidth);
        rasterizeBodyToMask(rd.offsetA, rd.offsetB, rd.totalWidth,
                            bodyMask, gridW, gridH, kBorderMaskCellSize);
    }

    // Pass 3: borders, skipping any chunk that lands on another road's body.
    for (const RoadDraw& rd : drawList) {
        if (!rd.hasBorder || rd.borderWidth <= 0.0f) {
            continue;
        }
        if (!staticViewportBounds.intersectsSegment(
                rd.offsetA, rd.offsetB, rd.borderWidth * 0.5f)) {
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
        drawIntersectionNode(target, intersection, /*tintByCongestion=*/false);
    }
}


void VisualizationEngine::drawDynamicLayer(sf::RenderTarget& target, const Graph& graph) const {
    const float detailScale = getDetailScale(target.getView());
    float fullThreshold = 0.35f;
    float mediumThreshold = 0.18f;
    // Keep Minimal visible through almost the entire Full-mode zoom range.
    float minimalThreshold = 0.008f;
    if (lodMode_ == LodMode::Medium) {
        fullThreshold = 0.70f;
        mediumThreshold = 0.28f;
        minimalThreshold = 0.08f;
    } else if (lodMode_ == LodMode::Low) {
        fullThreshold = 0.90f;
        mediumThreshold = 0.55f;
        minimalThreshold = 0.22f;
    }
    lodLevel_ = detailScale >= fullThreshold ? LodLevel::Full
              : detailScale >= mediumThreshold ? LodLevel::Medium
              : detailScale >= minimalThreshold ? LodLevel::Minimal
              : LodLevel::Low;
    const bool fullDetail = lodLevel_ == LodLevel::Full;
    const bool standardMarkers =
        fullDetail || lodLevel_ == LodLevel::Medium;

    // Medium keeps the map's functional landmarks visible, but the overlay
    // drawing routines omit expensive Full-only decoration such as traffic
    // light countdown text. Minimal uses compact markers, while Low is the
    // only tier that hides these overlays entirely.
    if (standardMarkers) {
        drawBusStops(target, graph);
        drawBusStations(target, graph);
        drawPOIs(target, graph);
    } else if (lodLevel_ == LodLevel::Minimal) {
        drawBusStops(target, graph);
        drawPOIs(target, graph);
    }

    // Layer 2: heat-map color block drawn on top of the gray road bodies.
    // Always drawn (at all LOD levels) so the heat map works like before.
    drawRoadCongestionOverlay(target, graph);
    drawBlockedLaneFills(target, graph);

    // Layer 3: intersection and roundabout heat-map tint. Drawn before the
    // lane markings so the road lane markings render on top of the
    // roundabout surface. Always drawn so the heat map works like before.
    auto intersections = graph.getAllIntersections();
    std::sort(intersections.begin(), intersections.end(), [](Intersection* lhs, Intersection* rhs) {
        return lhs->getId() < rhs->getId();
    });
    if (heatMapEnabled_) {
        const ViewportBounds viewportBounds(
            target.getView(),
            2.0f);
        for (auto* intersection : intersections) {
            const sf::Vector2f intersectionPosition =
                worldToScreen(
                    intersection->getX(),
                    intersection->getY());
            const float intersectionHalfExtent =
                getIntersectionBoxHalfExtent(intersection);
            if (!viewportBounds.containsPoint(
                    intersectionPosition,
                    intersectionHalfExtent)) {
                continue;
            }
            drawIntersectionNode(target, intersection, /*tintByCongestion=*/true);
        }
    }

    // Layer 4: lane markings drawn on top of the heat-map overlay and the
    // intersection/roundabout tint so the white lane dividers, carriageway
    // edges, and yellow centreline remain visible. Always drawn (with
    // zoom-based hiding inside drawLaneMarkings) so the render order is:
    //   1. gray road bodies (static layer)
    //   2. heat-map color block
    //   3. lane markings
    //   4. road names
    //   5. vehicles (drawn in renderFrame)
    drawLaneMarkings(target, buildRoadDrawList(graph));

    // Medium and Minimal both retain signals (without Full-only countdown
    // text); Low is the only tier that hides them.
    if (lodLevel_ != LodLevel::Low) {
        drawTrafficLights(target, graph);
    }

    // Road names are optional from the Overview panel.
    if (roadNamesVisible_) {
        auto roads = graph.getAllRoads();
        drawRoadNames(target, roads);
    }
}

void VisualizationEngine::drawRoadCongestionOverlay(sf::RenderTarget& target, const Graph& graph) const {
    if (!heatMapEnabled_) {
        return;
    }
    const ViewportBounds viewportBounds(
        target.getView(),
        2.0f);
    const auto roads = graph.getAllRoads();
    for (Road* road : roads) {
        // Bridges and tunnels both use the heat-map tint (a deep
        // green-to-red gradient) just like ordinary roads.
        if (road == nullptr) {
            continue;
        }
        auto* start = road->getStart();
        auto* end = road->getEnd();
        if (start == nullptr || end == nullptr) {
            continue;
        }
        const float laneWidth = getLaneWidthPixels(road);
        const float totalWidth =
            static_cast<float>(std::max(1, road->getLaneCount())) * laneWidth;
        const sf::Vector2f a = getRoadEntryPoint(road, start);
        const sf::Vector2f b = getRoadEntryPoint(road, end);
        if (!viewportBounds.intersectsSegment(
                a,
                b,
                totalWidth * 0.5f)) {
            continue;
        }
        drawRoadStrip(
            target,
            a,
            b,
            colorForRoad(road),
            totalWidth);
    }
}

void VisualizationEngine::drawBlockedLaneFills(sf::RenderTarget& target, const Graph& graph) const {
    if (!heatMapEnabled_) {
        return;
    }
    const ViewportBounds viewportBounds(
        target.getView(),
        2.0f);
    // Batch blocked-lane fills into a single vertex array.
    std::vector<sf::Vertex> vertices;
    const auto roads = graph.getAllRoads();
    vertices.reserve(roads.size() * 4u);
    for (Road* road : roads) {
        if (road == nullptr || road->isBlocked()) {
            continue;
        }
        const float laneWidth = getLaneWidthPixels(road);
        for (int i = 0; i < road->getLaneCount(); ++i) {
            if (!road->getLane(i).isBlocked()) {
                continue;
            }
            const Vec2 laneStart =
                RoadGeometry::laneEndpoint(*road, i, true);
            const Vec2 laneEnd =
                RoadGeometry::laneEndpoint(*road, i, false);
            const sf::Vector2f laneCenterA =
                worldToScreen(laneStart.x, laneStart.y);
            const sf::Vector2f laneCenterB =
                worldToScreen(laneEnd.x, laneEnd.y);
            const float laneFillWidth =
                std::max(1.0f, laneWidth - 1.0f);
            if (!viewportBounds.intersectsSegment(
                    laneCenterA,
                    laneCenterB,
                    laneFillWidth * 0.5f)) {
                continue;
            }
            appendStripQuad(
                vertices,
                laneCenterA,
                laneCenterB,
                sf::Color(180, 40, 40),
                laneFillWidth);
        }
    }
    if (!vertices.empty()) {
        target.draw(
            vertices.data(),
            vertices.size(),
            sf::Quads);
    }
}

void VisualizationEngine::drawGraph(sf::RenderTarget& target, const Graph& graph) const {
    drawStaticLayer(target, graph);
    drawDynamicLayer(target, graph);
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
}

bool VisualizationEngine::isHeatMapEnabled() const {
    return heatMapEnabled_;
}

void VisualizationEngine::setRoadNamesVisible(bool visible) {
    roadNamesVisible_ = visible;
}

bool VisualizationEngine::areRoadNamesVisible() const {
    return roadNamesVisible_;
}

void VisualizationEngine::setLodLevel(LodLevel level) {
    if (lodLevel_ == level) {
        return;
    }
    lodLevel_ = level;
}

VisualizationEngine::LodLevel VisualizationEngine::getLodLevel() const {
    return lodLevel_;
}

void VisualizationEngine::setLodMode(LodMode mode) {
    lodMode_ = mode;
}

VisualizationEngine::LodMode VisualizationEngine::getLodMode() const {
    return lodMode_;
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

    if (road->isTunnel()) {
        // Tunnels use a deep palette that tints the dark tunnel surface:
        // deep green (free flow) → dark yellow (moderate) → deep red
        // (heavy congestion), matching the tunnel's darker appearance.
        const sf::Color deepGreen(0, 90, 45);
        const sf::Color darkYellow(120, 100, 30);
        const sf::Color deepRed(150, 30, 30);
        if (normalized < 0.5f) {
            return mixColor(
                deepGreen, darkYellow, normalized * 2.0f);
        }
        return mixColor(
            darkYellow, deepRed, (normalized - 0.5f) * 2.0f);
    }

    if (road->isBridge()) {
        // Bridges use a deep palette that tints the blue bridge surface:
        // deep blue-green (free flow) → dark yellow (moderate) → deep red
        // (heavy congestion), matching the bridge's distinct appearance.
        const sf::Color deepBlueGreen(0, 110, 130);
        const sf::Color darkYellow(120, 100, 30);
        const sf::Color deepRed(150, 30, 30);
        if (normalized < 0.5f) {
            return mixColor(
                deepBlueGreen, darkYellow, normalized * 2.0f);
        }
        return mixColor(
            darkYellow, deepRed, (normalized - 0.5f) * 2.0f);
    }

    const sf::Color green(45, 190, 90);    // Free flow / 0 cars
    const sf::Color yellow(245, 190, 45);  // Moderate traffic
    const sf::Color red(220, 45, 45);      // Heavy congestion / Too many cars

    if (normalized < 0.5f) {
        return mixColor(green, yellow, normalized * 2.0f);
    }
    return mixColor(yellow, red, (normalized - 0.5f) * 2.0f);

}
