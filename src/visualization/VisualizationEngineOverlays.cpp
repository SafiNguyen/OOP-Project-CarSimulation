#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>

#include "model/Graph.h"
#include "model/BusStop.h"
#include "model/Intersection.h"
#include "model/PointOfInterest.h"
#include "model/Road.h"
#include "model/RoadGeometry.h"

namespace {

constexpr unsigned int kMinimumRoadLabelSize = 9u;
constexpr unsigned int kMaximumRoadLabelSize = 13u;

} // namespace

void VisualizationEngine::drawBusStops(sf::RenderTarget& target, const Graph& graph) const {
    for (const Road* road : graph.getAllRoads()) {
        if (road == nullptr || road->getStart() == nullptr || road->getEnd() == nullptr) {
            continue;
        }

        const sf::Vector2f roadStart = getRoadEntryPoint(road, road->getStart());
        const sf::Vector2f roadEnd = getRoadEntryPoint(road, road->getEnd());
        const sf::Vector2f normal = roadNormal(roadStart, roadEnd);
        const float laneWidth = getLaneWidthPixels(road);
        const float totalWidth =
            laneWidth *
            static_cast<float>(std::max(1, road->getLaneCount()));

        for (const auto& ownedStop : road->getBusStops()) {
            const BusStop* stop = ownedStop.get();
            if (stop == nullptr) {
                continue;
            }

            const float ratio = static_cast<float>(stop->getPositionRatio());
            const sf::Vector2f centerline = roadStart + (roadEnd - roadStart) * ratio;
            const int laneIndex = std::clamp(
                stop->getLaneIndex(), 0, road->getLaneCount() - 1);
            const float laneCenterOffset =
                -totalWidth * 0.5f +
                (static_cast<float>(laneIndex) + 0.5f) * laneWidth;
            const float edgeDirection =
                laneCenterOffset < 0.0f ? -1.0f : 1.0f;
            const sf::Vector2f laneCenter =
                centerline + normal * laneCenterOffset;
            const sf::Vector2f roadEdge =
                laneCenter + normal * (edgeDirection * laneWidth * 0.5f);
            const sf::Vector2f markerPos =
                roadEdge + normal * (edgeDirection * 9.0f);

            drawRoadStrip(
                target, roadEdge, markerPos,
                sf::Color(225, 235, 245), 2.5f);

            sf::RectangleShape pole({2.5f, 8.0f});
            pole.setOrigin(1.25f, 0.0f);
            pole.setPosition(markerPos.x, markerPos.y + 5.0f);
            pole.setFillColor(sf::Color(225, 235, 245));
            target.draw(pole);

            constexpr float signWidth = 14.0f;
            constexpr float signHeight = 12.0f;
            sf::RectangleShape sign({signWidth, signHeight});
            sign.setOrigin(signWidth * 0.5f, signHeight * 0.5f);
            sign.setPosition(markerPos);
            sign.setFillColor(sf::Color(35, 145, 230));
            sign.setOutlineThickness(2.0f);
            sign.setOutlineColor(sf::Color::White);
            target.draw(sign);

            const sf::Color symbolColor = sf::Color::White;
            sf::RectangleShape leftStroke({2.0f, 7.0f});
            leftStroke.setPosition(markerPos.x - 4.0f, markerPos.y - 3.5f);
            leftStroke.setFillColor(symbolColor);
            target.draw(leftStroke);

            sf::RectangleShape rightStroke({2.0f, 7.0f});
            rightStroke.setPosition(markerPos.x + 2.0f, markerPos.y - 3.5f);
            rightStroke.setFillColor(symbolColor);
            target.draw(rightStroke);

            sf::RectangleShape crossStroke({6.0f, 2.0f});
            crossStroke.setPosition(markerPos.x - 3.0f, markerPos.y - 1.0f);
            crossStroke.setFillColor(symbolColor);
            target.draw(crossStroke);

            if (font_) {
                sf::Text label;
                label.setFont(*font_);
                label.setString(stop->getName());
                label.setCharacterSize(9);
                label.setFillColor(sf::Color(120, 210, 255));
                label.setOutlineColor(sf::Color::Black);
                label.setOutlineThickness(1.0f);
                label.setPosition(markerPos.x + 10.0f, markerPos.y - 17.0f);
                target.draw(label);
            }
        }
    }
}

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
            const float laneWidth = getLaneWidthPixels(road);
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
        const float metricScale = static_cast<float>(
            RoadGeometry::metresPerWorldUnit(*intersection));
        const float laneWidth = static_cast<float>(
            RoadGeometry::LANE_WIDTH_METRES /
            std::max(1e-6f, metricScale) * scale_);
        const float circulationRadius = static_cast<float>(
            intersection->getTraversalRadiusMetres() /
            std::max(1e-6f, metricScale) * scale_);
        const float hubRadius =
            std::max(2.0f, circulationRadius - laneWidth * 0.5f);
        sf::CircleShape hub(hubRadius);
        hub.setOrigin(hubRadius, hubRadius);
        hub.setPosition(point);
        hub.setFillColor(sf::Color(100, 150, 100));
        hub.setOutlineThickness(laneWidth);
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
    if (road->getStart() == nullptr || road->getEnd() == nullptr) {
        return worldToScreen(intersection->getX(), intersection->getY());
    }
    const bool atStart = intersection == road->getStart();
    const Vec2 point =
        RoadGeometry::roadSurfaceEndpoint(*road, atStart);
    return worldToScreen(point.x, point.y);
}


sf::Vector2f VisualizationEngine::getRoadCenterlineEntryPoint(const Road* road, const Intersection* intersection) const {
    if (road == nullptr || intersection == nullptr) {
        return {};
    }
    if (road->getStart() == nullptr || road->getEnd() == nullptr) {
        return worldToScreen(intersection->getX(), intersection->getY());
    }
    const bool atStart = intersection == road->getStart();
    const Vec2 point =
        RoadGeometry::roadReferenceEndpoint(*road, atStart);
    return worldToScreen(point.x, point.y);
}

float VisualizationEngine::getIntersectionBoxHalfExtent(const Intersection* intersection) const {
    if (intersection == nullptr) {
        return 12.0f;
    }

    float halfExtent = std::max(
        0.0f,
        static_cast<float>(
            RoadGeometry::junctionBoundaryRadiusWorld(*intersection) *
            scale_));

    const auto includeRoadWidth = [this, &halfExtent](const std::vector<Road*>& roads) {
        for (const Road* road : roads) {
            if (road == nullptr) {
                continue;
            }

            const float carriagewayWidth =
                getLaneWidthPixels(road) *
                static_cast<float>(std::max(1, road->getLaneCount()));
            const float roadHalfWidth = RoadGeometry::hasReverseDirection(*road)
                ? carriagewayWidth
                : carriagewayWidth * 0.5f;
            halfExtent = std::max(halfExtent, roadHalfWidth + 1.0f);
        }
    };
    includeRoadWidth(intersection->getIncomingRoads());
    includeRoadWidth(intersection->getOutgoingRoads());
    return halfExtent;
}

float VisualizationEngine::getLaneWidthPixels(
    const Road* road) const {
    if (road == nullptr) return 1.0f;
    return std::max(
        MIN_LANE_WIDTH_PIXELS,
        static_cast<float>(
            RoadGeometry::LANE_WIDTH_METRES /
            RoadGeometry::metresPerWorldUnit(*road) * scale_));
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

        // 1. Draw driveway if POI is connected to a road
        if (poi->getConnectedRoad() != nullptr) {
            // Get position of the merging point on the road
            Vec2 roadPoint = RoadGeometry::sampleLane(*poi->getConnectedRoad(), 0, poi->getProgressOffset()).position;
            sf::Vector2f screenRoadPoint = worldToScreen(roadPoint.x, roadPoint.y);
            
            // Draw a line (thin rectangle) from building to the road
            sf::Vector2f dir = screenRoadPoint - pos;
            float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            if (length > 0) {
                float drivewayWidth = 4.0f * static_cast<float>(scale_); // 4 meters wide
                sf::RectangleShape driveway(sf::Vector2f(length, drivewayWidth));
                driveway.setOrigin(0.0f, drivewayWidth * 0.5f);
                driveway.setPosition(pos);
                driveway.setRotation(std::atan2(dir.y, dir.x) * 180.0f / 3.14159265f);
                driveway.setFillColor(sf::Color(100, 100, 100)); // Dark gray road
                target.draw(driveway);
                
                sf::RectangleShape centerLine(sf::Vector2f(length, 0.5f)); // Center dividing line
                centerLine.setOrigin(0.0f, 0.25f);
                centerLine.setPosition(pos);
                centerLine.setRotation(std::atan2(dir.y, dir.x) * 180.0f / 3.14159265f);
                centerLine.setFillColor(sf::Color(255, 200, 0, 150)); // Yellow line
                target.draw(centerLine);
            }
        }

        // 2. Draw building
        sf::RectangleShape building(sf::Vector2f(10.0f, 10.0f));
        building.setOrigin(5.0f, 5.0f);
        building.setPosition(pos);
        building.setFillColor(poiColor);
        building.setOutlineThickness(1.0f);
        building.setOutlineColor(sf::Color::White);
        target.draw(building);

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

    for (std::size_t roadIndex = 0; roadIndex < roads.size(); ++roadIndex) {
        const Road* road = roads[roadIndex];
        if (!road || road->getName().empty()) continue;
        const Intersection* start = road->getStart();
        const Intersection* end = road->getEnd();
        if (!start || !end) continue;

        // A two-way road is represented by two directional Road objects.
        // Draw one physical-road label instead of stacking both names.
        const int firstIntersectionId =
            std::min(start->getId(), end->getId());
        const int secondIntersectionId =
            std::max(start->getId(), end->getId());
        const bool labelAlreadyDrawn = std::any_of(
            roads.begin(),
            roads.begin() + static_cast<std::ptrdiff_t>(roadIndex),
            [&](const Road* candidate) {
                if (candidate == nullptr ||
                    candidate->getName() != road->getName() ||
                    candidate->getStart() == nullptr ||
                    candidate->getEnd() == nullptr) {
                    return false;
                }
                const int candidateFirstId = std::min(
                    candidate->getStart()->getId(),
                    candidate->getEnd()->getId());
                const int candidateSecondId = std::max(
                    candidate->getStart()->getId(),
                    candidate->getEnd()->getId());
                return candidateFirstId == firstIntersectionId &&
                       candidateSecondId == secondIntersectionId;
            });
        if (labelAlreadyDrawn) {
            continue;
        }

        // Labels belong on the shared physical centreline. Directional road
        // surfaces are offset to either side of this line.
        const sf::Vector2f a =
            getRoadCenterlineEntryPoint(road, start);
        const sf::Vector2f b =
            getRoadCenterlineEntryPoint(road, end);
        const sf::Vector2f mid = (a + b) * 0.5f;

        const float dx = b.x - a.x;
        const float dy = b.y - a.y;
        const float visibleLength = std::sqrt(dx * dx + dy * dy);

        float angle = std::atan2(dy, dx) * 180.0f / 3.14159265f;
        if (angle > 90.0f || angle < -90.0f) {
            angle += 180.0f;
        }

        sf::Text text;
        text.setFont(*font_);
        text.setString(road->getName());
        const float carriagewayWidth =
            getLaneWidthPixels(road) *
            static_cast<float>(std::max(1, road->getLaneCount()));
        const float physicalRoadWidth = carriagewayWidth *
            (RoadGeometry::hasReverseDirection(*road) ? 2.0f : 1.0f);
        unsigned int characterSize = static_cast<unsigned int>(std::lround(
            std::clamp(
                physicalRoadWidth * 0.36f,
                static_cast<float>(kMinimumRoadLabelSize),
                static_cast<float>(kMaximumRoadLabelSize))));
        text.setCharacterSize(characterSize);
        text.setStyle(sf::Text::Bold);
        text.setFillColor(sf::Color::White);
        text.setOutlineColor(sf::Color(12, 18, 22, 220));
        text.setOutlineThickness(1.0f);

        constexpr float kEdgeMargin = 6.0f;
        sf::FloatRect bounds = text.getLocalBounds();
        while (characterSize > kMinimumRoadLabelSize &&
               visibleLength < bounds.width + kEdgeMargin * 2.0f) {
            text.setCharacterSize(--characterSize);
            bounds = text.getLocalBounds();
        }
        if (visibleLength < bounds.width + kEdgeMargin * 2.0f) {
            continue;
        }

        text.setOrigin(bounds.left + bounds.width * 0.5f, bounds.top + bounds.height * 0.5f);
        text.setPosition(mid);
        text.setRotation(angle);


        constexpr float kPlatePaddingX = 3.0f;
        constexpr float kPlatePaddingY = 1.0f;
        sf::RectangleShape plate({bounds.width + kPlatePaddingX * 2.0f, bounds.height + kPlatePaddingY * 2.0f});
        plate.setOrigin(plate.getSize().x * 0.5f, plate.getSize().y * 0.5f);
        plate.setPosition(mid);
        plate.setRotation(angle);
        plate.setFillColor(sf::Color(12, 18, 22, 105));
        target.draw(plate);

        target.draw(text);
    }
}
