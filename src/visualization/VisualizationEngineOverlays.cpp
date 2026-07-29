#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>

#include "Graph.h"
#include "BusStop.h"
#include "Intersection.h"
#include "PointOfInterest.h"
#include "Road.h"
#include "RoadGeometry.h"

namespace {

constexpr unsigned int kMinimumRoadLabelSize = 9u;
constexpr unsigned int kMaximumRoadLabelSize = 13u;

void drawSevenSegmentNumber(
    sf::RenderTarget& target,
    int value,
    const sf::Vector2f& center,
    float boxSize,
    const sf::Color& color) {
    static constexpr unsigned char digitMasks[10] = {
        0x3F, // 0: A B C D E F
        0x06, // 1: B C
        0x5B, // 2: A B D E G
        0x4F, // 3: A B C D G
        0x66, // 4: B C F G
        0x6D, // 5: A C D F G
        0x7D, // 6: A C D E F G
        0x07, // 7: A B C
        0x7F, // 8: all segments
        0x6F  // 9: A B C D F G
    };

    const std::string digits =
        std::to_string(std::clamp(value, 0, 999));
    const float innerWidth = std::max(1.0f, boxSize - 2.0f);
    const float innerHeight = std::max(1.0f, boxSize - 2.0f);
    const float digitGap = 1.0f;
    const float digitWidth = std::max(
        2.0f,
        std::floor(
            (innerWidth -
             digitGap * static_cast<float>(digits.size() - 1)) /
            static_cast<float>(digits.size())));
    const float digitHeight =
        std::max(5.0f, std::floor(innerHeight));
    const float thickness =
        std::max(1.0f, std::floor(digitWidth * 0.24f));
    const float renderedWidth =
        digitWidth * static_cast<float>(digits.size()) +
        digitGap * static_cast<float>(digits.size() - 1);
    const float left = std::round(center.x - renderedWidth * 0.5f);
    const float top = std::round(center.y - digitHeight * 0.5f);
    const float verticalHeight =
        std::max(1.0f, (digitHeight - thickness * 3.0f) * 0.5f);

    const auto drawSegment =
        [&target, &color](
            float x,
            float y,
            float width,
            float height) {
            sf::RectangleShape segment({
                std::max(1.0f, std::floor(width)),
                std::max(1.0f, std::floor(height))
            });
            segment.setPosition(std::round(x), std::round(y));
            segment.setFillColor(color);
            target.draw(segment);
        };

    for (std::size_t index = 0; index < digits.size(); ++index) {
        const int digit = digits[index] - '0';
        const unsigned char mask = digitMasks[digit];
        const float x =
            left + static_cast<float>(index) *
                (digitWidth + digitGap);
        const float middleY =
            top + thickness + verticalHeight;
        const float rightX =
            x + digitWidth - thickness;
        const float lowerY =
            middleY + thickness;

        if ((mask & 0x01) != 0) {
            drawSegment(
                x + thickness, top,
                digitWidth - thickness * 2.0f, thickness);
        }
        if ((mask & 0x02) != 0) {
            drawSegment(
                rightX, top + thickness,
                thickness, verticalHeight);
        }
        if ((mask & 0x04) != 0) {
            drawSegment(
                rightX, lowerY,
                thickness, verticalHeight);
        }
        if ((mask & 0x08) != 0) {
            drawSegment(
                x + thickness,
                top + digitHeight - thickness,
                digitWidth - thickness * 2.0f,
                thickness);
        }
        if ((mask & 0x10) != 0) {
            drawSegment(
                x, lowerY,
                thickness, verticalHeight);
        }
        if ((mask & 0x20) != 0) {
            drawSegment(
                x, top + thickness,
                thickness, verticalHeight);
        }
        if ((mask & 0x40) != 0) {
            drawSegment(
                x + thickness, middleY,
                digitWidth - thickness * 2.0f, thickness);
        }
    }
}

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
    for (const Intersection* intersection :
         graph.getAllIntersections()) {
        if (intersection == nullptr) continue;
        for (const Road* road :
             intersection->getIncomingRoads()) {
            const TrafficLight* light =
                intersection->getLightForIncomingRoad(road);
            if (road == nullptr || light == nullptr) continue;

            const Vec2 stopWorld =
                RoadGeometry::stopLineCentre(*road);
            const Vec2 directionWorld =
                RoadGeometry::roadDirection(*road);
            const Vec2 outwardWorld =
                rightNormal(directionWorld);
            const double metricScale =
                RoadGeometry::metresPerWorldUnit(*road);
            const double halfWidthWorld =
                RoadGeometry::carriagewayWidthMetres(*road) *
                0.5 / metricScale;

            const Vec2 stopLeftWorld =
                stopWorld - outwardWorld * halfWidthWorld;
            const Vec2 stopRightWorld =
                stopWorld + outwardWorld * halfWidthWorld;
            const sf::Vector2f stopLeft =
                worldToScreen(stopLeftWorld.x, stopLeftWorld.y);
            const sf::Vector2f stopRight =
                worldToScreen(stopRightWorld.x, stopRightWorld.y);
            drawRoadStrip(
                target,
                stopLeft,
                stopRight,
                sf::Color(255, 255, 255, 235),
                std::max(
                    1.5f,
                    metresToScreenPixels(0.25, road)));

            // Anchor the complete signal assembly beyond the outer curb.
            // Keeping this calculation in world/metre space makes the
            // placement stable at every zoom level and prevents the housing
            // or countdown box from covering a carriageway or sidewalk.
            const double sidewalkOuterOffsetMetres =
                RoadGeometry::SIDEWALK_WIDTH_METRES +
                RoadGeometry::SIDEWALK_CURB_WIDTH_METRES;
            const Vec2 signalAnchorWorld =
                stopRightWorld +
                outwardWorld *
                    (sidewalkOuterOffsetMetres /
                     metricScale);
            const Vec2 outsideProbeWorld =
                signalAnchorWorld +
                outwardWorld * (1.0 / metricScale);
            const sf::Vector2f signalAnchor =
                worldToScreen(
                    signalAnchorWorld.x,
                    signalAnchorWorld.y);
            sf::Vector2f outwardDirection =
                worldToScreen(
                    outsideProbeWorld.x,
                    outsideProbeWorld.y) -
                signalAnchor;
            const float outwardLength =
                std::sqrt(
                    outwardDirection.x * outwardDirection.x +
                    outwardDirection.y * outwardDirection.y);
            if (outwardLength > 0.01f) {
                outwardDirection /= outwardLength;
            } else {
                outwardDirection = {0.0f, -1.0f};
            }

            const float laneWidth =
                getLaneWidthPixels(road);
            const float housingThickness =
                std::clamp(laneWidth * 0.82f, 8.0f, 11.0f);
            const float lampRadius =
                std::clamp(
                    housingThickness * 0.27f,
                    2.0f,
                    2.9f);
            const float lampSpacing =
                lampRadius * 2.0f + 1.2f;
            const float housingLength =
                lampSpacing * 2.0f +
                lampRadius * 2.0f + 3.0f;
            const float countdownSize =
                std::clamp(
                    std::max(housingThickness, laneWidth * 1.05f),
                    12.0f,
                    15.0f);
            const float edgeGap = 2.0f;
            const float housingAngle =
                std::atan2(
                    outwardDirection.y,
                    outwardDirection.x) *
                180.0f / 3.14159265f;
            const sf::Vector2f housingCenter =
                signalAnchor +
                outwardDirection *
                    (edgeGap + housingLength * 0.5f);
            const sf::Vector2f countdownCenter =
                signalAnchor +
                outwardDirection *
                    (edgeGap + housingLength +
                     countdownSize * 0.5f - 0.5f);

            sf::RectangleShape housing(
                {housingLength, housingThickness});
            housing.setOrigin(
                housingLength * 0.5f,
                housingThickness * 0.5f);
            housing.setPosition(housingCenter);
            housing.setRotation(housingAngle);
            housing.setFillColor(sf::Color(22, 24, 27, 245));
            housing.setOutlineThickness(0.8f);
            housing.setOutlineColor(sf::Color(115, 120, 125));
            target.draw(housing);

            const LightState state = light->getState();
            const sf::Color offColor(48, 50, 52);
            const auto drawLamp =
                [&target, lampRadius](
                    const sf::Vector2f& center,
                    const sf::Color& color) {
                    sf::CircleShape lamp(lampRadius);
                    lamp.setOrigin(lampRadius, lampRadius);
                    lamp.setPosition(center);
                    lamp.setFillColor(color);
                    lamp.setOutlineThickness(0.8f);
                    lamp.setOutlineColor(sf::Color::Black);
                    target.draw(lamp);
                };
            drawLamp(
                housingCenter -
                    outwardDirection * lampSpacing,
                state == LightState::RED
                    ? lightColor(LightState::RED)
                    : offColor);
            drawLamp(
                housingCenter,
                state == LightState::YELLOW
                    ? lightColor(LightState::YELLOW)
                    : offColor);
            drawLamp(
                housingCenter +
                    outwardDirection * lampSpacing,
                state == LightState::GREEN
                    ? lightColor(LightState::GREEN)
                    : offColor);

            // The countdown is a projection of the same simulation clock.
            // Its compact square cell is attached to the outer end of the
            // approach-oriented signal, matching the corner placement.
            sf::RectangleShape countdownBox(
                {countdownSize, countdownSize});
            countdownBox.setOrigin(
                countdownSize * 0.5f,
                countdownSize * 0.5f);
            countdownBox.setPosition(countdownCenter);
            countdownBox.setFillColor(
                sf::Color(9, 11, 14, 255));
            countdownBox.setOutlineThickness(0.8f);
            countdownBox.setOutlineColor(
                sf::Color(225, 230, 235));
            target.draw(countdownBox);

            const int displayedSeconds =
                static_cast<int>(std::ceil(
                    std::max(
                        0.0,
                        light->getRemainingSeconds() -
                            1e-9)));
            drawSevenSegmentNumber(
                target,
                displayedSeconds,
                countdownCenter,
                countdownSize,
                lightColor(state));
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
    const Vec2 surfacePoint =
        RoadGeometry::roadSurfaceEndpoint(*road, atStart);
    return worldToScreen(surfacePoint.x, surfacePoint.y);
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
    return static_cast<float>(
        road->getLaneWidthMetres() /
        RoadGeometry::metresPerWorldUnit(*road) * scale_);
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
