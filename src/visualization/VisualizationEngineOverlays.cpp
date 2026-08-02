#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "Graph.h"
#include "BusStop.h"
#include "Intersection.h"
#include "PointOfInterest.h"
#include "Road.h"
#include "RoadGeometry.h"
#include "SpawnPoint.h"

namespace {

constexpr unsigned int kMinimumRoadLabelSize = 9u;
constexpr unsigned int kMaximumRoadLabelSize = 13u;
constexpr float kRoundaboutCenterCropRadiusFraction = 0.245f;
constexpr int kRoundaboutCenterSegments = 64;

struct RoundaboutCenterAsset {
    sf::Texture texture;
    bool loaded = false;

    RoundaboutCenterAsset() {
        std::filesystem::path directory =
            std::filesystem::current_path();
        for (int depth = 0; depth < 6; ++depth) {
            const std::filesystem::path candidate =
                directory / "assets" / "roundabout.png";
            std::error_code error;
            if (std::filesystem::is_regular_file(
                    candidate, error) &&
                texture.loadFromFile(candidate.string())) {
                texture.setSmooth(true);
                texture.generateMipmap();
                loaded = true;
                return;
            }
            const std::filesystem::path parent =
                directory.parent_path();
            if (parent.empty() || parent == directory) {
                break;
            }
            directory = parent;
        }
        std::cerr
            << "Warning: assets/roundabout.png was not found; "
               "roundabout centres remain empty."
            << std::endl;
    }
};

const sf::Texture* roundaboutCenterTexture() {
    static RoundaboutCenterAsset asset;
    return asset.loaded ? &asset.texture : nullptr;
}

void drawRoundaboutCenterImage(
    sf::RenderTarget& target,
    const sf::Texture& texture,
    const sf::Vector2f& center,
    float radius) {
    if (radius <= 1.0f) {
        return;
    }

    const sf::Vector2u textureSize =
        texture.getSize();
    if (textureSize.x == 0u || textureSize.y == 0u) {
        return;
    }

    const sf::Vector2f textureCenter{
        static_cast<float>(textureSize.x) * 0.5f,
        static_cast<float>(textureSize.y) * 0.5f
    };
    const float textureRadius =
        static_cast<float>(
            std::min(textureSize.x, textureSize.y)) *
        kRoundaboutCenterCropRadiusFraction;
    std::vector<sf::Vertex> vertices;
    vertices.reserve(
        static_cast<std::size_t>(
            kRoundaboutCenterSegments + 2));
    vertices.emplace_back(
        center,
        sf::Color::White,
        textureCenter);

    constexpr float tau = 6.28318530718f;
    for (int segment = 0;
         segment <= kRoundaboutCenterSegments;
         ++segment) {
        const float angle =
            tau * static_cast<float>(segment) /
            static_cast<float>(
                kRoundaboutCenterSegments);
        const sf::Vector2f radial{
            std::cos(angle),
            std::sin(angle)
        };
        vertices.emplace_back(
            center + radial * radius,
            sf::Color::White,
            textureCenter +
                radial * textureRadius);
    }

    sf::RenderStates states;
    states.texture = &texture;
    target.draw(
        vertices.data(),
        vertices.size(),
        sf::TriangleFan,
        states);
}

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
    // Zoom-aware LOD: the sign/pole scale with the view so they stay
    // proportional to the road they sit on, and the whole stop is hidden
    // once the view is zoomed out past a threshold to avoid cluttering the
    // map with hundreds of tiny markers.
    const float detailScale = getDetailScale(target.getView());
    if (detailScale < 0.35f) {
        return;
    }
    // Cap the detail scale so markers never grow unboundedly when zooming
    // in very far. The base sizes are tuned for detailScale ~1.0.
    const float cappedScale = std::min(detailScale, 1.5f);
    const float signWidth = 8.0f * cappedScale;
    const float signHeight = 7.0f * cappedScale;
    const float poleWidth = 1.25f * cappedScale;
    const float poleHeight = 0.0f * cappedScale;
    const float markerOffset = 4.5f * cappedScale;
    const float outlineThickness = std::max(0.5f, 1.0f * cappedScale);
    const unsigned int labelSize = static_cast<unsigned int>(
        std::clamp(5.0f * cappedScale, 2.5f, 6.0f));

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
                roadEdge + normal * (edgeDirection * markerOffset);

            drawRoadStrip(
                target, roadEdge, markerPos,
                sf::Color(225, 235, 245), 2.5f * detailScale);

            sf::RectangleShape pole({poleWidth, poleHeight});
            pole.setOrigin(poleWidth * 0.5f, 0.0f);
            pole.setPosition(markerPos.x, markerPos.y + poleHeight * 0.6f);
            pole.setFillColor(sf::Color(225, 235, 245));
            target.draw(pole);

            sf::RectangleShape sign({signWidth, signHeight});
            sign.setOrigin(signWidth * 0.5f, signHeight * 0.5f);
            sign.setPosition(markerPos);
            sign.setFillColor(sf::Color(35, 145, 230));
            sign.setOutlineThickness(outlineThickness);
            sign.setOutlineColor(sf::Color::White);
            target.draw(sign);

            const std::string stopCode =
                stop->getCode().empty()
                    ? std::to_string(stop->getId())
                    : stop->getCode();
            if (font_) {
                sf::Text number;
                number.setFont(*font_);
                number.setString(stopCode);
                number.setCharacterSize(
                    stopCode.size() <= 2u ? labelSize : labelSize - 2u);
                number.setStyle(sf::Text::Bold);
                number.setFillColor(sf::Color::White);
                const sf::FloatRect bounds =
                    number.getLocalBounds();
                number.setOrigin(
                    bounds.left + bounds.width * 0.5f,
                    bounds.top + bounds.height * 0.5f);
                number.setPosition(
                    std::round(markerPos.x),
                    std::round(markerPos.y));
                target.draw(number);
            } else {
                int numericStopCode = 0;
                bool codeIsNumeric = !stopCode.empty();
                for (const char character : stopCode) {
                    if (character < '0' || character > '9') {
                        codeIsNumeric = false;
                        break;
                    }
                    numericStopCode = std::min(
                        999,
                        numericStopCode * 10 +
                            (character - '0'));
                }
                drawSevenSegmentNumber(
                    target,
                    codeIsNumeric
                        ? numericStopCode
                        : std::abs(stop->getId()) % 1000,
                    markerPos,
                    signHeight - 3.0f,
                    sf::Color::White);
            }
        }
    }
}

void VisualizationEngine::drawTrafficLights(sf::RenderTarget& target, const Graph& graph) const {
    // Zoom-aware LOD: traffic lights are hidden once the view is zoomed out
    // past a threshold. With thousands of intersections, drawing every
    // signal at a small zoom is both visually cluttered and expensive.
    const float detailScale = getDetailScale(target.getView());
    if (detailScale < 0.5f) {
        return;
    }

    const ViewportBounds viewportBounds(
        target.getView(),
        2.0f);
    for (const Intersection* intersection :
         graph.getAllIntersections()) {
        if (intersection == nullptr) continue;
        const sf::Vector2f interPos =
            worldToScreen(intersection->getX(), intersection->getY());
        if (!viewportBounds.containsPoint(interPos, 150.0f)) {
            continue;
        }
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
            const float renderedMinimumX =
                std::min(
                    {stopLeft.x,
                     stopRight.x,
                     housingCenter.x,
                     countdownCenter.x});
            const float renderedMaximumX =
                std::max(
                    {stopLeft.x,
                     stopRight.x,
                     housingCenter.x,
                     countdownCenter.x});
            const float renderedMinimumY =
                std::min(
                    {stopLeft.y,
                     stopRight.y,
                     housingCenter.y,
                     countdownCenter.y});
            const float renderedMaximumY =
                std::max(
                    {stopLeft.y,
                     stopRight.y,
                     housingCenter.y,
                     countdownCenter.y});
            const float renderedHalfThickness =
                std::max(housingThickness, countdownSize) *
                    0.5f +
                2.0f;
            const sf::FloatRect renderedBounds(
                renderedMinimumX,
                renderedMinimumY,
                renderedMaximumX - renderedMinimumX,
                renderedMaximumY - renderedMinimumY);
            if (!viewportBounds.intersectsRectangle(
                    renderedBounds,
                    renderedHalfThickness)) {
                continue;
            }

            drawRoadStrip(
                target,
                stopLeft,
                stopRight,
                sf::Color(255, 255, 255, 235),
                std::max(
                    1.5f,
                    metresToScreenPixels(0.25, road)));

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

            // The countdown text is the most expensive part of each signal
            // (per-frame text layout). At Medium/Low LOD we keep the lamp
            // housing but drop the countdown to save per-frame work.
            if (lodLevel_ == LodLevel::Full) {
                const int displayedSeconds =
                    static_cast<int>(std::ceil(
                        std::max(
                            0.0,
                            light->getRemainingSeconds() -
                                1e-9)));
                if (detailScale >= 0.7f) {
                    if (font_) {
                        sf::Text countdownText;
                        countdownText.setFont(*font_);
                        countdownText.setString(
                            std::to_string(displayedSeconds));
                        countdownText.setCharacterSize(
                            static_cast<unsigned int>(
                                std::max(8.0f, countdownSize * 0.7f)));
                        countdownText.setStyle(sf::Text::Bold);
                        countdownText.setFillColor(lightColor(state));
                        const sf::FloatRect bounds =
                            countdownText.getLocalBounds();
                        countdownText.setOrigin(
                            bounds.left + bounds.width * 0.5f,
                            bounds.top + bounds.height * 0.5f);
                        countdownText.setPosition(
                            std::round(countdownCenter.x),
                            std::round(countdownCenter.y));
                        target.draw(countdownText);
                    } else {
                        drawSevenSegmentNumber(
                            target,
                            displayedSeconds,
                            countdownCenter,
                            countdownSize,
                            lightColor(state));
                    }
                }
            }
        }
    }
}

void VisualizationEngine::drawBusStations(
    sf::RenderTarget& target,
    const Graph& graph) const {
    // Zoom-aware LOD: the terminal marker scales with the view so it stays
    // proportional to the road it sits on, and the label is hidden once the
    // view is zoomed out past a threshold.
    const float detailScale = getDetailScale(target.getView());
    if (detailScale < 0.35f) {
        return;
    }

    const float cappedScale = std::min(detailScale, 1.5f);
    const float terminalWidth = 7.0f * cappedScale;
    const float terminalHeight = 5.0f * cappedScale;
    const float bayWidth = 3.0f * cappedScale;
    const float bayHeight = 1.5f * cappedScale;
    const float outlineThickness = std::max(0.25f, 0.75f * cappedScale);
    const unsigned int labelSize = static_cast<unsigned int>(
        std::clamp(4.0f * cappedScale, 2.5f, 5.0f));
    const bool showLabel = detailScale >= 0.25f;

    const ViewportBounds viewportBounds(target.getView(), 10.0f);
    for (const BusStation* station :
         graph.getAllBusStations()) {
        if (station == nullptr ||
            station->getDepartureRoad() == nullptr) {
            continue;
        }

        const sf::Vector2f marker =
            worldToScreen(
                station->getX(),
                station->getY());
        if (!viewportBounds.containsPoint(marker, terminalWidth + 50.0f)) {
            continue;
        }

        const Road* departureRoad = station->getDepartureRoad();

        sf::RectangleShape terminal(
            {terminalWidth, terminalHeight});
        terminal.setOrigin(terminalWidth * 0.5f, terminalHeight * 0.5f);
        terminal.setPosition(marker);
        terminal.setFillColor(
            sf::Color(35, 185, 105));
        terminal.setOutlineThickness(outlineThickness);
        terminal.setOutlineColor(sf::Color::White);
        target.draw(terminal);

        sf::RectangleShape bay(
            {bayWidth, bayHeight});
        bay.setOrigin(bayWidth * 0.5f, bayHeight * 0.5f);
        bay.setPosition(marker);
        bay.setFillColor(sf::Color::White);
        target.draw(bay);

        if (font_ != nullptr && showLabel) {
            const Vec2 laneWorld =
                RoadGeometry::sampleLane(
                    *departureRoad,
                    station->getAccessLaneIndex() >= 0
                        ? station->getAccessLaneIndex()
                        : departureRoad->
                              getCurbLaneIndex(),
                    station->
                        getProgressOffset()).position;
            const sf::Vector2f lanePosition =
                worldToScreen(
                    laneWorld.x,
                    laneWorld.y);
            sf::Vector2f outward =
                marker - lanePosition;
            const float outwardLength =
                std::sqrt(
                    outward.x * outward.x +
                    outward.y * outward.y);
            if (outwardLength > 0.01f) {
                outward /= outwardLength;
            } else {
                outward = {0.0f, 1.0f};
            }
            sf::Text label;
            label.setFont(*font_);
            label.setString(station->getCode());
            label.setCharacterSize(labelSize);
            label.setStyle(sf::Text::Bold);
            label.setFillColor(
                sf::Color(120, 255, 175));
            label.setOutlineColor(sf::Color::Black);
            label.setOutlineThickness(1.0f);
            label.setPosition(
                marker.x + outward.x * 15.0f * detailScale,
                marker.y + outward.y * 15.0f * detailScale);
            target.draw(label);
        }
    }
}

void VisualizationEngine::drawIntersectionNode(sf::RenderTarget& target, const Intersection* intersection,
                                               bool tintByCongestion) const {
    if (intersection == nullptr) {
        return;
    }

    const sf::Vector2f point = worldToScreen(intersection->getX(), intersection->getY());
    if (intersection->isRoundabout()) {
        const float metricScale = static_cast<float>(
            RoadGeometry::metresPerWorldUnit(*intersection));
        const float roadWidth = static_cast<float>(
            std::max(
                RoadGeometry::LANE_WIDTH_METRES,
                intersection->getTraversalWidthMetres()) /
            std::max(1e-6f, metricScale) * scale_);
        const float circulationRadius = static_cast<float>(
            intersection->getTraversalRadiusMetres() /
            std::max(1e-6f, metricScale) * scale_);
        const float outerRadius =
            circulationRadius + roadWidth * 0.5f;
        // When tinting by congestion, the roundabout surface uses the
        // averaged heat-map color of its connected roads. The direction
        // arrows and centre image are drawn on top so they stay readable.
        const sf::Color roundaboutRoadColor =
            getIntersectionBoxColor(intersection, tintByCongestion);

        // Transition each physical road from its rectangular carriageway
        // into a wider mouth on the outer circle. The short flare starts
        // before the circle, avoiding the "rectangle touching a circle at
        // one point" silhouette.
        const Vec2 roundaboutCentre{
            intersection->getX(),
            intersection->getY()
        };
        const double outerRadiusWorld =
            static_cast<double>(outerRadius) /
            std::max(1e-6, scale_);
        const double roundaboutWidthWorld =
            intersection->getTraversalWidthMetres() /
            std::max(
                1e-6,
                RoadGeometry::metresPerWorldUnit(
                    *intersection));
        std::unordered_set<const Road*> connectedRoadsDrawn;
        const auto drawApproachConnector =
            [this,
             &target,
             intersection,
             &roundaboutCentre,
             outerRadiusWorld,
             roundaboutWidthWorld,
             &connectedRoadsDrawn,
             roundaboutRoadColor](const Road* road) {
                if (road == nullptr ||
                    connectedRoadsDrawn.count(road) != 0) {
                    return;
                }
                const auto touchesRoundabout =
                    [intersection](const Road* candidate) {
                        return candidate != nullptr &&
                            (candidate->getStart() == intersection ||
                             candidate->getEnd() == intersection);
                    };
                if (!touchesRoundabout(road)) {
                    return;
                }

                std::vector<const Road*> carriageways{road};
                const Road* reverse = road->getReverseRoad();
                if (touchesRoundabout(reverse)) {
                    carriageways.push_back(reverse);
                }
                for (const Road* carriageway : carriageways) {
                    connectedRoadsDrawn.insert(carriageway);
                }

                const bool referenceAtStart =
                    road->getStart() == intersection;
                const Vec2 outward = referenceAtStart
                    ? RoadGeometry::roadDirection(*road)
                    : -1.0 * RoadGeometry::roadDirection(*road);
                const Vec2 lateral = rightNormal(outward);
                double minimumLateral = 0.0;
                double maximumLateral = 0.0;
                bool hasEdge = false;
                for (const Road* carriageway : carriageways) {
                    const bool atStart =
                        carriageway->getStart() == intersection;
                    for (bool rightEdge : {false, true}) {
                        const Vec2 edge =
                            RoadGeometry::roadEdgeEndpoint(
                                *carriageway,
                                rightEdge,
                                atStart);
                        const double edgeOffset =
                            dot(
                                edge - roundaboutCentre,
                                lateral);
                        if (!hasEdge) {
                            minimumLateral = edgeOffset;
                            maximumLateral = edgeOffset;
                            hasEdge = true;
                        } else {
                            minimumLateral =
                                std::min(
                                    minimumLateral,
                                    edgeOffset);
                            maximumLateral =
                                std::max(
                                    maximumLateral,
                                    edgeOffset);
                        }
                    }
                }
                if (!hasEdge) {
                    return;
                }

                const double physicalRoadWidth =
                    maximumLateral - minimumLateral;
                const double transitionLength =
                    std::max(
                        roundaboutWidthWorld,
                        physicalRoadWidth * 0.55);
                const double mouthExpansion =
                    roundaboutWidthWorld * 0.35;
                const double maximumMouthOffset =
                    outerRadiusWorld * 0.82;
                const double mouthMinimum =
                    std::clamp(
                        minimumLateral - mouthExpansion,
                        -maximumMouthOffset,
                        maximumMouthOffset);
                const double mouthMaximum =
                    std::clamp(
                        maximumLateral + mouthExpansion,
                        -maximumMouthOffset,
                        maximumMouthOffset);
                const auto pointOnOuterCircle =
                    [&roundaboutCentre,
                     &outward,
                     &lateral,
                     outerRadiusWorld](double lateralOffset) {
                        const double radialOffset =
                            std::sqrt(std::max(
                                0.0,
                                outerRadiusWorld *
                                    outerRadiusWorld -
                                lateralOffset *
                                    lateralOffset));
                        return roundaboutCentre +
                            outward * radialOffset +
                            lateral * lateralOffset;
                    };
                const Vec2 transitionBase =
                    roundaboutCentre +
                    outward *
                        (outerRadiusWorld +
                         transitionLength);
                const Vec2 startA =
                    transitionBase +
                    lateral * minimumLateral;
                const Vec2 startB =
                    transitionBase +
                    lateral * maximumLateral;
                const Vec2 mouthA =
                    pointOnOuterCircle(mouthMinimum);
                const Vec2 mouthB =
                    pointOnOuterCircle(mouthMaximum);
                sf::ConvexShape connector(4);
                connector.setPoint(
                    0, worldToScreen(startA.x, startA.y));
                connector.setPoint(
                    1, worldToScreen(startB.x, startB.y));
                connector.setPoint(
                    2, worldToScreen(mouthB.x, mouthB.y));
                connector.setPoint(
                    3, worldToScreen(mouthA.x, mouthA.y));
                connector.setFillColor(roundaboutRoadColor);
                target.draw(connector);
            };
        for (const Road* road :
             intersection->getIncomingRoads()) {
            drawApproachConnector(road);
        }
        for (const Road* road :
             intersection->getOutgoingRoads()) {
            drawApproachConnector(road);
        }

        const float hubRadius =
            std::max(2.0f, circulationRadius - roadWidth * 0.5f);
        sf::CircleShape hub(hubRadius);
        hub.setOrigin(hubRadius, hubRadius);
        hub.setPosition(point);
        // Keep the island empty. A texture or sprite can later be placed at
        // the roundabout centre without being tinted by a built-in fill.
        hub.setFillColor(sf::Color::Transparent);
        hub.setOutlineThickness(roadWidth);
        hub.setOutlineColor(roundaboutRoadColor);
        target.draw(hub);
        if (const sf::Texture* centerTexture =
                roundaboutCenterTexture()) {
            drawRoundaboutCenterImage(
                target,
                *centerTexture,
                point,
                std::max(0.0f, hubRadius - 1.0f));
        }

        // Vehicles enter on their right and circulate counter-clockwise in
        // world space. Keep the direction arrows aligned with that path.
        // Deriving the screen tangent from two transformed world points also
        // accounts for the inverted screen Y axis.
        constexpr double pi = 3.14159265358979323846;
        constexpr double arrowStepRadians = 0.04;
        constexpr int arrowCount = 4;
        const double circulationRadiusWorld =
            intersection->getTraversalRadiusMetres() /
            std::max(
                1e-6,
                RoadGeometry::metresPerWorldUnit(
                    *intersection));
        const float arrowLength =
            std::clamp(roadWidth * 0.90f, 7.0f, 14.0f);
        const float arrowHalfWidth =
            std::clamp(roadWidth * 0.30f, 2.2f, 4.5f);
        const float shaftHalfWidth =
            arrowHalfWidth * 0.38f;
        for (int arrowIndex = 0;
             arrowIndex < arrowCount;
             ++arrowIndex) {
            const double angle =
                pi * 0.25 +
                static_cast<double>(arrowIndex) *
                    (2.0 * pi /
                     static_cast<double>(arrowCount));
            const Vec2 markerWorld =
                roundaboutCentre +
                Vec2{std::cos(angle), std::sin(angle)} *
                    circulationRadiusWorld;
            const Vec2 forwardProbeWorld =
                roundaboutCentre +
                Vec2{
                    std::cos(angle + arrowStepRadians),
                    std::sin(angle + arrowStepRadians)
                } * circulationRadiusWorld;
            const sf::Vector2f marker =
                worldToScreen(
                    markerWorld.x,
                    markerWorld.y);
            sf::Vector2f forward =
                worldToScreen(
                    forwardProbeWorld.x,
                    forwardProbeWorld.y) -
                marker;
            const float forwardLength =
                std::sqrt(
                    forward.x * forward.x +
                    forward.y * forward.y);
            if (forwardLength <= 0.001f) {
                continue;
            }
            forward /= forwardLength;
            const sf::Vector2f side{
                -forward.y,
                forward.x
            };
            const float tipOffset =
                arrowLength * 0.5f;
            const float headBaseOffset =
                arrowLength * 0.02f;
            const float tailOffset =
                -arrowLength * 0.5f;

            sf::ConvexShape arrow(7);
            arrow.setPoint(
                0,
                marker + forward * tipOffset);
            arrow.setPoint(
                1,
                marker +
                    forward * headBaseOffset +
                    side * arrowHalfWidth);
            arrow.setPoint(
                2,
                marker +
                    forward * headBaseOffset +
                    side * shaftHalfWidth);
            arrow.setPoint(
                3,
                marker +
                    forward * tailOffset +
                    side * shaftHalfWidth);
            arrow.setPoint(
                4,
                marker +
                    forward * tailOffset -
                    side * shaftHalfWidth);
            arrow.setPoint(
                5,
                marker +
                    forward * headBaseOffset -
                    side * shaftHalfWidth);
            arrow.setPoint(
                6,
                marker +
                    forward * headBaseOffset -
                    side * arrowHalfWidth);
            arrow.setFillColor(
                sf::Color(250, 250, 245, 225));
            arrow.setOutlineColor(
                sf::Color(30, 36, 38, 170));
            arrow.setOutlineThickness(0.5f);
            target.draw(arrow);
        }
    } else {
        const float halfExtent = getIntersectionBoxHalfExtent(intersection);

        sf::RectangleShape core({halfExtent * 2.0f, halfExtent * 2.0f});
        core.setOrigin(halfExtent, halfExtent);
        core.setPosition(point);
        core.setFillColor(getIntersectionBoxColor(intersection, tintByCongestion));
        core.setOutlineThickness(0.0f);
        target.draw(core);
    }

    if (!tintByCongestion && spriteTexture_ != nullptr) {
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

sf::Color VisualizationEngine::getIntersectionBoxColor(const Intersection* intersection, bool tintByCongestion) const {
    const sf::Color fallbackGray(110, 110, 110);
    if (intersection == nullptr) {
        return fallbackGray;
    }
    if (!tintByCongestion || !heatMapEnabled_) {
        return fallbackGray;
    }

    int r = 0;
    int g = 0;
    int b = 0;
    int count = 0;

    const auto accumulate = [&](const std::vector<Road*>& roads) {
        for (const Road* road : roads) {
            // Bridges and tunnels both participate in the heat-map tint
            // like ordinary roads.
            if (road == nullptr) {
                continue;
            }
            const sf::Color c = colorForRoad(road);
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

void VisualizationEngine::drawPOIDriveways(
    sf::RenderTarget& target,
    const Graph& graph) const {
    const auto drawDriveway =
        [&](const PointOfInterest* poi) {
        if (!poi) return;
        const Road* road = poi->getConnectedRoad();
        if (road == nullptr) {
            return;
        }

        const sf::Vector2f poiPosition =
            worldToScreen(poi->getX(), poi->getY());
        const int accessLane =
            poi->getAccessLaneIndex() >= 0
                ? poi->getAccessLaneIndex()
                : road->getCurbLaneIndex();
        const RoadGeometry::RoadAccessPath accessPath =
            RoadGeometry::makeRoadAccessPath(
                *road,
                accessLane,
                poi->getProgressOffset(),
                {poi->getX(), poi->getY()});
        const sf::Vector2f curbPosition =
            worldToScreen(
                accessPath.curb.x,
                accessPath.curb.y);
        const sf::Vector2f corner =
            worldToScreen(
                accessPath.corner.x,
                accessPath.corner.y);
        const float drivewayWidth = std::max(
            2.0f,
            metresToScreenPixels(4.0, road));
        const float borderWidth = drivewayWidth + 2.0f;
        const sf::Color borderColor(58, 62, 64);
        const sf::Color surfaceColor(100, 100, 100);
        const auto drawSegment =
            [&](const sf::Vector2f& from,
                const sf::Vector2f& to,
                const sf::Color& color,
                float width) {
                if (distanceBetween(from, to) > 0.5f) {
                    drawRoadStrip(
                        target, from, to, color, width);
                }
            };

        drawSegment(
            poiPosition, corner,
            borderColor, borderWidth);
        drawSegment(
            corner, curbPosition,
            borderColor, borderWidth);
        drawSegment(
            poiPosition, corner,
            surfaceColor, drivewayWidth);
        drawSegment(
            corner, curbPosition,
            surfaceColor, drivewayWidth);

        // Fill the inside of the 90-degree corner so two rectangular strips
        // read as one continuous driveway instead of leaving a pinhole gap.
        sf::CircleShape cornerFill(drivewayWidth * 0.5f);
        cornerFill.setOrigin(
            drivewayWidth * 0.5f,
            drivewayWidth * 0.5f);
        cornerFill.setPosition(corner);
        cornerFill.setFillColor(surfaceColor);
        target.draw(cornerFill);
    };

    for (const PointOfInterest* poi :
         graph.getAllPOIs()) {
        drawDriveway(poi);
    }
    for (const BusStation* station :
         graph.getAllBusStations()) {
        drawDriveway(station);
    }
}

void VisualizationEngine::drawPOIs(sf::RenderTarget& target, const Graph& graph) const {
    const float detailScale = getDetailScale(target.getView());
    if (detailScale < 0.35f) {
        return;
    }
    // Cap the detail scale so POI markers never grow unboundedly when
    // zooming in very far. The base sizes are tuned for detailScale ~1.0.
    const float cappedScale = std::min(detailScale, 1.5f);
    const auto& pois = graph.getAllPOIs();
    const bool showLabels = detailScale >= 0.6f;
    const float buildingSize = std::max(1.5f, 5.0f * cappedScale);
    const float halfSize = buildingSize * 0.5f;
    const float outlineThickness = std::max(0.25f, 0.5f * cappedScale);
    const unsigned int labelSize = static_cast<unsigned int>(
        std::clamp(5.0f * cappedScale, 3.0f, 7.0f));

    const ViewportBounds viewportBounds(target.getView(), 20.0f);
    for (const auto* poi : pois) {
        if (!poi) continue;
        sf::Vector2f pos = worldToScreen(poi->getX(), poi->getY());
        if (!viewportBounds.containsPoint(pos, buildingSize + 50.0f)) {
            continue;
        }

        sf::Color poiColor(255, 150, 0);
        if (poi->getType() == POIType::PARKING_LOT) poiColor = sf::Color(100, 100, 255);
        else if (poi->getType() == POIType::BUS_STATION) poiColor = sf::Color(50, 200, 50);
        else if (poi->getType() == POIType::HOSPITAL) poiColor = sf::Color(255, 50, 50);
        else if (poi->getType() == POIType::RESIDENTIAL_AREA) poiColor = sf::Color(90, 190, 220);
        else if (poi->getType() == POIType::SUPERMARKET) poiColor = sf::Color(200, 200, 50);

        // Draw building, scaled with the zoom level.
        sf::RectangleShape building(sf::Vector2f(buildingSize, buildingSize));
        building.setOrigin(halfSize, halfSize);
        building.setPosition(pos);
        building.setFillColor(poiColor);
        building.setOutlineThickness(outlineThickness);
        building.setOutlineColor(sf::Color::White);
        target.draw(building);

        if (font_ && showLabels) {
            sf::Text text;
            text.setFont(*font_);
            text.setString(poi->getName());
            text.setCharacterSize(labelSize);
            text.setFillColor(sf::Color::White);
            text.setOutlineColor(sf::Color::Black);
            text.setOutlineThickness(1.0f);
            sf::Vector2f roadAnchor = pos;
            if (const Road* road =
                    poi->getConnectedRoad()) {
                const int accessLane =
                    poi->getAccessLaneIndex() >= 0
                        ? poi->getAccessLaneIndex()
                        : road->getCurbLaneIndex();
                const auto accessPath =
                    RoadGeometry::makeRoadAccessPath(
                        *road,
                        accessLane,
                        poi->getProgressOffset(),
                        {poi->getX(), poi->getY()});
                roadAnchor = worldToScreen(
                    accessPath.curb.x,
                    accessPath.curb.y);
            }
            sf::Vector2f labelDirection =
                pos - roadAnchor;
            if (poi->isLabelOnLeft()) {
                labelDirection = {-1.0f, 0.0f};
            }

            const float labelGap = 8.0f * detailScale;
            const sf::FloatRect bounds =
                text.getLocalBounds();
            sf::Vector2f labelPosition;
            if (std::fabs(labelDirection.x) >=
                std::fabs(labelDirection.y)) {
                labelPosition.x =
                    labelDirection.x < 0.0f
                        ? pos.x - labelGap -
                              bounds.left - bounds.width
                        : pos.x + labelGap - bounds.left;
                labelPosition.y =
                    pos.y - bounds.top -
                    bounds.height * 0.5f;
            } else {
                labelPosition.x =
                    pos.x - bounds.left -
                    bounds.width * 0.5f;
                labelPosition.y =
                    labelDirection.y < 0.0f
                        ? pos.y - labelGap -
                              bounds.top - bounds.height
                        : pos.y + labelGap - bounds.top;
            }
            text.setPosition(labelPosition);
            target.draw(text);
        }
    }
}

void VisualizationEngine::drawRoadNames(sf::RenderTarget& target, const std::vector<Road*>& roads) const {
    if (!font_) return;

    // Zoom-aware LOD: road-name labels are hidden once the view is zoomed
    // out past a threshold. With thousands of roads, drawing every label
    // at a small zoom is both visually cluttered and expensive (per-label
    // text layout).
    const float detailScale = getDetailScale(target.getView());
    if (detailScale < 0.65f) {
        return;
    }

    const ViewportBounds viewportBounds(
        target.getView(),
        2.0f);
    // A two-way road is represented by two directional Road objects. Track
    // which physical road (identified by its unordered pair of endpoint
    // intersection ids) already has a label so we draw one per physical
    // road instead of stacking both names. Using a hash set keeps this
    // O(n) instead of the previous O(n²) scan over all earlier roads.
    std::unordered_set<std::uint64_t> labelledRoads;
    labelledRoads.reserve(roads.size());
    for (std::size_t roadIndex = 0; roadIndex < roads.size(); ++roadIndex) {
        const Road* road = roads[roadIndex];
        if (!road || road->getName().empty()) continue;
        const Intersection* start = road->getStart();
        const Intersection* end = road->getEnd();
        if (!start || !end) continue;

        const int firstIntersectionId =
            std::min(start->getId(), end->getId());
        const int secondIntersectionId =
            std::max(start->getId(), end->getId());
        const std::uint64_t roadKey =
            (static_cast<std::uint64_t>(
                 static_cast<unsigned int>(firstIntersectionId))
             << 32u) |
            static_cast<unsigned int>(secondIntersectionId);
        if (labelledRoads.count(roadKey) != 0) {
            continue;
        }
        labelledRoads.insert(roadKey);

        // Labels belong on the shared physical centreline. Directional road
        // surfaces are offset to either side of this line.
        const sf::Vector2f a =
            getRoadCenterlineEntryPoint(road, start);
        const sf::Vector2f b =
            getRoadCenterlineEntryPoint(road, end);
        constexpr float maximumLabelHalfHeight = 10.0f;
        if (!viewportBounds.intersectsSegment(
                a,
                b,
                maximumLabelHalfHeight)) {
            continue;
        }
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

        if (!viewportBounds.intersectsRectangle(
                text.getGlobalBounds(),
                4.0f)) {
            continue;
        }

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
