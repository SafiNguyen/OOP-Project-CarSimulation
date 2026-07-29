#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "model/Crosswalk.h"
#include "model/Graph.h"
#include "model/Road.h"
#include "model/RoadGeometry.h"

void VisualizationEngine::drawSidewalks(
    sf::RenderTarget& target,
    const Graph& graph) const {
    std::unordered_set<const Road*> drawn;
    const sf::Color curbColor(72, 76, 76);
    const sf::Color sidewalkColor(188, 186, 174);
    for (const Road* road : graph.getAllRoads()) {
        if (road == nullptr ||
            drawn.count(road) != 0) {
            continue;
        }

        const auto drawSide =
            [this, &target, curbColor, sidewalkColor](
                const Road& sideRoad,
                bool rightSide) {
                const Vec2 edgeStart =
                    RoadGeometry::sampleRoadEdge(
                        sideRoad,
                        rightSide,
                        0.0);
                const Vec2 edgeEnd =
                    RoadGeometry::sampleRoadEdge(
                        sideRoad,
                        rightSide,
                        sideRoad.getDistance());
                const Vec2 modelSidewalkStart =
                    RoadGeometry::sampleSidewalk(
                        sideRoad,
                        rightSide,
                        0.0);
                const Vec2 modelSidewalkEnd =
                    RoadGeometry::sampleSidewalk(
                        sideRoad,
                        rightSide,
                        sideRoad.getDistance());
                const float sidewalkWidth =
                    std::max(
                        8.0f,
                        metresToScreenPixels(
                            RoadGeometry::
                                SIDEWALK_WIDTH_METRES,
                            &sideRoad));
                const float curbWidth =
                    std::max(
                        1.0f,
                        metresToScreenPixels(
                            RoadGeometry::
                                SIDEWALK_CURB_WIDTH_METRES,
                            &sideRoad));

                const sf::Vector2f edgeStartScreen =
                    worldToScreen(edgeStart.x, edgeStart.y);
                const sf::Vector2f edgeEndScreen =
                    worldToScreen(edgeEnd.x, edgeEnd.y);
                sf::Vector2f outwardStart =
                    worldToScreen(
                        modelSidewalkStart.x,
                        modelSidewalkStart.y) -
                    edgeStartScreen;
                sf::Vector2f outwardEnd =
                    worldToScreen(
                        modelSidewalkEnd.x,
                        modelSidewalkEnd.y) -
                    edgeEndScreen;
                const auto normalize =
                    [](sf::Vector2f vector) {
                        const float length =
                            std::sqrt(
                                vector.x * vector.x +
                                vector.y * vector.y);
                        return length > 0.001f
                            ? vector / length
                            : sf::Vector2f{};
                    };
                outwardStart = normalize(outwardStart);
                outwardEnd = normalize(outwardEnd);
                if (outwardStart == sf::Vector2f{}) {
                    outwardStart = outwardEnd;
                }
                if (outwardEnd == sf::Vector2f{}) {
                    outwardEnd = outwardStart;
                }
                if (outwardStart == sf::Vector2f{} ||
                    outwardEnd == sf::Vector2f{}) {
                    return;
                }

                // Widths have minimum pixel sizes for readability. Derive
                // the visual centre from those actual widths, not only from
                // model metres, so zooming out can never push the sidewalk
                // back over the carriageway.
                constexpr float roadClearancePixels = 0.75f;
                const float centreOffset =
                    sidewalkWidth * 0.5f +
                    curbWidth +
                    roadClearancePixels;
                const sf::Vector2f sidewalkStart =
                    edgeStartScreen +
                    outwardStart * centreOffset;
                const sf::Vector2f sidewalkEnd =
                    edgeEndScreen +
                    outwardEnd * centreOffset;
                drawRoadStrip(
                    target,
                    sidewalkStart,
                    sidewalkEnd,
                    curbColor,
                    sidewalkWidth +
                        curbWidth * 2.0f);
                drawRoadStrip(
                    target,
                    sidewalkStart,
                    sidewalkEnd,
                    sidewalkColor,
                    sidewalkWidth);
            };

        Road* reverse = road->getReverseRoad();
        if (reverse != nullptr) {
            drawn.insert(road);
            drawn.insert(reverse);
            drawSide(*road, true);
            drawSide(*reverse, true);
        } else {
            drawn.insert(road);
            drawSide(*road, false);
            drawSide(*road, true);
        }
    }
}

void VisualizationEngine::drawCrosswalks(
    sf::RenderTarget& target,
    const Graph& graph) const {
    constexpr double stripeMetres = 0.30;
    constexpr double stripeGapMetres = 0.20;

    for (const Crosswalk* crosswalk :
         graph.getAllCrosswalks()) {
        if (crosswalk == nullptr ||
            crosswalk->getIncomingRoad() == nullptr) {
            continue;
        }
        const Road& incoming =
            *crosswalk->getIncomingRoad();
        const Vec2 direction =
            RoadGeometry::roadDirection(incoming);
        const double metricScale =
            RoadGeometry::metresPerWorldUnit(incoming);
        const Vec2 edgeA =
            RoadGeometry::sampleRoadEdge(
                incoming,
                true,
                crosswalk->
                    getCentreProgressMetres());
        Vec2 edgeB;
        if (Road* reverse =
                crosswalk->getReverseRoad()) {
            edgeB = RoadGeometry::sampleRoadEdge(
                *reverse,
                true,
                crosswalk->
                    getReverseCentreProgressMetres());
        } else {
            edgeB = RoadGeometry::sampleRoadEdge(
                incoming,
                false,
                crosswalk->
                    getCentreProgressMetres());
        }

        const double pitch =
            stripeMetres + stripeGapMetres;
        const int stripeCount = std::max(
            1,
            static_cast<int>(std::floor(
                crosswalk->getWidthMetres() /
                pitch)));
        const double occupiedLength =
            static_cast<double>(stripeCount - 1) *
            pitch;
        for (int index = 0;
             index < stripeCount;
             ++index) {
            const double longitudinalOffset =
                -occupiedLength * 0.5 +
                static_cast<double>(index) *
                    pitch;
            const Vec2 centreShift =
                direction *
                (longitudinalOffset /
                 metricScale);
            const Vec2 halfStripe =
                direction *
                (stripeMetres * 0.5 /
                 metricScale);

            sf::ConvexShape stripe;
            stripe.setPointCount(4);
            const Vec2 points[] = {
                edgeA + centreShift - halfStripe,
                edgeB + centreShift - halfStripe,
                edgeB + centreShift + halfStripe,
                edgeA + centreShift + halfStripe
            };
            for (std::size_t point = 0;
                 point < 4;
                 ++point) {
                stripe.setPoint(
                    point,
                    worldToScreen(
                        points[point].x,
                        points[point].y));
            }
            stripe.setFillColor(
                sf::Color(245, 245, 240, 225));
            target.draw(stripe);
        }
    }

    drawCrosswalkSignals(target, graph);
}

void VisualizationEngine::drawCrosswalkSignals(
    sf::RenderTarget& target,
    const Graph& graph) const {
    for (const Crosswalk* crosswalk :
         graph.getAllCrosswalks()) {
        if (crosswalk == nullptr ||
            crosswalk->getIncomingRoad() == nullptr) {
            continue;
        }
        const Road& incoming =
            *crosswalk->getIncomingRoad();
        const sf::Color signalColor =
            crosswalk->getSignalState() ==
                    PedestrianSignalState::Walk
                ? sf::Color(65, 220, 100)
                : crosswalk->getSignalState() ==
                          PedestrianSignalState::
                              Clearance
                      ? sf::Color(245, 185, 45)
                      : sf::Color(220, 65, 65);
        const sf::Vector2f signalPosition =
            worldToScreen(
                crosswalk->getSideAPosition().x,
                crosswalk->getSideAPosition().y);
        const float signalRadius = std::clamp(
            metresToScreenPixels(0.35, &incoming),
            2.5f,
            5.0f);
        sf::CircleShape signal(signalRadius);
        signal.setOrigin(signalRadius, signalRadius);
        signal.setPosition(signalPosition);
        signal.setFillColor(signalColor);
        signal.setOutlineThickness(1.0f);
        signal.setOutlineColor(sf::Color(25, 25, 25));
        target.draw(signal);
    }
}
