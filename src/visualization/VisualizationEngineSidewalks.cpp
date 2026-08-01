#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "Graph.h"
#include "Road.h"
#include "RoadGeometry.h"

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
