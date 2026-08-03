#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>

#include "Mapload.h"
#include "visualization/VisualizationEngine.h"
#include "visualization/VehicleRenderGeometry.h"
#include "visualization/VehicleSprite.h"
#include "visualization/SimulatorFactory.h"
#include "algorithm/DijkstraStrategy.h"
#include "Bus.h"
#include "BusStop.h"
#include "Car.h"
#include "EmergencyVehicle.h"
#include "Graph.h"
#include "Intersection.h"
#include "Motorbike.h"
#include "Road.h"
#include "RoadGeometry.h"
#include "simulation/TrafficSimulator.h"

namespace {

class PoseTestCar : public Car {
public:
    using Car::Car;
    void place(double progress, double speed) {
        progressOnCurrentRoad = progress;
        currentSpeed = speed;
    }
};

int countColorComponents(const sf::Image& image,
                         const sf::Color& color,
                         int minimumPixels) {
    const sf::Vector2u size = image.getSize();
    std::vector<unsigned char> visited(
        static_cast<std::size_t>(size.x) * size.y, 0u);
    std::vector<sf::Vector2u> pending;
    int components = 0;

    const auto indexOf = [size](unsigned int x, unsigned int y) {
        return static_cast<std::size_t>(y) * size.x + x;
    };

    for (unsigned int y = 0; y < size.y; ++y) {
        for (unsigned int x = 0; x < size.x; ++x) {
            const std::size_t startIndex = indexOf(x, y);
            if (visited[startIndex] || image.getPixel(x, y) != color) {
                continue;
            }

            int pixels = 0;
            pending.clear();
            pending.push_back({x, y});
            visited[startIndex] = 1u;
            while (!pending.empty()) {
                const sf::Vector2u current = pending.back();
                pending.pop_back();
                ++pixels;

                const int neighbors[4][2] = {
                    {-1, 0}, {1, 0}, {0, -1}, {0, 1}
                };
                for (const auto& neighbor : neighbors) {
                    const int nx = static_cast<int>(current.x) + neighbor[0];
                    const int ny = static_cast<int>(current.y) + neighbor[1];
                    if (nx < 0 || ny < 0 ||
                        nx >= static_cast<int>(size.x) ||
                        ny >= static_cast<int>(size.y)) {
                        continue;
                    }

                    const auto ux = static_cast<unsigned int>(nx);
                    const auto uy = static_cast<unsigned int>(ny);
                    const std::size_t neighborIndex = indexOf(ux, uy);
                    if (!visited[neighborIndex] &&
                        image.getPixel(ux, uy) == color) {
                        visited[neighborIndex] = 1u;
                        pending.push_back({ux, uy});
                    }
                }
            }

            if (pixels >= minimumPixels) {
                ++components;
            }
        }
    }
    return components;
}

} // namespace

int main() {
    const TurnSignalVisualSize normalSignal =
        getTurnSignalVisualSize(7.0f, 3.0f);
    const TurnSignalVisualSize zoomedOutSignal =
        getTurnSignalVisualSize(3.5f, 1.5f);
    assert(std::fabs(
        normalSignal.radius - zoomedOutSignal.radius * 2.0f) < 0.001f);
    assert(std::fabs(
        normalSignal.haloRadius -
        zoomedOutSignal.haloRadius * 2.0f) < 0.001f);
    assert(normalSignal.haloRadius < 3.0f);

    Intersection start(1, 0.0, 0.0);
    Intersection end(2, 10.0, 0.0);
    Road road(101, "Test", &start, &end, 10.0, 50.0, 1.0);
    VisualizationEngine engine;

    sf::Color normal = engine.colorForRoad(&road);
    assert(normal.r > 20 && normal.g > 100 && normal.b < 120);

    Road blockedRoad(102, "Test", &start, &end, 10.0, 50.0, 1.0);
    blockedRoad.blockRoad();
    sf::Color blocked = engine.colorForRoad(&blockedRoad);
    assert(blocked == sf::Color(180, 40, 40));

    Road congestedRoad(103, "Test", &start, &end, 10.0, 50.0, 3.0);
    sf::Color congested = engine.colorForRoad(&congestedRoad);
    assert(congested.r > congested.g && congested.r > congested.b);

    engine.setHeatMapEnabled(false);
    assert(!engine.isHeatMapEnabled());
    engine.setHeatMapEnabled(true);
    assert(engine.isHeatMapEnabled());

    Graph graph;
    graph.addIntersection(new Intersection(10, 0.0, 0.0));
    graph.addIntersection(new Intersection(11, 100.0, 0.0));
    auto* busRoad = new Road(
        201, "Bus Road", graph.getIntersection(10), graph.getIntersection(11),
        100.0, 50.0, 1.0, 2);
    graph.addRoad(busRoad);
    assert(busRoad->addBusStop(std::make_unique<BusStop>(
        501, "Rendered Stop", busRoad, 50.0, 1, 15.0)));
    assert(busRoad->addBusStop(std::make_unique<BusStop>(
        502, "Inner Lane Stop", busRoad, 25.0, 0, 15.0)));

    sf::RenderTexture target;
    assert(target.create(800, 600));
    engine.prepare(graph);
    target.clear(sf::Color::Black);
    engine.drawGraph(target, graph);
    target.display();

    const sf::Image rendered = target.getTexture().copyToImage();
    bool foundBusStopBlue = false;
    bool foundInnerLaneMarker = false;
    bool foundOuterLaneMarker = false;
    const float busRoadCenterY = engine.worldToScreen(0.0, 0.0).y;
    for (unsigned int y = 0; y < rendered.getSize().y; ++y) {
        for (unsigned int x = 0; x < rendered.getSize().x; ++x) {
            const sf::Color pixel = rendered.getPixel(x, y);
            if (pixel.r < 80 && pixel.g > 110 && pixel.b > 180) {
                foundBusStopBlue = true;
            }
            if (pixel == sf::Color(35, 145, 230)) {
                foundInnerLaneMarker =
                    foundInnerLaneMarker ||
                    static_cast<float>(y) < busRoadCenterY - 15.0f;
                foundOuterLaneMarker =
                    foundOuterLaneMarker ||
                    static_cast<float>(y) > busRoadCenterY + 15.0f;
            }
        }
    }
    assert(foundBusStopBlue);
    assert(foundInnerLaneMarker);
    assert(foundOuterLaneMarker);

    // The congestion strip is centred on the carriageway. Both lane
    // centres must receive the same heat-map color; drawing only from the
    // centreline to one side leaves the opposite lane gray.
    const sf::Color expectedRoadHeat = engine.colorForRoad(busRoad);
    for (int laneIndex = 0;
         laneIndex < busRoad->getLaneCount();
         ++laneIndex) {
        const Pose2D lanePose = RoadGeometry::sampleLane(
            *busRoad, laneIndex, 70.0);
        const sf::Vector2f lanePixel = engine.worldToScreen(
            lanePose.position.x, lanePose.position.y);
        assert(rendered.getPixel(
                   static_cast<unsigned int>(std::lround(lanePixel.x)),
                   static_cast<unsigned int>(std::lround(lanePixel.y))) ==
               expectedRoadHeat);
    }

    // Roads created from the debug panel are background links. At a visual
    // crossing, an existing road must remain on top regardless of graph
    // hash-map iteration order.
    Graph layeredRoadGraph;
    layeredRoadGraph.addIntersection(
        new Intersection(100, -50.0, 0.0));
    layeredRoadGraph.addIntersection(
        new Intersection(101, 50.0, 0.0));
    layeredRoadGraph.addIntersection(
        new Intersection(102, 0.0, -50.0));
    layeredRoadGraph.addIntersection(
        new Intersection(103, 0.0, 50.0));
    auto* existingLayerRoad = new Road(
        501, "Existing", layeredRoadGraph.getIntersection(100),
        layeredRoadGraph.getIntersection(101), 100.0, 40.0);
    auto* backgroundLayerRoad = new Road(
        999, "Custom Road", layeredRoadGraph.getIntersection(102),
        layeredRoadGraph.getIntersection(103), 100.0, 40.0);
    backgroundLayerRoad->setRenderBelowExistingRoads(true);
    backgroundLayerRoad->blockRoad();
    layeredRoadGraph.addRoad(existingLayerRoad);
    layeredRoadGraph.addRoad(backgroundLayerRoad);
    assert(backgroundLayerRoad->shouldRenderBelowExistingRoads());

    VisualizationEngine layeredRoadEngine({800u, 600u});
    layeredRoadEngine.prepare(layeredRoadGraph);
    sf::RenderTexture layeredRoadTarget;
    assert(layeredRoadTarget.create(800u, 600u));
    layeredRoadTarget.clear(sf::Color(30, 30, 30));
    layeredRoadEngine.drawGraph(
        layeredRoadTarget,
        layeredRoadGraph);
    layeredRoadTarget.display();
    const sf::Image layeredRoadImage =
        layeredRoadTarget.getTexture().copyToImage();
    const sf::Vector2f crossing =
        layeredRoadEngine.worldToScreen(0.0, 0.0);
    assert(layeredRoadImage.getPixel(
               static_cast<unsigned int>(std::lround(crossing.x)),
               static_cast<unsigned int>(std::lround(crossing.y))) ==
           layeredRoadEngine.colorForRoad(existingLayerRoad));
    const float customEdgeOffset =
        layeredRoadEngine.metresToScreenPixels(
            backgroundLayerRoad->getLaneWidthMetres() * 0.5,
            backgroundLayerRoad);
    assert(layeredRoadImage.getPixel(
               static_cast<unsigned int>(std::lround(
                   crossing.x + customEdgeOffset)),
               static_cast<unsigned int>(std::lround(crossing.y))) ==
           layeredRoadEngine.colorForRoad(existingLayerRoad));
    const sf::Vector2f customRoadInterior =
        layeredRoadEngine.worldToScreen(0.0, 25.0);
    assert(layeredRoadImage.getPixel(
               static_cast<unsigned int>(std::lround(
                   customRoadInterior.x + customEdgeOffset * 0.82f)),
               static_cast<unsigned int>(std::lround(
                   customRoadInterior.y))) ==
           layeredRoadEngine.colorForRoad(backgroundLayerRoad));

    Graph directionalStopGraph;
    directionalStopGraph.addIntersection(new Intersection(12, 0.0, 0.0));
    directionalStopGraph.addIntersection(new Intersection(13, 100.0, 0.0));
    auto* forwardStopRoad = new Road(
        210, "Forward", directionalStopGraph.getIntersection(12),
        directionalStopGraph.getIntersection(13), 100.0, 40.0, 1.0, 2);
    auto* reverseStopRoad = new Road(
        -210, "Reverse", directionalStopGraph.getIntersection(13),
        directionalStopGraph.getIntersection(12), 100.0, 40.0, 1.0, 2);
    directionalStopGraph.addRoad(forwardStopRoad);
    directionalStopGraph.addRoad(reverseStopRoad);
    assert(forwardStopRoad->addBusStop(std::make_unique<BusStop>(
        510, "Forward curb", forwardStopRoad, 30.0,
        forwardStopRoad->getCurbLaneIndex(), 5.0)));
    assert(reverseStopRoad->addBusStop(std::make_unique<BusStop>(
        511, "Reverse curb", reverseStopRoad, 30.0,
        reverseStopRoad->getCurbLaneIndex(), 5.0)));

    VisualizationEngine directionalStopEngine({800u, 600u});
    directionalStopEngine.prepare(directionalStopGraph);
    sf::RenderTexture directionalStopTarget;
    assert(directionalStopTarget.create(800u, 600u));
    directionalStopTarget.clear(sf::Color::Black);
    directionalStopEngine.drawGraph(
        directionalStopTarget, directionalStopGraph);
    directionalStopTarget.display();

    const sf::Image directionalStopImage =
        directionalStopTarget.getTexture().copyToImage();
    const float roadCenterY =
        directionalStopEngine.worldToScreen(0.0, 0.0).y;
    bool foundForwardCurbMarker = false;
    bool foundReverseCurbMarker = false;
    for (unsigned int y = 0; y < directionalStopImage.getSize().y; ++y) {
        for (unsigned int x = 0; x < directionalStopImage.getSize().x; ++x) {
            if (directionalStopImage.getPixel(x, y) !=
                sf::Color(35, 145, 230)) {
                continue;
            }
            foundForwardCurbMarker =
                foundForwardCurbMarker ||
                static_cast<float>(y) > roadCenterY + 15.0f;
            foundReverseCurbMarker =
                foundReverseCurbMarker ||
                static_cast<float>(y) < roadCenterY - 15.0f;
        }
    }
    assert(foundForwardCurbMarker);
    assert(foundReverseCurbMarker);

    Graph poseGraph;
    poseGraph.addIntersection(new Intersection(30, -50.0, 0.0));
    poseGraph.addIntersection(new Intersection(31, 0.0, 0.0));
    poseGraph.addIntersection(new Intersection(32, 0.0, 50.0));
    poseGraph.addRoad(new Road(
        401, "Pose incoming", poseGraph.getIntersection(30),
        poseGraph.getIntersection(31), 50.0, 20.0));
    poseGraph.addRoad(new Road(
        402, "Pose outgoing", poseGraph.getIntersection(31),
        poseGraph.getIntersection(32), 50.0, 20.0));
    PoseTestCar poseCar(
        900, 20.0, poseGraph.getIntersection(30),
        poseGraph.getIntersection(32));
    poseCar.setRoute(
        {poseGraph.getRoad(401), poseGraph.getRoad(402)});
    poseCar.place(49.0, 20.0);
    poseCar.update(0.1);
    assert(poseCar.getMovementState() ==
           MovementState::TraversingJunction);
    VisualizationEngine poseEngine({800u, 600u});
    poseEngine.prepare(poseGraph);
    Bus visualBus(
        901, 15.0, poseGraph.getIntersection(30),
        poseGraph.getIntersection(31));
    visualBus.setRoute({poseGraph.getRoad(401)});
    const VehicleScreenSize carScreenSize =
        getVehicleScreenSize(poseCar, poseEngine);
    const VehicleScreenSize busScreenSize =
        getVehicleScreenSize(visualBus, poseEngine);
    assert(std::fabs(
        busScreenSize.lengthPixels /
            carScreenSize.lengthPixels -
        visualBus.getLength() / poseCar.getLength()) < 1e-4f);
    assert(std::fabs(
        busScreenSize.widthPixels /
            carScreenSize.widthPixels -
        visualBus.getWidth() / poseCar.getWidth()) < 1e-4f);
    // The render-only minimum footprint is expressed in screen pixels and
    // preserves the physical aspect ratio.  It must not leak into the raw
    // metre-to-render-space conversion checked above.
    const VehicleScreenSize overviewCarSize =
        getVehicleVisualScreenSize(
            poseCar,
            poseEngine,
            100.0f);
    assert(overviewCarSize.lengthPixels / 100.0f >= 4.0f);
    assert(std::fabs(
        overviewCarSize.widthPixels /
            overviewCarSize.lengthPixels -
        carScreenSize.widthPixels /
            carScreenSize.lengthPixels) < 1e-4f);
    VehicleSprite poseSprite(&poseCar, &poseEngine);
    const Pose2D simulationPose = poseCar.getPose();
    const sf::Vector2f expectedScreen = poseEngine.worldToScreen(
        simulationPose.position.x, simulationPose.position.y);
    const sf::Vector2f spriteScreen = poseSprite.getPosition();
    assert(std::fabs(spriteScreen.x - expectedScreen.x) < 1e-4f);
    assert(std::fabs(spriteScreen.y - expectedScreen.y) < 1e-4f);
    assert(std::fabs(
        poseSprite.getAngle() +
        simulationPose.headingRadians * 180.0 /
            3.14159265358979323846) < 1e-4);

    Graph spawnGraph;
    spawnGraph.addIntersection(new Intersection(20, 0.0, 0.0));
    spawnGraph.addIntersection(new Intersection(21, 100.0, 0.0));
    spawnGraph.addRoad(new Road(
        301, "Outbound", spawnGraph.getIntersection(20),
        spawnGraph.getIntersection(21), 100.0, 50.0));
    spawnGraph.addRoad(new Road(
        302, "Inbound", spawnGraph.getIntersection(21),
        spawnGraph.getIntersection(20), 100.0, 50.0));

    DijkstraStrategy strategy;
    auto simulator = createDemoSimulator(spawnGraph, &strategy);
    int carCount = 0;
    int motorbikeCount = 0;
    int busCount = 0;
    int emergencyCount = 0;
    std::vector<Vehicle*> spawnedVehicles =
        simulator->getVehicles();
    const std::vector<Vehicle*> pendingVehicles =
        simulator->getPendingVehicles();
    spawnedVehicles.insert(
        spawnedVehicles.end(),
        pendingVehicles.begin(),
        pendingVehicles.end());
    for (Vehicle* vehicle : spawnedVehicles) {
        if (dynamic_cast<Bus*>(vehicle) != nullptr) {
            ++busCount;
        } else if (dynamic_cast<Motorbike*>(vehicle) != nullptr) {
            ++motorbikeCount;
        } else if (dynamic_cast<EmergencyVehicle*>(vehicle) != nullptr) {
            ++emergencyCount;
        } else if (dynamic_cast<Car*>(vehicle) != nullptr) {
            ++carCount;
        }
    }

    const int spawnedCount =
        carCount + motorbikeCount + busCount + emergencyCount;
    assert(spawnedCount == 1000);
    assert(simulator->getPendingVehicleCount() > 0);
    assert(busCount >= 45 && busCount <= 55);
    assert(emergencyCount >= 35 && emergencyCount <= 45);
    assert(carCount > busCount);
    assert(motorbikeCount > busCount);

    Graph map4Graph;
    std::string map4Error;
    std::string map4Path = "map4.json";
    if (!std::filesystem::exists(map4Path)) {
        map4Path = "../map4.json";
    }
    assert(MapLoad::loadGraphFromJsonFile(map4Path, map4Graph, &map4Error));
    assert(std::fabs(
        map4Graph.getRenderSettings().functionalMarkerScale -
        1.0f) < 0.001f);
    assert(std::fabs(
        map4Graph.getRenderSettings().minimumLaneWidthPixels) <
        0.001f);
    assert(std::fabs(
        map4Graph.getRenderSettings().minimumVehicleLengthPixels -
        4.0f) < 0.001f);
    assert(std::fabs(
        map4Graph.getRenderSettings().followZoomFactor -
        0.10f) < 0.001f);

    VisualizationEngine map4Engine({800u, 600u});
    map4Engine.prepare(map4Graph);
    sf::View map4CloseView(sf::FloatRect(0.0f, 0.0f, 800.0f, 600.0f));
    map4CloseView.zoom(0.2f);
    assert(std::fabs(
        map4Engine.getViewUnitsPerPixel(map4CloseView) - 0.2f) <
        0.001f);
    const float clampedCloseMarker =
        map4Engine.clampWorldSizeToPixels(
            map4CloseView,
            8.0f,
            1.5f,
            18.0f);
    assert(std::fabs(
        map4Engine.worldSizeToPixels(
            map4CloseView,
            clampedCloseMarker) -
        18.0f) < 0.001f);
    sf::View map4FarView(
        sf::FloatRect(0.0f, 0.0f, 800.0f, 600.0f));
    map4FarView.zoom(8.0f);
    const float clampedFarMarker =
        map4Engine.clampWorldSizeToPixels(
            map4FarView,
            8.0f,
            1.5f,
            18.0f);
    assert(std::fabs(
        map4Engine.worldSizeToPixels(
            map4FarView,
            clampedFarMarker) -
        1.5f) < 0.001f);
    const float closeFunctionalMarker =
        map4Engine.getFunctionalMarkerSize(
            map4CloseView,
            8.0f,
            1.5f,
            18.0f);
    const float farFunctionalMarker =
        map4Engine.getFunctionalMarkerSize(
            map4FarView,
            8.0f,
            1.5f,
            18.0f);
    assert(std::fabs(closeFunctionalMarker - 8.0f) < 0.001f);
    assert(std::fabs(farFunctionalMarker - 8.0f) < 0.001f);
    assert(
        map4Engine.worldSizeToPixels(
            map4CloseView,
            closeFunctionalMarker) >
        map4Engine.worldSizeToPixels(
            map4FarView,
            farFunctionalMarker));
    assert(std::fabs(
        map4Engine.getTextRenderScale(map4CloseView) - 1.0f) < 0.001f);
    sf::RenderTexture map4Target;
    assert(map4Target.create(800u, 600u));
    map4Target.clear(sf::Color(30, 30, 30));
    map4Engine.drawGraph(map4Target, map4Graph);
    map4Target.display();
    const sf::Image map4Image = map4Target.getTexture().copyToImage();
    const int map4Components = countColorComponents(
        map4Image, sf::Color(35, 145, 230), 20);
    assert(map4Components >= 4);

    // Regression: Medium LOD used to hide every functional map marker. map4
    // must retain its bus stops (and, through the same branch, stations,
    // POIs and traffic lights) when LOD drops from Full to Medium.
    map4Engine.setLodMode(VisualizationEngine::LodMode::Medium);
    sf::View mediumView = map4Target.getDefaultView();
    mediumView.zoom(2.0f);
    map4Target.setView(mediumView);
    map4Target.clear(sf::Color(30, 30, 30));
    map4Engine.drawGraph(map4Target, map4Graph);
    map4Target.display();
    assert(map4Engine.getLodLevel() ==
           VisualizationEngine::LodLevel::Medium);
    const sf::Image map4MediumImage =
        map4Target.getTexture().copyToImage();
    const int map4MediumComponents = countColorComponents(
        map4MediumImage, sf::Color(35, 145, 230), 1);
    assert(map4MediumComponents >= 4);

    // Large maps must still fit the prepared viewport. A previous scaling
    // shortcut multiplied every coordinate by 10, which made the static
    // cache cover only a small slice of the graph.
    Graph denseGraph;
    for (int index = 0; index < 500; ++index) {
        denseGraph.addIntersection(new Intersection(
            10000 + index,
            static_cast<double>(index),
            static_cast<double>(index % 25)));
    }
    VisualizationEngine denseEngine({800u, 600u});
    denseEngine.prepare(denseGraph);
    sf::View denseCloseView(
        sf::FloatRect(0.0f, 0.0f, 800.0f, 600.0f));
    denseCloseView.zoom(0.2f);
    assert(std::fabs(
        denseEngine.getTextRenderScale(denseCloseView) - 0.2f) < 0.001f);
    const auto& densePoints = denseEngine.getRoutePoints();
    const auto denseMinMaxX = std::minmax_element(
        densePoints.begin(), densePoints.end(),
        [](const sf::Vector2f& lhs, const sf::Vector2f& rhs) {
            return lhs.x < rhs.x;
        });
    assert(denseMinMaxX.second->x - denseMinMaxX.first->x <= 704.5f);

    // Manual caps are strict even at extreme zoom, and graph-density detail
    // is invariant under a proportional window resize.
    denseEngine.setLodMode(VisualizationEngine::LodMode::Low);
    sf::RenderTexture denseTarget;
    assert(denseTarget.create(800u, 600u));
    sf::View closeView = denseTarget.getDefaultView();
    closeView.zoom(0.1f);
    denseTarget.setView(closeView);
    denseEngine.drawDynamicLayer(denseTarget, denseGraph);
    assert(denseEngine.getLodLevel() ==
           VisualizationEngine::LodLevel::Low);

    denseEngine.setLodMode(VisualizationEngine::LodMode::Medium);
    denseEngine.drawDynamicLayer(denseTarget, denseGraph);
    assert(denseEngine.getLodLevel() ==
           VisualizationEngine::LodLevel::Medium);

    sf::View initialOverview(sf::FloatRect(0.0f, 0.0f, 800.0f, 600.0f));
    const float initialDetail = denseEngine.getDetailScale(initialOverview);
    denseEngine.setWindowSize({1600u, 1200u});
    denseEngine.prepare(denseGraph);
    sf::View resizedOverview(
        sf::FloatRect(0.0f, 0.0f, 1600.0f, 1200.0f));
    assert(std::fabs(
        denseEngine.getDetailScale(resizedOverview) - initialDetail) <
        0.001f);

    // Exercise the real dense-map label/marker path at close zoom. This
    // catches regressions in screen-space placement and viewport culling
    // without writing a diagnostic image into the repository.
    Graph vnuGraph;
    std::string vnuError;
    std::string vnuPath = "map_vnu_hcm_filtered.json";
    if (!std::filesystem::exists(vnuPath)) {
        vnuPath = "../map_vnu_hcm_filtered.json";
    }
    assert(MapLoad::loadGraphFromJsonFile(vnuPath, vnuGraph, &vnuError));
    assert(std::fabs(
        vnuGraph.getRenderSettings().functionalMarkerScale -
        0.3f) < 0.001f);
    assert(std::fabs(
        vnuGraph.getRenderSettings().minimumLaneWidthPixels -
        1.0f) < 0.001f);
    assert(std::fabs(
        vnuGraph.getRenderSettings().minimumVehicleLengthPixels -
        12.0f) < 0.001f);
    assert(std::fabs(
        vnuGraph.getRenderSettings().followZoomFactor -
        0.0000002f) < 0.00000001f);
    VisualizationEngine vnuEngine({800u, 600u});
    vnuEngine.prepare(vnuGraph);
    assert(std::fabs(
        vnuEngine.getMinimumVehicleLengthPixels() - 12.0f) <
        0.001f);
    vnuEngine.setLodMode(VisualizationEngine::LodMode::Full);
    sf::View vnuCloseView = denseTarget.getDefaultView();
    vnuCloseView.zoom(0.2f);
    denseTarget.setView(vnuCloseView);
    denseTarget.clear(sf::Color(30, 30, 30));
    vnuEngine.drawDynamicLayer(denseTarget, vnuGraph);
    denseTarget.display();
    assert(vnuEngine.getLodLevel() ==
           VisualizationEngine::LodLevel::Full);

    std::cout << "Visualization tests passed" << std::endl;
    return 0;
}
