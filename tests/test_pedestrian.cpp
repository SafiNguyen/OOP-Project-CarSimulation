#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "Mapload.h"
#include "algorithm/DijkstraStrategy.h"
#include "Car.h"
#include "Crosswalk.h"
#include "Graph.h"
#include "Intersection.h"
#include "Pedestrian.h"
#include "PedestrianRoute.h"
#include "Road.h"
#include "RoadGeometry.h"
#include "TrafficLight.h"
#include "simulation/TrafficSimulator.h"
#include "simulation/StatisticsManager.h"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "[FAIL] " << message << '\n';
    }
}

bool near(double first,
          double second,
          double tolerance = 1e-5) {
    return std::fabs(first - second) <= tolerance;
}

struct SignalizedCrosswalkFixture {
    Graph graph;
    Intersection* start = nullptr;
    Intersection* end = nullptr;
    Road* incoming = nullptr;
    Road* reverse = nullptr;
    Crosswalk* crosswalk = nullptr;

    explicit SignalizedCrosswalkFixture(
        CrosswalkTiming timing = {}) {
        graph.addIntersection(
            new Intersection(1, 0.0, 0.0));
        graph.addIntersection(
            new Intersection(2, 100.0, 0.0));
        start = graph.getIntersection(1);
        end = graph.getIntersection(2);
        graph.addRoad(new Road(
            101,
            "Incoming",
            start,
            end,
            100.0,
            15.0,
            1.0,
            1));
        graph.addRoad(new Road(
            -101,
            "Outgoing",
            end,
            start,
            100.0,
            15.0,
            1.0,
            1));
        incoming = graph.getRoad(101);
        reverse = graph.getRoad(-101);
        std::string error;
        check(
            end->configureTrafficSignalsAutomatically(
                1.0,
                0.5,
                0.5,
                &error),
            "fixture signal plan configures: " +
                error);
        auto crossing = std::make_unique<Crosswalk>(
            301,
            end,
            incoming,
            3.0,
            timing);
        crosswalk = crossing.get();
        check(
            graph.addCrosswalk(
                std::move(crossing)),
            "fixture crosswalk is owned by graph");
    }
};

class PositionedCar : public Car {
public:
    using Car::Car;

    void place(double progress, double speed) {
        progressOnCurrentRoad = progress;
        currentSpeed = speed;
    }
};

void testPedestrianLifecycle() {
    CrosswalkTiming timing;
    timing.minimumWaitSeconds = 0.5;
    timing.walkDurationSeconds = 1.0;
    timing.designWalkingSpeedMetresPerSecond =
        1.0;
    timing.clearanceBufferSeconds = 0.2;
    SignalizedCrosswalkFixture fixture(timing);

    auto route = buildCrosswalkJourney(
        *fixture.crosswalk,
        CrossingDirection::SideAToB,
        2.0,
        2.0);
    Pedestrian pedestrian(
        1,
        2.0,
        std::move(route));
    check(
        pedestrian.isValid(),
        "pedestrian accepts sidewalk-crosswalk-sidewalk route");
    const Pose2D initial = pedestrian.getPose();

    pedestrian.update(0.5);
    check(
        pedestrian.getState() ==
            PedestrianState::Walking,
        "pedestrian walks on sidewalk before reaching curb");
    check(
        distance(
            initial.position,
            pedestrian.getPose().position) >
            0.1,
        "sidewalk walking changes the model-space pose");

    pedestrian.update(0.5);
    check(
        pedestrian.getState() ==
            PedestrianState::WaitingToCross,
        "pedestrian waits after completing approach sidewalk");
    pedestrian.update(0.49);
    fixture.crosswalk->synchronizeSignalState(
        PedestrianSignalState::Walk);
    fixture.crosswalk->grantEligiblePedestrians();
    check(
        pedestrian.getState() ==
            PedestrianState::WaitingToCross,
        "minimum request wait prevents an early crossing");

    pedestrian.update(0.01);
    fixture.crosswalk->grantEligiblePedestrians();
    check(
        pedestrian.getState() ==
            PedestrianState::Crossing &&
            fixture.crosswalk->isOccupied(),
        "eligible pedestrian enters only during WALK");

    const double crossingTime =
        fixture.crosswalk->
            getCrossingLengthMetres() /
        pedestrian.getWalkingSpeed();
    fixture.crosswalk->synchronizeSignalState(
        PedestrianSignalState::Clearance);

    auto lateRoute = buildCrosswalkJourney(
        *fixture.crosswalk,
        CrossingDirection::SideBToA,
        0.5,
        1.0);
    Pedestrian latePedestrian(
        3,
        0.5,
        std::move(lateRoute));
    latePedestrian.update(1.0);
    latePedestrian.update(0.5);
    fixture.crosswalk->grantEligiblePedestrians();
    check(
        latePedestrian.getState() ==
            PedestrianState::WaitingToCross,
        "pedestrian arriving during CLEARANCE waits for the next WALK");

    pedestrian.update(crossingTime + 0.01);
    check(
        pedestrian.getState() ==
            PedestrianState::Walking &&
            !fixture.crosswalk->isOccupied(),
        "pedestrian continues through CLEARANCE and exits occupancy");

    pedestrian.update(2.0);
    check(
        pedestrian.hasArrived() &&
            pedestrian.getTotalWalkingTimeSeconds() > 0.0 &&
            pedestrian.getTotalWaitingTimeSeconds() >= 0.5 &&
            pedestrian.getTotalCrossingTimeSeconds() > 0.0,
        "pedestrian walks after crossing and completes the journey");
}

void testSharedSignalControllerAndSafetyHold() {
    CrosswalkTiming timing;
    timing.minimumWaitSeconds = 0.5;
    timing.walkDurationSeconds = 1.0;
    timing.designWalkingSpeedMetresPerSecond =
        1.0;
    timing.clearanceBufferSeconds = 0.1;
    SignalizedCrosswalkFixture fixture(timing);
    DijkstraStrategy strategy;
    TrafficSimulator simulator(
        &fixture.graph,
        &strategy);

    auto route = buildCrosswalkJourney(
        *fixture.crosswalk,
        CrossingDirection::SideAToB,
        0.5,
        1.0);
    check(
        simulator.addPedestrian(
            std::make_unique<Pedestrian>(
                2,
                0.5,
                std::move(route))),
        "simulator accepts an ownership-safe pedestrian");

    bool sawYellow = false;
    bool sawAllRed = false;
    bool sawWalk = false;
    bool sawClearance = false;
    bool redThroughoutPedestrianPhase = true;
    bool heldClearanceWhileOccupied = false;

    for (int step = 0; step < 1000; ++step) {
        simulator.update(0.05);
        const SignalStage stage =
            fixture.end->getSignalStage();
        sawYellow = sawYellow ||
                    stage == SignalStage::YELLOW;
        sawAllRed = sawAllRed ||
                    stage == SignalStage::ALL_RED;
        sawWalk = sawWalk ||
                  stage ==
                      SignalStage::PEDESTRIAN_WALK;
        sawClearance = sawClearance ||
                       stage ==
                           SignalStage::
                               PEDESTRIAN_CLEARANCE;
        if (stage ==
                SignalStage::PEDESTRIAN_WALK ||
            stage ==
                SignalStage::
                    PEDESTRIAN_CLEARANCE) {
            const TrafficLight* light =
                fixture.end->
                    getLightForIncomingRoad(
                        fixture.incoming);
            redThroughoutPedestrianPhase =
                redThroughoutPedestrianPhase &&
                light != nullptr &&
                light->getState() ==
                    LightState::RED;
        }
        if (stage ==
                SignalStage::
                    PEDESTRIAN_CLEARANCE &&
            fixture.crosswalk->isOccupied() &&
            fixture.end->
                    getSignalStageRemainingSeconds() <=
                1e-8) {
            heldClearanceWhileOccupied = true;
        }
    }

    check(
        sawYellow && sawAllRed &&
            sawWalk && sawClearance,
        "shared signal follows GREEN-YELLOW-ALL_RED-WALK-CLEARANCE");
    check(
        redThroughoutPedestrianPhase,
        "all vehicle signal heads remain red during pedestrian stages");
    check(
        heldClearanceWhileOccupied,
        "controller holds red after timed clearance while crossing is occupied");
    check(
        !simulator.getFinishedPedestrians().empty(),
        "simulator moves arrived pedestrian to completed ownership");
    const StatisticsSummary summary =
        simulator.getStatisticsManager()->getSummary();
    check(
        summary.totalPedestriansTracked == 1 &&
            summary.completedPedestrianTrips == 1 &&
            summary.averagePedestrianWaitSeconds >=
                timing.minimumWaitSeconds,
        "statistics expose completed pedestrian trip and crossing wait");
}

void testVehicleStopsBeforeCrosswalk() {
    CrosswalkTiming timing;
    timing.minimumWaitSeconds = 0.0;
    SignalizedCrosswalkFixture fixture(timing);
    PositionedCar car(
        10,
        12.0,
        fixture.start,
        fixture.end);
    car.setRoute({fixture.incoming});

    fixture.crosswalk->synchronizeSignalState(
        PedestrianSignalState::Walk);
    const double lineProgress =
        RoadGeometry::stopLineProgressMetres(
            *fixture.incoming);
    const double expectedCentreStop =
        lineProgress - car.getLength() * 0.5;
    car.place(
        expectedCentreStop - 0.5,
        8.0);
    car.update(1.0);

    check(
        car.getProgressOnRoad() <=
            expectedCentreStop + 1e-6,
        "vehicle centre stops with front bumper behind crosswalk stop line");
    check(
        car.getPauseReason() ==
            PauseReason::PedestrianCrossing,
        "vehicle exposes PedestrianCrossing pause reason");
    check(
        lineProgress <
            fixture.crosswalk->
                getStartProgressMetres(),
        "rendered stop line is upstream of zebra stripes");
}

void testPauseAndMapLoading() {
    CrosswalkTiming timing;
    timing.minimumWaitSeconds = 0.5;
    SignalizedCrosswalkFixture fixture(timing);
    DijkstraStrategy strategy;
    TrafficSimulator simulator(
        &fixture.graph,
        &strategy);
    auto route = buildCrosswalkJourney(
        *fixture.crosswalk,
        CrossingDirection::SideAToB,
        4.0,
        2.0);
    auto pedestrian = std::make_unique<Pedestrian>(
        20,
        1.0,
        std::move(route));
    const Vec2 initial =
        pedestrian->getPose().position;
    check(
        simulator.addPedestrian(
            std::move(pedestrian)),
        "pause fixture pedestrian is added");
    simulator.pause();
    simulator.update(2.0);
    check(
        near(
            distance(
                initial,
                simulator.getPedestrians().
                    front()->getPose().position),
            0.0),
        "paused simulator freezes pedestrian movement and timers");
    simulator.resume();
    simulator.update(0.1);
    check(
        distance(
            initial,
            simulator.getPedestrians().
                front()->getPose().position) >
            0.0,
        "resumed simulator advances pedestrian");

    const std::string validJson = R"JSON({
        "coordinateUnit": "m",
        "defaultDistanceUnit": "m",
        "defaultSpeedUnit": "m/s",
        "intersections": [
            {"id": 1, "x": 0, "y": 0},
            {"id": 2, "x": 100, "y": 0}
        ],
        "roads": [
            {"id": 101, "start": 1, "end": 2,
             "distance": 100, "speedLimit": 15,
             "twoWay": true}
        ],
        "trafficLights": [
            {"intersectionId": 2, "greenDuration": 5,
             "yellowDuration": 1, "allRedDuration": 1}
        ],
        "crosswalks": [
            {"id": 301, "intersectionId": 2,
             "incomingRoadId": 101, "width": 3,
             "minimumWait": 3, "walkDuration": 4,
             "designWalkingSpeed": 1.2,
             "clearanceBuffer": 1}
        ]
    })JSON";
    Graph loaded;
    std::string error;
    check(
        MapLoad::loadGraphFromJsonString(
            validJson,
            loaded,
            &error) &&
            loaded.getCrosswalk(301) != nullptr,
        "valid optional crosswalk JSON loads: " +
            error);

    const std::string legacyJson = R"JSON({
        "defaultDistanceUnit": "m",
        "defaultSpeedUnit": "m/s",
        "intersections": [
            {"id": 1, "x": 0, "y": 0},
            {"id": 2, "x": 100, "y": 0}
        ],
        "roads": [
            {"id": 101, "start": 1, "end": 2,
             "distance": 100, "speedLimit": 15,
             "twoWay": true}
        ],
        "trafficLights": [
            {"intersectionId": 2}
        ]
    })JSON";
    Graph legacy;
    error.clear();
    check(
        MapLoad::loadGraphFromJsonString(
            legacyJson,
            legacy,
            &error),
        "map without crosswalks remains backward compatible: " +
            error);

    std::string invalidJson = validJson;
    const std::string validRoad =
        "\"incomingRoadId\": 101";
    const std::size_t roadField =
        invalidJson.find(validRoad);
    invalidJson.replace(
        roadField,
        validRoad.size(),
        "\"incomingRoadId\": -101");
    Graph invalid;
    error.clear();
    check(
        !MapLoad::loadGraphFromJsonString(
            invalidJson,
            invalid,
            &error) &&
            error.find("does not end") !=
                std::string::npos,
        "loader rejects a crosswalk whose road is not incoming");
}

void testDemandAndSpeedMultiplier() {
    SignalizedCrosswalkFixture idleFixture;
    for (int step = 0; step < 200; ++step) {
        idleFixture.end->updateTrafficLights(0.05);
        check(
            idleFixture.end->getSignalStage() !=
                    SignalStage::PEDESTRIAN_WALK &&
                idleFixture.end->getSignalStage() !=
                    SignalStage::PEDESTRIAN_CLEARANCE,
            "controller does not insert a pedestrian phase without demand");
    }

    SignalizedCrosswalkFixture normalFixture;
    SignalizedCrosswalkFixture acceleratedFixture;
    DijkstraStrategy normalStrategy;
    DijkstraStrategy acceleratedStrategy;
    TrafficSimulator normal(
        &normalFixture.graph,
        &normalStrategy);
    TrafficSimulator accelerated(
        &acceleratedFixture.graph,
        &acceleratedStrategy);

    auto normalRoute = buildCrosswalkJourney(
        *normalFixture.crosswalk,
        CrossingDirection::SideAToB,
        4.0,
        1.0);
    auto acceleratedRoute = buildCrosswalkJourney(
        *acceleratedFixture.crosswalk,
        CrossingDirection::SideAToB,
        4.0,
        1.0);
    check(
        normal.addPedestrian(
            std::make_unique<Pedestrian>(
                30,
                1.0,
                std::move(normalRoute))) &&
            accelerated.addPedestrian(
                std::make_unique<Pedestrian>(
                    31,
                    1.0,
                    std::move(acceleratedRoute))),
        "speed-multiplier fixtures accept pedestrians");

    accelerated.setSpeedMultiplier(2.0);
    normal.update(0.1);
    accelerated.update(0.05);
    check(
        near(
            normal.getElapsedTime(),
            accelerated.getElapsedTime()) &&
            near(
                distance(
                    normal.getPedestrians().front()->
                        getPose().position,
                    accelerated.getPedestrians().front()->
                        getPose().position),
                0.0),
        "speed multiplier advances pedestrian movement and simulation time consistently");
}

} // namespace

int main() {
    testPedestrianLifecycle();
    testSharedSignalControllerAndSafetyHold();
    testVehicleStopsBeforeCrosswalk();
    testPauseAndMapLoading();
    testDemandAndSpeedMultiplier();

    if (failures == 0) {
        std::cout
            << "Pedestrian and crosswalk tests passed"
            << std::endl;
        return 0;
    }
    std::cerr << failures
              << " pedestrian/crosswalk test(s) failed"
              << std::endl;
    return 1;
}
