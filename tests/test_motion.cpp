#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Mapload.h"
#include "Bus.h"
#include "Car.h"
#include "EmergencyVehicle.h"
#include "Graph.h"
#include "Intersection.h"
#include "JunctionConnector.h"
#include "LaneMapping.h"
#include "MotionPath.h"
#include "Road.h"
#include "RoadGeometry.h"
#include "Roundabout.h"
#include "visualization/SimulatorFactory.h"
#include "algorithm/DijkstraStrategy.h"
#include "simulation/TrafficSimulator.h"

namespace {

constexpr double EPSILON = 1e-5;
int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "[FAIL] " << message << '\n';
    }
}

bool near(double a, double b, double tolerance = EPSILON) {
    return std::fabs(a - b) <= tolerance;
}

bool finitePose(const Pose2D& pose) {
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.headingRadians) &&
           std::isfinite(pose.curvature);
}

struct VehicleBounds {
    Vec2 centre;
    Vec2 forward;
    Vec2 side;
    double halfLength = 0.0;
    double halfWidth = 0.0;
};

VehicleBounds boundsOf(const Vehicle& vehicle) {
    const Pose2D pose = vehicle.getPose();
    const Road* road = vehicle.getCurrentRoad();
    const double metresPerWorldUnit =
        road != nullptr
            ? RoadGeometry::metresPerWorldUnit(*road)
            : 1.0;
    const Vec2 forward{
        std::cos(pose.headingRadians),
        std::sin(pose.headingRadians)
    };
    return {
        pose.position,
        forward,
        rightNormal(forward),
        vehicle.getLength() * 0.5 /
            metresPerWorldUnit,
        vehicle.getWidth() * 0.5 /
            metresPerWorldUnit
    };
}

bool overlap(const VehicleBounds& first,
             const VehicleBounds& second) {
    const Vec2 delta = second.centre - first.centre;
    const Vec2 axes[] = {
        first.forward, first.side,
        second.forward, second.side
    };
    for (const Vec2& axis : axes) {
        const double firstRadius =
            first.halfLength *
                std::fabs(dot(first.forward, axis)) +
            first.halfWidth *
                std::fabs(dot(first.side, axis));
        const double secondRadius =
            second.halfLength *
                std::fabs(dot(second.forward, axis)) +
            second.halfWidth *
                std::fabs(dot(second.side, axis));
        if (std::fabs(dot(delta, axis)) + 1e-8 >=
            firstRadius + secondRadius) {
            return false;
        }
    }
    return true;
}

class MotionTestCar : public Car {
public:
    using Car::Car;
    void place(double progress, double speed, int lane = 0) {
        if (currentRoad != nullptr &&
            currentLaneIndex != lane) {
            currentRoad->getLane(currentLaneIndex).removeVehicle(this);
            currentLaneIndex = lane;
            currentRoad->getLane(currentLaneIndex).addVehicle(this);
        }
        progressOnCurrentRoad = progress;
        currentSpeed = speed;
    }
};

class MotionTestEmergency : public EmergencyVehicle {
public:
    using EmergencyVehicle::EmergencyVehicle;
    void place(double progress, double speed, int lane = 0) {
        if (currentRoad != nullptr &&
            currentLaneIndex != lane) {
            currentRoad->getLane(currentLaneIndex).removeVehicle(this);
            currentLaneIndex = lane;
            currentRoad->getLane(currentLaneIndex).addVehicle(this);
        }
        progressOnCurrentRoad = progress;
        currentSpeed = speed;
    }
};

void testBezierGeometry() {
    BezierJunctionPath curve(
        {0.0, 0.0}, {10.0, 0.0},
        {10.0, 10.0}, {20.0, 10.0}, 1.0, 64);
    check(curve.getLength() > 0.0, "Bezier length is positive");

    const Pose2D start = curve.sampleByDistance(0.0);
    const Pose2D finish =
        curve.sampleByDistance(curve.getLength());
    check(near(start.position.x, 0.0) &&
          near(start.position.y, 0.0),
          "Bezier starts at P0");
    check(near(finish.position.x, 20.0) &&
          near(finish.position.y, 10.0),
          "Bezier ends at P3");
    check(std::fabs(start.headingRadians) < 1e-6 &&
          std::fabs(finish.headingRadians) < 1e-6,
          "Bezier endpoint tangents match controls");

    double previousX = -1.0;
    for (int index = 0; index <= 100; ++index) {
        const double s =
            curve.getLength() * index / 100.0;
        const Pose2D pose = curve.sampleByDistance(s);
        check(finitePose(pose), "Bezier samples stay finite");
        check(pose.position.x + 1e-9 >= previousX,
              "Bezier arc-length sampling is monotone");
        previousX = pose.position.x;

        if (index > 0 && index < 100) {
            const double delta = curve.getLength() * 1e-4;
            const Pose2D before =
                curve.sampleByDistance(s - delta);
            const Pose2D after =
                curve.sampleByDistance(s + delta);
            const double numericalHeading = std::atan2(
                after.position.y - before.position.y,
                after.position.x - before.position.x);
            check(std::fabs(
                      std::remainder(
                          pose.headingRadians - numericalHeading,
                          2.0 * 3.14159265358979323846)) < 2e-2,
                  "Heading follows the path derivative");
        }
    }

    check(near(curve.sampleByDistance(-10.0).position.x, 0.0) &&
          near(curve.sampleByDistance(1e9).position.x, 20.0),
          "Bezier distance sampling clamps outside its domain");

    BezierJunctionPath degenerate(
        {1.0, 2.0}, {1.0, 2.0},
        {1.0, 2.0}, {1.0, 2.0});
    check(finitePose(degenerate.sampleByDistance(0.0)),
          "Degenerate Bezier is safe");
}

void testLaneMapping() {
    Intersection west(1, -100.0, 0.0);
    Intersection centre(2, 0.0, 0.0);
    Intersection east(3, 100.0, 0.0);
    Intersection north(4, 0.0, 100.0);
    Intersection south(5, 0.0, -100.0);

    Road incoming1(10, "in1", &west, &centre, 100.0, 20.0, 1.0, 1);
    Road incoming2(11, "in2", &west, &centre, 100.0, 20.0, 1.0, 2);
    Road incoming3(12, "in3", &west, &centre, 100.0, 20.0, 1.0, 3);
    Road straight1(20, "s1", &centre, &east, 100.0, 20.0, 1.0, 1);
    Road straight2(21, "s2", &centre, &east, 100.0, 20.0, 1.0, 2);
    Road straight3(22, "s3", &centre, &east, 100.0, 20.0, 1.0, 3);
    Road left(23, "left", &centre, &north, 100.0, 20.0, 1.0, 2);
    Road right(24, "right", &centre, &south, 100.0, 20.0, 1.0, 2);
    Road reverse(25, "reverse", &centre, &west, 100.0, 20.0, 1.0, 2);

    check(TurnLanePolicy::map(incoming1, 0, straight1).outgoingLane == 0,
          "Lane mapping handles 1 -> 1");
    check(TurnLanePolicy::map(incoming1, 0, straight2).outgoingLane == 0,
          "Lane mapping handles 1 -> 2 deterministically");
    check(TurnLanePolicy::map(incoming2, 1, straight1).outgoingLane == 0,
          "Lane mapping handles 2 -> 1");
    check(TurnLanePolicy::map(incoming2, 1, straight3).outgoingLane == 2,
          "Lane mapping handles 2 -> 3 by relative position");
    check(TurnLanePolicy::map(incoming3, 2, straight2).outgoingLane == 1,
          "Lane mapping handles 3 -> 2 by relative position");

    const LaneMapping leftMap =
        TurnLanePolicy::map(incoming2, 1, left);
    const LaneMapping rightMap =
        TurnLanePolicy::map(incoming2, 0, right);
    check(leftMap.movement == MovementType::Left &&
          leftMap.incomingLane == 0 &&
          leftMap.outgoingLane == 0,
          "Left turns use median lanes");
    check(rightMap.movement == MovementType::Right &&
          rightMap.incomingLane == 1 &&
          rightMap.outgoingLane == 1,
          "Right turns use curb lanes");
    check(TurnLanePolicy::map(
              incoming2, 0, reverse, false).valid == false,
          "U-turn policy can reject a prohibited U-turn");

    straight2.blockLane(1);
    const LaneMapping blocked =
        TurnLanePolicy::map(incoming2, 1, straight2);
    check(blocked.valid && blocked.outgoingLane == 0,
          "Blocked destination lane maps to the nearest open lane");
}

struct JunctionFixture {
    Graph graph;
    Road* westIn = nullptr;
    Road* northOut = nullptr;
    Road* southIn = nullptr;
    Road* eastOut = nullptr;

    JunctionFixture() {
        graph.addIntersection(new Intersection(1, -100.0, 0.0));
        graph.addIntersection(new Intersection(2, 0.0, 0.0));
        graph.addIntersection(new Intersection(3, 0.0, 100.0));
        graph.addIntersection(new Intersection(4, 0.0, -100.0));
        graph.addIntersection(new Intersection(5, 100.0, 0.0));
        graph.addRoad(new Road(
            10, "west-in", graph.getIntersection(1),
            graph.getIntersection(2), 100.0, 20.0));
        graph.addRoad(new Road(
            11, "north-out", graph.getIntersection(2),
            graph.getIntersection(3), 100.0, 20.0));
        graph.addRoad(new Road(
            12, "south-in", graph.getIntersection(4),
            graph.getIntersection(2), 100.0, 20.0));
        graph.addRoad(new Road(
            13, "east-out", graph.getIntersection(2),
            graph.getIntersection(5), 100.0, 20.0));
        westIn = graph.getRoad(10);
        northOut = graph.getRoad(11);
        southIn = graph.getRoad(12);
        eastOut = graph.getRoad(13);
    }
};

void testConnectorCacheAndVehicleLifecycle() {
    JunctionFixture fixture;
    Intersection* centre = fixture.graph.getIntersection(2);
    auto connectorA = centre->getConnector(
        fixture.westIn, 0, fixture.northOut, 0);
    auto connectorB = centre->getConnector(
        fixture.westIn, 0, fixture.northOut, 0);
    check(connectorA != nullptr && connectorA == connectorB &&
          centre->getConnectorCacheSize() == 1,
          "Connector geometry is cached by movement key");

    MotionTestCar first(
        1, 20.0, fixture.graph.getIntersection(1),
        fixture.graph.getIntersection(3));
    first.setRoute({fixture.westIn, fixture.northOut});
    first.place(99.0, 20.0);
    first.update(0.1);
    check(first.getMovementState() ==
              MovementState::TraversingJunction &&
          first.getCurrentRoad() == fixture.westIn &&
          first.getCurrentRouteIndex() == 0 &&
          fixture.westIn->getLane(0).getVehicleCount() == 0 &&
          centre->isFull(),
          "Vehicle retains incoming road identity while traversing");

    MotionTestEmergency emergency(
        2, 30.0, fixture.graph.getIntersection(4),
        fixture.graph.getIntersection(5));
    emergency.setRoute({fixture.southIn, fixture.eastOut});
    emergency.place(99.0, 20.0);
    emergency.update(0.2);
    check(emergency.getMovementState() ==
              MovementState::WaitingAtIntersection &&
          emergency.getCurrentRoad() == fixture.southIn,
          "Emergency preemption never bypasses an occupied connector");

    for (int tick = 0;
         tick < 1000 &&
         first.getMovementState() ==
             MovementState::TraversingJunction;
         ++tick) {
        first.update(1.0 / 60.0);
    }
    check(first.getCurrentRoad() == fixture.northOut &&
          first.getCurrentRouteIndex() == 1 &&
          first.getMovementState() == MovementState::OnRoad &&
          !centre->isFull() &&
          fixture.northOut->getLane(0).getVehicleCount() == 1,
          "Traversal completion switches lane membership and releases reservation");

    emergency.update(0.2);
    check(emergency.getMovementState() !=
              MovementState::WaitingAtIntersection,
          "Conflicting vehicle resumes after reservation release");
    for (int tick = 0;
         tick < 1000 &&
         emergency.getMovementState() ==
             MovementState::TraversingJunction;
         ++tick) {
        emergency.update(1.0 / 60.0);
    }

    {
        JunctionFixture destructionFixture;
        Intersection* destructionCentre =
            destructionFixture.graph.getIntersection(2);
        {
            MotionTestCar disposable(
                3, 20.0,
                destructionFixture.graph.getIntersection(1),
                destructionFixture.graph.getIntersection(3));
            disposable.setRoute(
                {destructionFixture.westIn,
                 destructionFixture.northOut});
            disposable.place(99.0, 20.0);
            disposable.update(0.1);
            check(destructionCentre->isFull(),
                  "Traversal owns a reservation before destruction");
        }
        check(!destructionCentre->isFull(),
              "Vehicle destruction releases an active reservation");
    }
}

void testEmergencyRedLightPriorityAndCaution() {
    Graph graph;
    graph.addIntersection(
        new Intersection(1, -100.0, 0.0));
    graph.addIntersection(
        new Intersection(2, 0.0, 0.0));
    graph.addIntersection(
        new Intersection(3, 0.0, -100.0));
    graph.addIntersection(
        new Intersection(4, 100.0, 0.0));
    graph.addIntersection(
        new Intersection(5, 0.0, 100.0));
    graph.addRoad(new Road(
        20, "west-in",
        graph.getIntersection(1),
        graph.getIntersection(2),
        100.0, 20.0));
    graph.addRoad(new Road(
        21, "south-in",
        graph.getIntersection(3),
        graph.getIntersection(2),
        100.0, 20.0));
    graph.addRoad(new Road(
        22, "east-out",
        graph.getIntersection(2),
        graph.getIntersection(4),
        100.0, 20.0));
    graph.addRoad(new Road(
        23, "north-out",
        graph.getIntersection(2),
        graph.getIntersection(5),
        100.0, 20.0));

    Road* westIn = graph.getRoad(20);
    Road* southIn = graph.getRoad(21);
    Road* eastOut = graph.getRoad(22);
    Road* northOut = graph.getRoad(23);
    Intersection* centre = graph.getIntersection(2);
    std::string signalError;
    check(
        centre->configureTrafficSignals(
            {{westIn}, {southIn}},
            30.0, 3.0, 2.0,
            &signalError),
        "emergency fixture signal plan configures: " +
            signalError);
    check(
        centre->mustStopForRoad(southIn),
        "emergency fixture starts with the ambulance approach red");

    MotionTestEmergency emergency(
        80, 25.0,
        graph.getIntersection(3),
        graph.getIntersection(5));
    emergency.setRoute({southIn, northOut});
    emergency.place(65.0, 25.0);

    MotionTestCar conflictingCar(
        81, 20.0,
        graph.getIntersection(1),
        graph.getIntersection(4));
    conflictingCar.setRoute({westIn, eastOut});
    conflictingCar.place(65.0, 15.0);

    emergency.update(0.1);
    const double emergencyCautiousSpeed =
        emergency.getCurrentSpeed();
    check(
        centre->hasActiveEmergencyPriority() &&
            emergencyCautiousSpeed > 20.0 &&
            emergencyCautiousSpeed < 25.0,
        "emergency keeps most of its speed while cautiously approaching");

    conflictingCar.update(1.0);
    check(
        conflictingCar.getCurrentSpeed() <= 10.1 &&
            emergencyCautiousSpeed >
                conflictingCar.getCurrentSpeed() * 2.0,
        "conflicting approach yields with a substantially lower speed");

    emergency.place(99.0, 8.0);
    emergency.update(0.5);
    check(
        centre->mustStopForRoad(southIn) &&
            emergency.getMovementState() ==
                MovementState::TraversingJunction,
        "prioritized emergency enters safely while its signal remains red");
}

void testSignalizedMultiLaneQueueDischarge() {
    {
        Graph graph;
        graph.addIntersection(new Intersection(1, -100.0, 0.0));
        graph.addIntersection(new Intersection(2, 0.0, 0.0));
        graph.addIntersection(new Intersection(3, 0.0, -100.0));
        graph.addIntersection(new Intersection(4, 0.0, 100.0));
        graph.addRoad(new Road(
            20, "west-in", graph.getIntersection(1),
            graph.getIntersection(2), 100.0, 15.0, 1.0, 2));
        graph.addRoad(new Road(
            21, "south-out", graph.getIntersection(2),
            graph.getIntersection(3), 100.0, 15.0, 1.0, 2));
        graph.addRoad(new Road(
            22, "north-in", graph.getIntersection(4),
            graph.getIntersection(2), 100.0, 15.0));

        Road* westIn = graph.getRoad(20);
        Road* southOut = graph.getRoad(21);
        Road* northIn = graph.getRoad(22);
        Intersection* centre = graph.getIntersection(2);
        std::string signalError;
        check(centre->configureTrafficSignals(
                  {{northIn}, {westIn}},
                  1.0, 0.1, 0.1, &signalError),
              "Two-phase queue-discharge signal config is valid");

        MotionTestCar inner(
            301, 15.0, graph.getIntersection(1),
            graph.getIntersection(3));
        inner.setRoute({westIn, southOut});
        inner.place(
            RoadGeometry::stopLineProgressMetres(*westIn) -
                inner.getLength() * 0.5,
            0.0,
            0);
        inner.update(0.1);
        check(inner.getMovementState() ==
                  MovementState::WaitingAtIntersection,
              "Inner-lane right turn waits while its approach is red");

        centre->updateTrafficLights(1.25);
        bool enteredFromInnerLane = false;
        for (int tick = 0; tick < 60; ++tick) {
            inner.update(0.1);
            enteredFromInnerLane =
                enteredFromInnerLane ||
                inner.getMovementState() ==
                    MovementState::TraversingJunction ||
                inner.getCurrentRoad() == southOut;
            if (enteredFromInnerLane) break;
        }
        check(enteredFromInnerLane,
              "Inner-lane queue can discharge after red without a forced lane-change deadlock");
    }

    {
        Graph graph;
        graph.addIntersection(new Intersection(1, -100.0, 0.0));
        graph.addIntersection(new Intersection(2, 0.0, 0.0));
        graph.addIntersection(new Intersection(3, 100.0, 0.0));
        graph.addRoad(new Road(
            30, "west-in", graph.getIntersection(1),
            graph.getIntersection(2), 100.0, 15.0, 1.0, 2));
        graph.addRoad(new Road(
            31, "east-out", graph.getIntersection(2),
            graph.getIntersection(3), 100.0, 15.0, 1.0, 2));

        Road* westIn = graph.getRoad(30);
        Road* eastOut = graph.getRoad(31);
        Intersection* centre = graph.getIntersection(2);
        MotionTestCar inner(
            302, 15.0, graph.getIntersection(1),
            graph.getIntersection(3));
        MotionTestCar outer(
            303, 15.0, graph.getIntersection(1),
            graph.getIntersection(3));
        inner.setRoute({westIn, eastOut});
        outer.setRoute({westIn, eastOut});
        const double stopProgress =
            RoadGeometry::stopLineProgressMetres(*westIn) -
            inner.getLength() * 0.5;
        inner.place(stopProgress, 0.0, 0);
        outer.place(stopProgress, 0.0, 1);

        bool traversingTogether = false;
        for (int tick = 0; tick < 60; ++tick) {
            outer.update(0.1);
            inner.update(0.1);
            if (inner.getMovementState() ==
                    MovementState::TraversingJunction &&
                outer.getMovementState() ==
                    MovementState::TraversingJunction) {
                traversingTogether = true;
                break;
            }
        }
        check(centre->getCapacity() == 2 && traversingTogether,
              "Two non-conflicting lanes receive independent intersection capacity");
    }
}

double runFrameRateScenario(double dt) {
    JunctionFixture fixture;
    MotionTestCar vehicle(
        7, 15.0, fixture.graph.getIntersection(1),
        fixture.graph.getIntersection(3));
    vehicle.setRoute({fixture.westIn, fixture.northOut});
    vehicle.place(90.0, 15.0);
    const int steps = static_cast<int>(std::lround(4.0 / dt));
    for (int step = 0; step < steps; ++step) {
        vehicle.update(dt);
    }
    return vehicle.getCurrentRoad() == fixture.northOut
        ? 100.0 + vehicle.getProgressOnRoad()
        : vehicle.getProgressOnRoad();
}

void testFrameRateIndependence() {
    const double at60 = runFrameRateScenario(1.0 / 60.0);
    const double at20 = runFrameRateScenario(1.0 / 20.0);
    check(std::fabs(at60 - at20) < 0.35,
          "Junction traversal is stable between 60 Hz and 20 Hz (" +
              std::to_string(at60) + " vs " +
              std::to_string(at20) + ")");
}

void testRoundaboutGeometryAndCapacity() {
    Graph graph;
    graph.addIntersection(new Intersection(1, -100.0, 0.0));
    graph.addIntersection(new Roundabout(2, 0.0, 0.0, 20.0));
    graph.addIntersection(new Intersection(3, 100.0, 0.0));
    graph.addIntersection(new Intersection(4, 0.0, -100.0));
    graph.addIntersection(new Intersection(5, 0.0, 100.0));
    graph.addRoad(new Road(
        10, "west-in", graph.getIntersection(1),
        graph.getIntersection(2), 100.0, 15.0));
    graph.addRoad(new Road(
        11, "east-out", graph.getIntersection(2),
        graph.getIntersection(3), 100.0, 15.0));
    graph.addRoad(new Road(
        12, "south-in", graph.getIntersection(4),
        graph.getIntersection(2), 100.0, 15.0));
    graph.addRoad(new Road(
        13, "north-out", graph.getIntersection(2),
        graph.getIntersection(5), 100.0, 15.0));
    graph.addRoad(new Road(
        14, "south-out", graph.getIntersection(2),
        graph.getIntersection(4), 100.0, 15.0));
    graph.addRoad(new Road(
        15, "west-out", graph.getIntersection(2),
        graph.getIntersection(1), 100.0, 15.0));

    Intersection* roundabout = graph.getIntersection(2);
    auto westToEast = roundabout->getConnector(
        graph.getRoad(10), 0, graph.getRoad(11), 0);
    auto southToNorth = roundabout->getConnector(
        graph.getRoad(12), 0, graph.getRoad(13), 0);
    auto westToNorth = roundabout->getConnector(
        graph.getRoad(10), 0, graph.getRoad(13), 0);
    auto westToSouth = roundabout->getConnector(
        graph.getRoad(10), 0, graph.getRoad(14), 0);
    auto westUTurn = roundabout->getConnector(
        graph.getRoad(10), 0, graph.getRoad(15), 0);
    check(westToEast != nullptr &&
          westToEast->getLength() > 60.0,
          "Roundabout connector contains a real circular traversal");

    double minimumRadius = 1e9;
    for (int index = 0; index <= 300; ++index) {
        const Pose2D pose = westToEast->sampleByDistance(
            westToEast->getLength() * index / 300.0);
        minimumRadius = std::min(
            minimumRadius, std::hypot(
                pose.position.x - roundabout->getX(),
                pose.position.y - roundabout->getY()));
        check(finitePose(pose),
              "Roundabout samples stay finite");
    }
    check(minimumRadius >= 20.0 - 1e-3,
          "Roundabout path never cuts through the central island (min=" +
              std::to_string(minimumRadius) + ")");
    check(westToNorth != nullptr && westToSouth != nullptr &&
          westUTurn != nullptr &&
          westToNorth->getLength() < westToEast->getLength() &&
          westToEast->getLength() < westToSouth->getLength() &&
          westToSouth->getLength() < westUTurn->getLength(),
          "Roundabout supports first, second, far and U-turn exits clockwise (" +
              std::to_string(westToNorth->getLength()) + ", " +
              std::to_string(westToEast->getLength()) + ", " +
              std::to_string(westToSouth->getLength()) + ", " +
              std::to_string(westUTurn->getLength()) + ")");

    check(roundabout->tryEnterMovement(
              101, westToEast, 3.0),
          "First roundabout movement reserves a gap");
    roundabout->updateReservationProgress(
        101, westToEast->getLength() * 0.5);
    check(roundabout->tryEnterMovement(
              102, southToNorth, 3.0),
          "Separated roundabout movements can coexist");
    roundabout->exit(101);
    roundabout->exit(102);

    check(roundabout->tryEnterMovement(
              111, westToEast, 2.0, 4.5, 1.8),
          "Roundabout admits a leading car");
    roundabout->updateReservationProgress(111, 15.0);
    check(roundabout->tryEnterMovement(
              112, westToEast, 2.0, 4.5, 1.8),
          "Roundabout admits a following car at a safe merge gap");
    const double collisionLimitedAdvance =
        roundabout->limitTraversalAdvance(
            112, westToEast, 0.0, 12.0,
            4.5, 1.8, 2.0);
    check(collisionLimitedAdvance > 0.0 &&
              collisionLimitedAdvance < 12.0,
          "Oriented vehicle bounds prevent a faster roundabout follower from catching its leader");
    roundabout->exit(111);
    roundabout->exit(112);

    Car car(201, 20.0, graph.getIntersection(1),
            graph.getIntersection(3));
    Bus bus(202, 20.0, graph.getIntersection(1),
            graph.getIntersection(3));
    check(bus.getMaxLateralAcceleration() <
              car.getMaxLateralAcceleration(),
          "Bus curvature limit is lower than car curvature limit");

    Graph mapGraph;
    std::string error;
    std::string path = "map4.json";
    if (!std::filesystem::exists(path)) path = "../map4.json";
    check(MapLoad::loadGraphFromJsonFile(path, mapGraph, &error),
          "map4 loads with the SI radius contract");
    Intersection* mapRoundabout = mapGraph.getIntersection(2);
    check(mapRoundabout != nullptr &&
          mapRoundabout->isRoundabout() &&
          near(mapRoundabout->getTraversalRadiusMetres(), 50.0),
          "map4 radius 0.05 km becomes the same 50 m simulation radius");

    const std::string explicitUnits = R"JSON({
        "defaultDistanceUnit": "m",
        "defaultSpeedUnit": "m/s",
        "defaultRadiusUnit": "m",
        "intersections": [
            {"id": 1, "x": 0, "y": 0},
            {"id": 2, "x": 120, "y": 0,
             "type": "roundabout", "radius": 18}
        ],
        "roads": [
            {"id": 1, "start": 1, "end": 2,
             "distance": 120, "speedLimit": 15}
        ]
    })JSON";
    Graph explicitGraph;
    std::string explicitError;
    check(MapLoad::loadGraphFromJsonString(
              explicitUnits, explicitGraph, &explicitError) &&
          near(explicitGraph.getRoad(1)->getDistance(), 120.0) &&
          near(explicitGraph.getRoad(1)->getSpeedLimit(), 15.0) &&
          near(explicitGraph.getIntersection(2)
                   ->getTraversalRadiusMetres(), 18.0),
          "Explicit map metre and m/s units are normalized without legacy scaling");

    DijkstraStrategy strategy;
    auto simulator = createDemoSimulator(mapGraph, &strategy);
    check(simulator->getVehicles().size() +
              simulator->getPendingVehicleCount() == 1000,
          "map4 stress fixture creates 1000 routed vehicles");
    simulator->setSpeedMultiplier(100.0);
    simulator->update(0.1);
    simulator->update(0.1);
    simulator->setSpeedMultiplier(1.0);
    for (int sample = 0; sample < 200; ++sample) {
        simulator->update(0.1);
    }
    const std::size_t activeAfterWarmStart =
        simulator->getVehicles().size();
    check(activeAfterWarmStart >= 80u,
          "map4 warm start sustains at least 80 active vehicles after 40 simulated seconds (active=" +
              std::to_string(activeAfterWarmStart) + ")");
    int bridgeVehicles = 0;
    for (const Vehicle* vehicle : simulator->getVehicles()) {
        if (vehicle->getCurrentRoad() != nullptr &&
            std::abs(vehicle->getCurrentRoad()->getId()) == 105) {
            ++bridgeVehicles;
        }
    }
    bool stable = true;
    for (const Vehicle* vehicle : simulator->getVehicles()) {
        stable = stable &&
            vehicle->getCurrentRoad() != nullptr &&
            vehicle->getCurrentLaneIndex() >= 0 &&
            vehicle->getCurrentLaneIndex() <
                vehicle->getCurrentRoad()->getLaneCount() &&
            finitePose(vehicle->getPose());
    }
    stable = stable &&
        simulator->getVehicles().size() <=
            simulator->getMaximumActiveVehicles() &&
        simulator->getVehicles().size() +
            simulator->getPendingVehicleCount() +
            simulator->getFinishedVehicles().size() == 1000;
    int overlappingPairs = 0;
    for (std::size_t firstIndex = 0;
         firstIndex < simulator->getVehicles().size();
         ++firstIndex) {
        const Vehicle* first =
            simulator->getVehicles()[firstIndex];
        for (std::size_t secondIndex = firstIndex + 1;
             secondIndex < simulator->getVehicles().size();
             ++secondIndex) {
            const Vehicle* second =
                simulator->getVehicles()[secondIndex];
            if (overlap(
                    boundsOf(*first),
                    boundsOf(*second))) {
                ++overlappingPairs;
                if (overlappingPairs <= 8) {
                    std::cerr
                        << "[OVERLAP] " << first->getId()
                        << " road="
                        << (first->getCurrentRoad()
                                ? first->getCurrentRoad()->getId()
                                : 0)
                        << " lane=" << first->getCurrentLaneIndex()
                        << " progress="
                        << first->getProgressOnRoad()
                        << " with " << second->getId()
                        << " road="
                        << (second->getCurrentRoad()
                                ? second->getCurrentRoad()->getId()
                                : 0)
                        << " lane=" << second->getCurrentLaneIndex()
                        << " progress="
                        << second->getProgressOnRoad()
                        << '\n';
                }
            }
        }
    }
    check(overlappingPairs == 0,
          "map4 rendered vehicle rectangles do not physically overlap (pairs=" +
              std::to_string(overlappingPairs) + ")");
    check(bridgeVehicles <= 30,
          "map4 density admission prevents River Bridge from becoming a solid vehicle block (vehicles=" +
              std::to_string(bridgeVehicles) + ")");
    check(stable,
          "map4 keeps 1000 trips finite and within its active-density limit over 40 simulated seconds");
}

} // namespace

int main() {
    testBezierGeometry();
    testLaneMapping();
    testConnectorCacheAndVehicleLifecycle();
    testEmergencyRedLightPriorityAndCaution();
    testSignalizedMultiLaneQueueDischarge();
    testFrameRateIndependence();
    testRoundaboutGeometryAndCapacity();

    if (failures == 0) {
        std::cout << "Motion geometry and integration tests passed\n";
        return 0;
    }
    std::cerr << failures << " motion test(s) failed\n";
    return 1;
}
