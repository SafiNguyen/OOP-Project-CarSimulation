// ============================================================================
// test_vehicle.cpp
// ----------------------------------------------------------------------------
// Standalone unit tests for Vehicle, Car, EmergencyVehicle, Motorbike, and Bus.
// Verifies polymorphic speed calculations, route advancement, and 
// complex pausing mechanics (like Bus dwell times at stops).
// ============================================================================

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "../tests/TestFramework.h"
#include "../src/model/Graph.h"
#include "../src/model/Intersection.h"
#include "../src/model/Road.h"
#include "../src/model/Vehicle.h"
#include "../src/model/Car.h"
#include "../src/model/EmergencyVehicle.h"
#include "../src/model/Motorbike.h"
#include "../src/model/Bus.h"
#include "../src/model/BusStop.h"
#include "../src/model/TrafficLight.h"
#include "../src/algorithm/DijkstraStrategy.h" 
#include "../src/simulation/TrafficSimulator.h"

using TestFramework::reportResult;
using TestFramework::nearlyEqual;
using TestFramework::printSummary;

class LaneTestCar : public Car {
public:
    using Car::Car;

    void placeOnLane(Road& road, int laneIndex, double progress) {
        if (currentRoad != nullptr) {
            currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        }
        currentRoad = &road;
        currentLaneIndex = laneIndex;
        progressOnCurrentRoad = progress;
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
    }

    void setTestSpeed(double speed) {
        currentSpeed = speed;
    }
};

class TestBus : public Bus {
public:
    using Bus::Bus;

    void placeOnLane(Road& road, int laneIndex, double progress) {
        if (currentRoad != nullptr) {
            currentRoad->getLane(currentLaneIndex).removeVehicle(this);
        }
        currentRoad = &road;
        currentLaneIndex = laneIndex;
        progressOnCurrentRoad = progress;
        currentRoad->getLane(currentLaneIndex).addVehicle(this);
    }

    void setTestSpeed(double speed) {
        currentSpeed = speed;
    }

    void setTestProgress(double progress) {
        progressOnCurrentRoad = progress;
    }
};

// ----------------------------------------------------------------------------
// Test Cases
// ----------------------------------------------------------------------------

void test_Polymorphic_Speed_Calculation() {
    std::string testName = "Vehicle: Polymorphic speed respects congestion, weaving, and cruise factors";
    
    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    // Road: distance 100, speedLimit 50, congestion 3.0
    Road road(101, "Test", &i1, &i2, 100.0, 50.0, 3.0); 

    // All vehicles have engine base speed 60.0
    Car car(1, 60.0, &i1, &i2);
    EmergencyVehicle ambulance(2, 60.0, &i1, &i2);
    Motorbike bike(3, 60.0, &i1, &i2);
    Bus bus(4, 60.0, &i1, &i2);

    car.setRoute({&road});
    ambulance.setRoute({&road});
    bike.setRoute({&road});
    bus.setRoute({&road});

    // Calculations:
    // Car: min(60, 50) / 3.0 = 16.666...
    double carExpected = 50.0 / 3.0;
    
    // Ambulance: Ignores everything, runs at baseSpeed = 60.0
    double ambExpected = 60.0;
    
    // Motorbike: Weaving factor 0.5. Effective congestion = 1.0 + (3.0-1.0)*0.5 = 2.0
    // Speed: min(60, 50) / 2.0 = 25.0
    double bikeExpected = 25.0;

    // Bus: Cruise factor 0.85. Cap = 50 * 0.85 = 42.5
    // Speed: 42.5 / 3.0 = 14.166...
    double busExpected = (50.0 * 0.85) / 3.0;

    bool passed = nearlyEqual(car.calculateCurrentSpeed(), carExpected) &&
                  nearlyEqual(ambulance.calculateCurrentSpeed(), ambExpected) &&
                  nearlyEqual(bike.calculateCurrentSpeed(), bikeExpected) &&
                  nearlyEqual(bus.calculateCurrentSpeed(), busExpected);

    std::ostringstream d;
    d << "  Expected: Car=" << carExpected << ", Amb=" << ambExpected 
      << ", Bike=" << bikeExpected << ", Bus=" << busExpected << "\n";
    d << "  Actual:   Car=" << car.calculateCurrentSpeed() << ", Amb=" << ambulance.calculateCurrentSpeed() 
      << ", Bike=" << bike.calculateCurrentSpeed() << ", Bus=" << bus.calculateCurrentSpeed() << "\n";
    reportResult(testName, passed, d.str());
}

void test_EmergencyVehicle_BlockedRoad() {
    std::string testName = "Vehicle: EmergencyVehicle stops moving if road is totally blocked";
    
    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 50.0);
    road.blockRoad(); // Tai nạn!

    EmergencyVehicle ambulance(1, 60.0, &i1, &i2);
    ambulance.setRoute({&road});

    // Phải bằng 0. Nếu vẫn chạy xuyên tai nạn là bug.
    bool passed = nearlyEqual(ambulance.calculateCurrentSpeed(), 0.0);

    std::ostringstream d;
    d << "  Expected: speed = 0.0 (Road is blocked)\n";
    d << "  Actual:   speed = " << ambulance.calculateCurrentSpeed() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_DwellTime_StateMachine() {
    std::string testName = "Vehicle: Bus pauses at bus stops, counts down dwell time, then resumes";
    
    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 50.0, 1.0);
    
    // Add a bus stop at 20 meters
    road.addBusStop(20.0);

    // Bus starts with default 15s dwell time
    TestBus bus(1, 50.0, &i1, &i2, 15.0);
    bus.setRoute({&road});
    bus.setTestSpeed(50.0);

    bus.update(1.6); // Đi hơi lố 1 xíu để kích hoạt trạm

    // Kì vọng: Xe dừng đúng tại mốc 20.0, chuyển state sang Dwelling, timer = 15.0
    bool passStop = nearlyEqual(bus.getProgressRatio() * road.getDistance(), 20.0) &&
                    bus.isDwelling() &&
                    nearlyEqual(bus.getDwellTimer(), 15.0);

    // Chờ 10 giây...
    bus.update(10.0);
    bool passWait = bus.isDwelling() && nearlyEqual(bus.getDwellTimer(), 5.0);

    // Chờ thêm 6 giây (vượt mức 15s) -> Xe khởi hành lại
    bus.update(6.0);
    bool passResume = !bus.isDwelling() && nearlyEqual(bus.getDwellTimer(), 0.0);

    bool passed = passStop && passWait && passResume;

    std::ostringstream d;
    d << "  Expected: Stops exactly at 20.0, waits 15s, then resumes moving.\n";
    d << "  Actual:   Stopped=" << passStop << " Waited=" << passWait << " Resumed=" << passResume << "\n";
    reportResult(testName, passed, d.str());
}

void test_Vehicle_RouteAdvancement() {
    std::string testName = "Vehicle: Seamlessly advances to next road in route";
    
    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 10.0, 0.0);
    Intersection i3(3, 20.0, 0.0);
    
    Road r1(101, "Test", &i1, &i2, 10.0, 10.0, 1.0);
    Road r2(102, "Test", &i2, &i3, 10.0, 10.0, 1.0);

    LaneTestCar car(1, 10.0, &i1, &i3);
    car.setRoute({&r1, &r2});
    car.setTestSpeed(10.0);

    // Speed = 10m/s. Update 1.5s -> Moves 15m.
    // Expected: Finishes r1 (10m), enters r2, progresses 5m on r2.
    car.update(1.5);

    bool passed = (car.getCurrentRoad() == &r2) &&
                  nearlyEqual(car.getProgressRatio() * r2.getDistance(), 5.0);

    std::ostringstream d;
    d << "  Expected: currentRoad=r2, progressOnCurrentRoad=5.0\n";
    if (car.getCurrentRoad() != nullptr) {
        d << "  Actual:   currentRoad=" << car.getCurrentRoad()->getId() 
          << ", progressOnCurrentRoad=" << (car.getProgressRatio() * car.getCurrentRoad()->getDistance()) << "\n";
    } else {
        d << "  Actual:   currentRoad=nullptr\n";
    }
    reportResult(testName, passed, d.str());
}

void test_Vehicle_RecalculateRoute_PreservesProgressNoGrowth() {
    std::string testName = "Vehicle: recalculateRoute() called repeatedly mid-road preserves "
                            "progress and does not grow the route unboundedly";

    Graph g;
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 10.0, 0.0));
    g.addIntersection(new Intersection(3, 20.0, 0.0));
    g.addIntersection(new Intersection(4, 30.0, 0.0));
    g.addRoad(new Road(101, "Test Road", g.getIntersection(1), g.getIntersection(2), 10.0, 10.0, 1.0));
    g.addRoad(new Road(102, "Test Road", g.getIntersection(2), g.getIntersection(3), 10.0, 10.0, 1.0));
    g.addRoad(new Road(103, "Test Road", g.getIntersection(3), g.getIntersection(4), 10.0, 10.0, 1.0));

    DijkstraStrategy strategy; 

    Car car(1, 10.0, g.getIntersection(1), g.getIntersection(4));
    car.setRoute({g.getRoad(101), g.getRoad(102), g.getRoad(103)});

    car.update(0.5);
    double progressBefore = car.getProgressRatio() * car.getCurrentRoad()->getDistance();

    bool allSucceeded = true;
    for (int i = 0; i < 5; ++i) {
        if (!car.recalculateRoute(g, &strategy)) {
            allSucceeded = false;
        }
    }

    double progressAfter = car.getProgressRatio() * car.getCurrentRoad()->getDistance();
    bool passProgressUnchanged = nearlyEqual(progressBefore, progressAfter);
    bool passRoadUnchanged = (car.getCurrentRoad() == g.getRoad(101)); 


    car.update(25.0);
    bool passReachesDestination = car.hasReachedDestination();

    bool passed = allSucceeded && passProgressUnchanged && passRoadUnchanged && passReachesDestination;

    std::ostringstream d;
    d << "  Expected: progress unchanged after repeated recalculate, still on road 101, "
         "eventually reaches destination without infinite loop\n";
    d << "  Actual:   progressBefore=" << progressBefore << " progressAfter=" << progressAfter
      << " reached=" << passReachesDestination << "\n";
    reportResult(testName, passed, d.str());
}

void test_Vehicle_StopsAtRedTrafficLight() {
    std::string testName = "Vehicle: stops at a red traffic light before entering the intersection";

    Graph g;
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 80.0, 0.0));
    g.addIntersection(new Intersection(3, 80.0, -80.0));

    // Add the crossing road first so it becomes the active phase group.
    g.addRoad(new Road(100, "Cross Road", g.getIntersection(3), g.getIntersection(2), 80.0, 20.0, 1.0));
    g.addRoad(new Road(101, "Test Road", g.getIntersection(1), g.getIntersection(2), 80.0, 20.0, 1.0));

    Road* road = g.getRoad(101);
    g.getIntersection(2)->registerIncomingLight(road);
    TrafficLight* light = g.getIntersection(2)->getLightForIncomingRoad(road);
    if (light != nullptr) {
        light->forceState(LightState::RED);
    }
    bool hasRedLight = (light != nullptr) && light->mustStop();

    Car car(1, 30.0, g.getIntersection(1), g.getIntersection(2));
    car.setRoute({road});

    car.update(6.0);

    const bool passed = hasRedLight &&
                        (car.getCurrentRoad() == road) &&
                        !car.hasReachedDestination() &&
                        nearlyEqual(car.getCurrentSpeed(), 0.0) &&
                        nearlyEqual(
                            car.getProgressRatio() *
                                road->getDistance(),
                            road->getDistance() -
                                1.5 - car.getLength() * 0.5);

    std::ostringstream d;
    d << "  Expected: vehicle remains on the road and stops at the red light near the stop line.\n";
    d << "  Light was red: " << hasRedLight << "\n";
    d << "  Actual:   currentRoad=" << (car.getCurrentRoad() ? std::to_string(car.getCurrentRoad()->getId()) : "null")
      << ", speed=" << car.getCurrentSpeed()
      << ", reached=" << car.hasReachedDestination()
      << ", progress=" << (car.getProgressRatio() * road->getDistance()) << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_MultipleStopsUsePerStopDwellAndDoNotRepeat() {
    std::string testName = "Vehicle: Bus serves multiple stops in order with per-stop dwell";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 50.0);
    road.addBusStop(std::make_unique<BusStop>(501, "First", &road, 10.0, 0, 2.0));
    road.addBusStop(std::make_unique<BusStop>(502, "Second", &road, 30.0, 0, 4.0));

    Bus bus(1, 50.0, &i1, &i2);
    bus.setRoute({&road});
    bus.update(10.0);
    const bool firstStop = bus.isDwelling() &&
                           nearlyEqual(bus.getProgressOnRoad(), 10.0) &&
                           nearlyEqual(bus.getDwellTimer(), 2.0);

    bus.update(2.0);
    const bool firstResume = !bus.isDwelling() &&
                             bus.getNextBusStop() != nullptr &&
                             bus.getNextBusStop()->getId() == 502;

    bus.update(10.0);
    const bool secondStop = bus.isDwelling() &&
                            nearlyEqual(bus.getProgressOnRoad(), 30.0) &&
                            nearlyEqual(bus.getDwellTimer(), 4.0);

    bus.update(4.0);
    bus.update(0.5);
    const bool noRepeat = !bus.isDwelling() &&
                          bus.getProgressOnRoad() > 30.0 &&
                          bus.getNextBusStop() == nullptr;

    const bool passed = firstStop && firstResume && secondStop && noRepeat;
    std::ostringstream d;
    d << "  Expected: dwell 2s at 10m, then 4s at 30m, then move past 30m\n";
    d << "  Actual: first=" << firstStop << " second=" << secondStop
      << " noRepeat=" << noRepeat << "\n";
    reportResult(testName, passed, d.str());
}

void test_NonBusVehicle_IgnoresBusStops() {
    std::string testName = "Vehicle: Car ignores bus stops";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 50.0);
    road.addBusStop(std::make_unique<BusStop>(501, "Only buses", &road, 20.0, 0, 15.0));

    Car car(1, 50.0, &i1, &i2);
    car.setRoute({&road});
    car.update(1.5);

    const bool passed = !car.isPaused() &&
                        !nearlyEqual(car.getProgressOnRoad(), 20.0);
    std::ostringstream d;
    d << "  Expected: car does not enter a pause at 20m\n";
    d << "  Actual: paused=" << car.isPaused()
      << " progress=" << car.getProgressOnRoad() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_RedLightIsNotDwellAndDwellDoesNotBypassRed() {
    std::string testName = "Vehicle: Bus stop dwell is distinct from red-light stopping";

    Graph g;
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 80.0, 0.0));
    g.addIntersection(new Intersection(3, 80.0, -80.0));
    g.addRoad(new Road(100, "Cross", g.getIntersection(3), g.getIntersection(2), 80.0, 20.0));
    g.addRoad(new Road(101, "Bus Road", g.getIntersection(1), g.getIntersection(2), 80.0, 20.0));

    Road* road = g.getRoad(101);
    road->addBusStop(std::make_unique<BusStop>(501, "Before light", road, 20.0, 0, 1.0));
    g.getIntersection(2)->registerIncomingLight(road);
    TrafficLight* light = g.getIntersection(2)->getLightForIncomingRoad(road);
    if (light != nullptr) {
        light->forceState(LightState::RED);
    }

    Bus bus(1, 30.0, g.getIntersection(1), g.getIntersection(2));
    bus.setRoute({road});
    bus.update(10.0);
    const bool dwelled = bus.isDwelling() && nearlyEqual(bus.getDwellTimer(), 1.0);
    bus.update(1.0);
    bus.update(20.0);

    const bool passed = light != nullptr && light->mustStop() &&
                        dwelled &&
                        !bus.isDwelling() &&
                        bus.getPauseReason() != PauseReason::BusStop &&
                        bus.getCurrentRoad() == road &&
                        !bus.hasReachedDestination();
    std::ostringstream d;
    d << "  Expected: dwell at stop, then remain held by red without isDwelling\n";
    d << "  Actual: dwelled=" << dwelled
      << " finalDwelling=" << bus.isDwelling()
      << " reached=" << bus.hasReachedDestination() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_NoStopsKeepsNormalBehavior() {
    std::string testName = "Vehicle: Bus on a road without stops keeps moving normally";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "No stops", &i1, &i2, 100.0, 50.0);
    Bus bus(1, 50.0, &i1, &i2);
    bus.setRoute({&road});
    bus.update(1.0);

    const bool passed = !bus.isDwelling() &&
                        bus.getProgressOnRoad() > 0.0 &&
                        bus.getNextBusStop() == nullptr;
    std::ostringstream d;
    d << "  Expected: positive progress and no dwelling\n";
    d << "  Actual: progress=" << bus.getProgressOnRoad()
      << " dwelling=" << bus.isDwelling() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_UTurnRefreshesStopOnReverseRoad() {
    std::string testName = "Vehicle: Bus U-turn refreshes the next stop from the reverse road";

    Graph g;
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 100.0, 0.0));
    g.addRoad(new Road(101, "Forward", g.getIntersection(1), g.getIntersection(2),
                       100.0, 20.0));
    g.addRoad(new Road(102, "Reverse", g.getIntersection(2), g.getIntersection(1),
                       100.0, 20.0));

    Road* forward = g.getRoad(101);
    Road* reverse = g.getRoad(102);
    forward->addBusStop(std::make_unique<BusStop>(
        501, "Forward stop", forward, 90.0, 0, 1.0));
    reverse->addBusStop(std::make_unique<BusStop>(
        502, "Reverse stop", reverse, 80.0, 0, 1.0));
    const BusStop* stopA = forward->findBusStopById(501);
    const BusStop* stopB = reverse->findBusStopById(502);

    DijkstraStrategy strategy;
    TestBus bus(1, 20.0, g.getIntersection(1), g.getIntersection(1));
    bus.setRoute({forward});
    bus.setTestProgress(30.0);

    const bool turned = bus.performUTurn(g, &strategy);
    const BusStop* next = bus.getNextBusStop();
    const bool passed = turned &&
                        bus.getCurrentRoad() == reverse &&
                        next == stopB &&
                        next != stopA &&
                        next != nullptr &&
                        next->getRoad() == reverse;

    std::ostringstream d;
    d << "  Expected: reverse road and stop 502 after mirroring progress to 70m\n";
    d << "  Actual: turned=" << turned
      << " road=" << (bus.getCurrentRoad() ? bus.getCurrentRoad()->getId() : 0)
      << " stop=" << (next ? next->getId() : 0) << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_UTurnToRoadWithoutStopsClearsNextStop() {
    std::string testName = "Vehicle: Bus U-turn to a road without stops clears stale stop state";

    Graph g;
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 100.0, 0.0));
    g.addRoad(new Road(101, "Forward", g.getIntersection(1), g.getIntersection(2),
                       100.0, 20.0));
    g.addRoad(new Road(102, "Reverse", g.getIntersection(2), g.getIntersection(1),
                       100.0, 20.0));

    Road* forward = g.getRoad(101);
    Road* reverse = g.getRoad(102);
    forward->addBusStop(std::make_unique<BusStop>(
        501, "Old-road stop", forward, 90.0, 0, 1.0));

    DijkstraStrategy strategy;
    TestBus bus(1, 20.0, g.getIntersection(1), g.getIntersection(1));
    bus.setRoute({forward});
    bus.setTestProgress(30.0);

    const bool turned = bus.performUTurn(g, &strategy);
    const double progressAfterTurn = bus.getProgressOnRoad();
    bus.update(0.5);
    const bool passed = turned &&
                        bus.getCurrentRoad() == reverse &&
                        bus.getNextBusStop() == nullptr &&
                        bus.getNextStopPos() < 0.0 &&
                        !bus.isDwelling() &&
                        bus.getProgressOnRoad() > progressAfterTurn;

    std::ostringstream d;
    d << "  Expected: no next stop and no phantom dwell on reverse road\n";
    d << "  Actual: turned=" << turned
      << " nextPos=" << bus.getNextStopPos()
      << " dwelling=" << bus.isDwelling()
      << " progress=" << bus.getProgressOnRoad() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_UTurnSkipsReverseStopBehindMirroredProgress() {
    std::string testName = "Vehicle: Bus U-turn skips reverse-road stops behind its mirrored progress";

    Graph g;
    g.addIntersection(new Intersection(1, 0.0, 0.0));
    g.addIntersection(new Intersection(2, 100.0, 0.0));
    g.addRoad(new Road(101, "Forward", g.getIntersection(1), g.getIntersection(2),
                       100.0, 20.0));
    g.addRoad(new Road(102, "Reverse", g.getIntersection(2), g.getIntersection(1),
                       100.0, 20.0));

    Road* forward = g.getRoad(101);
    Road* reverse = g.getRoad(102);
    reverse->addBusStop(std::make_unique<BusStop>(
        501, "Behind", reverse, 50.0, 0, 1.0));
    reverse->addBusStop(std::make_unique<BusStop>(
        502, "Ahead", reverse, 80.0, 0, 1.0));

    DijkstraStrategy strategy;
    TestBus bus(1, 20.0, g.getIntersection(1), g.getIntersection(1));
    bus.setRoute({forward});
    bus.setTestProgress(30.0);

    const bool turned = bus.performUTurn(g, &strategy);
    const BusStop* next = bus.getNextBusStop();
    const bool passed = turned &&
                        nearlyEqual(bus.getProgressOnRoad(), 70.0) &&
                        next != nullptr &&
                        next->getId() == 502 &&
                        next->getRoad() == reverse;

    std::ostringstream d;
    d << "  Expected: stop 502 ahead of mirrored progress 70m\n";
    d << "  Actual: progress=" << bus.getProgressOnRoad()
      << " stop=" << (next ? next->getId() : 0) << "\n";
    reportResult(testName, passed, d.str());
}

void test_Vehicle_IntersectionPauseReasonPersistsAndResumes() {
    std::string testName = "Vehicle: intersection reservation wait uses Intersection reason until released";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 80.0, 0.0);
    Road road(101, "Approach", &i1, &i2, 80.0, 20.0);
    LaneTestCar car(1, 20.0, &i1, &i2);
    car.setRoute({&road});
    car.placeOnLane(road, 0, 78.5);
    car.setTestSpeed(0.0);

    const bool occupied = i2.tryEnter(900, &road);
    car.update(0.05);
    const double waitingProgress = car.getProgressOnRoad();
    const bool initiallyWaiting = occupied &&
                                  car.isPaused() &&
                                  car.getPauseReason() == PauseReason::Intersection &&
                                  !car.hasReachedDestination();

    car.update(0.5);
    const bool stillWaiting = car.isPaused() &&
                              car.getPauseReason() == PauseReason::Intersection &&
                              nearlyEqual(car.getProgressOnRoad(), waitingProgress) &&
                              !car.hasReachedDestination();

    i2.exit(900);
    car.update(0.1);
    const bool resumed = car.getProgressOnRoad() > waitingProgress &&
                         !car.isPaused() &&
                         car.getPauseReason() == PauseReason::None;

    const bool passed = initiallyWaiting && stillWaiting && resumed;
    std::ostringstream d;
    d << "  Expected: wait with Intersection, remain fixed, then resume with None\n";
    d << "  Actual: initial=" << initiallyWaiting
      << " waiting=" << stillWaiting
      << " resumed=" << resumed
      << " progress=" << car.getProgressOnRoad() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Vehicle_TrafficLightTakesPriorityOverIntersection() {
    std::string testName = "Vehicle: TrafficLight reason overrides a blocked intersection box";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 80.0, 0.0);
    Road road(101, "Signal approach", &i1, &i2, 80.0, 20.0);
    i2.addIncomingRoad(&road);
    i2.registerIncomingLight(&road);
    TrafficLight* light = i2.getLightForIncomingRoad(&road);
    if (light != nullptr) {
        light->forceState(LightState::RED);
    }

    LaneTestCar car(1, 20.0, &i1, &i2);
    car.setRoute({&road});
    car.placeOnLane(road, 0, 78.5);
    car.setTestSpeed(0.0);
    const bool occupied = i2.tryEnter(900, &road);

    car.update(0.05);
    const bool redReason = light != nullptr &&
                           occupied &&
                           car.isPaused() &&
                           car.getPauseReason() == PauseReason::TrafficLight;

    if (light != nullptr) {
        light->forceState(LightState::GREEN);
    }
    car.update(0.05);
    const bool greenButBlockedReason =
        car.isPaused() &&
        car.getPauseReason() == PauseReason::Intersection;

    i2.exit(900);
    const bool passed = redReason && greenButBlockedReason;
    std::ostringstream d;
    d << "  Expected: TrafficLight while red, then Intersection while green but occupied\n";
    d << "  Actual: redReason=" << redReason
      << " greenBlockedReason=" << greenButBlockedReason << "\n";
    reportResult(testName, passed, d.str());
}

void test_Vehicle_NonControlObstaclesDoNotUseIntersectionPause() {
    std::string testName = "Vehicle: lane and leader blocking do not use Intersection pause";

    bool blockedLaneKeepsNormalState = false;
    {
        Intersection i1(1, 0.0, 0.0);
        Intersection i2(2, 80.0, 0.0);
        Road road(101, "Blocked lane", &i1, &i2, 80.0, 20.0);
        LaneTestCar car(1, 20.0, &i1, &i2);
        car.setRoute({&road});
        car.placeOnLane(road, 0, 10.0);
        road.blockLane(0);
        i2.tryEnter(900, &road);

        car.update(0.5);
        blockedLaneKeepsNormalState =
            !car.isPaused() &&
            car.getPauseReason() == PauseReason::None &&
            nearlyEqual(car.getProgressOnRoad(), 10.0);
    }

    bool leaderKeepsNormalState = false;
    {
        Intersection i1(1, 0.0, 0.0);
        Intersection i2(2, 80.0, 0.0);
        Road road(101, "Queued approach", &i1, &i2, 80.0, 20.0);
        LaneTestCar follower(1, 20.0, &i1, &i2);
        LaneTestCar leader(2, 20.0, &i1, &i2);
        follower.setRoute({&road});
        leader.setRoute({&road});
        leader.placeOnLane(road, 0, 78.5);
        leader.setTestSpeed(0.0);
        follower.placeOnLane(road, 0, 72.0);
        follower.setTestSpeed(0.0);
        i2.tryEnter(900, &road);

        leader.update(0.05);
        follower.update(0.5);
        leaderKeepsNormalState =
            leader.isPaused() &&
            leader.getPauseReason() == PauseReason::Intersection &&
            !follower.isPaused() &&
            follower.getPauseReason() == PauseReason::None &&
            nearlyEqual(follower.getProgressOnRoad(), 72.0);
    }

    const bool passed =
        blockedLaneKeepsNormalState && leaderKeepsNormalState;
    std::ostringstream d;
    d << "  Expected: only the vehicle at the stop line uses Intersection pause\n";
    d << "  Actual: blockedLane=" << blockedLaneKeepsNormalState
      << " queuedFollower=" << leaderKeepsNormalState << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_DwellUsesRemainingUpdateTime() {
    std::string testName = "Vehicle: Bus uses update time remaining after dwell completes";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 200.0, 0.0);
    Road road(101, "Long bus road", &i1, &i2, 200.0, 20.0);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Short dwell", &road, 20.0, 0, 1.0));

    TestBus bus(1, 20.0, &i1, &i2);
    bus.setRoute({&road});
    bus.setTestProgress(19.9);
    bus.setTestSpeed(10.0);
    bus.update(0.05);
    const bool startedDwell = bus.isDwelling() &&
                              nearlyEqual(bus.getDwellTimer(), 1.0) &&
                              nearlyEqual(bus.getProgressOnRoad(), 20.0);

    bus.update(5.0);
    const bool passed = startedDwell &&
                        !bus.isDwelling() &&
                        nearlyEqual(bus.getDwellTimer(), 0.0) &&
                        bus.getProgressOnRoad() > 20.0;

    std::ostringstream d;
    d << "  Expected: consume 1s dwell and move with the remaining 4s\n";
    d << "  Actual: started=" << startedDwell
      << " timer=" << bus.getDwellTimer()
      << " progress=" << bus.getProgressOnRoad() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_IncompleteDwellConsumesOnlyAvailableTime() {
    std::string testName = "Vehicle: Bus remains stopped when available update time is shorter than dwell";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 200.0, 0.0);
    Road road(101, "Long bus road", &i1, &i2, 200.0, 20.0);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Long dwell", &road, 20.0, 0, 5.0));

    TestBus bus(1, 20.0, &i1, &i2);
    bus.setRoute({&road});
    bus.setTestProgress(19.9);
    bus.setTestSpeed(10.0);
    bus.update(0.05);
    const double stopProgress = bus.getProgressOnRoad();
    bus.update(2.0);

    const bool passed = bus.isDwelling() &&
                        bus.getPauseReason() == PauseReason::BusStop &&
                        nearlyEqual(bus.getDwellTimer(), 3.0) &&
                        nearlyEqual(bus.getProgressOnRoad(), stopProgress);
    std::ostringstream d;
    d << "  Expected: 3s dwell remains and progress stays fixed\n";
    d << "  Actual: timer=" << bus.getDwellTimer()
      << " progress=" << bus.getProgressOnRoad() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_DwellCompletionDoesNotBypassRedLight() {
    std::string testName = "Vehicle: Bus rechecks a red light after dwell completes";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 80.0, 0.0);
    Road road(101, "Signal bus road", &i1, &i2, 80.0, 20.0);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Before signal", &road, 70.0, 0, 1.0));
    i2.addIncomingRoad(&road);
    i2.registerIncomingLight(&road);
    TrafficLight* light = i2.getLightForIncomingRoad(&road);
    if (light != nullptr) {
        light->forceState(LightState::RED);
    }

    TestBus bus(1, 20.0, &i1, &i2);
    bus.setRoute({&road});
    bus.setTestProgress(69.9);
    bus.setTestSpeed(10.0);
    bus.update(0.05);
    const bool startedDwell = bus.isDwelling();
    bus.update(5.0);

    const bool passed = light != nullptr &&
                        startedDwell &&
                        !bus.isDwelling() &&
                        bus.isPaused() &&
                        bus.getPauseReason() == PauseReason::TrafficLight &&
                        bus.getCurrentRoad() == &road &&
                        bus.getProgressOnRoad() <=
                            road.getDistance() - 1.5 -
                                bus.getLength() * 0.5 + 1e-6 &&
                        !bus.hasReachedDestination();
    std::ostringstream d;
    d << "  Expected: finish dwell, then stop with its front bumper before the signal\n";
    d << "  Actual: reason=" << static_cast<int>(bus.getPauseReason())
      << " progress=" << bus.getProgressOnRoad()
      << " reached=" << bus.hasReachedDestination() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_DwellCompletionDoesNotBypassIntersection() {
    std::string testName = "Vehicle: Bus rechecks intersection reservation after dwell completes";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 80.0, 0.0);
    Road road(101, "Reserved bus road", &i1, &i2, 80.0, 20.0);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Before box", &road, 70.0, 0, 1.0));
    const bool occupied = i2.tryEnter(900, &road);

    TestBus bus(1, 20.0, &i1, &i2);
    bus.setRoute({&road});
    bus.setTestProgress(69.9);
    bus.setTestSpeed(10.0);
    bus.update(0.05);
    const bool startedDwell = bus.isDwelling();
    bus.update(5.0);

    const bool passed = occupied &&
                        startedDwell &&
                        !bus.isDwelling() &&
                        bus.isPaused() &&
                        bus.getPauseReason() == PauseReason::Intersection &&
                        bus.getCurrentRoad() == &road &&
                        bus.getProgressOnRoad() <=
                            road.getDistance() - 1.5 -
                                bus.getLength() * 0.5 + 1e-6 &&
                        !bus.hasReachedDestination();
    i2.exit(900);

    std::ostringstream d;
    d << "  Expected: finish dwell, then wait for the occupied box\n";
    d << "  Actual: reason=" << static_cast<int>(bus.getPauseReason())
      << " progress=" << bus.getProgressOnRoad()
      << " reached=" << bus.hasReachedDestination() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_ChangesOneLaneAtATimeTowardStop() {
    std::string testName = "Vehicle: Bus changes one safe lane at a time toward its stop lane";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 500.0, 0.0);
    Road road(101, "Three-lane bus road", &i1, &i2, 500.0, 20.0, 1.0, 3);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Lane 2 stop", &road, 300.0, 2, 1.0));

    TestBus bus(1, 2.0, &i1, &i2);
    bus.setRoute({&road});
    bus.placeOnLane(road, 0, 250.0);
    bus.setTestSpeed(2.0);

    bus.update(0.05);
    const bool firstAdjacentChange =
        bus.getCurrentLaneIndex() == 1 &&
        bus.getProgressOnRoad() < 300.0;

    bus.update(Vehicle::LANE_CHANGE_COOLDOWN + 0.1);
    const bool secondAdjacentChange =
        bus.getCurrentLaneIndex() == 2 &&
        bus.getProgressOnRoad() < 300.0;

    const bool passed = firstAdjacentChange && secondAdjacentChange;
    std::ostringstream d;
    d << "  Expected: lane 0 -> 1 -> 2 without skipping a lane\n";
    d << "  Actual: first=" << firstAdjacentChange
      << " second=" << secondAdjacentChange
      << " finalLane=" << bus.getCurrentLaneIndex()
      << " progress=" << bus.getProgressOnRoad() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_UsesAdaptiveDistanceAndDwellsInCurbLane() {
    std::string testName = "Vehicle: Bus prepares early, enters the curb lane, and dwells there";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 400.0, 0.0);
    Road road(101, "Adaptive bus approach", &i1, &i2, 400.0, 25.0, 1.0, 3);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Curb stop", &road, 300.0, road.getCurbLaneIndex(), 2.0));

    TestBus bus(1, 25.0, &i1, &i2);
    bus.setRoute({&road});
    bus.placeOnLane(road, 0, 150.0);
    bus.setTestSpeed(20.0);

    bus.update(0.05);
    const bool preparedEarlierThanLegacyDistance =
        bus.getCurrentLaneIndex() == 1 &&
        bus.getProgressOnRoad() < 240.0;

    bus.update(Vehicle::LANE_CHANGE_COOLDOWN + 0.1);
    const bool reachedCurbBeforeStop =
        bus.getCurrentLaneIndex() == road.getCurbLaneIndex() &&
        bus.getProgressOnRoad() < 300.0;

    for (int tick = 0; tick < 100 && !bus.isDwelling(); ++tick) {
        bus.update(0.25);
    }

    const bool passed =
        preparedEarlierThanLegacyDistance &&
        reachedCurbBeforeStop &&
        bus.isDwelling() &&
        bus.getCurrentLaneIndex() == road.getCurbLaneIndex() &&
        nearlyEqual(bus.getProgressOnRoad(), 300.0);

    std::ostringstream d;
    d << "  Expected: prepare beyond 60m, move 0 -> 1 -> 2, dwell at 300m\n";
    d << "  Actual: early=" << preparedEarlierThanLegacyDistance
      << " curbBeforeStop=" << reachedCurbBeforeStop
      << " lane=" << bus.getCurrentLaneIndex()
      << " progress=" << bus.getProgressOnRoad()
      << " dwelling=" << bus.isDwelling() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_RejectsUnsafeFrontGapTowardStopLane() {
    std::string testName = "Vehicle: Bus does not enter stop lane with an unsafe front gap";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 150.0, 0.0);
    Road road(101, "Bus approach", &i1, &i2, 150.0, 20.0, 1.0, 2);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Lane 1 stop", &road, 100.0, 1, 1.0));

    TestBus bus(1, 20.0, &i1, &i2);
    LaneTestCar frontVehicle(2, 10.0, &i1, &i2);
    bus.setRoute({&road});
    frontVehicle.setRoute({&road});
    bus.placeOnLane(road, 0, 50.0);
    frontVehicle.placeOnLane(road, 1, 60.0);
    bus.setTestSpeed(10.0);
    frontVehicle.setTestSpeed(5.0);

    bus.update(0.05);

    const bool passed =
        bus.getCurrentLaneIndex() == 0 &&
        !bus.isDwelling();
    std::ostringstream d;
    d << "  Expected: remain in lane 0 because lane 1 front gap is unsafe\n";
    d << "  Actual: lane=" << bus.getCurrentLaneIndex()
      << " progress=" << bus.getProgressOnRoad() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_RejectsUnsafeRearGapTowardStopLane() {
    std::string testName = "Vehicle: Bus does not enter stop lane with an unsafe rear gap";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 150.0, 0.0);
    Road road(101, "Bus approach", &i1, &i2, 150.0, 30.0, 1.0, 2);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Lane 1 stop", &road, 100.0, 1, 1.0));

    TestBus bus(1, 20.0, &i1, &i2);
    LaneTestCar rearVehicle(2, 30.0, &i1, &i2);
    bus.setRoute({&road});
    rearVehicle.setRoute({&road});
    bus.placeOnLane(road, 0, 50.0);
    rearVehicle.placeOnLane(road, 1, 45.0);
    bus.setTestSpeed(10.0);
    rearVehicle.setTestSpeed(25.0);

    bus.update(0.05);

    const bool passed =
        bus.getCurrentLaneIndex() == 0 &&
        !bus.isDwelling();
    std::ostringstream d;
    d << "  Expected: remain in lane 0 because lane 1 rear gap/TTC is unsafe\n";
    d << "  Actual: lane=" << bus.getCurrentLaneIndex()
      << " progress=" << bus.getProgressOnRoad() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_RetriesRequiredLaneWhenGapBecomesSafe() {
    std::string testName = "Vehicle: Bus retries its stop-lane merge after the gap becomes safe";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 180.0, 0.0);
    Road road(101, "Retry bus approach", &i1, &i2, 180.0, 20.0, 1.0, 2);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Curb stop", &road, 120.0, road.getCurbLaneIndex(), 1.0));

    TestBus bus(1, 20.0, &i1, &i2);
    LaneTestCar blocker(2, 10.0, &i1, &i2);
    bus.setRoute({&road});
    blocker.setRoute({&road});
    bus.placeOnLane(road, 0, 60.0);
    blocker.placeOnLane(road, 1, 70.0);
    bus.setTestSpeed(10.0);
    blocker.setTestSpeed(5.0);

    bus.update(0.05);
    const bool initiallyRejected =
        bus.getCurrentLaneIndex() == 0 &&
        !bus.isPaused();

    blocker.placeOnLane(road, 1, 150.0);
    blocker.setTestSpeed(10.0);
    bus.update(0.3);

    const bool passed =
        initiallyRejected &&
        bus.getCurrentLaneIndex() == road.getCurbLaneIndex() &&
        !bus.isDwelling();

    std::ostringstream d;
    d << "  Expected: reject unsafe first attempt, then merge on a later tick\n";
    d << "  Actual: initiallyRejected=" << initiallyRejected
      << " lane=" << bus.getCurrentLaneIndex()
      << " progress=" << bus.getProgressOnRoad() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_SlowsGraduallyWhileWaitingForStopLaneGap() {
    std::string testName = "Vehicle: Bus slows with deceleration while waiting for a stop-lane gap";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 160.0, 0.0);
    Road road(101, "Controlled bus approach", &i1, &i2, 160.0, 20.0, 1.0, 2);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Curb stop", &road, 100.0, road.getCurbLaneIndex(), 1.0));

    TestBus bus(1, 20.0, &i1, &i2);
    LaneTestCar blocker(2, 5.0, &i1, &i2);
    bus.setRoute({&road});
    blocker.setRoute({&road});
    bus.placeOnLane(road, 0, 90.0);
    blocker.placeOnLane(road, 1, 97.0);
    bus.setTestSpeed(10.0);
    blocker.setTestSpeed(2.0);

    bus.update(0.05);

    const bool passed =
        bus.getCurrentLaneIndex() == 0 &&
        bus.getCurrentSpeed() < 10.0 &&
        bus.getCurrentSpeed() > 0.0 &&
        !bus.isPaused() &&
        bus.getPauseReason() == PauseReason::None;

    std::ostringstream d;
    d << "  Expected: remain in lane, reduce speed gradually, and do not pause/dwell\n";
    d << "  Actual: lane=" << bus.getCurrentLaneIndex()
      << " speed=" << bus.getCurrentSpeed()
      << " reason=" << static_cast<int>(bus.getPauseReason()) << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_BlockedStopLaneIsMissedWithoutControlPause() {
    std::string testName = "Vehicle: Bus never enters a blocked stop lane and deterministically misses the stop";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 300.0, 0.0);
    Road road(101, "Blocked curb lane", &i1, &i2, 300.0, 20.0, 1.0, 2);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Blocked stop", &road, 100.0, road.getCurbLaneIndex(), 2.0));
    road.blockLane(road.getCurbLaneIndex());

    TestBus bus(1, 20.0, &i1, &i2);
    bus.setRoute({&road});
    bus.placeOnLane(road, 0, 70.0);
    bus.setTestSpeed(10.0);

    for (int tick = 0;
         tick < 200 && bus.getCurrentRoad() != nullptr &&
             bus.getProgressOnRoad() <= 101.0;
         ++tick) {
        bus.update(0.25);
    }

    const bool passed =
        bus.getCurrentLaneIndex() == 0 &&
        !bus.isDwelling() &&
        bus.getPauseReason() != PauseReason::BusStop &&
        bus.getPauseReason() != PauseReason::Intersection &&
        bus.getNextBusStop() == nullptr &&
        (bus.getCurrentRoad() == nullptr ||
         bus.getProgressOnRoad() > 100.0);

    std::ostringstream d;
    d << "  Expected: stay out of blocked lane, pass 100m, clear the missed stop\n";
    d << "  Actual: lane=" << bus.getCurrentLaneIndex()
      << " progress=" << bus.getProgressOnRoad()
      << " next=" << (bus.getNextBusStop() ? bus.getNextBusStop()->getId() : 0)
      << " reason=" << static_cast<int>(bus.getPauseReason()) << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_ReverseRoadUsesItsOwnCurbStopLane() {
    std::string testName = "Vehicle: Bus serves the reverse Road's curb stop without using the forward stop";

    Intersection west(1, 0.0, 0.0);
    Intersection east(2, 200.0, 0.0);
    Road forward(100, "Forward", &west, &east, 200.0, 20.0, 1.0, 2);
    Road reverse(-100, "Reverse", &east, &west, 200.0, 20.0, 1.0, 2);
    forward.addBusStop(std::make_unique<BusStop>(
        501, "Forward stop", &forward, 100.0, forward.getCurbLaneIndex(), 2.0));
    reverse.addBusStop(std::make_unique<BusStop>(
        504, "Reverse stop", &reverse, 100.0, reverse.getCurbLaneIndex(), 2.0));

    TestBus bus(1, 20.0, &east, &west);
    bus.setRoute({&reverse});
    bus.placeOnLane(reverse, 0, 40.0);
    bus.setTestSpeed(10.0);

    bus.update(0.05);
    const BusStop* selectedStop = bus.getNextBusStop();
    const bool selectedReverseStop =
        selectedStop != nullptr &&
        selectedStop->getId() == 504 &&
        selectedStop->getRoad() == &reverse &&
        bus.getCurrentLaneIndex() == reverse.getCurbLaneIndex();

    for (int tick = 0; tick < 100 && !bus.isDwelling(); ++tick) {
        bus.update(0.25);
    }

    const bool passed =
        selectedReverseStop &&
        bus.isDwelling() &&
        bus.getCurrentRoad() == &reverse &&
        bus.getCurrentLaneIndex() == reverse.getCurbLaneIndex() &&
        nearlyEqual(bus.getProgressOnRoad(), 100.0);

    std::ostringstream d;
    d << "  Expected: select stop 504, enter reverse curb lane, dwell at 100m\n";
    d << "  Actual: selected=" << (selectedStop ? selectedStop->getId() : 0)
      << " lane=" << bus.getCurrentLaneIndex()
      << " progress=" << bus.getProgressOnRoad()
      << " dwelling=" << bus.isDwelling() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Bus_DoesNotServeStopFromWrongLane() {
    std::string testName = "Vehicle: Bus skips a stop it cannot safely reach and serves the next valid stop";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 120.0, 0.0);
    Road road(101, "Two-stop bus road", &i1, &i2, 120.0, 20.0, 1.0, 2);
    road.addBusStop(std::make_unique<BusStop>(
        501, "Unreachable lane 1 stop", &road, 30.0, 1, 1.0));
    road.addBusStop(std::make_unique<BusStop>(
        502, "Reachable lane 0 stop", &road, 60.0, 0, 2.0));

    TestBus bus(1, 20.0, &i1, &i2);
    LaneTestCar laneOneVehicle(2, 5.0, &i1, &i2);
    bus.setRoute({&road});
    laneOneVehicle.setRoute({&road});
    bus.placeOnLane(road, 0, 20.0);
    laneOneVehicle.placeOnLane(road, 1, 35.0);
    bus.setTestSpeed(10.0);
    laneOneVehicle.setTestSpeed(0.0);

    bus.update(10.0);

    const BusStop* servedStop = bus.getNextBusStop();
    const bool passed =
        bus.isDwelling() &&
        bus.getCurrentLaneIndex() == 0 &&
        nearlyEqual(bus.getProgressOnRoad(), 60.0) &&
        servedStop != nullptr &&
        servedStop->getId() == 502 &&
        nearlyEqual(bus.getDwellTimer(), 2.0);
    std::ostringstream d;
    d << "  Expected: no dwell at 30m in wrong lane; dwell at stop 502 at 60m\n";
    d << "  Actual: lane=" << bus.getCurrentLaneIndex()
      << " progress=" << bus.getProgressOnRoad()
      << " stop=" << (servedStop ? servedStop->getId() : 0)
      << " dwelling=" << bus.isDwelling() << "\n";
    reportResult(testName, passed, d.str());
}

void test_LaneChange_DoesNotEnterEmergencyLane() {
    std::string testName = "Lane change: yielding vehicle does not enter the emergency lane";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 20.0, 1.0, 3);

    LaneTestCar subject(1, 20.0, &i1, &i2);
    LaneTestCar leader(2, 5.0, &i1, &i2);
    subject.setRoute({&road});
    leader.setRoute({&road});

    subject.placeOnLane(road, 1, 20.0);
    leader.placeOnLane(road, 1, 35.0);
    road.blockLane(0); // Lane 2 is the only attractive normal candidate.

    subject.notifyEmergencyApproaching(2);
    subject.update(0.05);

    const bool passed = subject.getCurrentLaneIndex() == 1;
    std::ostringstream d;
    d << "  Expected: remain in lane 1 instead of entering emergency lane 2\n";
    d << "  Actual:   lane=" << subject.getCurrentLaneIndex() << "\n";
    reportResult(testName, passed, d.str());
}

void test_YieldLaneChange_CanExitEmergencyLane() {
    std::string testName = "Lane change: vehicle on the emergency lane can still move out";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 20.0, 1.0, 3);

    LaneTestCar subject(1, 20.0, &i1, &i2);
    subject.setRoute({&road});
    subject.placeOnLane(road, 2, 20.0);

    subject.notifyEmergencyApproaching(2);
    subject.update(0.05);

    const bool passed = subject.getCurrentLaneIndex() == 1;
    std::ostringstream d;
    d << "  Expected: leave emergency lane 2 for nearest safe lane 1\n";
    d << "  Actual:   lane=" << subject.getCurrentLaneIndex() << "\n";
    reportResult(testName, passed, d.str());
}

void test_LaneChange_SelectsBestEligibleLane() {
    std::string testName = "Lane change: selects the best eligible adjacent lane";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 20.0, 1.0, 3);

    LaneTestCar subject(1, 20.0, &i1, &i2);
    LaneTestCar currentLeader(2, 20.0, &i1, &i2);
    LaneTestCar lane0Leader(3, 20.0, &i1, &i2);
    LaneTestCar lane2Leader(4, 20.0, &i1, &i2);
    subject.setRoute({&road});
    currentLeader.setRoute({&road});
    lane0Leader.setRoute({&road});
    lane2Leader.setRoute({&road});

    subject.placeOnLane(road, 1, 20.0);
    currentLeader.placeOnLane(road, 1, 34.5); // 10 m gap
    lane0Leader.placeOnLane(road, 0, 39.5);   // 15 m gap
    lane2Leader.placeOnLane(road, 2, 42.5);   // 18 m gap
    subject.setTestSpeed(10.0);
    currentLeader.setTestSpeed(10.0);
    lane0Leader.setTestSpeed(10.0);
    lane2Leader.setTestSpeed(10.0);

    subject.update(0.05);

    const bool passed = subject.getCurrentLaneIndex() == 2;
    std::ostringstream d;
    d << "  Expected: choose lane 2 with the largest safe forward gap\n";
    d << "  Actual:   lane=" << subject.getCurrentLaneIndex() << "\n";
    reportResult(testName, passed, d.str());
}

void test_LaneChange_RejectsFastRearFollower() {
    std::string testName = "Lane change: rejects a target lane with a fast rear follower";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 120.0, 0.0);
    Road road(101, "Test", &i1, &i2, 120.0, 30.0, 1.0, 3);

    LaneTestCar subject(1, 30.0, &i1, &i2);
    LaneTestCar currentLeader(2, 10.0, &i1, &i2);
    LaneTestCar fastFollower(3, 30.0, &i1, &i2);
    subject.setRoute({&road});
    currentLeader.setRoute({&road});
    fastFollower.setRoute({&road});

    subject.placeOnLane(road, 1, 40.0);
    currentLeader.placeOnLane(road, 1, 50.0);
    fastFollower.placeOnLane(road, 2, 30.0);
    subject.setTestSpeed(10.0);
    currentLeader.setTestSpeed(10.0);
    fastFollower.setTestSpeed(30.0);
    road.blockLane(0);

    subject.update(0.05);

    const bool passed = subject.getCurrentLaneIndex() == 1;
    std::ostringstream d;
    d << "  Expected: remain in lane 1 because lane 2 has unsafe rear TTC\n";
    d << "  Actual:   lane=" << subject.getCurrentLaneIndex() << "\n";
    reportResult(testName, passed, d.str());
}

void test_YieldLaneChange_DoesNotSkipBlockedAdjacentLane() {
    std::string testName = "Lane change: emergency yielding never skips across lanes";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 20.0, 1.0, 3);

    LaneTestCar subject(1, 20.0, &i1, &i2);
    subject.setRoute({&road});
    subject.placeOnLane(road, 0, 20.0);
    road.blockLane(1);

    subject.notifyEmergencyApproaching(0);
    subject.update(0.05);

    const bool passed = subject.getCurrentLaneIndex() == 0;
    std::ostringstream d;
    d << "  Expected: stay in lane 0; lane 2 cannot be reached by one change\n";
    d << "  Actual:   lane=" << subject.getCurrentLaneIndex() << "\n";
    reportResult(testName, passed, d.str());
}

void test_LaneChange_StopsOpportunisticChangeNearIntersection() {
    std::string testName = "Lane change: avoids opportunistic weaving near an intersection";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 20.0, 1.0, 2);

    LaneTestCar subject(1, 20.0, &i1, &i2);
    LaneTestCar leader(2, 10.0, &i1, &i2);
    subject.setRoute({&road});
    leader.setRoute({&road});
    subject.placeOnLane(road, 0, 90.0);
    leader.placeOnLane(road, 0, 98.0);
    subject.setTestSpeed(10.0);
    leader.setTestSpeed(10.0);

    subject.update(0.05);

    const bool passed = subject.getCurrentLaneIndex() == 0;
    std::ostringstream d;
    d << "  Expected: remain in lane 0 inside the 12 m no-change zone\n";
    d << "  Actual:   lane=" << subject.getCurrentLaneIndex() << "\n";
    reportResult(testName, passed, d.str());
}

void test_LaneChange_AllowsBlockedLaneEscapeNearIntersection() {
    std::string testName = "Lane change: blocked lane can still be escaped near an intersection";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 20.0, 1.0, 2);

    LaneTestCar subject(1, 20.0, &i1, &i2);
    subject.setRoute({&road});
    subject.placeOnLane(road, 0, 95.0);
    road.blockLane(0);

    subject.update(0.05);

    const bool passed = subject.getCurrentLaneIndex() == 1;
    std::ostringstream d;
    d << "  Expected: move to lane 1 despite being inside the no-change zone\n";
    d << "  Actual:   lane=" << subject.getCurrentLaneIndex() << "\n";
    reportResult(testName, passed, d.str());
}

void test_LaneChange_UsesAdaptiveIntersectionZoneOnShortRoad() {
    std::string testName = "Lane change: no-change zone scales down on short roads";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 20.0, 0.0);
    Road road(101, "Test", &i1, &i2, 20.0, 20.0, 1.0, 2);

    LaneTestCar subject(1, 20.0, &i1, &i2);
    LaneTestCar leader(2, 10.0, &i1, &i2);
    subject.setRoute({&road});
    leader.setRoute({&road});
    subject.placeOnLane(road, 0, 8.0);
    leader.placeOnLane(road, 0, 14.0);

    subject.update(0.05);

    const bool passed = subject.getCurrentLaneIndex() == 1;
    std::ostringstream d;
    d << "  Expected: lane change is allowed outside the final 20% (4 m)\n";
    d << "  Actual:   lane=" << subject.getCurrentLaneIndex() << "\n";
    reportResult(testName, passed, d.str());
}

void test_Yielding_DoesNotSlowVehiclesAlreadyOutsideEmergencyLane() {
    std::string testName = "Lane change: yielding does not slow an already clear lane";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    Road road(101, "Test", &i1, &i2, 100.0, 20.0, 1.0, 3);

    LaneTestCar subject(1, 20.0, &i1, &i2);
    subject.setRoute({&road});
    subject.placeOnLane(road, 1, 20.0);
    subject.setTestSpeed(20.0);

    subject.notifyEmergencyApproaching(2);
    subject.update(0.05);

    const bool passed = nearlyEqual(subject.getCurrentSpeed(), 20.0);
    std::ostringstream d;
    d << "  Expected: keep normal speed because lane 1 is already clear\n";
    d << "  Actual:   speed=" << subject.getCurrentSpeed() << "\n";
    reportResult(testName, passed, d.str());
}

void test_MixedLengthVehiclesUseBumperToBumperGap() {
    std::string testName =
        "Vehicle: mixed-length following uses both rectangular bodies";

    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 200.0, 0.0);
    Road road(101, "Mixed vehicle lane", &i1, &i2,
              200.0, 20.0);

    TestBus follower(1, 20.0, &i1, &i2);
    LaneTestCar leader(2, 0.0, &i1, &i2);
    follower.setRoute({&road});
    leader.setRoute({&road});
    follower.placeOnLane(road, 0, 50.0);
    leader.placeOnLane(road, 0, 60.0);
    follower.setTestSpeed(10.0);

    const double initialProgress =
        follower.getProgressOnRoad();
    follower.update(0.5);
    const double bodyGap =
        leader.getProgressOnRoad() -
        follower.getProgressOnRoad() -
        (leader.getLength() + follower.getLength()) * 0.5;
    const bool passed =
        nearlyEqual(follower.getProgressOnRoad(), initialProgress) &&
        bodyGap >= -1e-6;

    std::ostringstream d;
    d << "  Expected: a 12m bus must not advance into a 4.5m car\n";
    d << "  Actual: progress=" << follower.getProgressOnRoad()
      << " bodyGap=" << bodyGap << "\n";
    reportResult(testName, passed, d.str());
}

void test_SimulatorDefersUnsafeSpawnUntilEntranceIsClear() {
    std::string testName =
        "Simulator: burst spawning queues vehicles outside the road";

    Graph graph;
    graph.addIntersection(new Intersection(1, 0.0, 0.0));
    graph.addIntersection(new Intersection(2, 500.0, 0.0));
    graph.addRoad(new Road(
        101, "Spawn lane",
        graph.getIntersection(1),
        graph.getIntersection(2),
        500.0, 20.0));

    DijkstraStrategy strategy;
    TrafficSimulator simulator(&graph, &strategy);
    bool accepted = true;
    constexpr int vehicleCount = 12;
    for (int id = 0; id < vehicleCount; ++id) {
        Vehicle* vehicle = nullptr;
        if (id % 3 == 0) {
            vehicle = new Bus(
                id, 15.0,
                graph.getIntersection(1),
                graph.getIntersection(2));
        } else if (id % 3 == 1) {
            vehicle = new Car(
                id, 20.0,
                graph.getIntersection(1),
                graph.getIntersection(2));
        } else {
            vehicle = new Motorbike(
                id, 20.0,
                graph.getIntersection(1),
                graph.getIntersection(2));
        }
        accepted = simulator.addVehicle(vehicle) && accepted;
    }

    const bool initialQueueIsSafe =
        simulator.getVehicles().size() == 1 &&
        simulator.getPendingVehicleCount() ==
            vehicleCount - 1;
    bool bodyClearancesAreSafe = true;
    for (int step = 0; step < 100; ++step) {
        simulator.update(0.05);
        std::vector<Vehicle*> activeOnRoad;
        for (Vehicle* vehicle : simulator.getVehicles()) {
            if (vehicle->getCurrentRoad() == graph.getRoad(101)) {
                activeOnRoad.push_back(vehicle);
            }
        }
        std::sort(
            activeOnRoad.begin(),
            activeOnRoad.end(),
            [](const Vehicle* first, const Vehicle* second) {
                return first->getProgressOnRoad() <
                       second->getProgressOnRoad();
            });
        for (std::size_t index = 1;
             index < activeOnRoad.size();
             ++index) {
            const Vehicle* follower =
                activeOnRoad[index - 1];
            const Vehicle* leader =
                activeOnRoad[index];
            const double bodyGap =
                leader->getProgressOnRoad() -
                follower->getProgressOnRoad() -
                (leader->getLength() +
                 follower->getLength()) * 0.5;
            bodyClearancesAreSafe =
                bodyClearancesAreSafe &&
                bodyGap + 1e-6 >=
                    std::max(
                        follower->getMinGap(),
                        leader->getMinGap());
        }
    }

    const bool pendingPoolProgressed =
        simulator.getPendingVehicleCount() <
            vehicleCount - 1;
    const bool allVehiclesOwned =
        simulator.getVehicles().size() +
            simulator.getPendingVehicleCount() +
            simulator.getFinishedVehicles().size() ==
        vehicleCount;
    const bool passed =
        accepted &&
        initialQueueIsSafe &&
        pendingPoolProgressed &&
        bodyClearancesAreSafe &&
        allVehiclesOwned;

    std::ostringstream d;
    d << "  Expected: one safe initial spawn, then gradual activation\n";
    d << "  Actual: active=" << simulator.getVehicles().size()
      << " pending=" << simulator.getPendingVehicleCount()
      << " clearancesSafe=" << bodyClearancesAreSafe << "\n";
    reportResult(testName, passed, d.str());
}
// ----------------------------------------------------------------------------
// main
// ----------------------------------------------------------------------------

int main() {
    std::cout << "============================================================\n";
    std::cout << " VEHICLE MODEL & POLYMORPHISM UNIT TESTS\n";
    std::cout << "============================================================\n\n";

    test_Polymorphic_Speed_Calculation();
    test_EmergencyVehicle_BlockedRoad();
    test_Bus_DwellTime_StateMachine();
    test_Bus_MultipleStopsUsePerStopDwellAndDoNotRepeat();
    test_NonBusVehicle_IgnoresBusStops();
    test_Bus_RedLightIsNotDwellAndDwellDoesNotBypassRed();
    test_Bus_NoStopsKeepsNormalBehavior();
    test_Bus_UTurnRefreshesStopOnReverseRoad();
    test_Bus_UTurnToRoadWithoutStopsClearsNextStop();
    test_Bus_UTurnSkipsReverseStopBehindMirroredProgress();
    test_Vehicle_IntersectionPauseReasonPersistsAndResumes();
    test_Vehicle_TrafficLightTakesPriorityOverIntersection();
    test_Vehicle_NonControlObstaclesDoNotUseIntersectionPause();
    test_Bus_DwellUsesRemainingUpdateTime();
    test_Bus_IncompleteDwellConsumesOnlyAvailableTime();
    test_Bus_DwellCompletionDoesNotBypassRedLight();
    test_Bus_DwellCompletionDoesNotBypassIntersection();
    test_Bus_ChangesOneLaneAtATimeTowardStop();
    test_Bus_UsesAdaptiveDistanceAndDwellsInCurbLane();
    test_Bus_RejectsUnsafeFrontGapTowardStopLane();
    test_Bus_RejectsUnsafeRearGapTowardStopLane();
    test_Bus_RetriesRequiredLaneWhenGapBecomesSafe();
    test_Bus_SlowsGraduallyWhileWaitingForStopLaneGap();
    test_Bus_BlockedStopLaneIsMissedWithoutControlPause();
    test_Bus_ReverseRoadUsesItsOwnCurbStopLane();
    test_Bus_DoesNotServeStopFromWrongLane();
    test_Vehicle_RouteAdvancement();
    test_Vehicle_RecalculateRoute_PreservesProgressNoGrowth();
    test_Vehicle_StopsAtRedTrafficLight();
    test_LaneChange_DoesNotEnterEmergencyLane();
    test_YieldLaneChange_CanExitEmergencyLane();
    test_LaneChange_SelectsBestEligibleLane();
    test_LaneChange_RejectsFastRearFollower();
    test_YieldLaneChange_DoesNotSkipBlockedAdjacentLane();
    test_LaneChange_StopsOpportunisticChangeNearIntersection();
    test_LaneChange_AllowsBlockedLaneEscapeNearIntersection();
    test_LaneChange_UsesAdaptiveIntersectionZoneOnShortRoad();
    test_Yielding_DoesNotSlowVehiclesAlreadyOutsideEmergencyLane();
    test_MixedLengthVehiclesUseBumperToBumperGap();
    test_SimulatorDefersUnsafeSpawnUntilEntranceIsClear();


    printSummary();

    return (TestFramework::g_failed == 0) ? 0 : 1;
}
