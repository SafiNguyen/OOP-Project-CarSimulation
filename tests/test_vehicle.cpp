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
#include "../src/model/TrafficLight.h"
#include "../src/algorithm/DijkstraStrategy.h" 

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
    Bus bus(1, 50.0, &i1, &i2, 15.0);
    bus.setRoute({&road});

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

    Car car(1, 10.0, &i1, &i3);
    car.setRoute({&r1, &r2});

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
    TrafficLight* light = g.getIntersection(2)->getLightForIncomingRoad(road);
    bool hasRedLight = (light != nullptr) && light->mustStop();

    Car car(1, 30.0, g.getIntersection(1), g.getIntersection(2));
    car.setRoute({road});

    car.update(6.0);

    const bool passed = hasRedLight &&
                        (car.getCurrentRoad() == road) &&
                        !car.hasReachedDestination() &&
                        nearlyEqual(car.getCurrentSpeed(), 0.0) &&
                        nearlyEqual(car.getProgressRatio() * road->getDistance(), road->getDistance());

    std::ostringstream d;
    d << "  Expected: vehicle remains on the road and stops at the red light near the stop line.\n";
    d << "  Light was red: " << hasRedLight << "\n";
    d << "  Actual:   currentRoad=" << (car.getCurrentRoad() ? std::to_string(car.getCurrentRoad()->getId()) : "null")
      << ", speed=" << car.getCurrentSpeed()
      << ", reached=" << car.hasReachedDestination()
      << ", progress=" << (car.getProgressRatio() * road->getDistance()) << "\n";
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


    printSummary();

    return (TestFramework::g_failed == 0) ? 0 : 1;
}
