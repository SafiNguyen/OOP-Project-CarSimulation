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

using TestFramework::reportResult;
using TestFramework::nearlyEqual;
using TestFramework::printSummary;

// ----------------------------------------------------------------------------
// Test Cases
// ----------------------------------------------------------------------------

void test_Polymorphic_Speed_Calculation() {
    std::string testName = "Vehicle: Polymorphic speed respects congestion, weaving, and cruise factors";
    
    Intersection i1(1, 0.0, 0.0);
    Intersection i2(2, 100.0, 0.0);
    // Road: distance 100, speedLimit 50, congestion 3.0
    Road road(101, &i1, &i2, 100.0, 50.0, 3.0); 

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
    Road road(101, &i1, &i2, 100.0, 50.0);
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
    Road road(101, &i1, &i2, 100.0, 50.0, 1.0);
    
    // Add a bus stop at 20 meters
    road.addBusStop(20.0);

    // Bus starts with default 15s dwell time
    Bus bus(1, 50.0, &i1, &i2, 15.0);
    bus.setRoute({&road});

    // Speed should be 50 * 0.85 = 42.5 m/s
    // To reach 20m, it needs dt = 20 / 42.5 = ~0.47s
    bus.update(0.48); // Đi hơi lố 1 xíu để kích hoạt trạm

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
    
    Road r1(101, &i1, &i2, 10.0, 10.0, 1.0);
    Road r2(102, &i2, &i3, 10.0, 10.0, 1.0);

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

    printSummary();

    return (TestFramework::g_failed == 0) ? 0 : 1;
}