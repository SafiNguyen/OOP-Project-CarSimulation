#ifndef LANE_H
#define LANE_H

#include <vector>

class Vehicle;
constexpr double DEFAULT_LANE_CAPACITY = 10.0;


class Lane {
private:
    int laneIndex; 
    double capacity;
    bool blocked;
    std::vector<Vehicle*> vehicles;

public:
    explicit Lane(int laneIndex, double capacity = DEFAULT_LANE_CAPACITY);

    int getIndex() const;
    double getCapacity() const;
    void addVehicle(Vehicle* v);
    void removeVehicle(Vehicle* v);
    int getVehicleCount() const;
    const std::vector<Vehicle*>& getVehicles() const;

    bool isBlocked() const;
    void block();
    void unblock();
    
    ~Lane();
};

#endif