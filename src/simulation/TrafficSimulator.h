#ifndef TRAFFICSIMULATOR_H
#define TRAFFICSIMULATOR_H

#include <vector>
#include <memory>

class Graph;
class Vehicle;
class EventManager;
class TrafficEvent;
class PathFindingStrategy;
class StatisticsManager;


class TrafficSimulator {
public:
    
    TrafficSimulator(Graph* graph, PathFindingStrategy* strategy);

    ~TrafficSimulator();

   
    TrafficSimulator(const TrafficSimulator&) = delete;
    TrafficSimulator& operator=(const TrafficSimulator&) = delete;

    bool addVehicle(Vehicle* vehicle);

   
    void removeFinishedVehicles();

    void triggerEvent(std::unique_ptr<TrafficEvent> event);

    void update(double dt);


    void pause();
    void resume();
    bool isPaused() const;

    void setSpeedMultiplier(double factor);
    double getSpeedMultiplier() const;


    const std::vector<Vehicle*>& getVehicles() const;
    const Graph& getGraph() const;
    StatisticsManager* getStatisticsManager() const;

    double getElapsedTime() const;

private:
    Graph* graph;                                   
    PathFindingStrategy* pathFindingStrategy;  
    std::vector<Vehicle*> vehicles;      
    std::unique_ptr<EventManager> eventManager;     
    std::unique_ptr<StatisticsManager> statisticsManager;               

    bool paused;
    double speedMultiplier;
    double elapsedTime;
    long long tickCount;
};

#endif