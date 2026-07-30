#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <string>

enum class LightState {
    RED,
    YELLOW,
    GREEN
};

/**
 * TrafficLight
 * ------------
 * Controls one incoming approach at an intersection.
 * Each light is bound to the road vehicles travel on when approaching the junction.
 */

class TrafficLight {
private:
    int controlledRoadId;
    LightState currentState;
    double elapsedTime; // thời gian đã trôi qua
    double greenDuration;
    double yellowDuration;
    double redDuration;

    double durationForState(LightState state) const;
    LightState nextState(LightState state) const;

public:
    TrafficLight(int RoadId,
                 double greenDuration = 30.0,
                 double yellowDuration = 3.0,
                 double redDuration = 25.0,
                 LightState initialState = LightState::RED);

    // Getters
    TrafficLight(const TrafficLight&) = delete;
    TrafficLight& operator=(const TrafficLight&) = delete;
    int getControlledRoadId() const;
    LightState getState() const;
    double getElapsedTime() const;
    double getGreenDuration() const;
    double getYellowDuration() const;
    double getRedDuration() const;
    

    void update(double dt);
    /** GREEN only — vehicle may enter or cross the intersection. */
    bool canProceed() const;
    /** RED or YELLOW — vehicle must stop before the stop line. */
    bool mustStop() const;

    void setDurations(double green, double yellow, double red);
    void forceState(LightState state);

    std::string toString() const;

    ~TrafficLight();
};

#endif