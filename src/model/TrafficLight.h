#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <string>

enum class LightState {
    RED,
    YELLOW,
    GREEN
};

class Intersection;

// A signal head is a read-only projection of the intersection controller.
// Its standalone update/force API remains for backwards compatibility and
// focused model tests, while production intersections synchronize every
// head from one shared phase clock.
class TrafficLight {
private:
    friend class Intersection;

    int controlledRoadId;
    LightState currentState;
    double elapsedTime;
    double remainingSeconds;
    double greenDuration;
    double yellowDuration;
    double redDuration;

    double durationForState(LightState state) const;
    LightState nextState(LightState state) const;
    void synchronize(LightState state, double remaining);

public:
    TrafficLight(int roadId,
                 double greenDuration = 30.0,
                 double yellowDuration = 3.0,
                 double redDuration = 25.0,
                 LightState initialState = LightState::RED);

    TrafficLight(const TrafficLight&) = delete;
    TrafficLight& operator=(const TrafficLight&) = delete;

    int getControlledRoadId() const;
    LightState getState() const;
    double getElapsedTime() const;
    double getGreenDuration() const;
    double getYellowDuration() const;
    double getRedDuration() const;
    double getRemainingSeconds() const;

    void update(double dt);
    bool canProceed() const;
    bool mustStop() const;

    void setDurations(double green, double yellow, double red);
    void forceState(LightState state);

    std::string toString() const;

    ~TrafficLight();
};

#endif
