#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <string>

enum class LightState {
    RED,
    YELLOW,
    GREEN
};

class Intersection;

// Read-only signal-head projection. Intersection is the sole owner of signal
// timing and the only class allowed to synchronize state or durations.
class TrafficLight {
private:
    friend class Intersection;

    int controlledRoadId;
    LightState currentState;
    double remainingSeconds;
    double greenDuration;
    double yellowDuration;
    double redDuration;

    double durationForState(LightState state) const;
    void synchronize(LightState state, double remaining);
    void setDurations(double green, double yellow, double red);

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
    double getGreenDuration() const;
    double getYellowDuration() const;
    double getRedDuration() const;
    double getRemainingSeconds() const;

    bool canProceed() const;
    bool mustStop() const;

    std::string toString() const;

    ~TrafficLight();
};

#endif
