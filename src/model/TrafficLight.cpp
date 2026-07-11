#include "TrafficLight.h"

#include <algorithm>

namespace {

double clampPositive(double value, double fallback) {
    return value > 0.0 ? value : fallback;
}

const char* stateLabel(LightState state) {
    switch (state) {
        case LightState::GREEN:  return "GREEN";
        case LightState::YELLOW: return "YELLOW";
        case LightState::RED:    return "RED";
    }
    return "UNKNOWN";
}

} // namespace

TrafficLight::TrafficLight(int RoadId,
                           double greenDuration,
                           double yellowDuration,
                           double redDuration,
                           LightState initialState)
    : controlledRoadId(RoadId),
      currentState(initialState),
      elapsedTime(0.0),
      greenDuration(clampPositive(greenDuration, 30.0)),
      yellowDuration(clampPositive(yellowDuration, 3.0)),
      redDuration(clampPositive(redDuration, 25.0)) {}

int TrafficLight::getControlledRoadId() const {
    return controlledRoadId;
}

LightState TrafficLight::getState() const {
    return currentState;
}

double TrafficLight::getElapsedTime() const {
    return elapsedTime;
}

double TrafficLight::getGreenDuration() const {
    return greenDuration;
}

double TrafficLight::getYellowDuration() const {
    return yellowDuration;
}

double TrafficLight::getRedDuration() const {
    return redDuration;
}

double TrafficLight::durationForState(LightState state) const {
    switch (state) {
        case LightState::GREEN:  return greenDuration;
        case LightState::YELLOW: return yellowDuration;
        case LightState::RED:    return redDuration;
    }
    return redDuration;
}

LightState TrafficLight::nextState(LightState state) const {
    switch (state) {
        case LightState::GREEN:  return LightState::YELLOW;
        case LightState::YELLOW: return LightState::RED;
        case LightState::RED:    return LightState::GREEN;
    }
    return LightState::RED;
}

void TrafficLight::update(double dt) {
    if (dt <= 0.0) {
        return;
    }

    elapsedTime += dt;

    double currentDuration = durationForState(currentState);
    while (elapsedTime >= currentDuration) {
        elapsedTime -= currentDuration;
        currentState = nextState(currentState);
        currentDuration = durationForState(currentState);
    }
}

bool TrafficLight::canProceed() const {
    return currentState == LightState::GREEN;
}

bool TrafficLight::mustStop() const {
    return currentState == LightState::RED
        || currentState == LightState::YELLOW;
}

void TrafficLight::setDurations(double green, double yellow, double red) {
    greenDuration = clampPositive(green, greenDuration);
    yellowDuration = clampPositive(yellow, yellowDuration);
    redDuration = clampPositive(red, redDuration);
}

void TrafficLight::forceState(LightState state) {
    currentState = state;
    elapsedTime = 0.0;
}

std::string TrafficLight::toString() const {
    return "TrafficLight[RoadId: " + std::to_string(controlledRoadId) +
           ", State: " + stateLabel(currentState) +
           ", Elapsed: " + std::to_string(elapsedTime) + "s]";
}

TrafficLight::~TrafficLight() = default;
