#include "TrafficLight.h"

#include <algorithm>
#include <cmath>

namespace {

double positiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0
        ? value
        : fallback;
}

const char* stateLabel(LightState state) {
    switch (state) {
        case LightState::GREEN: return "GREEN";
        case LightState::YELLOW: return "YELLOW";
        case LightState::RED: return "RED";
    }
    return "UNKNOWN";
}

} // namespace

TrafficLight::TrafficLight(int roadId,
                           double green,
                           double yellow,
                           double red,
                           LightState initialState)
    : controlledRoadId(roadId),
      currentState(initialState),
      elapsedTime(0.0),
      remainingSeconds(0.0),
      greenDuration(positiveOr(green, 30.0)),
      yellowDuration(positiveOr(yellow, 3.0)),
      redDuration(positiveOr(red, 25.0)) {
    remainingSeconds = durationForState(currentState);
}

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

double TrafficLight::getRemainingSeconds() const {
    return std::max(0.0, remainingSeconds);
}

double TrafficLight::durationForState(LightState state) const {
    switch (state) {
        case LightState::GREEN: return greenDuration;
        case LightState::YELLOW: return yellowDuration;
        case LightState::RED: return redDuration;
    }
    return redDuration;
}

LightState TrafficLight::nextState(LightState state) const {
    switch (state) {
        case LightState::GREEN: return LightState::YELLOW;
        case LightState::YELLOW: return LightState::RED;
        case LightState::RED: return LightState::GREEN;
    }
    return LightState::RED;
}

void TrafficLight::update(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0) return;

    elapsedTime += dt;
    double duration = durationForState(currentState);
    while (elapsedTime >= duration) {
        elapsedTime -= duration;
        currentState = nextState(currentState);
        duration = durationForState(currentState);
    }
    remainingSeconds = std::max(0.0, duration - elapsedTime);
}

bool TrafficLight::canProceed() const {
    return currentState == LightState::GREEN;
}

bool TrafficLight::mustStop() const {
    return currentState != LightState::GREEN;
}

void TrafficLight::setDurations(double green,
                                double yellow,
                                double red) {
    greenDuration = positiveOr(green, greenDuration);
    yellowDuration = positiveOr(yellow, yellowDuration);
    redDuration = positiveOr(red, redDuration);
    const double duration = durationForState(currentState);
    elapsedTime = std::clamp(elapsedTime, 0.0, duration);
    remainingSeconds = duration - elapsedTime;
}

void TrafficLight::forceState(LightState state) {
    synchronize(state, durationForState(state));
}

void TrafficLight::synchronize(LightState state, double remaining) {
    currentState = state;
    const double duration = durationForState(state);
    remainingSeconds = std::max(
        0.0,
        std::isfinite(remaining) ? remaining : 0.0);
    elapsedTime = std::max(0.0, duration - remainingSeconds);
}

std::string TrafficLight::toString() const {
    return "TrafficLight[RoadId: " +
           std::to_string(controlledRoadId) +
           ", State: " + stateLabel(currentState) +
           ", Remaining: " +
           std::to_string(getRemainingSeconds()) + "s]";
}

TrafficLight::~TrafficLight() = default;
