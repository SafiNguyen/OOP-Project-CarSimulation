#ifndef TRAFFIC_EVENT_H
#define TRAFFIC_EVENT_H

#include <string>
#include "Graph.h"

class TrafficEvent {
protected:
    int roadId;
    double duration;
    double timeElapsed;
    bool active;

public:
    TrafficEvent(int rId, double dur) : roadId(rId), duration(dur), timeElapsed(0.0), active(true) {}
    virtual ~TrafficEvent() = default;

    // Polymorphism: Apply and remove the event
    virtual void apply(Graph& graph) = 0;
    virtual void remove(Graph& graph) = 0;
    
    virtual std::string getEventType() const = 0;

    void update(double dt, Graph& graph) {
        if (!active) return;
        timeElapsed += dt;
        if (timeElapsed >= duration) {
            remove(graph);
            active = false; // Event expired
        }
    }

    bool isActive() const { return active; }
    int getRoadId() const { return roadId; }
    double getDuration() const { return duration; }
    double getTimeElapsed() const { return timeElapsed; }
    void restoreTimeElapsed(double elapsed) {
        timeElapsed = elapsed < 0.0 ? 0.0 : elapsed;
        active = timeElapsed < duration;
    }
};

// Congestion Event
class CongestionEvent : public TrafficEvent {
private:
    double severity; // Congestion severity (e.g., 3.0 means speed is reduced by 3 times)
public:
    CongestionEvent(int rId, double dur, double severityLevel) 
        : TrafficEvent(rId, dur), severity(severityLevel) {}

    void apply(Graph& graph) override {
        graph.updateRoadCondition(roadId, severity, false); 
    }

    void remove(Graph& graph) override {
        graph.updateRoadCondition(roadId, 1.0, false); // Return to normal
    }

    std::string getEventType() const override { return "Congestion"; }
    double getSeverity() const { return severity; }
};

// Accident Event
class AccidentEvent : public TrafficEvent {
private:
    int laneIndex;
public:
    AccidentEvent(int rId, double dur, int lane = -1) : TrafficEvent(rId, dur), laneIndex(lane) {}

    void apply(Graph& graph) override {
        graph.updateRoadCondition(roadId, 1.0, true, laneIndex); // Block the specific lane (or all if -1)
    }

    void remove(Graph& graph) override {
        graph.updateRoadCondition(roadId, 1.0, false, laneIndex); // Reopen the lane
    }

    std::string getEventType() const override { return "Accident"; }
    int getLaneIndex() const { return laneIndex; }
};

// Road Closure Event
class RoadClosureEvent : public TrafficEvent {
public:
    RoadClosureEvent(int rId, double dur) : TrafficEvent(rId, dur) {}

    void apply(Graph& graph) override {
        graph.updateRoadCondition(roadId, 1.0, true); // Disable the road
    }

    void remove(Graph& graph) override {
        graph.updateRoadCondition(roadId, 1.0, false); // Re-enable the road
    }

    std::string getEventType() const override { return "Road Closure"; }
};

#endif
