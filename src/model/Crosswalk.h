#ifndef CROSSWALK_H
#define CROSSWALK_H

#include <vector>

#include "Geometry.h"

class Intersection;
class Pedestrian;
class Road;

enum class PedestrianSignalState {
    DontWalk,
    Walk,
    Clearance
};

struct CrosswalkTiming {
    double minimumWaitSeconds = 3.0;
    double walkDurationSeconds = 4.0;
    double designWalkingSpeedMetresPerSecond = 1.2;
    double clearanceBufferSeconds = 1.0;
};

// Static pedestrian infrastructure. Graph owns Crosswalk; references to
// roads, its intersection and pedestrians are non-owning.
class Crosswalk {
public:
    static constexpr double DEFAULT_WIDTH_METRES = 3.0;
    static constexpr double JUNCTION_EDGE_GAP_METRES = 0.5;
    static constexpr double STOP_LINE_GAP_METRES = 1.0;

    Crosswalk(int id,
              Intersection* intersection,
              Road* incomingRoad,
              double widthMetres = DEFAULT_WIDTH_METRES,
              CrosswalkTiming timing = {});

    Crosswalk(const Crosswalk&) = delete;
    Crosswalk& operator=(const Crosswalk&) = delete;

    bool isValid() const;
    int getId() const;
    Intersection* getIntersection() const;
    Road* getIncomingRoad() const;
    Road* getReverseRoad() const;
    double getWidthMetres() const;
    const CrosswalkTiming& getTiming() const;

    double getStartProgressMetres() const;
    double getEndProgressMetres() const;
    double getCentreProgressMetres() const;
    double getReverseCentreProgressMetres() const;
    double getStopLineProgressMetres() const;

    Vec2 getSideAPosition() const;
    Vec2 getSideBPosition() const;
    double getCrossingLengthMetres() const;
    double getClearanceDurationSeconds() const;

    void requestEntry(Pedestrian& pedestrian);
    void cancelRequest(int pedestrianId);
    void notifyEntered(Pedestrian& pedestrian);
    void notifyExited(Pedestrian& pedestrian);
    void grantEligiblePedestrians();

    void synchronizeSignalState(PedestrianSignalState state);
    PedestrianSignalState getSignalState() const;
    bool canStartCrossing(const Pedestrian& pedestrian) const;
    bool hasWaitingPedestrians() const;
    bool hasEligibleRequest() const;
    bool isOccupied() const;
    double getOldestRequestAgeSeconds() const;

private:
    static bool containsPedestrian(
        const std::vector<Pedestrian*>& pedestrians,
        int pedestrianId);
    static void erasePedestrian(
        std::vector<Pedestrian*>& pedestrians,
        int pedestrianId);

    int id_;
    Intersection* intersection_;
    Road* incomingRoad_;
    double widthMetres_;
    CrosswalkTiming timing_;
    PedestrianSignalState signalState_ =
        PedestrianSignalState::DontWalk;
    std::vector<Pedestrian*> waiting_;
    std::vector<Pedestrian*> occupants_;
};

#endif
