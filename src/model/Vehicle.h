    #ifndef VEHICLE_H
    #define VEHICLE_H

    #include <vector>

    class Intersection;
    class Road;
    class Graph;
    class PathFindingStrategy;
    
    class Vehicle {
    protected:
        int id;
        double baseSpeed;
        Intersection* spawnPoint; // The intersection where the vehicle starts its journey
        Intersection* destination; 
        Road* currentRoad;
        double progressOnCurrentRoad; 
        double currentSpeed;          // actual current speed (ramps towards target via accel/decel)
        std::vector<Road*> currentRoute;
        int currentRouteIndex;
        std::vector<Road*> travelHistory;
        bool paused;
        bool routeAssigned = false;
        int currentLaneIndex = 0; 


    public:
        Vehicle(int id, double speed, Intersection* start, Intersection* dest);
        Vehicle(const Vehicle&) = delete;
        Vehicle& operator=(const Vehicle&) = delete;
        Vehicle(Vehicle&&) = delete;
        Vehicle& operator=(Vehicle&&) = delete;

        virtual ~Vehicle();

        // Returns the theoretical MAX/target speed for the current road & vehicle
        // type (based on speed limit, congestion, blocked status, etc). This is
        // NOT the instantaneous speed the vehicle is actually moving at - see
        // getCurrentSpeed() for that.
        virtual double calculateCurrentSpeed() const = 0;
        virtual void onRoadChanged() {}

        // --- Vehicle physics (acceleration / deceleration) ---
        // Subclasses may override these to give each vehicle type its own
        // "feel" (e.g. a Bus accelerates slower than a Motorbike, an
        // EmergencyVehicle brakes/accelerates the fastest).
        // Units: distance-units / second^2 (consistent with baseSpeed's
        // distance-units / second).
        virtual double getAcceleration() const { return 15.0; }
        virtual double getDeceleration() const { return 25.0; }

        // The vehicle's actual current speed (after ramping via
        // acceleration/deceleration towards calculateCurrentSpeed()).
        double getCurrentSpeed() const { return currentSpeed; }

        virtual bool shouldPauseAt(double currentPos,
                                    double projectedPos,
                                    double& pausePos) { return false; }
        virtual bool mustStopForTrafficLight(Intersection* nextIntersection) const;
        virtual void onPauseStarted() {}
        virtual bool updatePause(double dt) { return true; }
        virtual void update(double dt);
        void setRoute(const std::vector<Road*>& route);

        // Getters
        int getId() const { return id; }
        Intersection* getSpawnPoint() const { return spawnPoint; }
        Intersection* getDestination() const { return destination; }
        double getBaseSpeed() const { return baseSpeed; }
        Road* getCurrentRoad() const { return currentRoad; }
        bool isPaused() const{ return paused; }
        bool hasReachedDestination() const {
            return routeAssigned
                && currentRoad == nullptr
                && currentRouteIndex >= (int)currentRoute.size();
        }
        double getProgressRatio() const;
        int getCurrentLaneIndex() const { return currentLaneIndex; }


        void addTravelHistory(Road* road) { travelHistory.push_back(road); }
        const std::vector<Road*>& getTravelHistory() const { return travelHistory; }

        // --- Dynamic Routing Functions ---
        bool isRoadInUpcomingRoute(int roadId) const;
        bool recalculateRoute(const Graph& graph, PathFindingStrategy* strategy);
        bool performUTurn(const Graph& graph, PathFindingStrategy* strategy);

        private:
        /** Advance to the next road in the route.  Returns false if route ends. */
        bool advanceToNextRoad();
    };

    #endif