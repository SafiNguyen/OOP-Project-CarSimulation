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
        std::vector<Road*> currentRoute;
        int currentRouteIndex;
        std::vector<Road*> travelHistory;
        bool paused;
        bool routeAssigned = false;


    public:
        Vehicle(int id, double speed, Intersection* start, Intersection* dest);
        Vehicle(const Vehicle&) = delete;
        Vehicle& operator=(const Vehicle&) = delete;
        Vehicle(Vehicle&&) = delete;
        Vehicle& operator=(Vehicle&&) = delete;

        virtual ~Vehicle() = default;

        virtual double calculateCurrentSpeed() const = 0;
        virtual void onRoadChanged() {}

        virtual bool shouldPauseAt(double currentPos,
                                    double projectedPos,
                                    double& pausePos) { return false; }
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

        void addTravelHistory(Road* road) { travelHistory.push_back(road); }
        const std::vector<Road*>& getTravelHistory() const { return travelHistory; }

        // --- Dynamic Routing Functions ---
        bool isRoadInUpcomingRoute(int roadId) const    ;
        bool recalculateRoute(const Graph& graph, PathFindingStrategy* strategy);

        private:
        /** Advance to the next road in the route.  Returns false if route ends. */
        bool advanceToNextRoad();
    };

    #endif