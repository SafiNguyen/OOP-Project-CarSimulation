# Urban Traffic Simulator

A C++ Object-Oriented software application that simulates traffic movement within a virtual city environment.

## Architecture

The project follows a modular, object-oriented design:

1. **Model Layer (`src/model`)**: Core data structures representing the city graph.
   - `Graph`: Contains roads and intersections.
   - `Intersection`: Nodes in the graph.
   - `Road`: Edges in the graph, handles speed limits, congestion, and weights.
   - `Vehicle`, `Car`, `Bus`, `Motorbike`, `EmergencyVehicle`: Moving entities with independent logic and speed calculations.

2. **Algorithm Layer (`src/algorithm`)**: Route planning algorithms using the Strategy Pattern.
   - `PathFindingStrategy` interface.
   - `BFSStrategy`, `DijkstraStrategy`, `AStarStrategy`.

3. **Simulation Layer (`src/simulation`)**: The engine driving the events and movement.
   - `TrafficSimulator`: Main loop manager handling ticks and updates.
   - `EventManager`: Triggers dynamic events (accidents, congestion) and notifies vehicles.
   - `StatisticsManager`: Collects travel time, recalculation counts, and algorithm benchmarks.

4. **Visualization Layer (`src/visualization`)**: SFML-based UI.
   - `VisualizationEngine`: Renders the static graph and view mapping.
   - `VehicleSprite`: Draws the simulation-owned `Pose2D`; it does not build
     turn geometry or interpolate between roads.
   - `StatsPanel`: Displays real-time simulation statistics.

## Motion and junction architecture

- `Geometry.h` provides SFML-independent `Vec2` and `Pose2D` model types.
- `MotionPath` has straight, cubic Bézier, circular-arc, composite, and
  roundabout implementations. Bézier paths precompute a 48-sample arc-length
  table, so vehicles advance in metres rather than treating parameter `t` as
  distance.
- `Intersection` caches `JunctionConnector` objects by incoming road/lane and
  outgoing road/lane. Normal intersections use cubic Bézier connectors;
  `Roundabout` overrides connector construction with entry, clockwise
  circulating arc, and exit segments.
- `TurnLanePolicy` classifies movements from model-space dot/cross products.
  Straight movements map relative lane position, right turns use curb lanes,
  and left/U-turn movements use median lanes.
- `Vehicle` uses the state sequence `OnRoad -> WaitingAtIntersection ->
  TraversingJunction -> OnRoad`. Incoming lane membership is removed when the
  connector is entered; route index, current road, and outgoing lane change
  only after the connector and rear-clearance distance are complete.
- Reservations are RAII-safe and live for the traversal distance, not a render
  timer. Normal junctions currently use the conservative whole-intersection
  policy. Roundabouts allow multiple separated reservations using entry-gap
  checks and reported connector progress.
- Curve speed is limited by `sqrt(maxLateralAcceleration / abs(curvature))`.
  Bus limits are lowest, followed by cars, emergency vehicles, and motorbikes.

## Map units

Internal physics uses metres, seconds, m/s, and radians. The JSON loader accepts
these root defaults (and per-road overrides where noted):

```json
{
  "coordinateUnit": "world",
  "defaultDistanceUnit": "legacy",
  "defaultSpeedUnit": "km/h",
  "defaultRadiusUnit": "km"
}
```

- `distanceUnit`: `m`, `km`, `mi`, or `legacy`. The named legacy adapter is
  exactly 20 metres per historical map distance unit.
- `speedUnit`: `m/s` or `km/h`.
- `radiusUnit`: `m`, `km`, or `mi`; radius must be finite and positive.
- Coordinates remain map/world coordinates. `RoadGeometry` derives a local
  metres-per-world-unit transform from road distance and endpoint geometry, so
  old maps with non-identical coordinate and distance scales remain compatible.
- Lane width is 3.5 metres. Rendering converts the same model geometry to
  screen space; roundabout rendering and traversal use the same radius.

## Build Instructions

**Prerequisites:**
- CMake 3.15 or newer
- A C++17 compatible compiler (e.g. GCC, Clang, MSVC)

**Building the Project:**
```bash
mkdir build
cd build
cmake ..
cmake --build .
ctest --test-dir . --output-on-failure
```

## Running the Simulation

After building, the executable `UrbanTrafficSimulator` will be generated in the `build/bin` (or `build/Debug/bin` on MSVC) directory.

```bash
# Run the simulator
./bin/UrbanTrafficSimulator
```

Alternatively, you can provide a JSON map file as an argument:
```bash
./bin/UrbanTrafficSimulator ../map.json
```

**Controls:**
- **Middle Mouse Click + Drag**: Pan the map.
- **Scroll Wheel** or **+/-**: Zoom in/out.
- **Space**: Pause / Resume the simulation.
- **R**: Reset view.
- **Esc**: Exit.
