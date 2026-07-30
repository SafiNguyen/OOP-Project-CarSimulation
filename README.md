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
  "coordinateUnit": "m",
  "defaultDistanceUnit": "m",
  "defaultSpeedUnit": "km/h",
  "defaultRadiusUnit": "m",
  "defaultLaneWidth": 3.5
}
```

- `distanceUnit`: `m`, `km`, `mi`, or `legacy`. The named legacy adapter is
  exactly 20 metres per historical map distance unit.
- `speedUnit`: `m/s` or `km/h`.
- `radiusUnit`: `m`, `km`, or `mi`; radius must be finite and positive.
- Coordinates remain map/world coordinates. `RoadGeometry` derives a local
  metres-per-world-unit transform from road distance and endpoint geometry, so
  old maps with non-identical coordinate and distance scales remain compatible.
- `laneWidth` may override `defaultLaneWidth` per road. Both are metres and
  must be finite and positive.
- Lane centres, road surfaces, lane/edge markings, stop lines, junction
  connectors, vehicle poses and rendering all come from `RoadGeometry`.
- Opposite directions are paired by reversed endpoints and cached on `Road`;
  road-id signs are not used to infer physical pairing.

## Traffic signal plans

Signalized intersections use one shared simulation-time controller. A plan is
declared explicitly and must list every incoming directional road exactly once:

```json
{
  "trafficLights": [
    {
      "intersectionId": 5,
      "enabled": true,
      "greenDuration": 25.0,
      "yellowDuration": 3.0,
      "allRedDuration": 1.5,
      "allowRightTurnOnRed": true,
      "phases": [
        { "incomingRoadIds": [105, 108] },
        { "incomingRoadIds": [117, 120] }
      ]
    }
  ]
}
```

The cycle is `GREEN -> YELLOW -> ALL_RED -> next GREEN`. Signal heads expose
the controller-derived remaining simulation seconds; the renderer does not
maintain a separate timer. A compact entry may omit `phases`; every incoming
road is then included automatically and geometrically opposite approaches are
grouped into the same phase:

```json
{
  "intersectionId": 2,
  "greenDuration": 25.0,
  "yellowDuration": 3.0,
  "allRedDuration": 1.5
}
```

The bundled maps use compact plans for T-junctions and roundabouts, while
keeping explicit phase plans at regular four-way intersections. A signalized
roundabout still applies its circulating-traffic yield and safe-gap checks
after an entry receives green.

`allowRightTurnOnRed` is optional and defaults to `true`. A right-turning
vehicle first stops at the stop line for at least 0.5 simulation seconds, then
yields to active pedestrian phases, green-priority approaches, junction
reservations and unavailable outgoing space. Setting it to `false` retains the
ordinary red-light stop rule.

Vehicles expose a model-owned left/right turn signal. Junction signals are
derived from `TurnLanePolicy`; lane changes signal for at least one simulation
second before committing. The renderer draws deterministic amber rear lamps
in vehicle-local coordinates, independently of emergency red/blue lighting.

## Pedestrians and signalized crosswalks

Pedestrians are independent simulation agents rather than `Vehicle`
subclasses. A pedestrian follows a route made from model-space sidewalk and
crosswalk segments, using the state sequence `Walking -> WaitingToCross ->
Crossing -> Walking -> Arrived`. `TrafficSimulator` owns active/completed
pedestrians, while `Graph` owns static `Crosswalk` infrastructure.

Crosswalk requests join the intersection's shared signal controller. When a
request has waited long enough, the sequence becomes `GREEN -> YELLOW ->
ALL_RED -> PEDESTRIAN_WALK -> PEDESTRIAN_CLEARANCE -> GREEN`. All vehicle
heads remain red during the pedestrian stages, new junction reservations are
rejected, and green is held if a slow pedestrian still occupies the crossing.
Pedestrians arriving during clearance wait for the next cycle.

Crosswalks are optional and backward-compatible:

```json
{
  "crosswalks": [
    {
      "id": 301,
      "intersectionId": 2,
      "incomingRoadId": 101,
      "enabled": true,
      "width": 3.0,
      "minimumWait": 3.0,
      "walkDuration": 4.0,
      "designWalkingSpeed": 1.2,
      "clearanceBuffer": 1.0
    }
  ]
}
```

The incoming road must end at the configured signalized intersection. When a
reverse direction exists, one crossing spans the paired physical road without
duplicate rendering. `RoadGeometry` derives sidewalk centres, zebra geometry
and an upstream stop line in metres; roads without crosswalks retain their
previous stop-line behavior. The deterministic demo factory creates a small
number of pedestrians with visible sidewalk approach and departure segments
for every configured crosswalk.

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

For deterministic visual QA without a desktop window, the same executable can
render a simulation snapshot:

```bash
./bin/UrbanTrafficSimulator ../map2.json \
  --snapshot map2.png --width 1600 --height 900 \
  --wall-seconds 5 --speed 2
```

**Controls:**
- **Middle Mouse Click + Drag**: Pan the map.
- **Scroll Wheel** or **+/-**: Zoom in/out.
- **Space**: Pause / Resume the simulation.
- **R**: Reset view.
- **Esc**: Exit.
