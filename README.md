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
   - `VehicleSprite`: Draws dynamic vehicles on screen based on progress and type.
   - `StatsPanel`: Displays real-time simulation statistics.

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
