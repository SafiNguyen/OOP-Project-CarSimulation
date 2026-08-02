cmake_minimum_required(VERSION 3.15)
project(UrbanTrafficSimulator CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
find_package(OpenGL REQUIRED)

FetchContent_Declare(
    SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG        2.6.1
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.89.9
    GIT_SHALLOW    TRUE
)

FetchContent_Declare(
    imgui-sfml
    GIT_REPOSITORY https://github.com/SFML/imgui-sfml.git
    GIT_TAG        v2.6
    GIT_SHALLOW    TRUE
)

if (MINGW)
    set(SFML_USE_SYSTEM_DEPS ON CACHE BOOL "Use system deps for SFML on MinGW" FORCE)
    set(SFML_BUILD_AUDIO OFF CACHE BOOL "Build SFML audio module" FORCE)
    set(SFML_BUILD_NETWORK OFF CACHE BOOL "Build SFML network module" FORCE)
endif()

# Populate sources only — do NOT call MakeAvailable on imgui-sfml
FetchContent_MakeAvailable(SFML nlohmann_json imgui)
FetchContent_Populate(imgui-sfml)   # only downloads, skips its CMakeLists

# Build imgui + imgui-sfml as a plain static library ourselves
add_library(imgui-sfml-lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui-sfml_SOURCE_DIR}/imgui-SFML.cpp
)

target_include_directories(imgui-sfml-lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui-sfml_SOURCE_DIR}
)

target_link_libraries(imgui-sfml-lib PUBLIC
    sfml-graphics
    sfml-window
    sfml-system
    OpenGL::GL
)

# Needed for imgui-sfml to find SFML headers
target_include_directories(imgui-sfml-lib PUBLIC
    ${sfml_SOURCE_DIR}/include
)

set(CORE_SOURCE_FILES
    src/InputHandling.cpp
    src/algorithm/AStarStrategy.cpp
    src/algorithm/BFSStrategy.cpp
    src/algorithm/DijkstraStrategy.cpp
    src/loader/DemoMap.cpp
    src/loader/GraphBuilder.cpp
    src/loader/MapLoading.cpp
    src/loader/Mapload.cpp
    src/loader/MapParser.cpp
    src/model/Graph.cpp
    src/model/MotionPath.cpp
    src/model/infrastructure/Intersection.cpp
    src/model/infrastructure/JunctionConnector.cpp
    src/model/infrastructure/Lane.cpp
    src/model/infrastructure/LaneMapping.cpp
    src/model/infrastructure/Road.cpp
    src/model/infrastructure/RoadGeometry.cpp
    src/model/infrastructure/Roundabout.cpp
    src/model/infrastructure/TrafficLight.cpp
    src/model/vehicle/BusService.cpp
    src/model/vehicle/BusSnapshot.cpp
    src/model/vehicle/BusStop.cpp
    src/model/vehicle/Vehicle.cpp
    src/model/vehicle/VehicleFactory.cpp
    src/model/vehicle/VehicleJunction.cpp
    src/model/vehicle/VehicleLaneChange.cpp
    src/model/vehicle/VehicleMovement.cpp
    src/model/vehicle/VehiclePOI.cpp
    src/model/vehicle/VehicleSignaling.cpp
    src/model/vehicle/components/CarFollowingModel.cpp
    src/model/vehicle/components/JunctionTraversalState.cpp
    src/model/vehicle/components/LaneChangePolicy.cpp
    src/model/vehicle/components/RouteFollower.cpp
    src/model/vehicle/components/VehicleBehavior.cpp
    src/model/vehicle/components/VehicleDynamics.cpp
    src/simulation/BusTripPlanner.cpp
    src/simulation/EventManager.cpp
    src/simulation/SnapshotManager.cpp
    src/simulation/StatisticsManager.cpp
    src/simulation/TimePlaybackController.cpp
    src/simulation/TrafficSimulator.cpp
    src/simulation/VehicleSpawnPolicy.cpp
    src/ui/DebugConsole.cpp
    src/ui/DebugConsoleAccident.cpp
    src/ui/DebugConsoleAddRoad.cpp
    src/ui/DebugConsoleAlgorithm.cpp
    src/ui/DebugConsoleSpawnVehicle.cpp
    src/ui/DebugConsoleTopBar.cpp
    src/ui/DebugConsoleTrafficLight.cpp
    src/ui/MapFileDialog.cpp
    src/ui/StatsPanel.cpp
    src/ui/UiTheme.cpp
    src/ui/VehicleInspector.cpp
    src/visualization/Camera.cpp
    src/visualization/Rendering.cpp
    src/visualization/SimulatorFactory.cpp
    src/visualization/VehicleAssets.cpp
    src/visualization/VehicleSprite.cpp
    src/visualization/VisualizationEngine.cpp
    src/visualization/VisualizationEngineGeometry.cpp
    src/visualization/VisualizationEngineOverlays.cpp
    src/visualization/VisualizationEngineSidewalks.cpp
)

add_library(urban_core STATIC ${CORE_SOURCE_FILES})

target_include_directories(urban_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model/vehicle
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model/infrastructure
    ${CMAKE_CURRENT_SOURCE_DIR}/src/loader
    ${sfml_SOURCE_DIR}/include
    ${imgui_SOURCE_DIR}
    ${imgui-sfml_SOURCE_DIR}
)

target_link_libraries(urban_core PUBLIC
    nlohmann_json::nlohmann_json
)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/InputHandling.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    urban_core
    imgui-sfml-lib
    sfml-graphics
    sfml-window
    sfml-system
)

add_executable(test_pathfinding
    tests/test_pathfinding.cpp
)

target_link_libraries(test_pathfinding PRIVATE
    urban_core
)

enable_testing()
add_test(NAME pathfinding COMMAND test_pathfinding)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        $<TARGET_FILE_DIR:sfml-system>
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/assets
        $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets
)

add_executable(test_graph tests/test_graph.cpp)
target_link_libraries(test_graph PRIVATE urban_core)
add_test(NAME graph_test COMMAND test_graph)

add_executable(test_vehicle tests/test_vehicle.cpp)
target_link_libraries(test_vehicle PRIVATE urban_core)
add_test(NAME vehicle_test COMMAND test_vehicle)

add_executable(test_visualization tests/test_visualization.cpp)
target_link_libraries(test_visualization PRIVATE
    urban_core
    sfml-graphics
    sfml-window
    sfml-system
)
add_test(NAME visualization_test COMMAND test_visualization)

add_executable(test_map_parser test_map_parser.cpp)
target_link_libraries(test_map_parser PRIVATE urban_core)
add_test(NAME map_parser_test COMMAND test_map_parser)
set_tests_properties(map_parser_test PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(test_crossing_render tests/test_crossing_render.cpp)
target_link_libraries(test_crossing_render PRIVATE
    urban_core
    sfml-graphics
    sfml-window
    sfml-system
)
add_test(NAME crossing_render_test COMMAND test_crossing_render)

add_executable(test_motion tests/test_motion.cpp)
target_link_libraries(test_motion PRIVATE urban_core)
add_test(NAME motion_test COMMAND test_motion)
set_tests_properties(motion_test PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
