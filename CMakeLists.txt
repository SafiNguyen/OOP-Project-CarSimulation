cmake_minimum_required(VERSION 3.15)
project(UrbanTrafficSimulator CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

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
    opengl32
)

# Needed for imgui-sfml to find SFML headers
target_include_directories(imgui-sfml-lib PUBLIC
    ${sfml_SOURCE_DIR}/include
)

file(GLOB CORE_SOURCE_FILES CONFIGURE_DEPENDS
    "src/model/*.cpp"
    "src/algorithm/*.cpp"
    "src/mapload.cpp"
)

add_library(urban_core STATIC ${CORE_SOURCE_FILES})

target_include_directories(urban_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(urban_core PUBLIC
    nlohmann_json::nlohmann_json
)

add_executable(${PROJECT_NAME} src/main.cpp)

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
)

add_executable(test_graph tests/test_graph.cpp)
target_link_libraries(test_graph PRIVATE urban_core)
add_test(NAME graph_test COMMAND test_graph)

add_executable(test_vehicle tests/test_vehicle.cpp)
target_link_libraries(test_vehicle PRIVATE urban_core)
add_test(NAME vehicle_test COMMAND test_vehicle)