# CMake generated Testfile for 
# Source directory: C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation
# Build directory: C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/build-mingw
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(pathfinding "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/build-mingw/bin/test_pathfinding.exe")
set_tests_properties(pathfinding PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;119;add_test;C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;0;")
add_test(graph_test "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/build-mingw/bin/test_graph.exe")
set_tests_properties(graph_test PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;129;add_test;C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;0;")
add_test(vehicle_test "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/build-mingw/bin/test_vehicle.exe")
set_tests_properties(vehicle_test PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;133;add_test;C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;0;")
add_test(visualization_test "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/build-mingw/bin/test_visualization.exe")
set_tests_properties(visualization_test PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;142;add_test;C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;0;")
add_test(map_parser_test "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/build-mingw/bin/test_map_parser.exe")
set_tests_properties(map_parser_test PROPERTIES  WORKING_DIRECTORY "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation" _BACKTRACE_TRIPLES "C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;146;add_test;C:/Users/ADMIN/Documents/GitHub/OOP-Project-CarSimulation/CMakeLists.txt;0;")
subdirs("_deps/sfml-build")
subdirs("_deps/nlohmann_json-build")
