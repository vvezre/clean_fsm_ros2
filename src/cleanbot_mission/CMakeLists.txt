cmake_minimum_required(VERSION 3.8)
project(cleanbot_mission)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cleanbot_common REQUIRED)
find_package(cleanbot_config REQUIRED)
find_package(cleanbot_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(mission_core
  src/maintenance_gate.cpp
  src/mission_state_machine.cpp
  src/straight_edge_guard.cpp
  src/mission_checkpoint.cpp
  src/waypoint_plan.cpp
  src/vehicle_state_builder.cpp
)
target_include_directories(mission_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mission_core cleanbot_interfaces)
target_link_libraries(mission_core nlohmann_json::nlohmann_json)

add_executable(mission_manager_node src/mission_manager_node.cpp)
target_link_libraries(mission_manager_node mission_core)
ament_target_dependencies(
  mission_manager_node
  rclcpp
  rclcpp_action
  cleanbot_common
  cleanbot_config
  cleanbot_interfaces
)
target_link_libraries(mission_manager_node nlohmann_json::nlohmann_json)

install(TARGETS mission_core mission_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_maintenance_gate test/test_maintenance_gate.cpp)
  target_link_libraries(test_maintenance_gate mission_core)
  ament_add_gtest(test_mission_state_machine test/test_mission_state_machine.cpp)
  target_link_libraries(test_mission_state_machine mission_core)
  ament_add_gtest(test_mission_checkpoint test/test_mission_checkpoint.cpp)
  target_link_libraries(test_mission_checkpoint mission_core nlohmann_json::nlohmann_json)
  ament_target_dependencies(test_mission_checkpoint cleanbot_interfaces)
  ament_add_gtest(test_straight_edge_guard test/test_straight_edge_guard.cpp)
  target_link_libraries(test_straight_edge_guard mission_core)
  ament_add_gtest(test_waypoint_plan test/test_waypoint_plan.cpp)
  target_link_libraries(test_waypoint_plan mission_core)
  ament_add_gtest(
    test_vehicle_state_builder
    test/test_vehicle_state_builder.cpp
  )
  target_link_libraries(test_vehicle_state_builder mission_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(mission_core)
ament_export_dependencies(cleanbot_interfaces nlohmann_json)
ament_package()
