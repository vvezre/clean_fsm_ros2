# 文件作用：任务管理节点的 CMake 构建、安装与测试声明。
cmake_minimum_required(VERSION 3.8)
project(cleanbot_mission)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Windows 编译器按 UTF-8 解码源码，保证中文注释不会受系统代码页影响。
if(MSVC)
  add_compile_options(/utf-8)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rmw REQUIRED)
find_package(cleanbot_common REQUIRED)
find_package(cleanbot_config REQUIRED)
find_package(cleanbot_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(mission_core
  src/maintenance_gate.cpp
  src/maintenance_runtime.cpp
  src/maintenance_store.cpp
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
ament_target_dependencies(
  mission_core
  cleanbot_common
  cleanbot_interfaces
)
target_link_libraries(mission_core nlohmann_json::nlohmann_json)

add_executable(mission_manager_node src/mission_manager_node.cpp)
target_link_libraries(mission_manager_node mission_core)
ament_target_dependencies(
  mission_manager_node
  rclcpp
  rclcpp_action
  rmw
  cleanbot_common
  cleanbot_config
  cleanbot_interfaces
)
target_link_libraries(mission_manager_node nlohmann_json::nlohmann_json)

add_executable(maintenance_store_init src/maintenance_store_init.cpp)
target_link_libraries(maintenance_store_init mission_core)

install(TARGETS mission_core mission_manager_node maintenance_store_init
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_maintenance_gate test/test_maintenance_gate.cpp)
  target_link_libraries(test_maintenance_gate mission_core)
  ament_add_gtest(
    test_maintenance_runtime
    test/test_maintenance_runtime.cpp
  )
  target_link_libraries(test_maintenance_runtime mission_core)
  ament_add_gtest(
    test_maintenance_store
    test/test_maintenance_store.cpp
  )
  target_link_libraries(
    test_maintenance_store
    mission_core
    nlohmann_json::nlohmann_json
  )
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
ament_export_dependencies(
  cleanbot_common
  cleanbot_interfaces
  nlohmann_json
)
ament_package()
