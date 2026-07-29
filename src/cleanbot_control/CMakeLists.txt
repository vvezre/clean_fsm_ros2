cmake_minimum_required(VERSION 3.8)
project(cleanbot_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cleanbot_common REQUIRED)
find_package(cleanbot_config REQUIRED)
find_package(cleanbot_interfaces REQUIRED)

add_library(control_core
  src/tracking_core.cpp
  src/command_arbiter_core.cpp
  src/joystick_mapper.cpp
)
target_include_directories(control_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(tracking_node src/tracking_node.cpp)
target_link_libraries(tracking_node control_core)
ament_target_dependencies(tracking_node rclcpp cleanbot_common cleanbot_config cleanbot_interfaces)

add_executable(command_arbiter_node src/command_arbiter_node.cpp)
target_link_libraries(command_arbiter_node control_core)
ament_target_dependencies(command_arbiter_node rclcpp cleanbot_common cleanbot_config cleanbot_interfaces)

install(TARGETS control_core tracking_node command_arbiter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tracking_core test/test_tracking_core.cpp)
  target_link_libraries(test_tracking_core control_core)
  ament_add_gtest(test_command_arbiter_core test/test_command_arbiter_core.cpp)
  target_link_libraries(test_command_arbiter_core control_core)
  ament_add_gtest(test_joystick_mapper test/test_joystick_mapper.cpp)
  target_link_libraries(test_joystick_mapper control_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(control_core)
ament_package()
