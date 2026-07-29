cmake_minimum_required(VERSION 3.8)
project(cleanbot_gateway)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cleanbot_common REQUIRED)
find_package(cleanbot_config REQUIRED)
find_package(cleanbot_control REQUIRED)
find_package(cleanbot_interfaces REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(gateway_core
  src/cloud_command.cpp
  src/cloud_message_codec.cpp
)
target_include_directories(gateway_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gateway_core cleanbot_control)
target_link_libraries(gateway_core nlohmann_json::nlohmann_json)

add_executable(cloud_gateway_node src/cloud_gateway_node.cpp)
target_link_libraries(
  cloud_gateway_node
  gateway_core
  PahoMqttCpp::paho-mqttpp3
)
ament_target_dependencies(
  cloud_gateway_node
  rclcpp
  cleanbot_common
  cleanbot_config
  cleanbot_control
  cleanbot_interfaces
)

install(TARGETS gateway_core cloud_gateway_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cloud_command test/test_cloud_command.cpp)
  target_link_libraries(test_cloud_command gateway_core)
  ament_add_gtest(test_cloud_message_codec test/test_cloud_message_codec.cpp)
  target_link_libraries(test_cloud_message_codec gateway_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(gateway_core)
ament_export_dependencies(cleanbot_control nlohmann_json)
ament_package()
