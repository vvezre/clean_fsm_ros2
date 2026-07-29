cmake_minimum_required(VERSION 3.8)
project(cleanbot_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cleanbot_common REQUIRED)
find_package(cleanbot_config REQUIRED)
find_package(cleanbot_interfaces REQUIRED)

add_library(lower_machine_protocol
  src/ack_codec.cpp
  src/command_encoder.cpp
  src/command_lifecycle.cpp
  src/frame_buffer.cpp
  src/gateway_readiness.cpp
  src/status_frame_parser.cpp
  src/write_queue.cpp
)
target_include_directories(lower_machine_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(lower_machine_transport src/lower_machine_serial.cpp)
target_include_directories(lower_machine_transport
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lower_machine_transport
  lower_machine_protocol
  Boost::system
  Threads::Threads
)

add_executable(lower_machine_node src/lower_machine_node.cpp)
target_link_libraries(lower_machine_node lower_machine_protocol lower_machine_transport)
ament_target_dependencies(
  lower_machine_node
  rclcpp
  cleanbot_common
  cleanbot_config
  cleanbot_interfaces
)

install(TARGETS lower_machine_protocol lower_machine_transport lower_machine_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_buffer test/test_frame_buffer.cpp)
  target_link_libraries(test_frame_buffer lower_machine_protocol)
  ament_add_gtest(test_status_frame_parser test/test_status_frame_parser.cpp)
  target_link_libraries(test_status_frame_parser lower_machine_protocol)
  ament_add_gtest(test_command_encoder test/test_command_encoder.cpp)
  target_link_libraries(test_command_encoder lower_machine_protocol)
  ament_add_gtest(test_ack_codec test/test_ack_codec.cpp)
  target_link_libraries(test_ack_codec lower_machine_protocol)
  ament_add_gtest(test_command_lifecycle test/test_command_lifecycle.cpp)
  target_link_libraries(test_command_lifecycle lower_machine_protocol)
  ament_add_gtest(test_write_queue test/test_write_queue.cpp)
  target_link_libraries(test_write_queue lower_machine_protocol)
  ament_add_gtest(test_gateway_readiness test/test_gateway_readiness.cpp)
  target_link_libraries(test_gateway_readiness lower_machine_protocol)
endif()

ament_export_include_directories(include)
ament_export_libraries(lower_machine_protocol lower_machine_transport)
ament_package()
