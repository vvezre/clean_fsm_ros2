cmake_minimum_required(VERSION 3.8)
project(cleanbot_http)

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
find_package(cleanbot_control REQUIRED)
find_package(cleanbot_interfaces REQUIRED)

add_library(http_control_core
  src/http_control_router.cpp
  src/joystick_sequence_guard.cpp
  src/service_response_mapping.cpp
  src/user_control_request.cpp
  src/vehicle_state_json.cpp
)
target_include_directories(http_control_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(http_control_core cleanbot_control)

add_library(http_transport
  src/http_server.cpp
  src/http_session.cpp
)
target_include_directories(http_transport
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  http_transport
  http_control_core
  Boost::system
  Threads::Threads
)

add_executable(http_gateway_node src/http_gateway_node.cpp)
target_link_libraries(
  http_gateway_node
  http_transport
  Boost::system
  Threads::Threads
)
ament_target_dependencies(
  http_gateway_node
  rclcpp
  cleanbot_common
  cleanbot_config
  cleanbot_control
  cleanbot_interfaces
)

install(TARGETS http_control_core http_transport http_gateway_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_http_control_router test/test_http_control_router.cpp)
  target_link_libraries(test_http_control_router http_control_core)

  ament_add_gtest(test_http_server test/test_http_server.cpp)
  target_link_libraries(
    test_http_server
    http_transport
    Boost::system
    Threads::Threads
  )

  ament_add_gtest(
    test_user_control_request
    test/test_user_control_request.cpp
  )
  target_link_libraries(test_user_control_request http_control_core)

  ament_add_gtest(
    test_vehicle_state_json
    test/test_vehicle_state_json.cpp
  )
  target_link_libraries(test_vehicle_state_json http_control_core)

  ament_add_gtest(
    test_service_response_mapping
    test/test_service_response_mapping.cpp
  )
  target_link_libraries(test_service_response_mapping http_control_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(http_control_core http_transport)
ament_export_dependencies(cleanbot_control)
ament_package()
