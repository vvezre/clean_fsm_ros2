# 文件作用：RTK 串口、差分定位节点的 CMake 构建、安装与测试声明。
cmake_minimum_required(VERSION 3.8)
project(cleanbot_rtk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Windows 编译器按 UTF-8 解码源码，保证中文注释不会受系统代码页影响。
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

add_library(rtk_protocol
  src/nmea_line_buffer.cpp
  src/nmea_parser.cpp
  src/ntrip_protocol.cpp
  src/rtcm3_frame_buffer.cpp
  src/rtk_validity.cpp
  src/rtk_sample_synchronizer.cpp
  src/vehicle_center_transform.cpp
)
target_include_directories(rtk_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(rtk_transport
  src/ntrip_client.cpp
  src/rtk_serial.cpp
)
target_include_directories(rtk_transport
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rtk_transport rtk_protocol Boost::system Threads::Threads)

add_executable(rtk_node src/rtk_node.cpp)
target_link_libraries(rtk_node rtk_protocol rtk_transport)
ament_target_dependencies(
  rtk_node
  rclcpp
  cleanbot_common
  cleanbot_config
  cleanbot_interfaces
)

install(TARGETS rtk_protocol rtk_transport rtk_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nmea_line_buffer test/test_nmea_line_buffer.cpp)
  target_link_libraries(test_nmea_line_buffer rtk_protocol)
  ament_add_gtest(test_nmea_parser test/test_nmea_parser.cpp)
  target_link_libraries(test_nmea_parser rtk_protocol)
  ament_add_gtest(test_rtk_sample_synchronizer test/test_rtk_sample_synchronizer.cpp)
  target_link_libraries(test_rtk_sample_synchronizer rtk_protocol)
  ament_add_gtest(test_rtk_validity test/test_rtk_validity.cpp)
  target_link_libraries(test_rtk_validity rtk_protocol)
  ament_add_gtest(test_vehicle_center_transform test/test_vehicle_center_transform.cpp)
  target_link_libraries(test_vehicle_center_transform rtk_protocol)
  ament_add_gtest(test_ntrip_protocol test/test_ntrip_protocol.cpp)
  target_link_libraries(test_ntrip_protocol rtk_protocol)
  ament_add_gtest(test_rtcm3_frame_buffer test/test_rtcm3_frame_buffer.cpp)
  target_link_libraries(test_rtcm3_frame_buffer rtk_protocol)
endif()

ament_export_include_directories(include)
ament_export_libraries(rtk_protocol rtk_transport)
ament_package()
