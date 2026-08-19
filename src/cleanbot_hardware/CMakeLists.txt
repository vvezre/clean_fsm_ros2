# 文件作用：构建旧下位机协议库、异步串口传输库和ROS2适配节点。
# lower_machine_protocol只处理字节协议和发送队列，不依赖ROS2；
# lower_machine_transport封装Boost.Asio串口；lower_machine_node负责ROS2接口和配置接入。
cmake_minimum_required(VERSION 3.8)
project(cleanbot_hardware)

# 整个工程目标为C++17；MSVC额外强制UTF-8，保证中文注释和日志可正确编译。
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

# 协议库可独立单元测试：负责19字节命令、23字节状态、帧重组和优先级队列。
add_library(lower_machine_protocol
  src/command_encoder.cpp
  src/frame_buffer.cpp
  src/status_frame_parser.cpp
  src/write_queue.cpp
)
target_include_directories(lower_machine_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 传输库独占物理串口，在专用I/O线程中完成异步读写和断线重连。
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

# ROS2节点把/control/final_cmd转换为旧串口协议，并发布硬件与命令状态。
add_executable(lower_machine_node src/lower_machine_node.cpp)
target_link_libraries(lower_machine_node lower_machine_protocol lower_machine_transport)
ament_target_dependencies(
  lower_machine_node
  rclcpp
  cleanbot_common
  cleanbot_config
  cleanbot_interfaces
)

# 安装节点、两层库和公共头文件，供bringup启动及其他包链接使用。
install(TARGETS lower_machine_protocol lower_machine_transport lower_machine_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  # 四组测试分别锁定拆包、状态解析、命令编码和发送队列行为。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_buffer test/test_frame_buffer.cpp)
  target_link_libraries(test_frame_buffer lower_machine_protocol)
  ament_add_gtest(test_status_frame_parser test/test_status_frame_parser.cpp)
  target_link_libraries(test_status_frame_parser lower_machine_protocol)
  ament_add_gtest(test_command_encoder test/test_command_encoder.cpp)
  target_link_libraries(test_command_encoder lower_machine_protocol)
  ament_add_gtest(test_write_queue test/test_write_queue.cpp)
  target_link_libraries(test_write_queue lower_machine_protocol)
endif()

ament_export_include_directories(include)
ament_export_libraries(lower_machine_protocol lower_machine_transport)
ament_package()
