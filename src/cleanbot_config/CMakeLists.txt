# 文件作用：配置管理节点的 CMake 构建、安装与测试声明。
cmake_minimum_required(VERSION 3.8)
project(cleanbot_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Windows 编译器按 UTF-8 解码源码，保证中文注释不会受系统代码页影响。
if(MSVC)
  add_compile_options(/utf-8)
endif()

find_package(ament_cmake REQUIRED)
find_package(cleanbot_common REQUIRED)
find_package(cleanbot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SQLite3 REQUIRED)

add_library(config_core
  src/config_registry.cpp
  src/sqlite_config_repository.cpp
)
target_include_directories(config_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(config_core SQLite::SQLite3)

add_library(config_client src/config_client.cpp)
target_include_directories(config_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(config_client
  cleanbot_common
  cleanbot_interfaces
  rclcpp
)

add_executable(config_manager_node src/config_manager_node.cpp)
target_link_libraries(config_manager_node config_core)
ament_target_dependencies(config_manager_node
  cleanbot_common
  cleanbot_interfaces
  rclcpp
)
target_link_libraries(config_manager_node SQLite::SQLite3)

install(TARGETS config_core config_client config_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(config_client)
ament_export_dependencies(cleanbot_common cleanbot_interfaces rclcpp)

ament_package()
