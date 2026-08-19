# 文件作用：清扫建模节点的 CMake 构建、安装与测试声明。
cmake_minimum_required(VERSION 3.8)
project(cleanbot_modeling)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Windows 编译器按 UTF-8 解码源码，保证中文注释不会受系统代码页影响。
if(MSVC)
  add_compile_options(/utf-8)
endif()

find_package(ament_cmake REQUIRED)
find_package(cleanbot_common REQUIRED)
find_package(cleanbot_config REQUIRED)
find_package(cleanbot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(SQLite3 REQUIRED)

add_library(modeling_core
  src/point_sampler.cpp
  src/geometry.cpp
  src/region_recognizer.cpp
  src/coverage_planner.cpp
  src/task_plan_builder.cpp
  src/goal_submission.cpp
)
target_include_directories(modeling_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(modeling_repository
  src/sqlite_model_repository.cpp
)
target_include_directories(modeling_repository
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(modeling_repository modeling_core SQLite::SQLite3)

add_executable(modeling_manager_node src/modeling_manager_node.cpp)
target_link_libraries(
  modeling_manager_node
  modeling_core
  modeling_repository
  SQLite::SQLite3
)
ament_target_dependencies(
  modeling_manager_node
  cleanbot_common
  cleanbot_config
  cleanbot_interfaces
  rclcpp
  rclcpp_action
)

install(TARGETS modeling_core modeling_repository modeling_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_point_sampler test/test_point_sampler.cpp)
  target_link_libraries(test_point_sampler modeling_core)
  ament_add_gtest(test_region_recognizer test/test_region_recognizer.cpp)
  target_link_libraries(test_region_recognizer modeling_core)
  ament_add_gtest(test_coverage_planner test/test_coverage_planner.cpp)
  target_link_libraries(test_coverage_planner modeling_core)
  ament_add_gtest(test_task_plan_builder test/test_task_plan_builder.cpp)
  target_link_libraries(test_task_plan_builder modeling_core)
  ament_add_gtest(
    test_sqlite_model_repository
    test/test_sqlite_model_repository.cpp
  )
  target_link_libraries(
    test_sqlite_model_repository
    modeling_repository
    SQLite::SQLite3
  )
  ament_add_gtest(
    test_goal_submission
    test/test_goal_submission.cpp
  )
  target_link_libraries(test_goal_submission modeling_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(modeling_core modeling_repository)
ament_package()
