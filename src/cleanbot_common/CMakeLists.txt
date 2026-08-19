# 文件作用：构建cleanbot_common公共头文件库和其单元测试。
cmake_minimum_required(VERSION 3.8)
project(cleanbot_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Windows 编译器必须按 UTF-8 读取中文注释，避免代码页误判破坏后续语法。
if(MSVC)
  add_compile_options(/utf-8)
endif()

# ament_cmake和rclcpp提供ROS2构建规则及QoS类型依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 安装公共头文件，供其他模块通过cleanbot_common命名空间复用。
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  # 测试C++标准、发布者代际跟踪和公共接口契约。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cpp_standard test/test_cpp_standard.cpp)
  ament_target_dependencies(test_cpp_standard rclcpp)

  ament_add_gtest(
    test_publisher_epoch_tracker
    test/test_publisher_epoch_tracker.cpp
  )
  target_include_directories(test_publisher_epoch_tracker PRIVATE include)
endif()

ament_package()
