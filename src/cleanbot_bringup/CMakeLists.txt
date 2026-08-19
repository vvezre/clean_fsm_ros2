# 文件作用：系统启动、配置和可视化资源安装规则。
cmake_minimum_required(VERSION 3.8)
project(cleanbot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
