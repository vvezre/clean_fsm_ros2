# 诊断专用启动文件：只启动诊断节点和其配置来源。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 只启动统一诊断节点，适合无图形界面的树莓派。
    config_file = os.path.join(
        get_package_share_directory("cleanbot_bringup"),
        "config",
        "cleanbot_tools.yaml",
    )
    return LaunchDescription([
        # 该节点只订阅状态并发布 /diagnostics，不会发送车辆控制命令。
        Node(
            package="cleanbot_diagnostics",
            executable="cleanbot_diagnostics_node",
            name="cleanbot_diagnostics_node",
            output="screen",
            parameters=[config_file],
        ),
    ])
