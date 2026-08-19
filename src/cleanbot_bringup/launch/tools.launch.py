# 工具启动文件：启动Foxglove桥接、诊断和可视化辅助节点。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


FOXGLOVE_TOPIC_WHITELIST = [
    "^/rtk/fix$",
    "^/tracking/(debug|status|target)$",
    "^/vehicle/state$",
    "^/hardware/(status|command_status)$",
    "^/config/status$",
    "^/system/maintenance_state$",
    "^/modeling/model$",
    "^/cleanbot/visualization/.*$",
    "^/diagnostics$",
    "^/rosout$",
    "^/tf(_static)?$",
]


def generate_launch_description():
    # 按开关启动只读适配、诊断、远程查看和本机 GUI 工具。
    share_directory = get_package_share_directory("cleanbot_bringup")
    config_file = os.path.join(
        share_directory,
        "config",
        "cleanbot_tools.yaml",
    )
    rviz_config = os.path.join(
        share_directory,
        "config",
        "cleanbot.rviz",
    )

    enable_visualization = LaunchConfiguration("enable_visualization")
    enable_diagnostics = LaunchConfiguration("enable_diagnostics")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_rqt_graph = LaunchConfiguration("enable_rqt_graph")
    enable_plotjuggler = LaunchConfiguration("enable_plotjuggler")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_visualization",
            default_value="true",
            description="Start the read-only visualization adapter.",
        ),
        DeclareLaunchArgument(
            "enable_diagnostics",
            default_value="true",
            description="Start the read-only diagnostics node.",
        ),
        DeclareLaunchArgument(
            "enable_foxglove",
            default_value="false",
            description="Expose ROS2 data through Foxglove Bridge on the LAN.",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Read-only Foxglove WebSocket port.",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
            description="Start RViz on a machine with a graphical display.",
        ),
        DeclareLaunchArgument(
            "enable_rqt_graph",
            default_value="false",
            description="Start rqt_graph on a development machine.",
        ),
        DeclareLaunchArgument(
            "enable_plotjuggler",
            default_value="false",
            description="Start PlotJuggler on a development machine.",
        ),
        # 适配现有业务消息，发布 RViz/Foxglove 使用的位姿、路径和 Marker。
        Node(
            package="cleanbot_visualization",
            executable="cleanbot_visualization_node",
            name="cleanbot_visualization_node",
            output="screen",
            parameters=[config_file],
            condition=IfCondition(enable_visualization),
        ),
        # 汇总配置、硬件、RTK、纠偏、任务和维护状态到 /diagnostics。
        Node(
            package="cleanbot_diagnostics",
            executable="cleanbot_diagnostics_node",
            name="cleanbot_diagnostics_node",
            output="screen",
            parameters=[config_file],
            condition=IfCondition(enable_diagnostics),
        ),
        # 默认关闭；启用后仅向局域网客户端暴露白名单 Topic 和节点图。
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            condition=IfCondition(enable_foxglove),
            parameters=[{
                "port": ParameterValue(foxglove_port, value_type=int),
                "topic_whitelist": FOXGLOVE_TOPIC_WHITELIST,
                "client_topic_whitelist": ["(?!)"],
                "service_whitelist": ["(?!)"],
                "param_whitelist": ["(?!)"],
                "capabilities": ["connectionGraph"],
                "remote_access": False,
            }],
        ),
        # 以下三个 GUI 进程只适合带桌面的开发机，树莓派默认不启动。
        Node(
            package="rviz2",
            executable="rviz2",
            name="cleanbot_rviz",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(enable_rviz),
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
            output="screen",
            condition=IfCondition(enable_rqt_graph),
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "plotjuggler", "plotjuggler"],
            output="screen",
            condition=IfCondition(enable_plotjuggler),
        ),
    ])
