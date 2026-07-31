import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("cleanbot_bringup"),
        "config",
        "system.yaml",
    )
    # 配置中心节点：先把系统参数装载起来，给后续所有节点提供统一配置源。
    config_manager = Node(
        package="cleanbot_config",
        executable="config_manager_node",
        name="config_manager_node",
        output="screen",
        parameters=[config_file],
    )
    # 下位机节点：独占串口，负责协议编解码、ACK 和硬件状态发布。
    lower_machine = Node(
        package="cleanbot_hardware",
        executable="lower_machine_node",
        name="lower_machine_node",
        output="screen",
    )
    # RTK 节点：处理串口定位数据和 NTRIP 差分数据，输出 /rtk/fix。
    rtk = Node(
        package="cleanbot_rtk",
        executable="rtk_node",
        name="rtk_node",
        output="screen",
    )
    # 跟踪节点：把目标线段和 RTK 实时位置转成任务控制量。
    tracking = Node(
        package="cleanbot_control",
        executable="tracking_node",
        name="tracking_node",
        output="screen",
    )
    # 命令仲裁节点：在手动、任务、安全和急停之间选出唯一最终控制命令。
    command_arbiter = Node(
        package="cleanbot_control",
        executable="command_arbiter_node",
        name="command_arbiter_node",
        output="screen",
    )
    # 任务节点：负责清扫、多路点、返航、暂停、恢复和检查点管理。
    mission = Node(
        package="cleanbot_mission",
        executable="mission_manager_node",
        name="mission_manager_node",
        output="screen",
    )
    # 建模节点：负责采样、建模、识别、路径规划和计划执行。
    modeling = Node(
        package="cleanbot_modeling",
        executable="modeling_manager_node",
        name="modeling_manager_node",
        output="screen",
    )
    # HTTP 网关节点：把网页和接口请求翻译成 ROS2 Service / Topic 调用。
    http_gateway = Node(
        package="cleanbot_http",
        executable="http_gateway_node",
        name="http_gateway_node",
        output="screen",
    )
    return LaunchDescription([
        config_manager,
        lower_machine,
        rtk,
        tracking,
        command_arbiter,
        mission,
        modeling,
        http_gateway,
    ])
