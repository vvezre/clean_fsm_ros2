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
    config_manager = Node(
        package="cleanbot_config",
        executable="config_manager_node",
        name="config_manager_node",
        output="screen",
        parameters=[config_file],
    )
    lower_machine = Node(
        package="cleanbot_hardware",
        executable="lower_machine_node",
        name="lower_machine_node",
        output="screen",
    )
    rtk = Node(
        package="cleanbot_rtk",
        executable="rtk_node",
        name="rtk_node",
        output="screen",
    )
    tracking = Node(
        package="cleanbot_control",
        executable="tracking_node",
        name="tracking_node",
        output="screen",
    )
    command_arbiter = Node(
        package="cleanbot_control",
        executable="command_arbiter_node",
        name="command_arbiter_node",
        output="screen",
    )
    mission = Node(
        package="cleanbot_mission",
        executable="mission_manager_node",
        name="mission_manager_node",
        output="screen",
    )
    modeling = Node(
        package="cleanbot_modeling",
        executable="modeling_manager_node",
        name="modeling_manager_node",
        output="screen",
    )
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
