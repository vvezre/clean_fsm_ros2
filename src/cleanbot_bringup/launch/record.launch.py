# 录包启动文件：声明需要记录的核心控制、定位和硬件话题。

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


DEFAULT_TOPICS = " ".join([
    "/rtk/fix",
    "/tracking/debug",
    "/tracking/status",
    "/vehicle/state",
    "/hardware/status",
    "/hardware/command_status",
    "/config/status",
    "/system/maintenance_state",
])


def _record_action(context):
    # 读取录包参数并创建对应的rosbag录制进程动作。
    # 生成 rosbag2 命令，并在执行前剔除所有 /control/ Topic。
    requested = LaunchConfiguration("topics").perform(context).split()
    topics = [
        topic
        for topic in requested
        if topic.startswith("/") and not topic.startswith("/control/")
    ]
    if not topics:
        raise RuntimeError("No safe rosbag topics were selected.")

    command = [
        "ros2",
        "bag",
        "record",
        "-s",
        LaunchConfiguration("storage_id").perform(context),
    ]
    output = LaunchConfiguration("output").perform(context).strip()
    if output:
        command.extend(["-o", output])
    command.extend(topics)
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    # 构造rosbag录制进程及其启动参数。
    # 启动可配置的状态/调试数据录包，不录制控制指令。
    return LaunchDescription([
        DeclareLaunchArgument(
            "storage_id",
            default_value="sqlite3",
            description="rosbag2 storage plugin, for example sqlite3 or mcap.",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="cleanbot_bag",
            description="Output bag directory.",
        ),
        DeclareLaunchArgument(
            "topics",
            default_value=DEFAULT_TOPICS,
            description="Space-separated status/debug topics; control topics are filtered.",
        ),
        OpaqueFunction(function=_record_action),
    ])
