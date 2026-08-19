# 可视化节点：把车辆、RTK、轨迹和清扫模型转换为RViz标记与TF。

from collections import deque
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from cleanbot_interfaces.msg import (
    CleaningModel,
    RtkFix,
    TrackingStatus,
    TrackingTarget,
    VehicleState,
)

from .geo import heading_to_quaternion, lat_lon_to_local
from .visualization_rules import (
    Origin,
    is_rtk_fresh,
    model_group_geometry,
    segment_local_points,
    select_origin,
    vehicle_color,
)


def _qos(depth=10, transient_local=False):
    # 创建可视化订阅使用的QoS配置。
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else DurabilityPolicy.VOLATILE
        ),
    )


class VisualizationNode(Node):
    # 订阅车辆状态和模型数据并发布Marker、Path及车辆TF。
    # 把现有业务Topic转换为RViz/Foxglove可直接显示的只读数据。

    def __init__(self):
        # 创建可视化订阅、发布器、TF广播器和运行时缓存。
        super().__init__("cleanbot_visualization_node")
        self.declare_parameter("visualization.origin_lat", 0.0)
        self.declare_parameter("visualization.origin_lon", 0.0)
        self.declare_parameter("visualization.origin_valid", False)
        self.declare_parameter("visualization.rtk_timeout_sec", 2.0)
        self.declare_parameter("visualization.path_history_size", 500)
        self._configured_origin = Origin(
            float(self.get_parameter("visualization.origin_lat").value),
            float(self.get_parameter("visualization.origin_lon").value),
        )
        self._configured_origin_valid = bool(
            self.get_parameter("visualization.origin_valid").value
        )
        self._rtk_timeout_sec = float(
            self.get_parameter("visualization.rtk_timeout_sec").value
        )
        self._path = deque(
            maxlen=max(
                1,
                int(self.get_parameter("visualization.path_history_size").value),
            )
        )
        self._model_origin = None
        self._last_rtk = None
        self._last_rtk_received_sec = None
        self._last_vehicle_state = None
        self._last_tracking_status = None
        self._last_pose = None

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/cleanbot/visualization/vehicle_pose",
            _qos(),
        )
        self.vehicle_path_publisher = self.create_publisher(
            Path,
            "/cleanbot/visualization/vehicle_path",
            _qos(),
        )
        self.target_path_publisher = self.create_publisher(
            Path,
            "/cleanbot/visualization/target_path",
            _qos(),
        )
        self.model_marker_publisher = self.create_publisher(
            MarkerArray,
            "/cleanbot/visualization/cleaning_model",
            _qos(),
        )
        self.status_marker_publisher = self.create_publisher(
            MarkerArray,
            "/cleanbot/visualization/status_marker",
            _qos(),
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            RtkFix, "/rtk/fix", self._on_rtk, _qos(depth=5)
        )
        self.create_subscription(
            TrackingTarget,
            "/tracking/target",
            self._on_target,
            _qos(depth=1),
        )
        self.create_subscription(
            TrackingStatus,
            "/tracking/status",
            self._on_tracking_status,
            _qos(depth=5),
        )
        self.create_subscription(
            VehicleState,
            "/vehicle/state",
            self._on_vehicle_state,
            _qos(depth=1, transient_local=True),
        )
        self.create_subscription(
            CleaningModel,
            "/modeling/model",
            self._on_model,
            _qos(depth=1, transient_local=True),
        )
        # RTK 节点停止发布后不会再触发订阅回调，因此用定时器刷新超时颜色。
        status_period = min(1.0, max(0.1, self._rtk_timeout_sec * 0.5))
        self.create_timer(status_period, self._publish_status_marker)

    def _now_sec(self):
        # 返回当前ROS时钟的浮点秒数。
        return self.get_clock().now().nanoseconds / 1e9

    def _rtk_is_fresh(self):
        # 根据RTK接收时间判断定位数据是否仍然新鲜。
        if self._last_rtk is None or self._last_rtk_received_sec is None:
            return False
        receive_age_sec = max(
            0.0,
            self._now_sec() - self._last_rtk_received_sec,
        )
        return is_rtk_fresh(
            self._last_rtk.gga_age_sec,
            receive_age_sec,
            self._rtk_timeout_sec,
        )

    def _origin(self):
        # 根据模型和参数选择当前地图坐标原点。
        if self._model_origin is not None:
            return self._model_origin
        if self._configured_origin_valid:
            return self._configured_origin
        return None

    def _on_model(self, message):
        # 接收清扫模型并刷新模型标记缓存。
        # 更新显示坐标原点，并发布清扫模型点、区域和连接线。
        next_origin = select_origin(
            message.origin_lat,
            message.origin_lon,
            message.origin_valid,
            self._configured_origin.lat,
            self._configured_origin.lon,
            self._configured_origin_valid,
        )
        if next_origin != self._model_origin:
            self._path.clear()
            self._last_pose = None
        self._model_origin = next_origin
        self._publish_model(message)

    def _on_rtk(self, message):
        # 接收RTK定位消息并更新车辆位置与朝向缓存。
        # 接收有效RTK后发布车辆位姿、轨迹及map到base_link的TF。
        self._last_rtk = message
        self._last_rtk_received_sec = self._now_sec()
        origin = self._origin()
        fresh = self._rtk_is_fresh()
        if (
            origin is None
            or not message.coordinate_valid
            or not math.isfinite(message.lat)
            or not math.isfinite(message.lon)
            or not fresh
        ):
            self._publish_status_marker()
            return
        try:
            east, north = lat_lon_to_local(
                origin.lat, origin.lon, message.lat, message.lon
            )
        except ValueError:
            self._publish_status_marker()
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = east
        pose.pose.position.y = north
        pose.pose.position.z = 0.0
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = heading_to_quaternion(message.heading_deg)
        self.pose_publisher.publish(pose)
        self._last_pose = pose
        self._path.append(pose)

        path = Path()
        path.header = pose.header
        path.poses = list(self._path)
        self.vehicle_path_publisher.publish(path)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = east
        transform.transform.translation.y = north
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        self._publish_status_marker(pose)

    def _on_target(self, message):
        # 接收当前跟踪目标并触发目标线段可视化。
        # 把纠偏节点使用的当前目标线转换为RViz Path。
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        points = segment_local_points(message.segment, self._origin())
        if message.active and points is not None:
            for x, y in points:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.target_path_publisher.publish(path)

    def _on_tracking_status(self, message):
        # 接收跟踪状态并更新车辆颜色、文字和调试信息。
        self._last_tracking_status = message
        self._publish_status_marker()

    def _on_vehicle_state(self, message):
        # 接收车辆状态并发布车辆位姿、路径和状态标记。
        self._last_vehicle_state = message
        self._publish_status_marker()

    def _publish_model(self, message):
        # 将模型分组转换为Polygon或LineStrip标记并发布。
        # 把模型局部厘米坐标转换为米制MarkerArray。
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        origin = self._origin()
        if origin is not None:
            marker_id = 0
            for group in message.groups:
                geometry = model_group_geometry(group)
                point_marker = Marker()
                point_marker.header.frame_id = "map"
                point_marker.header.stamp = self.get_clock().now().to_msg()
                point_marker.ns = "model_points"
                point_marker.id = marker_id
                marker_id += 1
                point_marker.type = Marker.SPHERE_LIST
                point_marker.action = Marker.ADD
                point_marker.pose.orientation.w = 1.0
                point_marker.scale.x = 0.18
                point_marker.scale.y = 0.18
                point_marker.scale.z = 0.08
                point_marker.color.r = 0.2
                point_marker.color.g = 0.65
                point_marker.color.b = 1.0
                point_marker.color.a = 1.0
                point_marker.points = [
                    Point(x=x, y=y, z=0.0)
                    for x, y in geometry["points"]
                ]
                if point_marker.points:
                    markers.markers.append(point_marker)

                for area_line in geometry["areas"]:
                    area_marker = Marker()
                    area_marker.header = point_marker.header
                    area_marker.ns = "model_sub_areas"
                    area_marker.id = marker_id
                    marker_id += 1
                    area_marker.type = Marker.LINE_STRIP
                    area_marker.action = Marker.ADD
                    area_marker.pose.orientation.w = 1.0
                    area_marker.scale.x = 0.08
                    area_marker.color.r = 0.1
                    area_marker.color.g = 0.9
                    area_marker.color.b = 0.3
                    area_marker.color.a = 1.0
                    area_marker.points = [
                        Point(x=x, y=y, z=0.0)
                        for x, y in area_line
                    ]
                    markers.markers.append(area_marker)

                if geometry["connectors"]:
                    connector_marker = Marker()
                    connector_marker.header = point_marker.header
                    connector_marker.ns = "model_connectors"
                    connector_marker.id = marker_id
                    marker_id += 1
                    connector_marker.type = Marker.LINE_LIST
                    connector_marker.action = Marker.ADD
                    connector_marker.pose.orientation.w = 1.0
                    connector_marker.scale.x = 0.12
                    connector_marker.color.r = 1.0
                    connector_marker.color.g = 0.55
                    connector_marker.color.b = 0.1
                    connector_marker.color.a = 1.0
                    for start, end in geometry["connectors"]:
                        connector_marker.points.append(
                            Point(x=start[0], y=start[1], z=0.0)
                        )
                        connector_marker.points.append(
                            Point(x=end[0], y=end[1], z=0.0)
                        )
                    markers.markers.append(connector_marker)
        self.model_marker_publisher.publish(markers)

    def _publish_status_marker(self, pose=None):
        # 发布车辆状态标记及其文本说明。
        # 在最近有效位置发布车辆健康颜色和文本；超时后自动变红。
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vehicle_status"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.2
        if self._last_rtk is None:
            color = (1.0, 0.15, 0.1, 1.0)
        else:
            color = vehicle_color(
                self._last_rtk.fixed_valid,
                self._last_rtk.coordinate_valid,
                self._rtk_is_fresh(),
            )
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        display_pose = pose if pose is not None else self._last_pose
        if display_pose is not None:
            marker.pose = display_pose.pose
        marker_array = MarkerArray()
        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = "vehicle_status_text"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.orientation.w = 1.0
        if display_pose is not None:
            text_marker.pose.position.x = display_pose.pose.position.x
            text_marker.pose.position.y = display_pose.pose.position.y
        text_marker.pose.position.z = 0.8
        text_marker.scale.z = 0.22
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = self._status_text()
        marker_array.markers.append(text_marker)
        self.status_marker_publisher.publish(marker_array)

    def _status_text(self):
        # 根据最新车辆、RTK和跟踪缓存生成状态文本。
        lines = []
        if self._last_vehicle_state is not None:
            state = self._last_vehicle_state
            lines.append(
                f"vehicle: {state.control_state}/{state.current_action}"
            )
            lines.append(f"health: {state.health_state}")
        if self._last_tracking_status is not None:
            tracking = self._last_tracking_status
            lines.append(
                f"tracking: {tracking.code or 'OK'} "
                f"distance={tracking.distance_to_target_m:.2f}m"
            )
        if self._last_rtk is None:
            lines.append("RTK: no data")
        else:
            rtk = self._last_rtk
            lines.append(
                f"RTK: quality={rtk.fix_quality} fixed={rtk.fixed_valid}"
            )
        return "\n".join(lines)


def main(args=None):
    # 初始化ROS2、运行可视化节点并在退出时清理上下文。
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
