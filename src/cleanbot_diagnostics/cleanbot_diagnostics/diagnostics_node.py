# 统一诊断节点：汇总各ROS2子系统状态并发布诊断级别。

from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cleanbot_interfaces.msg import (
    CommandExecutionStatus,
    ConfigStatus,
    HardwareStatus,
    MaintenanceState,
    RtkFix,
    TrackingStatus,
    VehicleState,
)

from .rules import freshness_level, status_level


def _qos(depth=5, transient_local=False):
    # 创建诊断订阅使用的可靠、有限深度QoS配置。
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else DurabilityPolicy.VOLATILE
        ),
    )


@dataclass
class Observation:
    # 保存一个被诊断主题的最新值、接收时间和健康摘要。
    message: object = None
    received_sec: float = -1.0


class DiagnosticsNode(Node):
    # 订阅系统状态，按规则计算并发布DiagnosticArray。
    # 汇总各业务节点的只读状态，并统一发布标准 /diagnostics。

    def __init__(self):
        # 创建订阅、诊断更新器和各状态源的观察缓存。
        super().__init__("cleanbot_diagnostics_node")
        self.declare_parameter("diagnostics.publish_period_sec", 1.0)
        self.declare_parameter("diagnostics.config_timeout_sec", 5.0)
        self.declare_parameter("diagnostics.hardware_timeout_sec", 2.0)
        self.declare_parameter("diagnostics.rtk_timeout_sec", 2.0)
        self.declare_parameter("diagnostics.tracking_timeout_sec", 1.0)
        self.declare_parameter("diagnostics.mission_timeout_sec", 2.0)
        self.declare_parameter("diagnostics.maintenance_timeout_sec", 2.0)
        self._observations = {
            name: Observation()
            for name in (
                "config",
                "hardware",
                "command",
                "rtk",
                "tracking",
                "mission",
                "maintenance",
            )
        }
        self._timeouts = {
            "config": float(
                self.get_parameter("diagnostics.config_timeout_sec").value
            ),
            "hardware": float(
                self.get_parameter("diagnostics.hardware_timeout_sec").value
            ),
            "rtk": float(
                self.get_parameter("diagnostics.rtk_timeout_sec").value
            ),
            "tracking": float(
                self.get_parameter("diagnostics.tracking_timeout_sec").value
            ),
            "mission": float(
                self.get_parameter("diagnostics.mission_timeout_sec").value
            ),
            "maintenance": float(
                self.get_parameter(
                    "diagnostics.maintenance_timeout_sec"
                ).value
            ),
        }
        publish_period = float(
            self.get_parameter("diagnostics.publish_period_sec").value
        )
        self._updater = Updater(self, publish_period)
        self._updater.setHardwareID("cleanbot")
        self._updater.add("config", self._diagnose_config)
        self._updater.add("hardware", self._diagnose_hardware)
        self._updater.add("rtk", self._diagnose_rtk)
        self._updater.add("tracking", self._diagnose_tracking)
        self._updater.add("mission", self._diagnose_mission)
        self._updater.add("maintenance", self._diagnose_maintenance)
        self.create_subscription(
            ConfigStatus,
            "/config/status",
            lambda message: self._observe("config", message),
            _qos(depth=1, transient_local=True),
        )
        self.create_subscription(
            HardwareStatus,
            "/hardware/status",
            lambda message: self._observe("hardware", message),
            _qos(),
        )
        self.create_subscription(
            CommandExecutionStatus,
            "/hardware/command_status",
            lambda message: self._observe("command", message),
            _qos(depth=20),
        )
        self.create_subscription(
            RtkFix,
            "/rtk/fix",
            lambda message: self._observe("rtk", message),
            _qos(),
        )
        self.create_subscription(
            TrackingStatus,
            "/tracking/status",
            lambda message: self._observe("tracking", message),
            _qos(),
        )
        self.create_subscription(
            VehicleState,
            "/vehicle/state",
            lambda message: self._observe("mission", message),
            _qos(depth=1, transient_local=True),
        )
        self.create_subscription(
            MaintenanceState,
            "/system/maintenance_state",
            lambda message: self._observe("maintenance", message),
            _qos(depth=1, transient_local=True),
        )

    def _now(self):
        # 返回当前ROS时钟的秒数，供新鲜度计算使用。
        return self.get_clock().now().nanoseconds / 1e9

    def _observe(self, name, message):
        # 记录指定主题的最新消息及其接收时间。
        # 保存最新消息及本节点接收时刻，用于判断发布链路是否中断。
        observation = self._observations[name]
        observation.message = message
        observation.received_sec = self._now()

    def _age_and_level(self, name):
        # 计算观察项年龄并映射为诊断级别。
        observation = self._observations[name]
        if observation.message is None:
            return None, DiagnosticStatus.ERROR
        age = max(0.0, self._now() - observation.received_sec)
        return age, freshness_level(age, self._timeouts[name])

    @staticmethod
    def _summary(stat, level, message):
        # 把规则结果写入DiagnosticStatusWrapper并返回摘要文本。
        levels = {
            "OK": DiagnosticStatus.OK,
            "WARN": DiagnosticStatus.WARN,
            "ERROR": DiagnosticStatus.ERROR,
        }
        stat.summary(levels[level], message)

    def _diagnose_config(self, stat: DiagnosticStatusWrapper):
        # 诊断配置中心连接状态和配置快照新鲜度。
        # 报告配置中心就绪状态；配置消息是持久快照，不按心跳超时。
        age, freshness = self._age_and_level("config")
        message = self._observations["config"].message
        if message is None:
            stat.summary(DiagnosticStatus.ERROR, "config status not received")
            return stat
        level, summary = status_level(
            message.config_ready,
            "OK",
            message.error_message or message.state or "config ready",
        )
        self._summary(stat, level, summary)
        stat.add("age_sec", f"{age:.3f}")
        stat.add("revision", str(message.revision))
        stat.add("error_code", message.error_code)
        return stat

    def _diagnose_hardware(self, stat: DiagnosticStatusWrapper):
        # 诊断下位机串口连接、故障和硬件状态新鲜度。
        # 报告串口、下位机故障、电池以及最近一次命令执行结果。
        age, freshness = self._age_and_level("hardware")
        message = self._observations["hardware"].message
        if message is None:
            stat.summary(DiagnosticStatus.ERROR, "hardware status not received")
            return stat
        ok = message.connected and not message.fault
        level, summary = status_level(
            ok,
            freshness,
            message.fault or "hardware ready",
            stale_is_error=True,
        )
        self._summary(stat, level, summary)
        stat.add("age_sec", f"{age:.3f}")
        stat.add("connected", str(message.connected))
        stat.add("battery_percent", f"{message.battery_percent:.1f}")
        stat.add("fault", message.fault)
        command = self._observations["command"]
        if command.message is None:
            stat.add("command_status", "not received")
        else:
            command_age = max(0.0, self._now() - command.received_sec)
            stat.add("command_age_sec", f"{command_age:.3f}")
            stat.add("command_id", str(command.message.command_id))
            stat.add("command_state", str(command.message.state))
            stat.add("command_detail", command.message.detail)
        return stat

    def _diagnose_rtk(self, stat: DiagnosticStatusWrapper):
        # 诊断RTK定位质量、固定解和数据接收年龄。
        # 报告RTK串口、坐标、固定解、航向及NTRIP状态。
        age, freshness = self._age_and_level("rtk")
        message = self._observations["rtk"].message
        if message is None:
            stat.summary(DiagnosticStatus.ERROR, "RTK status not received")
            return stat
        ok = message.serial_connected and message.coordinate_valid
        level, summary = status_level(
            ok,
            freshness,
            "RTK ready" if ok else "RTK coordinate or serial invalid",
            stale_is_error=True,
        )
        self._summary(stat, level, summary)
        stat.add("age_sec", f"{age:.3f}")
        stat.add("fixed_valid", str(message.fixed_valid))
        stat.add("fix_quality", str(message.fix_quality))
        stat.add("heading_valid", str(message.heading_valid))
        stat.add("ntrip_connected", str(message.ntrip_connected))
        return stat

    def _diagnose_tracking(self, stat: DiagnosticStatusWrapper):
        # 诊断视觉/跟踪控制状态及其数据新鲜度。
        # 报告纠偏是否阻塞、目标距离和错误码。
        age, freshness = self._age_and_level("tracking")
        message = self._observations["tracking"].message
        if message is None:
            stat.summary(DiagnosticStatus.WARN, "tracking status not received")
            return stat
        ok = not message.blocked
        level, summary = status_level(
            ok,
            freshness,
            message.message or message.code or "tracking ready",
        )
        self._summary(stat, level, summary)
        stat.add("age_sec", f"{age:.3f}")
        stat.add("active", str(message.active))
        stat.add("distance_to_target_m", f"{message.distance_to_target_m:.3f}")
        stat.add("code", message.code)
        return stat

    def _diagnose_mission(self, stat: DiagnosticStatusWrapper):
        # 诊断任务执行状态和任务状态消息新鲜度。
        # 报告车辆控制状态、当前任务动作及任务段进度。
        age, freshness = self._age_and_level("mission")
        message = self._observations["mission"].message
        if message is None:
            stat.summary(DiagnosticStatus.WARN, "mission state not received")
            return stat
        ok = message.health_state not in ("fault", "error")
        level, summary = status_level(
            ok,
            freshness,
            message.message or message.health_state,
        )
        self._summary(stat, level, summary)
        stat.add("age_sec", f"{age:.3f}")
        stat.add("control_state", message.control_state)
        stat.add("current_action", message.current_action)
        stat.add("current_segment", str(message.current_segment))
        return stat

    def _diagnose_maintenance(self, stat: DiagnosticStatusWrapper):
        # 诊断维护门控状态、代际和维护状态消息新鲜度。
        # 报告维护门是否生效及车辆是否已达到安全就绪条件。
        age, freshness = self._age_and_level("maintenance")
        message = self._observations["maintenance"].message
        if message is None:
            stat.summary(DiagnosticStatus.WARN, "maintenance state not received")
            return stat
        ok = message.ready
        level, summary = status_level(
            ok,
            "OK",
            message.message or message.blocker_code or "maintenance ready",
        )
        self._summary(stat, level, summary)
        stat.add("age_sec", f"{age:.3f}")
        stat.add("gate_active", str(message.gate_active))
        stat.add("ready", str(message.ready))
        stat.add("blocker_code", message.blocker_code)
        return stat


def main(args=None):
    # 初始化ROS2、运行诊断节点并在退出时释放资源。
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
