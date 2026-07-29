/*
 * 文件作用：车辆控制命令仲裁节点，是运动命令进入下位机前的唯一决策出口。
 *
 * 输入：订阅急停、安全、手动、任务、视觉五类车辆命令，以及独立的滚刷命令。
 * 输出：只向 /control/final_cmd 发布当前生效的唯一车辆控制命令。
 *
 * 主流程：接收多来源命令 -> 固定来源优先级 -> 更新命令租约 -> 处理急停和操作意图
 *          -> 选择当前有效命令 -> 合并滚刷状态 -> 发布最终命令。
 * 安全边界：发布者不能自行提高优先级；命令租约超时后自动输出刹车；软件急停后新的用户操作
 *          可以恢复车辆移动，但滚刷保持关闭，必须由用户重新打开。
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_client.hpp"
#include "cleanbot_control/command_arbiter_core.hpp"
#include "cleanbot_interfaces/msg/brush_command.hpp"
#include "cleanbot_interfaces/msg/vehicle_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cleanbot {
namespace control {

class CommandArbiterNode : public rclcpp::Node {
 public:
  CommandArbiterNode() : Node("command_arbiter_node") {
    // 1. /control/final_cmd是唯一最终出口，下位机节点只订阅该主题。
    final_publisher_ = create_publisher<cleanbot_interfaces::msg::VehicleCommand>(
        "/control/final_cmd", common::latest_command_qos());

    // 2. 分别订阅五类车辆命令。来源类型由订阅主题确定，不相信消息自带的priority。
    emergency_subscription_ = subscribe(
        "/control/emergency_cmd", CommandSource::kEmergency);
    safety_subscription_ = subscribe(
        "/control/safety_cmd", CommandSource::kSafety);
    manual_subscription_ = subscribe(
        "/control/manual_cmd", CommandSource::kManual);
    mission_subscription_ = subscribe(
        "/control/mission_cmd", CommandSource::kMission);
    vision_subscription_ = subscribe(
        "/control/vision_cmd", CommandSource::kVision);

    // 3. 滚刷使用独立控制主题，摇杆运动不会隐式打开滚刷。
    brush_subscription_ = create_subscription<cleanbot_interfaces::msg::BrushCommand>(
        "/control/brush_cmd",
        common::latest_command_qos(),
        std::bind(&CommandArbiterNode::onBrush, this, std::placeholders::_1));

    // 4. 50毫秒周期检查命令租约，即使没有新消息，也能在来源超时后及时发布刹车。
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CommandArbiterNode::publishIfChanged, this));

    // 5. 租约参数只从配置中心读取；配置未就绪期间节点持续输出安全刹车。
    config_client_ = std::make_unique<config::ConfigClient>(
        this,
        std::vector<std::string>{
            "control.manual_command_lease_ms",
            "control.mission_command_lease_ms",
            "control.vision_command_lease_ms",
        },
        false,
        [this](const config::ConfigSnapshot& snapshot, const bool initial) {
          configure(snapshot, initial);
        });
    publishIfChanged();
  }

 private:
  using VehicleCommand = cleanbot_interfaces::msg::VehicleCommand;

  // 创建指定来源的订阅器，并把所有车辆命令统一转交onCommand处理。
  rclcpp::Subscription<VehicleCommand>::SharedPtr subscribe(
      const std::string& topic, const CommandSource source) {
    return create_subscription<VehicleCommand>(
        topic,
        common::latest_command_qos(),
        [this, source](const VehicleCommand::SharedPtr message) {
          onCommand(source, *message);
        });
  }

  // 更新某一来源的候选命令；优先级由本节点按来源重新赋值，防止上游伪造高优先级。
  void onCommand(const CommandSource source, const VehicleCommand& message) {
    if (!configured_ || !arbiter_) {
      publishIfChanged();
      return;
    }
    auto command = toCore(message);
    command.priority = fixedPriority(source);
    if (arbiter_->update(source, command, monotonicMs())) {
      publishIfChanged();
    }
  }

  // 更新独立滚刷状态。急停发生后核心仲裁器会强制清零滚刷，需要用户重新发送开启命令。
  void onBrush(const cleanbot_interfaces::msg::BrushCommand::SharedPtr message) {
    if (!configured_ || !arbiter_) {
      publishIfChanged();
      return;
    }
    arbiter_->set_brush(message->enabled, message->speed, message->operator_intent);
    publishIfChanged();
  }

  // 计算当前仲裁结果；只有输出内容变化时才发布，避免无意义地重复占用下位机串口。
  void publishIfChanged() {
    ControlCommand output;
    if (!configured_ || !arbiter_) {
      output.source = "config_not_ready";
      output.priority = VehicleCommand::PRIORITY_SAFETY;
      output.active = true;
      output.brake = true;
      output.brush_speed = 0;
    } else {
      output = arbiter_->output(monotonicMs());
    }
    if (has_last_output_ && sameOutput(output, last_output_)) {
      return;
    }
    last_output_ = output;
    has_last_output_ = true;
    auto message = toMessage(output);
    message.stamp = now();
    message.command_id = next_output_command_id_++;
    final_publisher_->publish(message);
  }

  void configure(const config::ConfigSnapshot& snapshot, const bool initial) {
    if (!initial && arbiter_) {
      RCLCPP_WARN(
          get_logger(),
          "command lease configuration changed; restart required before it becomes active");
      return;
    }
    ArbiterParameters parameters;
    parameters.manual_lease_ms = static_cast<std::uint64_t>(
        snapshot.get_integer("control.manual_command_lease_ms"));
    parameters.mission_lease_ms = static_cast<std::uint64_t>(
        snapshot.get_integer("control.mission_command_lease_ms"));
    parameters.vision_lease_ms = static_cast<std::uint64_t>(
        snapshot.get_integer("control.vision_command_lease_ms"));
    arbiter_ = std::make_unique<CommandArbiterCore>(parameters);
    configured_ = true;
    has_last_output_ = false;
    publishIfChanged();
    RCLCPP_INFO(get_logger(), "command arbiter configuration ready");
  }

  // ROS2消息转换为不依赖ROS2的核心结构，便于仲裁算法独立单元测试。
  static ControlCommand toCore(const VehicleCommand& message) {
    ControlCommand command;
    command.stamp_ms = stampMs(message.stamp);
    command.command_id = message.command_id;
    command.request_id = message.request_id == 0u
        ? message.command_id
        : message.request_id;
    command.source = message.source;
    command.priority = message.priority;
    command.active = message.active;
    command.operator_intent = message.operator_intent;
    command.status = message.status;
    command.x_speed = message.x_speed;
    command.z_speed = message.z_speed;
    command.steering_offset = message.steering_offset;
    command.brush_speed = message.brush_speed;
    command.target_distance = message.target_distance;
    command.target_rotation = message.target_rotation;
    command.heading_deg = message.heading_deg;
    command.brake = message.brake;
    command.charge = message.charge;
    return command;
  }

  // 核心仲裁结果转换回ROS2消息，由本节点补充输出时间戳和新的最终command_id。
  static VehicleCommand toMessage(const ControlCommand& command) {
    VehicleCommand message;
    message.request_id = command.request_id;
    message.source = command.source;
    message.priority = command.priority;
    message.active = command.active;
    message.operator_intent = command.operator_intent;
    message.status = command.status;
    message.x_speed = command.x_speed;
    message.z_speed = command.z_speed;
    message.steering_offset = command.steering_offset;
    message.brush_speed = command.brush_speed;
    message.target_distance = command.target_distance;
    message.target_rotation = command.target_rotation;
    message.heading_deg = command.heading_deg;
    message.brake = command.brake;
    message.charge = command.charge;
    return message;
  }

  // 固定优先级顺序：急停 > 安全 > 手动 > 任务 > 视觉。
  static std::uint8_t fixedPriority(const CommandSource source) {
    if (source == CommandSource::kEmergency) {
      return VehicleCommand::PRIORITY_EMERGENCY;
    }
    if (source == CommandSource::kSafety) {
      return VehicleCommand::PRIORITY_SAFETY;
    }
    if (source == CommandSource::kManual) {
      return VehicleCommand::PRIORITY_MANUAL;
    }
    if (source == CommandSource::kMission) {
      return VehicleCommand::PRIORITY_MISSION;
    }
    return VehicleCommand::PRIORITY_VISION;
  }

  static std::uint64_t stampMs(const builtin_interfaces::msg::Time& stamp) {
    if (stamp.sec < 0) {
      return 0u;
    }
    return static_cast<std::uint64_t>(stamp.sec) * 1000u +
        static_cast<std::uint64_t>(stamp.nanosec) / 1000000u;
  }

  // 比较实际控制字段，保证定时检查只在仲裁输出发生变化时触发发布。
  static bool sameOutput(const ControlCommand& left, const ControlCommand& right) {
    return left.request_id == right.request_id &&
        left.source == right.source && left.priority == right.priority &&
        left.active == right.active && left.operator_intent == right.operator_intent &&
        left.status == right.status && left.x_speed == right.x_speed &&
        left.z_speed == right.z_speed &&
        left.steering_offset == right.steering_offset &&
        left.brush_speed == right.brush_speed &&
        left.target_distance == right.target_distance &&
        left.target_rotation == right.target_rotation &&
        left.heading_deg == right.heading_deg && left.brake == right.brake &&
        left.charge == right.charge;
  }

  static std::uint64_t monotonicMs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  std::unique_ptr<CommandArbiterCore> arbiter_;
  std::unique_ptr<config::ConfigClient> config_client_;
  bool configured_{false};
  ControlCommand last_output_;
  bool has_last_output_{false};
  std::uint64_t next_output_command_id_{1u};
  rclcpp::Publisher<VehicleCommand>::SharedPtr final_publisher_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr emergency_subscription_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr safety_subscription_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr manual_subscription_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr mission_subscription_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr vision_subscription_;
  rclcpp::Subscription<cleanbot_interfaces::msg::BrushCommand>::SharedPtr brush_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace control
}  // namespace cleanbot

// 程序入口：初始化ROS2并运行仲裁节点，订阅回调和租约定时器共同维护最终控制输出。
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanbot::control::CommandArbiterNode>());
  rclcpp::shutdown();
  return 0;
}
