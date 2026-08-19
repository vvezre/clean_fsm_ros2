/*
 * 文件作用：兼容旧Python串口协议的ROS2下位机适配节点。
 *
 * 输入：订阅 /control/final_cmd，接收命令仲裁节点输出的唯一车辆命令。
 * 输出：发布 /hardware/status 和 /hardware/command_status。
 *
 * 本节点独占下位机串口。普通命令编码为19字节并默认连续发送5次，只接收固定23字节
 * 状态帧。旧协议没有独立ACK，定距和转向完成由状态帧第12/13字节的0xBB判断。
 * CommandEncoder保留了旧初始化航向特殊布局，但当前VehicleCommand入口尚未选择该编码。
 */

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_client.hpp"
#include "cleanbot_hardware/command_encoder.hpp"
#include "cleanbot_hardware/lower_machine_serial.hpp"
#include "cleanbot_hardware/status_frame_parser.hpp"
#include "cleanbot_interfaces/msg/command_execution_status.hpp"
#include "cleanbot_interfaces/msg/hardware_status.hpp"
#include "cleanbot_interfaces/msg/vehicle_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cleanbot {
namespace hardware {

class LowerMachineNode : public rclcpp::Node {
 public:
  // 建立ROS2发布订阅并等待配置中心就绪；配置未就绪前不会打开串口。
  LowerMachineNode() : Node("lower_machine_node") {
    status_publisher_ = create_publisher<cleanbot_interfaces::msg::HardwareStatus>(
        "/hardware/status", common::hardware_status_qos());
    command_status_publisher_ =
        create_publisher<cleanbot_interfaces::msg::CommandExecutionStatus>(
            "/hardware/command_status", common::command_status_qos());
    command_subscription_ = create_subscription<cleanbot_interfaces::msg::VehicleCommand>(
        "/control/final_cmd", common::latest_command_qos(),
        std::bind(&LowerMachineNode::onCommand, this, std::placeholders::_1));

    config_client_ = std::make_unique<config::ConfigClient>(
        this,
        std::vector<std::string>{
            "hardware.lower_machine_port",
            "hardware.lower_machine_baudrate",
            "hardware.command_repeat_count",
        },
        false,
        // 配置回调作用：接收最新配置快照，并刷新本节点对应的运行参数。
        [this](const config::ConfigSnapshot& snapshot, const bool initial) {
          configure(snapshot, initial);
        });
  }

  // 节点退出前停止串口线程，避免异步回调访问已销毁的ROS2对象。
  ~LowerMachineNode() override {
    if (serial_) {
      serial_->stop();
    }
  }

 private:
  // 保存最近送入串口流程的ROS2业务标识。旧串口帧不携带这些字段，只能在节点内关联。
  struct ActiveCommand {
    std::uint64_t command_id{0u};
    std::uint64_t request_id{0u};
    std::string source;
    std::uint8_t command_type{0u};
    bool expects_completion{false};
  };

  // 识别旧Python中不使用普通XOR布局的模式控制命令。
  // 初始化航向使用另一种特殊布局，当前尚未从VehicleCommand入口触发。
  static bool isLegacyControl(const std::uint8_t status) {
    return status == 0xe0u || status == 0xeau || status == 0xfau || status == 0xfbu;
  }

  // 定距、转向、充电和模式控制属于有限动作，不能被后续连续速度命令替换。
  static bool isFiniteCommand(
      const cleanbot_interfaces::msg::VehicleCommand& command) {
    return command.charge || command.target_distance != 0 ||
        command.target_rotation != 0 || command.status == 2u ||
        command.status == 3u || command.status == 5u ||
        isLegacyControl(command.status);
  }

  // 只有定距和转向动作使用状态帧中的移动/旋转完成标志。
  static bool expectsCompletion(
      const cleanbot_interfaces::msg::VehicleCommand& command) {
    return command.target_distance != 0 || command.target_rotation != 0 ||
        command.status == 2u || command.status == 3u;
  }

  // 刹车优先级最高，有限动作其次，连续速度命令最低。
  static WritePriority writePriorityFor(
      const cleanbot_interfaces::msg::VehicleCommand& command) {
    if (command.brake) {
      return WritePriority::kSafety;
    }
    return isFiniteCommand(command) ? WritePriority::kFinite : WritePriority::kContinuous;
  }

  // 连续运动只保留尚未发送的最新值，避免纠偏高频输出堆积旧速度。
  static std::string coalescingKeyFor(
      const cleanbot_interfaces::msg::VehicleCommand& command) {
    if (command.brake) {
      return "brake";
    }
    return isFiniteCommand(command) ? "" : "motion";
  }

  // 最终命令入口：检查配置和串口、选择普通/特殊编码、生成重复批次并异步发送。
  void onCommand(const cleanbot_interfaces::msg::VehicleCommand::SharedPtr command) {
    if (!configured_ || !serial_) {
      publishCommandStatus(
          command->command_id, command->request_id, command->source, command->status,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_REJECTED,
          0u, 0u, "config_not_ready");
      return;
    }
    if (!serial_->connected()) {
      publishCommandStatus(
          command->command_id, command->request_id, command->source, command->status,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_REJECTED,
          1u, 0u, "serial_not_connected");
      return;
    }

    // ROS2消息字段先完整映射到协议中间模型，编码器再负责范围截断和字节序。
    CommandFields fields;
    fields.status = command->status;
    fields.power_on = 1u;
    fields.hardware_control = 0u;
    fields.x_speed = command->x_speed;
    fields.z_speed = command->z_speed;
    fields.steering_offset = command->steering_offset;
    fields.brush_speed = command->brush_speed;
    fields.target_distance = command->target_distance;
    fields.target_rotation = command->target_rotation;
    fields.heading_deg = command->heading_deg;
    fields.brake = command->brake;
    fields.charge = command->charge;

    // E0/EA/FA/FB严格使用旧模式布局；其余命令使用带XOR的普通19字节布局。
    const auto frame = isLegacyControl(command->status)
        ? command_encoder_.encodeLegacyControl(command->status, fields.power_on)
        : command_encoder_.encode(fields);
    const bool expects_completion = expectsCompletion(*command);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      // 旧协议没有sequence/ACK，只能记住最近命令。新的命令会替换本地关联记录，
      // 因而完成事件只能按“最近有限动作”近似关联，不能达到新版协议的精确匹配。
      active_command_ = ActiveCommand{
          command->command_id, command->request_id, command->source,
          frame[1], expects_completion};
      if (expects_completion) {
        // The old Python implementation reset these flags before finite moves
        // and rotations, so completion is detected on the next rising edge.
        last_move_finished_ = false;
        last_rotate_finished_ = false;
      }
    }

    publishCommandStatus(
        command->command_id, command->request_id, command->source, frame[1],
        cleanbot_interfaces::msg::CommandExecutionStatus::STATE_QUEUED,
        0u, 0u, "legacy_command_queued");

    // 旧Python通过连续重复同一帧提高下位机收到概率；重复批次不是ACK机制。
    const auto burst = command_encoder_.repeat(frame, repeat_count_);
    serial_->send(
        burst,
        writePriorityFor(*command),
        coalescingKeyFor(*command),
        [this, command, frame](const SendEvent event, const std::string& detail) {
          if (event == SendEvent::kWritten) {
            // SENT只代表Boost.Asio已把批次写入本机串口，不代表下位机已经解析或执行。
            publishCommandStatus(
                command->command_id, command->request_id, command->source, frame[1],
                cleanbot_interfaces::msg::CommandExecutionStatus::STATE_SENT,
                0u, 0u, detail);
          } else if (event != SendEvent::kQueued) {
            publishCommandStatus(
                command->command_id, command->request_id, command->source, frame[1],
                cleanbot_interfaces::msg::CommandExecutionStatus::STATE_REJECTED,
                2u, 0u, detail);
          }
        });
  }

  // 状态帧入口：只接受23字节旧协议帧，发布硬件状态并处理0xBB完成上升沿。
  void onFrame(const std::vector<std::uint8_t>& frame) {
    const auto result = status_parser_.parse(frame);
    if (!result.parsed) {
      RCLCPP_WARN(get_logger(), "legacy status frame rejected: %s", result.error.c_str());
      return;
    }

    const auto& parsed = result.status;
    // 把两个0xBB布尔位合并成ROS2完成事件，并用上升沿抑制连续状态帧重复上报。
    const auto completion_event = completionEvent(parsed.move_finished, parsed.rotate_finished);
    ActiveCommand active;
    bool completion_new = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      completion_new = completion_event != 0u &&
          ((parsed.move_finished && !last_move_finished_) ||
           (parsed.rotate_finished && !last_rotate_finished_));
      last_move_finished_ = parsed.move_finished;
      last_rotate_finished_ = parsed.rotate_finished;
      active = active_command_;
    }

    cleanbot_interfaces::msg::HardwareStatus status;
    status.stamp = now();
    status.frame_sequence = ++frame_sequence_;
    status.connected = true;
    status.status = parsed.status;
    status.power_on = parsed.power_on;
    status.hardware_state = parsed.hardware_state;
    status.x_speed_raw = parsed.x_speed_raw;
    status.z_speed_raw = parsed.z_speed_raw;
    status.x_speed = parsed.x_speed;
    status.z_speed = parsed.z_speed;
    status.brush_speed = parsed.brush_speed;
    status.edge_clear = parsed.edge_clear;
    status.battery_percent = parsed.battery_percent;
    status.pack_voltage = parsed.pack_voltage;
    status.air_state = parsed.air_state;
    status.move_finished = parsed.move_finished;
    status.rotate_finished = parsed.rotate_finished;
    status.angle = parsed.angle;
    status.odometer = parsed.odometer;
    status.command_sequence = 0u;
    status.completion_event = completion_event;
    status.completion_new = completion_new;
    status_publisher_->publish(status);

    // 只有最近命令确实需要完成反馈时才发布COMPLETED；连续速度和模式命令不消费0xBB。
    if (completion_new && active.expects_completion && active.command_id != 0u) {
      publishCommandStatus(
          active.command_id, active.request_id, active.source, active.command_type,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_COMPLETED,
          0u, completion_event, "legacy_status_completion");
    }
  }

  // 把移动和旋转完成位合并成ROS2完成事件：1移动、2旋转、3同时完成。
  static std::uint8_t completionEvent(
      const bool move_finished, const bool rotate_finished) {
    if (move_finished && rotate_finished) {
      return 0x03u;
    }
    if (move_finished) {
      return 0x01u;
    }
    return rotate_finished ? 0x02u : 0x00u;
  }

  // 发布ROS2侧命令生命周期；旧协议没有sequence和ACK，因此sequence固定为0。
  // QUEUED/SENT来自本机发送流程，COMPLETED才来自23字节状态帧中的0xBB上升沿。
  void publishCommandStatus(
      const std::uint64_t command_id,
      const std::uint64_t request_id,
      const std::string& source,
      const std::uint8_t command_type,
      const std::uint8_t state,
      const std::uint8_t result,
      const std::uint8_t completion_event,
      const std::string& detail) {
    cleanbot_interfaces::msg::CommandExecutionStatus status;
    status.stamp = now();
    status.command_id = command_id;
    status.request_id = request_id;
    status.source = source;
    status.sequence = 0u;
    status.command_type = command_type;
    status.state = state;
    status.result = result;
    status.completion_event = completion_event;
    status.detail = detail;
    command_status_publisher_->publish(status);
  }

  // 从配置中心读取串口和重复次数，只在首次配置时创建并启动串口传输层。
  // 运行中端口或波特率变化要求重启，避免热切换时出现两个串口所有者。
  void configure(const config::ConfigSnapshot& snapshot, const bool initial) {
    if (!initial && configured_) {
      RCLCPP_WARN(get_logger(), "lower-machine configuration changed; restart required");
      return;
    }
    const auto port = snapshot.get_string("hardware.lower_machine_port");
    const auto baudrate = snapshot.get_integer("hardware.lower_machine_baudrate");
    repeat_count_ = static_cast<std::size_t>(
        std::max<std::int64_t>(1, snapshot.get_integer("hardware.command_repeat_count")));
    serial_ = std::make_unique<LowerMachineSerial>(
        port, static_cast<unsigned int>(baudrate),
        std::bind(&LowerMachineNode::onFrame, this, std::placeholders::_1),
        std::bind(
            &LowerMachineNode::onConnectionChanged, this,
            std::placeholders::_1, std::placeholders::_2));
    configured_ = true;
    serial_->start();
    RCLCPP_INFO(
        get_logger(), "legacy lower-machine protocol ready port=%s baudrate=%lld repeats=%zu",
        port.c_str(), static_cast<long long>(baudrate), repeat_count_);
  }

  // 串口连接后先发送旧格式刹车批次；断线时上报当前命令中断和硬件离线。
  void onConnectionChanged(const bool connected, const std::string& detail) {
    if (connected) {
      RCLCPP_INFO(get_logger(), "%s", detail.c_str());
      CommandFields stop;
      stop.brake = true;
      const auto frame = command_encoder_.encode(stop);
      serial_->send(
          command_encoder_.repeat(frame, repeat_count_),
          WritePriority::kSafety,
          "reconnect-stop");
      return;
    }

    ActiveCommand active;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active = active_command_;
    }
    if (active.command_id != 0u) {
      publishCommandStatus(
          active.command_id, active.request_id, active.source, active.command_type,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_TRANSPORT_LOST,
          0u, 0u, "serial_transport_lost");
    }
    RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    cleanbot_interfaces::msg::HardwareStatus status;
    status.stamp = now();
    status.frame_sequence = frame_sequence_.load();
    status.connected = false;
    status.edge_clear = false;
    status.fault = detail;
    status_publisher_->publish(status);
  }

  CommandEncoder command_encoder_;
  StatusFrameParser status_parser_;
  std::unique_ptr<config::ConfigClient> config_client_;
  std::unique_ptr<LowerMachineSerial> serial_;
  std::mutex state_mutex_;
  ActiveCommand active_command_;
  bool last_move_finished_{false};
  bool last_rotate_finished_{false};
  bool configured_{false};
  std::size_t repeat_count_{5u};
  std::atomic<std::uint64_t> frame_sequence_{0u};
  rclcpp::Publisher<cleanbot_interfaces::msg::HardwareStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<cleanbot_interfaces::msg::CommandExecutionStatus>::SharedPtr
      command_status_publisher_;
  rclcpp::Subscription<cleanbot_interfaces::msg::VehicleCommand>::SharedPtr command_subscription_;
};

}  // namespace hardware
}  // namespace cleanbot

// ROS2进程入口：运行下位机节点直到收到退出信号。
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanbot::hardware::LowerMachineNode>());
  rclcpp::shutdown();
  return 0;
}
