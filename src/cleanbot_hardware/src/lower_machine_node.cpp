/*
 * 文件作用：下位机串口网关节点。
 *
 * 输入：订阅 /control/final_cmd，接收命令仲裁节点输出的唯一车辆控制命令。
 * 输出：发布 /hardware/status 和 /hardware/command_status，分别提供下位机实时状态与命令执行状态。
 *
 * 主流程：ROS2命令 -> 编码协议帧 -> 按优先级写入串口 -> 接收状态帧/ACK
 *          -> 更新命令生命周期 -> 发布硬件状态和命令结果。
 * 安全边界：本节点独占下位机串口；串口重连后先下发刹车，只有收到刹车ACK和一帧有效
 *          状态数据，才允许普通运动命令继续下发。
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_client.hpp"
#include "cleanbot_hardware/ack_codec.hpp"
#include "cleanbot_hardware/command_encoder.hpp"
#include "cleanbot_hardware/command_lifecycle.hpp"
#include "cleanbot_hardware/gateway_readiness.hpp"
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
  // 初始化参数、ROS2发布订阅、下位机串口和命令ACK重试定时器。
  LowerMachineNode() : Node("lower_machine_node") {
    // 1. 先建立ROS2通信。此时只接收消息，不打开下位机串口。
    status_publisher_ = create_publisher<cleanbot_interfaces::msg::HardwareStatus>(
        "/hardware/status", common::hardware_status_qos());
    command_status_publisher_ =
        create_publisher<cleanbot_interfaces::msg::CommandExecutionStatus>(
        "/hardware/command_status", common::command_status_qos());
    command_subscription_ = create_subscription<cleanbot_interfaces::msg::VehicleCommand>(
        "/control/final_cmd", common::latest_command_qos(),
        std::bind(&LowerMachineNode::onCommand, this, std::placeholders::_1));

    // 2. 等待配置中心READY；只有成功读取全部硬件参数后才创建并启动串口。
    config_client_ = std::make_unique<config::ConfigClient>(
        this,
        std::vector<std::string>{
            "hardware.lower_machine_port",
            "hardware.lower_machine_baudrate",
            "hardware.command_ack_timeout_ms",
            "hardware.command_max_retries",
            "hardware.command_history_size",
        },
        false,
        [this](const config::ConfigSnapshot& snapshot, const bool initial) {
          configure(snapshot, initial);
        });
  }

  // 节点销毁前停止串口I/O线程，避免回调继续访问已释放的节点对象。
  ~LowerMachineNode() override {
    if (serial_) {
      serial_->stop();
    }
  }

 private:
  struct CommandCorrelation {
    std::uint64_t command_id{0u};
    std::uint64_t request_id{0u};
    std::string source;
  };

  // 最终命令接收链路：消息字段映射 -> 分配sequence -> 安全检查 -> 编码和入队。
  void onCommand(const cleanbot_interfaces::msg::VehicleCommand::SharedPtr command) {
    if (!configured_ || !serial_ || !command_lifecycle_) {
      publishCommandStatus(
          command->command_id,
          command->request_id,
          command->source,
          0u,
          command->status,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_REJECTED,
          0u,
          0u,
          "config_not_ready");
      return;
    }
    CommandFields fields;
    fields.status = command->status;
    fields.x_speed = command->x_speed;
    fields.z_speed = command->z_speed;
    fields.steering_offset = command->steering_offset;
    fields.brush_speed = command->brush_speed;
    fields.target_distance = command->target_distance;
    fields.target_rotation = command->target_rotation;
    fields.heading_deg = command->heading_deg;
    fields.brake = command->brake;
    fields.charge = command->charge;
    {
      std::lock_guard<std::mutex> lock(protocol_mutex_);
      fields.sequence = nextSequenceLocked();
    }
    const auto frame = command_encoder_.encode(fields);

    // 串口未连接时不缓存运动命令，直接明确上报拒绝，避免重连后执行旧命令。
    if (!serial_->connected()) {
      publishCommandStatus(
          command->command_id,
          command->request_id,
          command->source,
          fields.sequence,
          frame[1],
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_REJECTED,
          1u,
          0u,
          "serial_not_connected");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(protocol_mutex_);
      // 重连握手未完成时只允许刹车命令，普通控制命令一律拒绝。
      if (!command->brake && !gateway_readiness_.ready()) {
        publishCommandStatus(
            command->command_id,
            command->request_id,
            command->source,
            fields.sequence,
            frame[1],
            cleanbot_interfaces::msg::CommandExecutionStatus::STATE_REJECTED,
            2u,
            0u,
            "gateway_not_ready");
        return;
      }
    }
    const auto delivery_policy = deliveryPolicyFor(*command);
    const auto write_priority = writePriorityFor(*command);
    sendTrackedCommand(
        frame,
        write_priority,
        coalescingKeyFor(*command),
        delivery_policy,
        CommandCorrelation{
            command->command_id,
            command->request_id,
            command->source});
  }

  // 串口完整帧入口：先区分ACK帧和23字节状态帧，再进入各自处理链路。
  void onFrame(const std::vector<std::uint8_t>& frame) {
    if (frame.size() == AckCodec::kFrameLength && frame.front() == AckCodec::kFrameStart) {
      onAckFrame(frame);
      return;
    }

    const auto result = status_parser_.parse(frame);
    if (!result.parsed) {
      RCLCPP_WARN(get_logger(), "lower-machine frame rejected: %s", result.error.c_str());
      return;
    }

    const auto& parsed = result.status;
    bool became_ready = false;
    {
      std::lock_guard<std::mutex> lock(protocol_mutex_);
      // 有效状态帧是重连恢复条件之一；另一个条件是重连刹车已收到ACK。
      const bool was_ready = gateway_readiness_.ready();
      gateway_readiness_.observe_valid_status();
      became_ready = !was_ready && gateway_readiness_.ready();
    }
    if (became_ready) {
      RCLCPP_INFO(get_logger(), "lower-machine gateway ready after brake ACK and status frame");
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
    status.command_sequence = parsed.command_sequence;
    status.completion_event = completionEvent(parsed.move_finished, parsed.rotate_finished);
    status.completion_new = false;

    // 下位机上报移动/转向完成后进行幂等去重，并回发A2表示上位机已经收到完成事件。
    if (status.completion_event != 0u) {
      CompletionDisposition disposition;
      {
        std::lock_guard<std::mutex> lock(protocol_mutex_);
        disposition = command_lifecycle_->observe_completion(
            parsed.command_sequence, status.completion_event);
      }
      if (disposition == CompletionDisposition::kUnknownCommand) {
        RCLCPP_WARN(
            get_logger(),
            "completion ignored for unknown sequence=%u event=%u",
            parsed.command_sequence, status.completion_event);
      } else {
        status.completion_new = disposition == CompletionDisposition::kFirstSeen;
        serial_->send(
            ack_codec_.encode_completion_ack(
                parsed.command_sequence, status.completion_event),
            WritePriority::kProtocolAck,
            "a2-" + std::to_string(parsed.command_sequence) + "-" +
                std::to_string(status.completion_event));
        RCLCPP_INFO(
            get_logger(),
            "completion %s and A2 sent sequence=%u event=%u",
            status.completion_new ? "accepted" : "duplicate",
            parsed.command_sequence,
            status.completion_event);
        if (status.completion_new) {
          const auto correlation = commandCorrelationFor(parsed.command_sequence);
          publishCommandStatus(
              correlation.command_id,
              correlation.request_id,
              correlation.source,
              parsed.command_sequence,
              parsed.status,
              cleanbot_interfaces::msg::CommandExecutionStatus::STATE_COMPLETED,
              0u,
              status.completion_event,
              "lower_machine_completion");
        }
      }
    }
    status_publisher_->publish(status);
  }

  // 处理下位机A1命令确认：匹配sequence、结束等待ACK状态，并发布接受或拒绝结果。
  void onAckFrame(const std::vector<std::uint8_t>& frame) {
    const auto parsed = ack_codec_.parse(frame);
    if (!parsed.parsed) {
      RCLCPP_WARN(get_logger(), "ACK frame rejected: %s", parsed.error.c_str());
      return;
    }
    if (parsed.ack.frame_type != AckCodec::kCommandAck) {
      RCLCPP_WARN(
          get_logger(), "unexpected ACK type=0x%02x", parsed.ack.frame_type);
      return;
    }

    AckDisposition disposition;
    bool became_ready = false;
    bool reconnect_brake_rejected = false;
    {
      std::lock_guard<std::mutex> lock(protocol_mutex_);
      const bool was_ready = gateway_readiness_.ready();
      // 如果该ACK属于重连刹车，则同步更新网关就绪状态。
      const bool reconnect_ack = gateway_readiness_.observe_command_ack(
          parsed.ack.sequence,
          parsed.ack.subject_type,
          parsed.ack.result == AckCodec::kAccepted);
      became_ready = reconnect_ack && !was_ready && gateway_readiness_.ready();
      reconnect_brake_rejected =
          parsed.ack.sequence == gateway_readiness_.brake_sequence() &&
          parsed.ack.subject_type == 0u && parsed.ack.result != AckCodec::kAccepted;
      disposition = command_lifecycle_->handle_command_ack(parsed.ack);
    }
    if (became_ready) {
      RCLCPP_INFO(get_logger(), "lower-machine gateway ready after brake ACK and status frame");
    }
    if (disposition == AckDisposition::kAccepted) {
      const auto correlation = commandCorrelationFor(parsed.ack.sequence);
      publishCommandStatus(
          correlation.command_id,
          correlation.request_id,
          correlation.source,
          parsed.ack.sequence,
          parsed.ack.subject_type,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_ACKNOWLEDGED,
          parsed.ack.result,
          0u,
          "command_acknowledged");
      RCLCPP_DEBUG(
          get_logger(), "A1 accepted sequence=%u type=%u",
          parsed.ack.sequence, parsed.ack.subject_type);
    } else if (disposition == AckDisposition::kRejected) {
      const auto correlation = commandCorrelationFor(parsed.ack.sequence);
      publishCommandStatus(
          correlation.command_id,
          correlation.request_id,
          correlation.source,
          parsed.ack.sequence,
          parsed.ack.subject_type,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_REJECTED,
          parsed.ack.result,
          0u,
          "command_rejected");
      RCLCPP_ERROR(
          get_logger(), "A1 rejected sequence=%u type=%u result=%u",
          parsed.ack.sequence, parsed.ack.subject_type, parsed.ack.result);
      if (reconnect_brake_rejected) {
        startReconnectBrakeHandshake();
      }
    } else {
      RCLCPP_WARN(
          get_logger(), "A1 unmatched sequence=%u type=%u",
          parsed.ack.sequence, parsed.ack.subject_type);
    }
  }

  // 定时处理命令可靠性：有限命令/刹车按原帧重发，连续运动命令只保留最新值。
  void onRetryTimer() {
    if (!configured_ || !serial_ || !command_lifecycle_) {
      return;
    }
    RetryBatch batch;
    {
      std::lock_guard<std::mutex> lock(protocol_mutex_);
      batch = command_lifecycle_->collect_due_retries(monotonicMs());
    }
    for (const auto& frame : batch.frames) {
      serial_->send(
          frame,
          writePriorityForFrame(frame),
          coalescingKeyForFrame(frame));
    }
    bool reconnect_brake_timed_out = false;
    for (const auto& timed_out : batch.timed_out_commands) {
      {
        std::lock_guard<std::mutex> lock(protocol_mutex_);
        reconnect_brake_timed_out = reconnect_brake_timed_out ||
            timed_out.sequence == gateway_readiness_.brake_sequence();
      }
      const auto correlation = commandCorrelationFor(timed_out.sequence);
      publishCommandStatus(
          correlation.command_id,
          correlation.request_id,
          correlation.source,
          timed_out.sequence,
          timed_out.command_type,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_TIMED_OUT,
          0u,
          0u,
          "command_ack_timeout");
      RCLCPP_ERROR(get_logger(), "A1 timeout sequence=%u", timed_out.sequence);
    }
    if (reconnect_brake_timed_out && serial_->connected()) {
      // 重连刹车多次未确认时重新开始握手，期间仍禁止普通运动命令。
      startReconnectBrakeHandshake();
    }
  }

  // 将一条新命令交给串口，并根据异步入队/写入结果维护ACK生命周期和ROS2状态。
  void sendTrackedCommand(
      const std::vector<std::uint8_t>& frame,
      const WritePriority priority,
      const std::string& coalescing_key,
      const CommandDeliveryPolicy policy,
      const CommandCorrelation& correlation) {
    if (!serial_ || frame.size() < CommandEncoder::kCommandLength) {
      return;
    }
    const auto sequence = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(frame[17]) << 8u) |
        static_cast<std::uint16_t>(frame[18]));
    const auto command_type = frame[1];
    serial_->send(
        frame,
        priority,
        coalescing_key,
        [this, frame, sequence, command_type, policy, correlation](
            const SendEvent event, const std::string& detail) {
          onSendEvent(
              event,
              detail,
              frame,
              sequence,
              command_type,
              policy,
              correlation);
        });
  }

  // 串口事件只表达传输事实；本方法负责转换为带业务关联的命令生命周期状态。
  void onSendEvent(
      const SendEvent event,
      const std::string& detail,
      const std::vector<std::uint8_t>& frame,
      const std::uint16_t sequence,
      const std::uint8_t command_type,
      const CommandDeliveryPolicy policy,
      const CommandCorrelation& correlation) {
    if (event == SendEvent::kQueued) {
      {
        std::lock_guard<std::mutex> lock(protocol_mutex_);
        rememberCommandCorrelation(sequence, correlation);
        command_lifecycle_->track(
            sequence, command_type, frame, monotonicMs(), policy);
      }
      publishCommandStatus(
          correlation.command_id,
          correlation.request_id,
          correlation.source,
          sequence,
          command_type,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_QUEUED,
          0u,
          0u,
          detail);
      return;
    }
    if (event == SendEvent::kWritten) {
      publishCommandStatus(
          correlation.command_id,
          correlation.request_id,
          correlation.source,
          sequence,
          command_type,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_SENT,
          0u,
          0u,
          detail);
      return;
    }
    if (event == SendEvent::kSuperseded) {
      {
        std::lock_guard<std::mutex> lock(protocol_mutex_);
        command_lifecycle_->cancel_pending(sequence);
        forgetCommandCorrelationLocked(sequence);
      }
      publishCommandStatus(
          correlation.command_id,
          correlation.request_id,
          correlation.source,
          sequence,
          command_type,
          cleanbot_interfaces::msg::CommandExecutionStatus::STATE_SUPERSEDED,
          0u,
          0u,
          detail);
      return;
    }

    publishCommandStatus(
        correlation.command_id,
        correlation.request_id,
        correlation.source,
        sequence,
        command_type,
        cleanbot_interfaces::msg::CommandExecutionStatus::STATE_REJECTED,
        event == SendEvent::kQueueRejected ? 3u : 1u,
        0u,
        detail);
  }

  // 将业务标识、协议sequence和当前生命周期状态统一发布到命令状态Topic。
  void publishCommandStatus(
      const std::uint64_t command_id,
      const std::uint64_t request_id,
      const std::string& source,
      const std::uint16_t sequence,
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
    status.sequence = sequence;
    status.command_type = command_type;
    status.state = state;
    status.result = result;
    status.completion_event = completion_event;
    status.detail = detail;
    command_status_publisher_->publish(status);
  }

  // 保存协议sequence到ROS2业务命令标识的映射，并按容量淘汰最旧记录。
  void rememberCommandCorrelation(
      const std::uint16_t sequence,
      const CommandCorrelation& correlation) {
    const auto existing = command_correlations_.find(sequence);
    if (existing != command_correlations_.end()) {
      command_id_order_.erase(
          std::remove(command_id_order_.begin(), command_id_order_.end(), sequence),
          command_id_order_.end());
    }
    command_correlations_[sequence] = correlation;
    command_id_order_.push_back(sequence);
    while (command_id_order_.size() > command_history_size_) {
      const auto expired = command_id_order_.front();
      command_id_order_.pop_front();
      command_correlations_.erase(expired);
    }
  }

  // 在protocol_mutex_保护下删除指定sequence的业务关联和历史顺序记录。
  void forgetCommandCorrelationLocked(const std::uint16_t sequence) {
    command_correlations_.erase(sequence);
    command_id_order_.erase(
        std::remove(command_id_order_.begin(), command_id_order_.end(), sequence),
        command_id_order_.end());
  }

  // 为断线或握手重启时被清理的命令逐条发布终止状态，并删除业务关联。
  void publishInterruptedCommands(
      const std::vector<InterruptedCommand>& interrupted,
      const std::uint8_t state,
      const std::string& detail) {
    for (const auto& command : interrupted) {
      CommandCorrelation correlation;
      {
        std::lock_guard<std::mutex> lock(protocol_mutex_);
        const auto found = command_correlations_.find(command.sequence);
        if (found != command_correlations_.end()) {
          correlation = found->second;
        }
        forgetCommandCorrelationLocked(command.sequence);
      }
      publishCommandStatus(
          correlation.command_id,
          correlation.request_id,
          correlation.source,
          command.sequence,
          command.command_type,
          state,
          0u,
          0u,
          detail);
    }
  }

  // 根据下位机返回的sequence查找原始command_id、request_id和命令来源。
  CommandCorrelation commandCorrelationFor(const std::uint16_t sequence) {
    std::lock_guard<std::mutex> lock(protocol_mutex_);
    const auto found = command_correlations_.find(sequence);
    return found == command_correlations_.end()
        ? CommandCorrelation{}
        : found->second;
  }

  // 将移动/转向完成布尔值组合成协议完成事件位：移动1、转向2、同时完成3。
  static std::uint8_t completionEvent(const bool move_finished, const bool rotate_finished) {
    if (move_finished && rotate_finished) {
      return 0x03u;
    }
    if (move_finished) {
      return 0x01u;
    }
    return rotate_finished ? 0x02u : 0x00u;
  }

  // 返回不受系统时间校准影响的单调毫秒时间，用于可靠计算ACK超时。
  static std::uint64_t monotonicMs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  // 读取串口和协议参数，构建命令生命周期管理器并启动串口线程。
  void configure(const config::ConfigSnapshot& snapshot, const bool initial) {
    if (!initial && configured_) {
      RCLCPP_WARN(
          get_logger(),
          "lower-machine configuration changed; restart required before it becomes active");
      return;
    }
    const auto port = snapshot.get_string("hardware.lower_machine_port");
    const auto baudrate = snapshot.get_integer("hardware.lower_machine_baudrate");
    const auto ack_timeout_ms = snapshot.get_integer("hardware.command_ack_timeout_ms");
    const auto max_retries = snapshot.get_integer("hardware.command_max_retries");
    const auto history_size = snapshot.get_integer("hardware.command_history_size");
    command_history_size_ = static_cast<std::size_t>(history_size);
    command_lifecycle_ = std::make_unique<CommandLifecycle>(
        static_cast<std::uint64_t>(ack_timeout_ms),
        static_cast<std::size_t>(max_retries),
        command_history_size_);
    serial_ = std::make_unique<LowerMachineSerial>(
        port,
        static_cast<unsigned int>(baudrate),
        std::bind(&LowerMachineNode::onFrame, this, std::placeholders::_1),
        std::bind(
            &LowerMachineNode::onConnectionChanged,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    configured_ = true;
    serial_->start();
    retry_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&LowerMachineNode::onRetryTimer, this));
    RCLCPP_INFO(
        get_logger(),
        "lower-machine configuration ready port=%s baudrate=%lld ack_timeout_ms=%lld max_retries=%lld",
        port.c_str(),
        static_cast<long long>(baudrate),
        static_cast<long long>(ack_timeout_ms),
        static_cast<long long>(max_retries));
  }

  // 在protocol_mutex_保护下分配非零16位协议sequence，并在溢出后从1继续。
  std::uint16_t nextSequenceLocked() {
    const auto sequence = next_command_sequence_++;
    if (next_command_sequence_ == 0u) {
      next_command_sequence_ = 1u;
    }
    return sequence;
  }

  // 判断命令是否为必须完成并等待确认的有限动作，而不是可被新值覆盖的连续运动命令。
  static bool isFiniteCommand(
      const cleanbot_interfaces::msg::VehicleCommand& command) {
    return command.charge || command.target_distance != 0 ||
        command.target_rotation != 0 || command.status == 2u ||
        command.status == 3u || command.status == 5u;
  }

  // 连续运动命令采用最新值策略；刹车、转向、定距和充电采用原帧重试策略。
  static CommandDeliveryPolicy deliveryPolicyFor(
      const cleanbot_interfaces::msg::VehicleCommand& command) {
    // 连续控制采用“最新值覆盖旧值”；刹车和有限动作必须等待ACK并允许原帧重试。
    return !command.brake && !isFiniteCommand(command)
        ? CommandDeliveryPolicy::kLatestOnly
        : CommandDeliveryPolicy::kRetryUntilAck;
  }

  // 为新ROS2命令选择串口写优先级：刹车最高，其次有限动作，最后连续控制。
  static WritePriority writePriorityFor(
      const cleanbot_interfaces::msg::VehicleCommand& command) {
    if (command.brake) {
      return WritePriority::kSafety;
    }
    return isFiniteCommand(command)
        ? WritePriority::kFinite
        : WritePriority::kContinuous;
  }

  // 连续运动和普通刹车分别使用固定合并键，只替换尚未开始写入的同类旧命令。
  static std::string coalescingKeyFor(
      const cleanbot_interfaces::msg::VehicleCommand& command) {
    if (command.brake) {
      return "brake";
    }
    return !command.brake && !isFiniteCommand(command) ? "motion" : "";
  }

  // 从已编码重发帧中恢复写入优先级，保证重发顺序与首次发送一致。
  static WritePriority writePriorityForFrame(
      const std::vector<std::uint8_t>& frame) {
    return classify_command_frame(frame);
  }

  // 为重发的连续运动帧恢复合并键，避免重试队列累积过时速度。
  static std::string coalescingKeyForFrame(
      const std::vector<std::uint8_t>& frame) {
    return writePriorityForFrame(frame) == WritePriority::kContinuous ? "motion" : "";
  }

  // 清理断线前命令，发送新的高优先级刹车，并等待A1和有效状态帧完成恢复握手。
  void startReconnectBrakeHandshake() {
    // 清除断线前尚未完成的命令，生成新的刹车sequence并将网关置为等待恢复状态。
    CommandFields fields;
    fields.brake = true;
    std::vector<InterruptedCommand> interrupted;
    {
      std::lock_guard<std::mutex> lock(protocol_mutex_);
      interrupted = command_lifecycle_->clear_pending();
      fields.sequence = nextSequenceLocked();
      gateway_readiness_.on_connected(fields.sequence);
    }
    publishInterruptedCommands(
        interrupted,
        cleanbot_interfaces::msg::CommandExecutionStatus::STATE_SUPERSEDED,
        "reconnect_handshake_restarted");
    const auto frame = command_encoder_.encode(fields);
    sendTrackedCommand(
        frame,
        WritePriority::kSafety,
        "reconnect_brake",
        CommandDeliveryPolicy::kRetryUntilAck,
        CommandCorrelation{0u, 0u, "reconnect_brake"});
    RCLCPP_WARN(
        get_logger(),
        "lower-machine connected; reconnect brake sent sequence=%u, waiting for ACK and status",
        fields.sequence);
  }

  // 串口连接状态回调：连接时启动刹车握手；断开时清理旧命令并发布离线状态。
  void onConnectionChanged(const bool connected, const std::string& detail) {
    if (connected) {
      RCLCPP_INFO(get_logger(), "%s", detail.c_str());
      startReconnectBrakeHandshake();
      return;
    }
    std::vector<InterruptedCommand> interrupted;
    {
      std::lock_guard<std::mutex> lock(protocol_mutex_);
      gateway_readiness_.on_disconnected();
      interrupted = command_lifecycle_->clear_pending();
    }
    publishInterruptedCommands(
        interrupted,
        cleanbot_interfaces::msg::CommandExecutionStatus::STATE_TRANSPORT_LOST,
        "serial_transport_lost");
    RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    cleanbot_interfaces::msg::HardwareStatus status;
    status.stamp = now();
    status.frame_sequence = frame_sequence_.load();
    status.connected = false;
    status.edge_clear = false;
    status.fault = detail;
    status_publisher_->publish(status);
  }

  AckCodec ack_codec_;
  CommandEncoder command_encoder_;
  StatusFrameParser status_parser_;
  GatewayReadiness gateway_readiness_;
  std::unique_ptr<config::ConfigClient> config_client_;
  std::unique_ptr<CommandLifecycle> command_lifecycle_;
  std::unique_ptr<LowerMachineSerial> serial_;
  bool configured_{false};
  std::mutex protocol_mutex_;
  std::size_t command_history_size_{256u};
  std::unordered_map<std::uint16_t, CommandCorrelation> command_correlations_;
  std::deque<std::uint16_t> command_id_order_;
  std::uint16_t next_command_sequence_{1u};
  std::atomic<std::uint64_t> frame_sequence_{0u};
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::Publisher<cleanbot_interfaces::msg::HardwareStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<cleanbot_interfaces::msg::CommandExecutionStatus>::SharedPtr
      command_status_publisher_;
  rclcpp::Subscription<cleanbot_interfaces::msg::VehicleCommand>::SharedPtr command_subscription_;
};

}  // namespace hardware
}  // namespace cleanbot

// 程序入口：初始化ROS2，持续处理订阅、定时器和串口回调，退出时释放ROS2资源。
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanbot::hardware::LowerMachineNode>());
  rclcpp::shutdown();
  return 0;
}
