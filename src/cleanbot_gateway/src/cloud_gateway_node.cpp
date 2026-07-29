/*
 * 文件作用：小程序/云平台控制入口。
 *
 * 云平台继续使用现有 REST -> MQTT 链路。本节点订阅本机的 RAILCAR/S/{设备编号}
 * 主题，将 joystick_move 转换为手动控制 Topic，将 parking/stop 转换为软件急停
 * Topic，并通过 RAILCAR/R/{设备编号} 回传 ACK 和 command_result。
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <mqtt/async_client.h>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_client.hpp"
#include "cleanbot_gateway/cloud_command.hpp"
#include "cleanbot_gateway/cloud_message_codec.hpp"
#include "cleanbot_interfaces/msg/vehicle_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cleanbot {
namespace gateway {

class CloudGatewayNode : public rclcpp::Node, public virtual mqtt::callback {
 public:
  CloudGatewayNode() : Node("cloud_gateway_node") {
    manual_publisher_ = create_publisher<cleanbot_interfaces::msg::VehicleCommand>(
        "/control/manual_cmd", common::latest_command_qos());
    emergency_publisher_ = create_publisher<cleanbot_interfaces::msg::VehicleCommand>(
        "/control/emergency_cmd", common::latest_command_qos());

    reconnect_timer_ = create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&CloudGatewayNode::ensureMqttConnected, this));
    connect_listener_ = std::make_unique<ConnectListener>(this);

    config_client_ = std::make_unique<config::ConfigClient>(
        this,
        std::vector<std::string>{
            "cloud.mqtt.enabled",
            "cloud.mqtt.host",
            "cloud.mqtt.port",
            "cloud.mqtt.username",
            "cloud.mqtt.password",
            "cloud.mqtt.keepalive_sec",
            "cloud.mqtt.qos",
            "cloud.command_max_age_sec",
            "device.company_code",
            "device.product_model",
            "device.product_id",
            "motion.manual_max_speed",
        },
        true,
        [this](const config::ConfigSnapshot& snapshot, const bool initial) {
          configure(snapshot, initial);
        });
  }

  ~CloudGatewayNode() override {
    shutting_down_.store(true);
    reconnect_timer_.reset();
    mqtt::token_ptr disconnect_token;
    {
      std::lock_guard<std::mutex> lock(mqtt_mutex_);
      try {
        if (mqtt_client_ && mqtt_client_->is_connected()) {
          disconnect_token = mqtt_client_->disconnect();
        }
      } catch (const mqtt::exception&) {
      }
    }
    if (disconnect_token) {
      try {
        disconnect_token->wait();
      } catch (const mqtt::exception&) {
      }
    }
    {
      std::lock_guard<std::mutex> lock(mqtt_mutex_);
      mqtt_client_.reset();
    }
  }

 private:
  using VehicleCommand = cleanbot_interfaces::msg::VehicleCommand;

  class ConnectListener : public virtual mqtt::iaction_listener {
   public:
    explicit ConnectListener(CloudGatewayNode* owner) : owner_(owner) {}

    void on_failure(const mqtt::token& token) override {
      if (owner_ != nullptr) {
        owner_->onConnectFinished(false, token.get_message_id());
      }
    }

    void on_success(const mqtt::token& token) override {
      if (owner_ != nullptr) {
        owner_->onConnectFinished(true, token.get_message_id());
      }
    }

   private:
    CloudGatewayNode* owner_{nullptr};
  };

  void configure(const config::ConfigSnapshot& snapshot, const bool initial) {
    CloudCommandParameters command_parameters;
    command_parameters.command_max_age_sec =
        snapshot.get_double("cloud.command_max_age_sec");
    command_parameters.joystick.max_linear_speed = static_cast<std::int32_t>(
        snapshot.get_integer("motion.manual_max_speed"));
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      translator_ = std::make_unique<CloudCommandTranslator>(command_parameters);
    }

    const bool enabled = snapshot.get_boolean("cloud.mqtt.enabled");
    if (!initial && mqtt_initialized_) {
      RCLCPP_INFO(
          get_logger(),
          "manual maximum speed updated to %d and is effective immediately",
          command_parameters.joystick.max_linear_speed);
      return;
    }

    mqtt_enabled_.store(enabled);
    if (!enabled) {
      RCLCPP_WARN(
          get_logger(),
          "cloud MQTT gateway is disabled; local ROS2 control remains available");
      return;
    }

    identity_.company_code = snapshot.get_string("device.company_code");
    identity_.product_model = snapshot.get_string("device.product_model");
    identity_.product_id = snapshot.get_string("device.product_id");
    input_topic_ = "RAILCAR/S/" + identity_.product_model + identity_.product_id;
    output_topic_ = "RAILCAR/R/" + identity_.product_model + identity_.product_id;
    mqtt_qos_ = static_cast<int>(snapshot.get_integer("cloud.mqtt.qos"));
    codec_ = std::make_unique<CloudMessageCodec>(identity_);

    const auto server_uri =
        "tcp://" + snapshot.get_string("cloud.mqtt.host") + ":" +
        std::to_string(snapshot.get_integer("cloud.mqtt.port"));
    const auto client_id =
        "cleanbot-" + identity_.product_model + identity_.product_id;

    mqtt::connect_options options;
    options.set_clean_session(true);
    options.set_automatic_reconnect(1, 30);
    options.set_keep_alive_interval(static_cast<int>(
        snapshot.get_integer("cloud.mqtt.keepalive_sec")));
    const auto username = snapshot.get_string("cloud.mqtt.username");
    const auto password = snapshot.get_string("cloud.mqtt.password");
    if (!username.empty()) {
      options.set_user_name(username);
    }
    if (!password.empty()) {
      options.set_password(password);
    }

    {
      std::lock_guard<std::mutex> lock(mqtt_mutex_);
      connect_options_ = options;
      mqtt_client_ = std::make_unique<mqtt::async_client>(server_uri, client_id);
      mqtt_client_->set_callback(*this);
    }
    mqtt_initialized_ = true;
    RCLCPP_INFO(
        get_logger(),
        "cloud gateway configured: subscribe=%s publish=%s",
        input_topic_.c_str(), output_topic_.c_str());
    ensureMqttConnected();
  }

  void ensureMqttConnected() {
    if (shutting_down_.load() || !mqtt_enabled_.load() || !mqtt_initialized_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    if (!mqtt_client_ || mqtt_client_->is_connected() ||
        connect_in_progress_.exchange(true)) {
      return;
    }
    try {
      mqtt_client_->connect(connect_options_, nullptr, *connect_listener_);
    } catch (const mqtt::exception& exception) {
      connect_in_progress_.store(false);
      RCLCPP_ERROR(
          get_logger(), "cloud MQTT connect failed: %s", exception.what());
    }
  }

  void onConnectFinished(const bool success, const int message_id) {
    connect_in_progress_.store(false);
    if (!success && !shutting_down_.load()) {
      RCLCPP_WARN(
          get_logger(),
          "cloud MQTT initial connection failed, message_id=%d; retry scheduled",
          message_id);
    }
  }

  void connected(const std::string& cause) override {
    connect_in_progress_.store(false);
    if (shutting_down_.load()) {
      return;
    }
    try {
      std::lock_guard<std::mutex> lock(mqtt_mutex_);
      if (mqtt_client_) {
        mqtt_client_->subscribe(input_topic_, mqtt_qos_);
      }
      RCLCPP_INFO(
          get_logger(), "cloud MQTT connected (%s), subscribed to %s",
          cause.c_str(), input_topic_.c_str());
    } catch (const mqtt::exception& exception) {
      RCLCPP_ERROR(
          get_logger(), "cloud MQTT subscribe failed: %s", exception.what());
    }
  }

  void connection_lost(const std::string& cause) override {
    connect_in_progress_.store(false);
    if (!shutting_down_.load()) {
      RCLCPP_WARN(
          get_logger(), "cloud MQTT connection lost: %s", cause.c_str());
    }
  }

  void message_arrived(mqtt::const_message_ptr message) override {
    if (shutting_down_.load() || !codec_ || !message) {
      return;
    }
    const auto decoded = codec_->decode(
        message->get_payload_str(), message->is_retained());
    if (!decoded.success) {
      RCLCPP_WARN(
          get_logger(), "cloud command rejected while decoding: %s (%s)",
          decoded.message.c_str(), decoded.code.c_str());
      if (!decoded.command.command_id.empty()) {
        publishResult(decoded.command, false, decoded.code, decoded.message);
      }
      return;
    }

    publishAck(decoded.command, "accepted");
    CloudCommandDecision decision;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!translator_) {
        publishResult(
            decoded.command, false, "GATEWAY_NOT_READY",
            "cloud command translator is not ready");
        return;
      }
      decision = translator_->translate(decoded.command, unixSeconds());
    }
    if (!decision.accepted) {
      publishResult(
          decoded.command, false, decision.code, decision.message);
      return;
    }

    if (decision.kind == CloudCommandKind::kManualJoystick) {
      publishManualCommand(decoded.command, decision);
    } else if (decision.kind == CloudCommandKind::kEmergencyStop) {
      publishEmergencyStop(decoded.command);
    }
    publishResult(decoded.command, true, decision.code, decision.message);
  }

  void delivery_complete(mqtt::delivery_token_ptr) override {}

  void publishManualCommand(
      const CloudCommandInput& input,
      const CloudCommandDecision& decision) {
    VehicleCommand command;
    command.stamp = now();
    command.command_id = next_command_id_.fetch_add(1u);
    command.request_id = requestId(input.command_id);
    command.source = "cloud_joystick";
    command.priority = VehicleCommand::PRIORITY_MANUAL;
    command.active = true;
    command.operator_intent = true;
    command.status = decision.brake ? 0u : 1u;
    command.x_speed = decision.x_speed;
    command.steering_offset = decision.steering_offset;
    command.brake = decision.brake;
    manual_publisher_->publish(command);
  }

  void publishEmergencyStop(const CloudCommandInput& input) {
    VehicleCommand command;
    command.stamp = now();
    command.command_id = next_command_id_.fetch_add(1u);
    command.request_id = requestId(input.command_id);
    command.source = "cloud_emergency_stop";
    command.priority = VehicleCommand::PRIORITY_EMERGENCY;
    command.active = true;
    command.operator_intent = true;
    command.status = 0u;
    command.brake = true;
    emergency_publisher_->publish(command);
  }

  void publishAck(
      const CloudCommandInput& input,
      const std::string& status) {
    publishMqtt(codec_->encode_ack(input, status, unixSeconds()));
  }

  void publishResult(
      const CloudCommandInput& input,
      const bool success,
      const std::string& code,
      const std::string& message) {
    publishMqtt(codec_->encode_result(
        input, success, code, message, unixSeconds()));
  }

  void publishMqtt(const std::string& payload) {
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    if (!mqtt_client_ || !mqtt_client_->is_connected()) {
      RCLCPP_WARN(get_logger(), "cannot publish cloud result: MQTT is disconnected");
      return;
    }
    try {
      mqtt_client_->publish(output_topic_, payload.data(), payload.size(), mqtt_qos_, false);
    } catch (const mqtt::exception& exception) {
      RCLCPP_ERROR(
          get_logger(), "cloud MQTT publish failed: %s", exception.what());
    }
  }

  static std::uint64_t requestId(const std::string& value) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const unsigned char character : value) {
      hash ^= static_cast<std::uint64_t>(character);
      hash *= 1099511628211ull;
    }
    return hash == 0u ? 1u : hash;
  }

  static std::int64_t unixSeconds() {
    return std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
  }

  std::unique_ptr<config::ConfigClient> config_client_;
  std::unique_ptr<CloudCommandTranslator> translator_;
  std::unique_ptr<CloudMessageCodec> codec_;
  std::mutex command_mutex_;

  CloudIdentity identity_;
  std::string input_topic_;
  std::string output_topic_;
  int mqtt_qos_{1};
  std::atomic<bool> mqtt_enabled_{false};
  bool mqtt_initialized_{false};
  std::atomic<bool> connect_in_progress_{false};
  std::atomic<bool> shutting_down_{false};
  std::mutex mqtt_mutex_;
  mqtt::connect_options connect_options_;
  std::unique_ptr<mqtt::async_client> mqtt_client_;
  std::unique_ptr<ConnectListener> connect_listener_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  std::atomic<std::uint64_t> next_command_id_{1u};
  rclcpp::Publisher<VehicleCommand>::SharedPtr manual_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr emergency_publisher_;
};

}  // namespace gateway
}  // namespace cleanbot

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanbot::gateway::CloudGatewayNode>());
  rclcpp::shutdown();
  return 0;
}
