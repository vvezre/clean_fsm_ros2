#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cleanbot_interfaces/msg/config_changed.hpp"
#include "cleanbot_interfaces/msg/config_status.hpp"
#include "cleanbot_interfaces/srv/get_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cleanbot {
namespace config {

class ConfigSnapshot {
 public:
  ConfigSnapshot() = default;
  explicit ConfigSnapshot(std::map<std::string, std::string> values);

  bool contains(const std::string& key) const;
  std::string get_string(const std::string& key) const;
  std::int64_t get_integer(const std::string& key) const;
  double get_double(const std::string& key) const;
  bool get_boolean(const std::string& key) const;
  const std::map<std::string, std::string>& values() const;

 private:
  const std::string& required_value(const std::string& key) const;
  std::map<std::string, std::string> values_;
};

class ConfigClient {
 public:
  using ReadyCallback = std::function<void(const ConfigSnapshot&, bool initial)>;
  using StatusCallback =
      std::function<void(const cleanbot_interfaces::msg::ConfigStatus&)>;

  ConfigClient(
      rclcpp::Node* node,
      std::vector<std::string> keys,
      bool include_sensitive,
      ReadyCallback ready_callback,
      StatusCallback status_callback = StatusCallback());

  bool ready() const;
  ConfigSnapshot snapshot() const;
  void request_refresh();

 private:
  void on_status(const cleanbot_interfaces::msg::ConfigStatus::SharedPtr message);
  void on_changed(const cleanbot_interfaces::msg::ConfigChanged::SharedPtr message);
  void refresh_if_possible();
  bool change_is_relevant(const std::vector<std::string>& changed_keys) const;

  rclcpp::Node* node_{nullptr};
  std::vector<std::string> keys_;
  bool include_sensitive_{false};
  ReadyCallback ready_callback_;
  StatusCallback status_callback_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> refresh_requested_{false};
  bool request_in_flight_{false};
  bool ever_loaded_{false};
  mutable std::mutex mutex_;
  ConfigSnapshot snapshot_;
  rclcpp::Client<cleanbot_interfaces::srv::GetConfig>::SharedPtr client_;
  rclcpp::Subscription<cleanbot_interfaces::msg::ConfigStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<cleanbot_interfaces::msg::ConfigChanged>::SharedPtr changed_subscription_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
};

}  // namespace config
}  // namespace cleanbot
