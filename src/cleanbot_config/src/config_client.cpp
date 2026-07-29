#include "cleanbot_config/config_client.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <utility>

#include "cleanbot_common/qos_profiles.hpp"

namespace cleanbot {
namespace config {

ConfigSnapshot::ConfigSnapshot(std::map<std::string, std::string> values)
    : values_(std::move(values)) {}

bool ConfigSnapshot::contains(const std::string& key) const {
  return values_.find(key) != values_.end();
}

std::string ConfigSnapshot::get_string(const std::string& key) const {
  return required_value(key);
}

std::int64_t ConfigSnapshot::get_integer(const std::string& key) const {
  const auto& value = required_value(key);
  errno = 0;
  char* end = nullptr;
  const auto parsed = std::strtoll(value.c_str(), &end, 10);
  if (errno != 0 || end == value.c_str() || *end != '\0') {
    throw std::runtime_error("configuration value is not an integer: " + key);
  }
  return static_cast<std::int64_t>(parsed);
}

double ConfigSnapshot::get_double(const std::string& key) const {
  const auto& value = required_value(key);
  errno = 0;
  char* end = nullptr;
  const auto parsed = std::strtod(value.c_str(), &end);
  if (errno != 0 || end == value.c_str() || *end != '\0') {
    throw std::runtime_error("configuration value is not a number: " + key);
  }
  return parsed;
}

bool ConfigSnapshot::get_boolean(const std::string& key) const {
  auto value = required_value(key);
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](const unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
  if (value == "true" || value == "1") {
    return true;
  }
  if (value == "false" || value == "0") {
    return false;
  }
  throw std::runtime_error("configuration value is not a boolean: " + key);
}

const std::map<std::string, std::string>& ConfigSnapshot::values() const {
  return values_;
}

const std::string& ConfigSnapshot::required_value(const std::string& key) const {
  const auto found = values_.find(key);
  if (found == values_.end()) {
    throw std::runtime_error("configuration key is missing: " + key);
  }
  return found->second;
}

ConfigClient::ConfigClient(
    rclcpp::Node* node,
    std::vector<std::string> keys,
    const bool include_sensitive,
    ReadyCallback ready_callback,
    StatusCallback status_callback)
    : node_(node),
      keys_(std::move(keys)),
      include_sensitive_(include_sensitive),
      ready_callback_(std::move(ready_callback)),
      status_callback_(std::move(status_callback)) {
  client_ = node_->create_client<cleanbot_interfaces::srv::GetConfig>("/config/get");
  status_subscription_ =
      node_->create_subscription<cleanbot_interfaces::msg::ConfigStatus>(
          "/config/status",
          common::latched_status_qos(),
          std::bind(&ConfigClient::on_status, this, std::placeholders::_1));
  changed_subscription_ =
      node_->create_subscription<cleanbot_interfaces::msg::ConfigChanged>(
          "/config/changed",
          common::latched_status_qos(),
          std::bind(&ConfigClient::on_changed, this, std::placeholders::_1));
  refresh_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ConfigClient::refresh_if_possible, this));
}

bool ConfigClient::ready() const {
  return ready_.load();
}

ConfigSnapshot ConfigClient::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

void ConfigClient::request_refresh() {
  refresh_requested_.store(true);
}

void ConfigClient::on_status(
    const cleanbot_interfaces::msg::ConfigStatus::SharedPtr message) {
  if (status_callback_) {
    status_callback_(*message);
  }
  if (!message->config_ready) {
    ready_.store(false);
    return;
  }
  request_refresh();
}

void ConfigClient::on_changed(
    const cleanbot_interfaces::msg::ConfigChanged::SharedPtr message) {
  if (change_is_relevant(message->changed_keys)) {
    request_refresh();
  }
}

void ConfigClient::refresh_if_possible() {
  if (!refresh_requested_.load() || request_in_flight_ || !client_->service_is_ready()) {
    return;
  }
  request_in_flight_ = true;
  refresh_requested_.store(false);
  auto request = std::make_shared<cleanbot_interfaces::srv::GetConfig::Request>();
  request->keys = keys_;
  request->include_sensitive = include_sensitive_;
  client_->async_send_request(
      request,
      [this](rclcpp::Client<cleanbot_interfaces::srv::GetConfig>::SharedFuture future) {
        request_in_flight_ = false;
        const auto response = future.get();
        if (!response->success) {
          ready_.store(false);
          RCLCPP_ERROR(
              node_->get_logger(), "configuration fetch failed: %s",
              response->message.c_str());
          return;
        }
        std::map<std::string, std::string> values;
        for (const auto& entry : response->entries) {
          if (!entry.configured) {
            ready_.store(false);
            RCLCPP_ERROR(
                node_->get_logger(), "required configuration missing: %s",
                entry.key.c_str());
            return;
          }
          values[entry.key] = entry.value_text;
        }
        const bool initial = !ever_loaded_;
        ConfigSnapshot loaded(std::move(values));
        {
          std::lock_guard<std::mutex> lock(mutex_);
          snapshot_ = loaded;
          ever_loaded_ = true;
        }
        ready_.store(true);
        if (ready_callback_) {
          ready_callback_(loaded, initial);
        }
      });
}

bool ConfigClient::change_is_relevant(
    const std::vector<std::string>& changed_keys) const {
  if (keys_.empty()) {
    return true;
  }
  for (const auto& changed : changed_keys) {
    if (std::find(keys_.begin(), keys_.end(), changed) != keys_.end()) {
      return true;
    }
  }
  return false;
}

}  // namespace config
}  // namespace cleanbot
