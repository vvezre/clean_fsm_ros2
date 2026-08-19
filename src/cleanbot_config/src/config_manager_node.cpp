/*
 * 文件作用：配置管理节点实现：提供参数校验、持久化和配置快照服务。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_registry.hpp"
#include "cleanbot_config/sqlite_config_repository.hpp"
#include "cleanbot_interfaces/msg/config_changed.hpp"
#include "cleanbot_interfaces/msg/config_entry.hpp"
#include "cleanbot_interfaces/msg/config_status.hpp"
#include "cleanbot_interfaces/srv/get_config.hpp"
#include "cleanbot_interfaces/srv/set_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cleanbot {
namespace config {
namespace {

// 将内部配置值类型转换为配置服务返回的稳定文本名称。
std::string value_type_name(const ConfigValueType type) {
  switch (type) {
    case ConfigValueType::kInteger:
      return "integer";
    case ConfigValueType::kDouble:
      return "double";
    case ConfigValueType::kBoolean:
      return "boolean";
    case ConfigValueType::kString:
    default:
      return "string";
  }
}

}  // namespace

class ConfigManagerNode : public rclcpp::Node {
 public:
  // 构造配置管理节点，打开配置仓库并创建查询、更新服务和状态发布器。
  ConfigManagerNode() : Node("config_manager_node") {
    state_ = "STARTING";
    const auto database_path = declare_parameter<std::string>(
        "database_path", "/var/lib/cleanbot/cleanbot.db");
    profile_ = declare_parameter<std::string>("profile", "default");
    const auto minimum_free_space_mb = std::max<std::int64_t>(
        0, declare_parameter<std::int64_t>("minimum_free_space_mb", 64));

    status_publisher_ = create_publisher<cleanbot_interfaces::msg::ConfigStatus>(
        "/config/status", common::latched_status_qos());
    changed_publisher_ = create_publisher<cleanbot_interfaces::msg::ConfigChanged>(
        "/config/changed", common::latched_status_qos());
    get_service_ = create_service<cleanbot_interfaces::srv::GetConfig>(
        "/config/get",
        std::bind(
            &ConfigManagerNode::on_get, this,
            std::placeholders::_1, std::placeholders::_2));
    set_service_ = create_service<cleanbot_interfaces::srv::SetConfig>(
        "/config/set",
        std::bind(
            &ConfigManagerNode::on_set, this,
            std::placeholders::_1, std::placeholders::_2));

    state_ = "CHECKING";
    publish_status();
    repository_ = std::make_unique<SqliteConfigRepository>(
        database_path,
        static_cast<std::uint64_t>(minimum_free_space_mb) * 1024u * 1024u);
    repository_status_ = repository_->open_and_initialize(registry_);
    if (!repository_status_.healthy) {
      state_ = "ERROR";
      error_code_ = repository_status_.code;
      error_message_ = repository_status_.message;
    } else {
      values_ = repository_->read_all();
      evaluate_configuration();
    }
    publish_status();
    status_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ConfigManagerNode::publish_status, this));
    RCLCPP_INFO(
        get_logger(), "configuration manager state=%s profile=%s database=%s",
        state_.c_str(), profile_.c_str(), database_path.c_str());
  }

 private:
  // 汇总当前内存配置，检查必填项和取值合法性，并把节点状态切到 READY/ERROR。
  void evaluate_configuration() {
    missing_required_keys_ = registry_.missing_required(values_);
    for (const auto& item : registry_.definitions()) {
      if (item.second.has_default && values_.find(item.first) == values_.end()) {
        missing_required_keys_.push_back(item.first);
      }
    }
    std::sort(missing_required_keys_.begin(), missing_required_keys_.end());
    missing_required_keys_.erase(
        std::unique(missing_required_keys_.begin(), missing_required_keys_.end()),
        missing_required_keys_.end());

    for (const auto& value : values_) {
      const auto validation = registry_.validate_value(value.first, value.second);
      if (!validation.valid) {
        state_ = "ERROR";
        error_code_ = validation.code;
        error_message_ = validation.message;
        return;
      }
    }
    if (!missing_required_keys_.empty()) {
      state_ = "ERROR";
      error_code_ = "CONFIG_REQUIRED_MISSING";
      error_message_ = "required configuration is incomplete";
      return;
    }
    state_ = "READY";
    error_code_.clear();
    error_message_.clear();
  }

  // 读取配置的只读入口：支持按 key 查询，也支持一次性返回全部非敏感项。
  void on_get(
      const std::shared_ptr<cleanbot_interfaces::srv::GetConfig::Request> request,
      std::shared_ptr<cleanbot_interfaces::srv::GetConfig::Response> response) {
    std::vector<std::string> keys = request->keys;
    if (keys.empty()) {
      for (const auto& item : registry_.definitions()) {
        keys.push_back(item.first);
      }
    }
    for (const auto& key : keys) {
      const auto* definition = registry_.find(key);
      if (definition == nullptr) {
        response->success = false;
        response->code = "CONFIG_KEY_UNKNOWN";
        response->message = "unknown configuration key: " + key;
        return;
      }
      cleanbot_interfaces::msg::ConfigEntry entry;
      entry.key = key;
      entry.value_type = value_type_name(definition->value_type);
      entry.sensitive = definition->sensitive;
      const auto value = values_.find(key);
      entry.configured = value != values_.end();
      if (entry.configured && (!entry.sensitive || request->include_sensitive)) {
        entry.value_text = value->second;
      }
      response->entries.push_back(entry);
    }
    response->success = true;
    response->code = "OK";
    response->message = "configuration returned";
    response->revision = repository_status_.revision;
  }

  // 写入配置的服务入口：先做重复 key 和仓库健康检查，再原子写入并刷新缓存。
  void on_set(
      const std::shared_ptr<cleanbot_interfaces::srv::SetConfig::Request> request,
      std::shared_ptr<cleanbot_interfaces::srv::SetConfig::Response> response) {
    if (!repository_status_.healthy) {
      response->code = repository_status_.code;
      response->message = repository_status_.message;
      return;
    }
    std::map<std::string, std::string> updates;
    for (const auto& entry : request->entries) {
      if (!updates.emplace(entry.key, entry.value_text).second) {
        response->code = "CONFIG_KEY_DUPLICATED";
        response->message = "configuration key is duplicated: " + entry.key;
        return;
      }
    }
    const auto result = repository_->write_values(
        updates,
        request->updated_by.empty() ? "ros2-service" : request->updated_by,
        registry_);
    response->success = result.success;
    response->code = result.code;
    response->message = result.message;
    response->revision = result.revision;
    response->restart_required = result.restart_required;
    if (!result.success) {
      if (result.code.find("CONFIG_VALUE_") != 0u &&
          result.code != "CONFIG_KEY_UNKNOWN") {
        state_ = "DEGRADED";
        repository_status_.healthy = false;
        repository_status_.writable = false;
        error_code_ = result.code;
        error_message_ = result.message;
        publish_status();
      }
      return;
    }

    repository_status_.revision = result.revision;
    values_ = repository_->read_all();
    evaluate_configuration();
    cleanbot_interfaces::msg::ConfigChanged changed;
    changed.stamp = now();
    changed.revision = result.revision;
    changed.changed_keys = result.changed_keys;
    changed.restart_required = result.restart_required;
    changed_publisher_->publish(changed);
    publish_status();
  }

  // 周期性发布配置就绪状态，给下游节点和界面做启动门控与故障展示。
  void publish_status() {
    cleanbot_interfaces::msg::ConfigStatus status;
    status.stamp = now();
    status.config_ready = state_ == "READY";
    status.database_writable = repository_status_.writable;
    status.revision = repository_status_.revision;
    status.state = state_;
    status.error_code = error_code_;
    status.error_message = error_message_;
    status.missing_required_keys = missing_required_keys_;
    status_publisher_->publish(status);
  }

  ConfigRegistry registry_;
  std::unique_ptr<ConfigRepository> repository_;
  RepositoryStatus repository_status_;
  std::map<std::string, std::string> values_;
  std::vector<std::string> missing_required_keys_;
  std::string profile_;
  std::string state_{"STARTING"};
  std::string error_code_;
  std::string error_message_;
  rclcpp::Publisher<cleanbot_interfaces::msg::ConfigStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<cleanbot_interfaces::msg::ConfigChanged>::SharedPtr changed_publisher_;
  rclcpp::Service<cleanbot_interfaces::srv::GetConfig>::SharedPtr get_service_;
  rclcpp::Service<cleanbot_interfaces::srv::SetConfig>::SharedPtr set_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace config
}  // namespace cleanbot

// 初始化 ROS 2，运行配置管理节点，并在退出前完成清理。
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanbot::config::ConfigManagerNode>());
  rclcpp::shutdown();
  return 0;
}
