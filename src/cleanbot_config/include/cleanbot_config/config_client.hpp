// 文件作用：声明配置快照和ROS2配置客户端，供业务节点统一读取运行参数。
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
  // 构造空配置快照，通常用于客户端尚未首次加载配置的阶段。
  ConfigSnapshot() = default;
  // 用键值表构造不可变快照，供调用方按类型读取配置。
  explicit ConfigSnapshot(std::map<std::string, std::string> values);

  // 判断指定配置键是否存在于当前快照。
  bool contains(const std::string& key) const;
  // 读取字符串配置；键不存在时抛出配置缺失异常。
  std::string get_string(const std::string& key) const;
  // 读取并校验整数配置。
  std::int64_t get_integer(const std::string& key) const;
  // 读取并校验浮点配置。
  double get_double(const std::string& key) const;
  // 读取并校验布尔配置。
  bool get_boolean(const std::string& key) const;
  // 返回完整原始键值表，供批量检查或诊断使用。
  const std::map<std::string, std::string>& values() const;

 private:
  // 查找必填键；不存在时生成统一错误信息。
  const std::string& required_value(const std::string& key) const;
  std::map<std::string, std::string> values_;
};

class ConfigClient {
 public:
  // 配置首次加载或相关配置变更后的通知回调。
  using ReadyCallback = std::function<void(const ConfigSnapshot&, bool initial)>;
  // 配置中心状态变更通知回调。
  using StatusCallback =
      std::function<void(const cleanbot_interfaces::msg::ConfigStatus&)>;

  // 创建配置客户端并订阅配置状态、变更通知及读取服务。
  ConfigClient(
      rclcpp::Node* node,
      std::vector<std::string> keys,
      bool include_sensitive,
      ReadyCallback ready_callback,
      StatusCallback status_callback = StatusCallback());

  // 返回当前是否已经取得一份可用配置快照。
  bool ready() const;
  // 复制并返回最近一次成功读取的配置快照。
  ConfigSnapshot snapshot() const;
  // 请求下一次定时器周期刷新配置，避免并发发起多个服务调用。
  void request_refresh();

 private:
  // 处理配置中心就绪状态和版本号变化。
  void on_status(const cleanbot_interfaces::msg::ConfigStatus::SharedPtr message);
  // 处理配置键变更通知，并筛选本客户端关心的键。
  void on_changed(const cleanbot_interfaces::msg::ConfigChanged::SharedPtr message);
  // 在服务可用且未请求中的条件下发起配置读取。
  void refresh_if_possible();
  // 判断变更键集合是否与本客户端订阅的键有交集。
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
  bool status_ready_seen_{false};
  std::uint64_t status_revision_{0u};
  mutable std::mutex mutex_;
  ConfigSnapshot snapshot_;
  rclcpp::Client<cleanbot_interfaces::srv::GetConfig>::SharedPtr client_;
  rclcpp::Subscription<cleanbot_interfaces::msg::ConfigStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<cleanbot_interfaces::msg::ConfigChanged>::SharedPtr changed_subscription_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
};

}  // namespace config
}  // namespace cleanbot
