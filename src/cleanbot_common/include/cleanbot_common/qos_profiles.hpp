// 文件作用：集中定义所有Cleanbot ROS2话题使用的QoS配置。
// 作用：统一命令、状态、诊断和调试话题的可靠性、队列深度及持久化策略。
#pragma once

#include <cstddef>

#include "rclcpp/qos.hpp"

namespace cleanbot {
namespace common {

inline rclcpp::QoS latest_command_qos() {
  // 方法作用：只保留最新控制命令，避免旧命令排队后延迟执行。
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

inline rclcpp::QoS latched_status_qos() {
  // 方法作用：保存最后一条状态，让晚加入的订阅者立即获得当前快照。
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

inline rclcpp::QoS status_qos() {
  // 方法作用：为普通状态话题提供可靠传输和有限历史队列。
  return rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
}

inline rclcpp::QoS hardware_status_qos() {
  // 方法作用：为硬件状态话题提供可靠传输和短历史队列。
  return rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
}

inline rclcpp::QoS command_status_qos() {
  // 方法作用：为命令生命周期状态保留较长历史，便于任务侧关联。
  return rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
}

inline rclcpp::QoS rtk_fix_qos() {
  // 方法作用：为RTK定位结果提供可靠传输和有限历史队列。
  return rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
}

inline rclcpp::QoS debug_qos() {
  // 方法作用：为高频调试数据使用低开销的尽力传输策略。
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

}  // namespace common
}  // namespace cleanbot
