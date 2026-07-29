#pragma once

#include <cstddef>

#include "rclcpp/qos.hpp"

namespace cleanbot {
namespace common {

inline rclcpp::QoS latest_command_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

inline rclcpp::QoS latched_status_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

inline rclcpp::QoS status_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
}

inline rclcpp::QoS hardware_status_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
}

inline rclcpp::QoS command_status_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
}

inline rclcpp::QoS rtk_fix_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
}

inline rclcpp::QoS debug_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

}  // namespace common
}  // namespace cleanbot
