/*
 * 文件作用：摇杆映射实现：把输入轴值转换为车辆速度和控制命令。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_control/joystick_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace cleanbot {
namespace control {

// 保存摇杆死区、速度和转向输出限制参数。
JoystickMapper::JoystickMapper(const JoystickParameters& parameters)
    : parameters_(parameters) {}

// 对摇杆输入限幅、去死区后映射为线速度、转向偏移和制动状态。
JoystickOutput JoystickMapper::map(double dir_x, double dir_y) const {
  dir_x = std::max(-1.0, std::min(1.0, dir_x));
  dir_y = std::max(-1.0, std::min(1.0, dir_y));
  const double magnitude = std::min(1.0, std::hypot(dir_x, dir_y));
  JoystickOutput result;
  const double dead_zone = std::max(0.0, std::min(0.99, parameters_.dead_zone));
  if (magnitude <= dead_zone) {
    return result;
  }

  const double effective = (magnitude - dead_zone) / (1.0 - dead_zone);
  const double unit_x = dir_x / magnitude;
  const double unit_y = dir_y / magnitude;
  const double direction_sign = unit_y < -std::abs(parameters_.reverse_threshold) ? -1.0 : 1.0;
  result.x_speed = static_cast<std::int32_t>(std::llround(
      effective * unit_y * std::abs(parameters_.max_linear_speed)));
  result.steering_offset = static_cast<std::int32_t>(std::llround(
      -effective * unit_x * direction_sign *
      std::abs(parameters_.max_steering_offset)));
  result.brake = false;
  return result;
}

}  // namespace control
}  // namespace cleanbot
