#include "cleanbot_control/joystick_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace cleanbot {
namespace control {

JoystickMapper::JoystickMapper(const JoystickParameters& parameters)
    : parameters_(parameters) {}

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
