#ifndef CLEANBOT_CONTROL__JOYSTICK_MAPPER_HPP_
#define CLEANBOT_CONTROL__JOYSTICK_MAPPER_HPP_

#include <cstdint>

namespace cleanbot {
namespace control {

struct JoystickParameters {
  double dead_zone{0.12};
  double reverse_threshold{0.20};
  std::int32_t max_linear_speed{350};
  std::int32_t max_steering_offset{1000};
};

struct JoystickOutput {
  std::int32_t x_speed{0};
  std::int32_t steering_offset{0};
  bool brake{true};
};

class JoystickMapper {
 public:
  explicit JoystickMapper(const JoystickParameters& parameters = JoystickParameters());
  JoystickOutput map(double dir_x, double dir_y) const;

 private:
  JoystickParameters parameters_;
};

}  // namespace control
}  // namespace cleanbot

#endif  // CLEANBOT_CONTROL__JOYSTICK_MAPPER_HPP_
