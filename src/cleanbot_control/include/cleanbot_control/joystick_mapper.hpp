#ifndef CLEANBOT_CONTROL__JOYSTICK_MAPPER_HPP_
#define CLEANBOT_CONTROL__JOYSTICK_MAPPER_HPP_

#include <cstdint>

// 文件作用：声明遥控器摇杆输入到车辆线速度和转向偏移量的映射规则。
namespace cleanbot {
namespace control {

// 摇杆映射的死区、反向阈值和输出上限参数。
struct JoystickParameters {
  double dead_zone{0.12};
  double reverse_threshold{0.20};
  std::int32_t max_linear_speed{350};
  std::int32_t max_steering_offset{1000};
};

// 摇杆映射完成后的车辆控制输出。
struct JoystickOutput {
  std::int32_t x_speed{0};
  std::int32_t steering_offset{0};
  bool brake{true};
};

class JoystickMapper {
 public:
  // 使用给定映射参数创建摇杆转换器。
  explicit JoystickMapper(const JoystickParameters& parameters = JoystickParameters());
  // 将二维摇杆方向转换为线速度、转向偏移量和制动状态。
  JoystickOutput map(double dir_x, double dir_y) const;

 private:
  JoystickParameters parameters_;
};

}  // namespace control
}  // namespace cleanbot

#endif  // CLEANBOT_CONTROL__JOYSTICK_MAPPER_HPP_
