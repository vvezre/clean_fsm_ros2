#pragma once

#include <cstdint>
#include <string>

#include "cleanbot_control/joystick_mapper.hpp"
#include "cleanbot_http/joystick_sequence_guard.hpp"

namespace cleanbot {
namespace http {

enum class HttpControlAction {
  kNone,
  kManual,
  kEmergencyStop,
  kVehicleState,
  kMissionPause,
  kMissionResume,
  kExecuteModelPlan,
};

struct HttpControlResult {
  int status_code{500};
  std::string body;
  std::string content_type{"application/json; charset=utf-8"};
  HttpControlAction action{HttpControlAction::kNone};
  std::int32_t x_speed{0};
  std::int32_t steering_offset{0};
  bool brake{true};
  std::string plan_id;
  std::int32_t brush_speed{0};
};

// 只负责HTTP方法/路径解析和控制参数转换，不依赖ROS2或网络套接字。
class HttpControlRouter {
 public:
  explicit HttpControlRouter(
      const cleanbot::control::JoystickParameters& parameters =
          cleanbot::control::JoystickParameters());

  void updateParameters(
      const cleanbot::control::JoystickParameters& parameters);

  HttpControlResult route(
      const std::string& method,
      const std::string& target) const;

 private:
  cleanbot::control::JoystickMapper joystick_mapper_;
  mutable JoystickSequenceGuard sequence_guard_;
};

}  // namespace http
}  // namespace cleanbot
