#pragma once

#include <cstdint>
#include <string>

#include "cleanbot_control/joystick_mapper.hpp"
#include "cleanbot_http/joystick_sequence_guard.hpp"

// 文件作用：声明 HTTP 路径到车辆控制动作的纯解析与路由接口。
namespace cleanbot {
namespace http {

// HTTP 控制请求被识别后的业务动作类别。
enum class HttpControlAction {
  kNone,
  kManual,
  kEmergencyStop,
  kVehicleState,
  kMissionPause,
  kMissionResume,
  kExecuteModelPlan,
};

// HTTP 路由结果，包括响应内容和需要交由 ROS 节点执行的动作。
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

  // 热更新摇杆映射参数，供运行期配置变更调用。
  void updateParameters(
      const cleanbot::control::JoystickParameters& parameters);

  // 解析 HTTP 方法与目标路径，生成业务响应和后续控制动作。
  HttpControlResult route(
      const std::string& method,
      const std::string& target) const;

 private:
  cleanbot::control::JoystickMapper joystick_mapper_;
  mutable JoystickSequenceGuard sequence_guard_;
};

}  // namespace http
}  // namespace cleanbot
