#pragma once

#include <cstdint>
#include <string>

#include "cleanbot_control/joystick_mapper.hpp"

// 文件作用：声明云端控制消息的输入模型、合法性判定和车辆控制转换逻辑。
namespace cleanbot {
namespace gateway {

// 云端命令可被转换成的控制类别。
enum class CloudCommandKind {
  kRejected,
  kManualJoystick,
  kEmergencyStop,
};

// 从云端 JSON 消息解析出的原始命令字段。
struct CloudCommandInput {
  std::string command_id;
  std::string trace_id;
  std::string command;
  std::int64_t timestamp_sec{0};
  double distance{0.0};
  double dir_x{0.0};
  double dir_y{0.0};
  bool has_joystick_params{false};
  bool retained{false};
};

// 云端命令的时效性限制和摇杆转换参数。
struct CloudCommandParameters {
  double command_max_age_sec{2.0};
  cleanbot::control::JoystickParameters joystick;
};

// 云端命令翻译后的接受状态、错误信息和车辆输出。
struct CloudCommandDecision {
  bool accepted{false};
  std::string code;
  std::string message;
  CloudCommandKind kind{CloudCommandKind::kRejected};
  std::int32_t x_speed{0};
  std::int32_t steering_offset{0};
  bool brake{true};
};

class CloudCommandTranslator {
 public:
  // 使用给定时效和摇杆参数创建命令翻译器。
  explicit CloudCommandTranslator(
      const CloudCommandParameters& parameters = CloudCommandParameters());

  // 校验并将云端命令转换为手动控制或急停动作。
  CloudCommandDecision translate(
      const CloudCommandInput& input,
      std::int64_t now_sec) const;

 private:
  CloudCommandParameters parameters_;
  cleanbot::control::JoystickMapper joystick_mapper_;
};

}  // namespace gateway
}  // namespace cleanbot
