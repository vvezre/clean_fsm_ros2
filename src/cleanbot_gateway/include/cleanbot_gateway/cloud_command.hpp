#pragma once

#include <cstdint>
#include <string>

#include "cleanbot_control/joystick_mapper.hpp"

namespace cleanbot {
namespace gateway {

enum class CloudCommandKind {
  kRejected,
  kManualJoystick,
  kEmergencyStop,
};

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

struct CloudCommandParameters {
  double command_max_age_sec{2.0};
  cleanbot::control::JoystickParameters joystick;
};

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
  explicit CloudCommandTranslator(
      const CloudCommandParameters& parameters = CloudCommandParameters());

  CloudCommandDecision translate(
      const CloudCommandInput& input,
      std::int64_t now_sec) const;

 private:
  CloudCommandParameters parameters_;
  cleanbot::control::JoystickMapper joystick_mapper_;
};

}  // namespace gateway
}  // namespace cleanbot
