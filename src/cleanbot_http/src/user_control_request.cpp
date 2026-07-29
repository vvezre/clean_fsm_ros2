#include "cleanbot_http/user_control_request.hpp"

#include <cstdint>

namespace cleanbot {
namespace http {
namespace {

UserControlDecision rejection(
    const std::string& code,
    const std::string& message) {
  UserControlDecision result;
  result.code = code;
  result.message = message;
  return result;
}

bool withinLimit(const std::int32_t value, const std::int32_t limit) {
  if (limit < 0) {
    return false;
  }
  const auto wide_value = static_cast<std::int64_t>(value);
  const auto wide_limit = static_cast<std::int64_t>(limit);
  return wide_value >= -wide_limit && wide_value <= wide_limit;
}

}  // namespace

UserControlDecision build_manual_command(
    const std::string& source,
    const std::int32_t x_speed,
    const std::int32_t z_speed,
    const bool brake,
    const std::int32_t maximum_x_speed,
    const std::int32_t maximum_z_speed) {
  if (source.empty()) {
    return rejection(
        "MANUAL_SOURCE_REQUIRED", "manual control source is required");
  }
  if (!withinLimit(x_speed, maximum_x_speed)) {
    return rejection(
        "MANUAL_X_SPEED_OUT_OF_RANGE",
        "manual x speed exceeds the configured limit");
  }
  if (!withinLimit(z_speed, maximum_z_speed)) {
    return rejection(
        "MANUAL_Z_SPEED_OUT_OF_RANGE",
        "manual z speed exceeds the configured limit");
  }

  UserControlDecision result;
  result.accepted = true;
  result.code = "MANUAL_COMMAND_ACCEPTED";
  result.message = "manual command accepted into the control pipeline";
  result.command.source = source;
  result.command.x_speed = brake ? 0 : x_speed;
  result.command.z_speed = brake ? 0 : z_speed;
  result.command.brake = brake;
  return result;
}

UserControlDecision build_emergency_command(
    const std::string& source,
    const std::string& reason) {
  if (source.empty()) {
    return rejection(
        "EMERGENCY_SOURCE_REQUIRED", "emergency stop source is required");
  }
  if (reason.empty()) {
    return rejection(
        "EMERGENCY_REASON_REQUIRED", "emergency stop reason is required");
  }

  UserControlDecision result;
  result.accepted = true;
  result.code = "EMERGENCY_STOP_ACCEPTED";
  result.message = "software emergency stop accepted into the control pipeline";
  result.command.source = source;
  result.command.x_speed = 0;
  result.command.z_speed = 0;
  result.command.brake = true;
  return result;
}

}  // namespace http
}  // namespace cleanbot
