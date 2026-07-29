#include "cleanbot_gateway/cloud_command.hpp"

#include <cmath>

namespace cleanbot {
namespace gateway {
namespace {

CloudCommandDecision reject(
    const std::string& code,
    const std::string& message) {
  CloudCommandDecision decision;
  decision.code = code;
  decision.message = message;
  return decision;
}

bool finite_in_range(const double value, const double minimum, const double maximum) {
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

}  // namespace

CloudCommandTranslator::CloudCommandTranslator(
    const CloudCommandParameters& parameters)
    : parameters_(parameters), joystick_mapper_(parameters.joystick) {}

CloudCommandDecision CloudCommandTranslator::translate(
    const CloudCommandInput& input,
    const std::int64_t now_sec) const {
  if (input.command == "parking" || input.command == "stop") {
    CloudCommandDecision decision;
    decision.accepted = true;
    decision.code = "OK";
    decision.message = "software emergency stop accepted";
    decision.kind = CloudCommandKind::kEmergencyStop;
    decision.brake = true;
    return decision;
  }

  if (input.command != "joystick_move" && input.command != "joystickMove") {
    return reject("COMMAND_UNSUPPORTED", "unsupported cloud control command");
  }
  if (input.retained) {
    return reject(
        "JOYSTICK_RETAINED_REJECTED",
        "retained joystick commands are not allowed");
  }
  if (input.timestamp_sec <= 0) {
    return reject(
        "JOYSTICK_TIMESTAMP_REQUIRED",
        "joystick command timestamp is required");
  }
  const double age_sec = static_cast<double>(now_sec - input.timestamp_sec);
  if (age_sec < -5.0 || age_sec > parameters_.command_max_age_sec) {
    return reject(
        "JOYSTICK_COMMAND_EXPIRED",
        "joystick command is stale or has an invalid timestamp");
  }
  if (!input.has_joystick_params ||
      !finite_in_range(input.distance, 0.0, 100.0) ||
      !finite_in_range(input.dir_x, -1.0, 1.0) ||
      !finite_in_range(input.dir_y, -1.0, 1.0)) {
    return reject(
        "JOYSTICK_PARAMS_INVALID",
        "distance must be 0..100 and dirX/dirY must be -1..1");
  }

  const auto mapped = input.distance <= 0.0
      ? cleanbot::control::JoystickOutput()
      : joystick_mapper_.map(input.dir_x, input.dir_y);
  CloudCommandDecision decision;
  decision.accepted = true;
  decision.code = "OK";
  decision.message = mapped.brake
      ? "joystick released; brake requested"
      : "joystick command accepted";
  decision.kind = CloudCommandKind::kManualJoystick;
  decision.x_speed = mapped.x_speed;
  decision.steering_offset = mapped.steering_offset;
  decision.brake = mapped.brake;
  return decision;
}

}  // namespace gateway
}  // namespace cleanbot
