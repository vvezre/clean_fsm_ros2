#include "cleanbot_http/http_control_router.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

namespace cleanbot {
namespace http {
namespace {

constexpr const char* kJoystickPrefix = "/vehicle/joystickMove/";
constexpr const char* kJoystickPath = "/vehicle/joystickMove";
constexpr const char* kParkingPath = "/vehicle/parking";
constexpr const char* kVehicleStatePath = "/api/v1/vehicle/state";
constexpr const char* kMissionPausePath = "/api/v1/mission/pause";
constexpr const char* kMissionResumePath = "/api/v1/mission/resume";
constexpr const char* kExecuteModelPlanPath =
    "/api/v1/modeling/execute-plan";

std::vector<std::string> split(
    const std::string& value,
    char delimiter);

std::size_t find_character(const std::string& value, const char target) {
  for (std::size_t index = 0u; index < value.size(); ++index) {
    if (value[index] == target) {
      return index;
    }
  }
  return std::string::npos;
}

HttpControlResult text_success(const HttpControlAction action) {
  HttpControlResult result;
  result.status_code = 200;
  result.body = "1";
  result.content_type = "text/plain; charset=utf-8";
  result.action = action;
  return result;
}

HttpControlResult empty_success() {
  HttpControlResult result;
  result.status_code = 204;
  result.body.clear();
  result.content_type = "text/plain; charset=utf-8";
  return result;
}

HttpControlResult business_action(const HttpControlAction action) {
  HttpControlResult result;
  result.status_code = 200;
  result.body.clear();
  result.action = action;
  return result;
}

HttpControlResult error_result(
    const int status_code,
    const std::string& code,
    const std::string& message) {
  HttpControlResult result;
  result.status_code = status_code;
  result.body =
      "{\"success\":false,\"code\":\"" + code +
      "\",\"message\":\"" + message + "\",\"data\":{}}";
  return result;
}

std::string request_path(const std::string& target) {
  for (std::size_t index = 0; index < target.size(); ++index) {
    if (target[index] == '?') {
      return target.substr(0, index);
    }
  }
  return target;
}

struct JoystickSequence {
  bool legacy{false};
  bool valid{false};
  std::string session_id;
  std::uint64_t sequence{0u};
};

bool parse_uint64(const std::string& value, std::uint64_t& parsed) {
  if (value.empty()) {
    return false;
  }
  for (const unsigned char character : value) {
    if (character < '0' || character > '9') {
      return false;
    }
  }
  errno = 0;
  char* end = nullptr;
  const auto converted = std::strtoull(value.c_str(), &end, 10);
  if (errno == ERANGE || end == value.c_str() || *end != '\0') {
    return false;
  }
  parsed = static_cast<std::uint64_t>(converted);
  return true;
}

bool ascii_alphanumeric(const unsigned char character) {
  return (character >= '0' && character <= '9') ||
      (character >= 'A' && character <= 'Z') ||
      (character >= 'a' && character <= 'z');
}

bool valid_session_id(const std::string& value) {
  if (value.empty() || value.size() > 128u) {
    return false;
  }
  for (const unsigned char character : value) {
    if (!ascii_alphanumeric(character) &&
        character != '-' && character != '_' && character != '.') {
      return false;
    }
  }
  return true;
}

bool valid_plan_id(const std::string& value) {
  if (value.empty() || value.size() > 128u) {
    return false;
  }
  for (const unsigned char character : value) {
    if (!ascii_alphanumeric(character) &&
        character != '-' && character != '_' && character != '.') {
      return false;
    }
  }
  return true;
}

bool parse_execute_plan_query(
    const std::string& target,
    std::string& plan_id,
    std::int32_t& brush_speed) {
  const auto query_at = find_character(target, '?');
  if (query_at == std::string::npos || query_at + 1u >= target.size()) {
    return false;
  }

  bool has_plan_id = false;
  bool has_brush_speed = false;
  std::uint64_t parsed_brush_speed = 0u;
  const auto pairs = split(target.substr(query_at + 1u), '&');
  for (const auto& pair : pairs) {
    const auto equals_at = find_character(pair, '=');
    if (equals_at == std::string::npos || equals_at == 0u ||
        equals_at + 1u >= pair.size()) {
      return false;
    }
    const auto key = pair.substr(0u, equals_at);
    const auto value = pair.substr(equals_at + 1u);
    if (key == "planId") {
      if (has_plan_id || !valid_plan_id(value)) {
        return false;
      }
      has_plan_id = true;
      plan_id = value;
    } else if (key == "brushSpeed") {
      if (has_brush_speed ||
          !parse_uint64(value, parsed_brush_speed) ||
          parsed_brush_speed == 0u ||
          parsed_brush_speed >
              static_cast<std::uint64_t>(
                  std::numeric_limits<std::int32_t>::max())) {
        return false;
      }
      has_brush_speed = true;
    } else {
      return false;
    }
  }
  if (!has_plan_id || !has_brush_speed) {
    return false;
  }
  brush_speed = static_cast<std::int32_t>(parsed_brush_speed);
  return true;
}

JoystickSequence parse_joystick_sequence(const std::string& target) {
  JoystickSequence result;
  const auto query_at = find_character(target, '?');
  if (query_at == std::string::npos || query_at + 1u >= target.size()) {
    result.legacy = true;
    result.valid = true;
    return result;
  }

  bool has_session = false;
  bool has_sequence = false;
  std::string session_value;
  std::string sequence_value;
  const auto pairs = split(target.substr(query_at + 1u), '&');
  for (const auto& pair : pairs) {
    const auto equals_at = find_character(pair, '=');
    const std::string key = equals_at == std::string::npos
        ? pair
        : pair.substr(0u, equals_at);
    if (key != "sessionId" && key != "sequence") {
      continue;
    }
    if (equals_at == std::string::npos) {
      return result;
    }
    if (key == "sessionId") {
      if (has_session) {
        return result;
      }
      has_session = true;
      session_value = pair.substr(equals_at + 1u);
    } else {
      if (has_sequence) {
        return result;
      }
      has_sequence = true;
      sequence_value = pair.substr(equals_at + 1u);
    }
  }

  if (!has_session && !has_sequence) {
    result.legacy = true;
    result.valid = true;
    return result;
  }
  if (!has_session || !has_sequence ||
      !valid_session_id(session_value) ||
      !parse_uint64(sequence_value, result.sequence)) {
    return result;
  }
  result.session_id = session_value;
  result.valid = true;
  return result;
}

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.size() >= prefix.size() &&
      value.compare(0, prefix.size(), prefix) == 0;
}

std::vector<std::string> split(const std::string& value, const char delimiter) {
  std::vector<std::string> parts;
  std::size_t begin = 0;
  for (std::size_t index = 0; index < value.size(); ++index) {
    if (value[index] == delimiter) {
      parts.push_back(value.substr(begin, index - begin));
      begin = index + 1;
    }
  }
  parts.push_back(value.substr(begin));
  return parts;
}

bool parse_finite_double(const std::string& value, double& parsed) {
  if (value.empty()) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  parsed = std::strtod(value.c_str(), &end);
  return errno != ERANGE && end != value.c_str() && *end == '\0' &&
      std::isfinite(parsed);
}

bool in_range(const double value, const double minimum, const double maximum) {
  return value >= minimum && value <= maximum;
}

}  // namespace

HttpControlRouter::HttpControlRouter(
    const cleanbot::control::JoystickParameters& parameters)
    : joystick_mapper_(parameters) {}

void HttpControlRouter::updateParameters(
    const cleanbot::control::JoystickParameters& parameters) {
  joystick_mapper_ = cleanbot::control::JoystickMapper(parameters);
}

HttpControlResult HttpControlRouter::route(
    const std::string& method,
    const std::string& target) const {
  if (method == "OPTIONS") {
    return empty_success();
  }

  const auto path = request_path(target);
  const bool is_parking = path == kParkingPath;
  const bool is_joystick =
      path == kJoystickPath || starts_with(path, kJoystickPrefix);
  const bool is_vehicle_state = path == kVehicleStatePath;
  const bool is_mission_pause = path == kMissionPausePath;
  const bool is_mission_resume = path == kMissionResumePath;
  const bool is_execute_model_plan = path == kExecuteModelPlanPath;
  if (!is_parking && !is_joystick && !is_vehicle_state &&
      !is_mission_pause && !is_mission_resume &&
      !is_execute_model_plan) {
    return error_result(404, "ROUTE_NOT_FOUND", "route not found");
  }

  if (is_vehicle_state) {
    if (method != "GET") {
      return error_result(405, "METHOD_NOT_ALLOWED", "method not allowed");
    }
    if (target != path) {
      return error_result(
          400, "QUERY_NOT_ALLOWED", "query parameters are not allowed");
    }
    return business_action(HttpControlAction::kVehicleState);
  }

  if (is_mission_pause || is_mission_resume) {
    if (method != "POST") {
      return error_result(405, "METHOD_NOT_ALLOWED", "method not allowed");
    }
    if (target != path) {
      return error_result(
          400, "QUERY_NOT_ALLOWED", "query parameters are not allowed");
    }
    return business_action(
        is_mission_pause
            ? HttpControlAction::kMissionPause
            : HttpControlAction::kMissionResume);
  }

  if (is_execute_model_plan) {
    if (method != "POST") {
      return error_result(405, "METHOD_NOT_ALLOWED", "method not allowed");
    }
    auto result = business_action(HttpControlAction::kExecuteModelPlan);
    if (!parse_execute_plan_query(
            target, result.plan_id, result.brush_speed)) {
      return error_result(
          400,
          "EXECUTE_PLAN_QUERY_INVALID",
          "planId and positive brushSpeed are required");
    }
    return result;
  }

  if (method != "GET") {
    return error_result(405, "METHOD_NOT_ALLOWED", "method not allowed");
  }

  if (is_parking) {
    auto result = text_success(HttpControlAction::kEmergencyStop);
    result.brake = true;
    return result;
  }

  const auto parameter_text = starts_with(path, kJoystickPrefix)
      ? path.substr(std::string(kJoystickPrefix).size())
      : std::string();
  const auto values = split(parameter_text, '/');
  double distance = 0.0;
  double dir_x = 0.0;
  double dir_y = 0.0;
  if (values.size() != 3u ||
      !parse_finite_double(values[0], distance) ||
      !parse_finite_double(values[1], dir_x) ||
      !parse_finite_double(values[2], dir_y) ||
      !in_range(distance, 0.0, 100.0) ||
      !in_range(dir_x, -1.0, 1.0) ||
      !in_range(dir_y, -1.0, 1.0)) {
    return error_result(
        400,
        "JOYSTICK_PARAMS_INVALID",
        "distance must be 0..100 and dirX/dirY must be -1..1");
  }

  const auto request_sequence = parse_joystick_sequence(target);
  if (!request_sequence.valid) {
    return error_result(
        400,
        "JOYSTICK_SEQUENCE_INVALID",
        "sessionId and sequence must be provided together");
  }
  if (!request_sequence.legacy &&
      !sequence_guard_.accept(
          request_sequence.session_id, request_sequence.sequence)) {
    return error_result(
        409,
        "STALE_JOYSTICK_COMMAND",
        "stale joystick command");
  }

  const auto mapped = distance <= 0.0
      ? cleanbot::control::JoystickOutput()
      : joystick_mapper_.map(dir_x, dir_y);
  auto result = text_success(HttpControlAction::kManual);
  result.x_speed = mapped.x_speed;
  result.steering_offset = mapped.steering_offset;
  result.brake = mapped.brake;
  return result;
}

}  // namespace http
}  // namespace cleanbot
