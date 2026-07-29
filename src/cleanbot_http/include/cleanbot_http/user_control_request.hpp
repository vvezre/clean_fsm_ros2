#pragma once

#include <cstdint>
#include <string>

namespace cleanbot {
namespace http {

struct UserControlCommand {
  std::string source;
  bool active{true};
  bool operator_intent{true};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  bool brake{true};
};

struct UserControlDecision {
  bool accepted{false};
  std::string code;
  std::string message;
  UserControlCommand command;
};

UserControlDecision build_manual_command(
    const std::string& source,
    std::int32_t x_speed,
    std::int32_t z_speed,
    bool brake,
    std::int32_t maximum_x_speed,
    std::int32_t maximum_z_speed);

UserControlDecision build_emergency_command(
    const std::string& source,
    const std::string& reason);

}  // namespace http
}  // namespace cleanbot
