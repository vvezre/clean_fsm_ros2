#pragma once

#include <cstdint>
#include <string>

namespace cleanbot {
namespace http {

struct VehicleStateJsonData {
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0u};
  std::string control_state;
  std::string health_state;
  std::string fault_state;
  std::string current_action;
  std::string message;
  bool start_ready{false};
  bool parking{true};
  bool cleaning{false};
  bool rtk_fixed{false};
  bool in_garage{false};
  double battery_percent{-1.0};
  std::uint32_t current_segment{0u};
  std::uint32_t total_segments{0u};
};

std::string vehicle_state_json(const VehicleStateJsonData& state);

std::string business_response_json(
    bool success,
    const std::string& code,
    const std::string& message,
    const std::string& data_json = "{}");

}  // namespace http
}  // namespace cleanbot
