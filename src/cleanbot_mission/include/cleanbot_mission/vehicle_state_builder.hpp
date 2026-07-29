#pragma once

#include <cstdint>
#include <string>

namespace cleanbot {
namespace mission {

struct VehicleStateInput {
  static constexpr std::uint8_t PRIORITY_VISION = 20u;
  static constexpr std::uint8_t PRIORITY_MISSION = 40u;
  static constexpr std::uint8_t PRIORITY_MANUAL = 60u;
  static constexpr std::uint8_t PRIORITY_SAFETY = 80u;
  static constexpr std::uint8_t PRIORITY_EMERGENCY = 100u;

  bool configured{false};
  bool hardware_ready{false};
  bool rtk_ready{false};
  bool mission_active{false};
  bool goal_reserved{false};
  bool has_final_command{false};
  bool final_command_active{false};
  bool final_command_brake{true};
  std::uint8_t final_command_priority{0u};
  std::string current_action{"idle"};
  std::string lifecycle_message;
  std::string pause_reason;
  std::string fault_code;
  std::string fault_message;
  double battery_percent{-1.0};
  std::uint32_t current_segment{0u};
  std::uint32_t total_segments{0u};
};

struct VehicleStateSnapshot {
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

VehicleStateSnapshot build_vehicle_state(const VehicleStateInput& input);

}  // namespace mission
}  // namespace cleanbot
