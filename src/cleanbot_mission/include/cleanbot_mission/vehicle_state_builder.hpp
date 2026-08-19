#pragma once

#include <cstdint>
#include <string>

// 文件作用：声明将任务、硬件、RTK 和最终控制状态汇总为车辆对外状态的规则。
namespace cleanbot {
namespace mission {

// 构建车辆状态快照所需的原始运行时输入。
struct VehicleStateInput {
  // 各类控制来源的约定优先级，用于解释最终控制权归属。
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

// 面向状态话题和 HTTP 接口的车辆状态快照。
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

// 基于原始输入推导控制、健康、故障和作业状态。
VehicleStateSnapshot build_vehicle_state(const VehicleStateInput& input);

}  // namespace mission
}  // namespace cleanbot
