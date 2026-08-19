/*
 * 文件作用：车辆状态构建实现：把硬件、定位和任务信息汇总成状态快照。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_mission/vehicle_state_builder.hpp"

#include <cmath>

namespace cleanbot {
namespace mission {
namespace {

// 判断正在执行的任务是否存在有效故障码。
bool hasActiveFault(const VehicleStateInput& input) {
  return input.mission_active && !input.fault_code.empty();
}

// 根据配置状态和最终命令优先级推导当前控制权来源。
std::string controlState(const VehicleStateInput& input) {
  if (!input.configured) {
    return "not_ready";
  }
  if (!input.has_final_command || !input.final_command_active) {
    return "idle";
  }

  switch (input.final_command_priority) {
    case VehicleStateInput::PRIORITY_EMERGENCY:
      return "emergency";
    case VehicleStateInput::PRIORITY_SAFETY:
      return "safety";
    case VehicleStateInput::PRIORITY_MANUAL:
      return "manual";
    case VehicleStateInput::PRIORITY_MISSION:
      return "mission";
    case VehicleStateInput::PRIORITY_VISION:
      return "vision";
    default:
      return "idle";
  }
}

// 综合配置、故障、硬件和 RTK 状态推导车辆健康等级。
std::string healthState(const VehicleStateInput& input) {
  if (!input.configured) {
    return "not_ready";
  }
  if (hasActiveFault(input)) {
    return "fault";
  }
  if (!input.hardware_ready || !input.rtk_ready) {
    return "degraded";
  }
  return "ready";
}

// 按故障、暂停、生命周期信息的优先级选择对外状态说明。
std::string stateMessage(const VehicleStateInput& input) {
  if (hasActiveFault(input) && !input.fault_message.empty()) {
    return input.fault_message;
  }
  if (!input.pause_reason.empty()) {
    return input.pause_reason;
  }
  if (!input.lifecycle_message.empty()) {
    return input.lifecycle_message;
  }
  return input.configured ? "vehicle state available" : "configuration not ready";
}

}  // namespace

// 汇总任务与车辆输入，生成供话题和 HTTP 使用的状态快照。
VehicleStateSnapshot build_vehicle_state(const VehicleStateInput& input) {
  VehicleStateSnapshot state;
  state.control_state = controlState(input);
  state.health_state = healthState(input);
  state.fault_state = hasActiveFault(input) ? input.fault_code : "NONE";
  state.current_action =
      input.current_action.empty() ? "idle" : input.current_action;
  state.message = stateMessage(input);
  state.start_ready =
      input.configured && input.hardware_ready && input.rtk_ready &&
      !hasActiveFault(input) && !input.mission_active && !input.goal_reserved;
  state.parking =
      !input.has_final_command || !input.final_command_active ||
      input.final_command_brake;
  state.cleaning =
      input.mission_active && state.current_action == "cleaning";
  state.rtk_fixed = input.rtk_ready;
  state.in_garage = false;
  state.battery_percent =
      std::isfinite(input.battery_percent) ? input.battery_percent : -1.0;
  state.current_segment = input.current_segment;
  state.total_segments = input.total_segments;
  return state;
}

}  // namespace mission
}  // namespace cleanbot
