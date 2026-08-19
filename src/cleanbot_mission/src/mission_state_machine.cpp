/*
 * 文件作用：任务状态机实现：约束任务段开始、完成、暂停、失败和取消转换。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_mission/mission_state_machine.hpp"

namespace cleanbot {
namespace mission {

// 校验段数和起始位置后进入任务准备状态。
bool MissionStateMachine::start(
    const std::size_t total_segments,
    const std::size_t start_segment) {
  if (total_segments == 0u || start_segment >= total_segments ||
      (state_ != MissionState::kIdle && !terminal())) {
    return false;
  }
  total_segments_ = total_segments;
  current_segment_ = start_segment;
  clearExpectedEvents();
  state_ = MissionState::kPreparing;
  return true;
}

// 登记当前段的转向请求号并切换为转向状态。
bool MissionStateMachine::begin_turn(const std::uint64_t request_id) {
  if (state_ != MissionState::kPreparing || request_id == 0u) {
    return false;
  }
  expected_turn_request_id_ = request_id;
  expected_tracking_generation_ = 0u;
  state_ = MissionState::kTurning;
  return true;
}

// 仅匹配当前请求号时完成转向并回到准备状态。
bool MissionStateMachine::complete_turn(const std::uint64_t request_id) {
  if (state_ != MissionState::kTurning ||
      request_id != expected_turn_request_id_) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kPreparing;
  return true;
}

// 登记跟踪代次并从准备状态进入直线跟踪。
bool MissionStateMachine::begin_tracking(const std::uint64_t generation) {
  if (state_ != MissionState::kPreparing || generation == 0u) {
    return false;
  }
  expected_turn_request_id_ = 0u;
  expected_tracking_generation_ = generation;
  state_ = MissionState::kTracking;
  return true;
}

// 匹配跟踪代次后推进任务段，必要时结束任务。
bool MissionStateMachine::complete_tracking(const std::uint64_t generation) {
  if (state_ != MissionState::kTracking ||
      generation != expected_tracking_generation_) {
    return false;
  }
  clearExpectedEvents();
  ++current_segment_;
  state_ = current_segment_ >= total_segments_
      ? MissionState::kCompleted
      : MissionState::kPreparing;
  return true;
}

// 暂停当前非终止任务，并废弃等待中的异步事件。
bool MissionStateMachine::pause() {
  if (state_ == MissionState::kIdle || state_ == MissionState::kPaused || terminal()) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kPaused;
  return true;
}

// 从暂停状态回到准备状态，由上层重新下发当前段动作。
bool MissionStateMachine::resume() {
  if (state_ != MissionState::kPaused) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kPreparing;
  return true;
}

// RTK 异常时保存任务段并切换到恢复等待状态。
bool MissionStateMachine::begin_rtk_recovery() {
  if (state_ == MissionState::kIdle || state_ == MissionState::kPaused || terminal()) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kRtkRecovering;
  return true;
}

// RTK 恢复后回到准备状态以重新规划当前段。
bool MissionStateMachine::recover_rtk() {
  if (state_ != MissionState::kRtkRecovering) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kPreparing;
  return true;
}

// 将可取消任务置为取消终止状态。
bool MissionStateMachine::cancel() {
  if (state_ == MissionState::kIdle || terminal()) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kCancelled;
  return true;
}

// 将非终止任务置为失败终止状态。
bool MissionStateMachine::fail() {
  if (state_ == MissionState::kIdle || terminal()) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kFailed;
  return true;
}

// 返回当前状态机状态。
MissionState MissionStateMachine::state() const { return state_; }

// 返回当前正在准备或执行的任务段索引。
std::size_t MissionStateMachine::current_segment() const {
  return current_segment_;
}

// 返回任务的总段数。
std::size_t MissionStateMachine::total_segments() const {
  return total_segments_;
}

// 返回当前等待回执的转向请求号。
std::uint64_t MissionStateMachine::expected_turn_request_id() const {
  return expected_turn_request_id_;
}

// 返回当前等待完成的跟踪代次。
std::uint64_t MissionStateMachine::expected_tracking_generation() const {
  return expected_tracking_generation_;
}

// 判断任务是否已完成、取消或失败而不可继续转换。
bool MissionStateMachine::terminal() const {
  return state_ == MissionState::kCompleted ||
      state_ == MissionState::kCancelled ||
      state_ == MissionState::kFailed;
}

// 清除等待中的转向请求和跟踪代次。
void MissionStateMachine::clearExpectedEvents() {
  expected_turn_request_id_ = 0u;
  expected_tracking_generation_ = 0u;
}

}  // namespace mission
}  // namespace cleanbot
