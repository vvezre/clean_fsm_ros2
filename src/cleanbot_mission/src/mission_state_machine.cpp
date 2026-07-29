#include "cleanbot_mission/mission_state_machine.hpp"

namespace cleanbot {
namespace mission {

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

bool MissionStateMachine::begin_turn(const std::uint64_t request_id) {
  if (state_ != MissionState::kPreparing || request_id == 0u) {
    return false;
  }
  expected_turn_request_id_ = request_id;
  expected_tracking_generation_ = 0u;
  state_ = MissionState::kTurning;
  return true;
}

bool MissionStateMachine::complete_turn(const std::uint64_t request_id) {
  if (state_ != MissionState::kTurning ||
      request_id != expected_turn_request_id_) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kPreparing;
  return true;
}

bool MissionStateMachine::begin_tracking(const std::uint64_t generation) {
  if (state_ != MissionState::kPreparing || generation == 0u) {
    return false;
  }
  expected_turn_request_id_ = 0u;
  expected_tracking_generation_ = generation;
  state_ = MissionState::kTracking;
  return true;
}

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

bool MissionStateMachine::pause() {
  if (state_ == MissionState::kIdle || state_ == MissionState::kPaused || terminal()) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kPaused;
  return true;
}

bool MissionStateMachine::resume() {
  if (state_ != MissionState::kPaused) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kPreparing;
  return true;
}

bool MissionStateMachine::begin_rtk_recovery() {
  if (state_ == MissionState::kIdle || state_ == MissionState::kPaused || terminal()) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kRtkRecovering;
  return true;
}

bool MissionStateMachine::recover_rtk() {
  if (state_ != MissionState::kRtkRecovering) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kPreparing;
  return true;
}

bool MissionStateMachine::cancel() {
  if (state_ == MissionState::kIdle || terminal()) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kCancelled;
  return true;
}

bool MissionStateMachine::fail() {
  if (state_ == MissionState::kIdle || terminal()) {
    return false;
  }
  clearExpectedEvents();
  state_ = MissionState::kFailed;
  return true;
}

MissionState MissionStateMachine::state() const { return state_; }

std::size_t MissionStateMachine::current_segment() const {
  return current_segment_;
}

std::size_t MissionStateMachine::total_segments() const {
  return total_segments_;
}

std::uint64_t MissionStateMachine::expected_turn_request_id() const {
  return expected_turn_request_id_;
}

std::uint64_t MissionStateMachine::expected_tracking_generation() const {
  return expected_tracking_generation_;
}

bool MissionStateMachine::terminal() const {
  return state_ == MissionState::kCompleted ||
      state_ == MissionState::kCancelled ||
      state_ == MissionState::kFailed;
}

void MissionStateMachine::clearExpectedEvents() {
  expected_turn_request_id_ = 0u;
  expected_tracking_generation_ = 0u;
}

}  // namespace mission
}  // namespace cleanbot
