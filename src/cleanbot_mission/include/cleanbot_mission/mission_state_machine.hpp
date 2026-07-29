#ifndef CLEANBOT_MISSION__MISSION_STATE_MACHINE_HPP_
#define CLEANBOT_MISSION__MISSION_STATE_MACHINE_HPP_

#include <cstddef>
#include <cstdint>

namespace cleanbot {
namespace mission {

enum class MissionState {
  kIdle = 0,
  kPreparing = 1,
  kTurning = 2,
  kTracking = 3,
  kPaused = 4,
  kRtkRecovering = 5,
  kCompleted = 6,
  kCancelled = 7,
  kFailed = 8,
};

class MissionStateMachine {
 public:
  bool start(std::size_t total_segments, std::size_t start_segment = 0u);
  bool begin_turn(std::uint64_t request_id);
  bool complete_turn(std::uint64_t request_id);
  bool begin_tracking(std::uint64_t generation);
  bool complete_tracking(std::uint64_t generation);
  bool pause();
  bool resume();
  bool begin_rtk_recovery();
  bool recover_rtk();
  bool cancel();
  bool fail();

  MissionState state() const;
  std::size_t current_segment() const;
  std::size_t total_segments() const;
  std::uint64_t expected_turn_request_id() const;
  std::uint64_t expected_tracking_generation() const;
  bool terminal() const;

 private:
  void clearExpectedEvents();

  MissionState state_{MissionState::kIdle};
  std::size_t current_segment_{0u};
  std::size_t total_segments_{0u};
  std::uint64_t expected_turn_request_id_{0u};
  std::uint64_t expected_tracking_generation_{0u};
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__MISSION_STATE_MACHINE_HPP_
