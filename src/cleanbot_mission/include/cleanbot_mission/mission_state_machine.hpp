#ifndef CLEANBOT_MISSION__MISSION_STATE_MACHINE_HPP_
#define CLEANBOT_MISSION__MISSION_STATE_MACHINE_HPP_

#include <cstddef>
#include <cstdint>

// 文件作用：声明覆盖任务从准备、转向、跟踪到终止的状态机和事件约束。
namespace cleanbot {
namespace mission {

// 任务执行生命周期中的所有状态。
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
  // 以总段数和可选起始段启动任务。
  bool start(std::size_t total_segments, std::size_t start_segment = 0u);
  // 在当前段登记一次等待完成的转向请求。
  bool begin_turn(std::uint64_t request_id);
  // 使用匹配请求号完成转向。
  bool complete_turn(std::uint64_t request_id);
  // 在当前段登记一次等待完成的直线跟踪。
  bool begin_tracking(std::uint64_t generation);
  // 使用匹配代次完成直线跟踪并推进段号。
  bool complete_tracking(std::uint64_t generation);
  // 暂停可暂停的任务状态。
  bool pause();
  // 从暂停状态恢复任务。
  bool resume();
  // 因 RTK 不可用切换到恢复等待状态。
  bool begin_rtk_recovery();
  // RTK 恢复后返回可执行状态。
  bool recover_rtk();
  // 取消未结束任务。
  bool cancel();
  // 将任务置为失败终止状态。
  bool fail();

  // 返回当前任务状态。
  MissionState state() const;
  // 返回当前正在执行的段序号。
  std::size_t current_segment() const;
  // 返回任务总段数。
  std::size_t total_segments() const;
  // 返回当前等待的转向请求号。
  std::uint64_t expected_turn_request_id() const;
  // 返回当前等待的跟踪代次。
  std::uint64_t expected_tracking_generation() const;
  // 返回任务是否已进入完成、取消或失败的终止状态。
  bool terminal() const;

 private:
  // 清除等待中的异步转向和跟踪事件标识。
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
