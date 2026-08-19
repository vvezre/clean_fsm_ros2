/*
 * 文件作用：目标提交实现：把建模结果转换为任务目标并提交给任务执行器。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_modeling/goal_submission.hpp"

namespace cleanbot {
namespace modeling {

// 将 Action 目标接受、拒绝或超时状态转换为稳定业务结果。
GoalSubmissionDecision classify_goal_response(const GoalResponseState state) {
  GoalSubmissionDecision decision;
  switch (state) {
    case GoalResponseState::kAccepted:
      decision.accepted = true;
      decision.code = "MISSION_GOAL_ACCEPTED";
      decision.message = "cleaning mission goal accepted";
      return decision;
    case GoalResponseState::kRejected:
      decision.code = "MISSION_GOAL_REJECTED";
      decision.message = "cleaning mission goal rejected";
      return decision;
    case GoalResponseState::kTimeout:
      decision.code = "MISSION_GOAL_RESPONSE_TIMEOUT";
      decision.message =
          "cleaning mission goal response exceeded the deadline";
      return decision;
  }
  decision.code = "MISSION_GOAL_RESPONSE_INVALID";
  decision.message = "cleaning mission goal response is invalid";
  return decision;
}

}  // namespace modeling
}  // namespace cleanbot
