#include "cleanbot_modeling/goal_submission.hpp"

namespace cleanbot {
namespace modeling {

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
