#include "cleanbot_http/service_response_mapping.hpp"

namespace cleanbot {
namespace http {
namespace {

HttpBusinessDecision fixedDecision(
    const int status_code,
    const std::string& code,
    const std::string& message) {
  HttpBusinessDecision decision;
  decision.status_code = status_code;
  decision.code = code;
  decision.message = message;
  return decision;
}

HttpBusinessDecision respondedDecision(
    const DownstreamResponse& response,
    const int accepted_status) {
  HttpBusinessDecision decision;
  decision.status_code = response.accepted ? accepted_status : 409;
  decision.success = response.accepted;
  decision.code = response.code;
  decision.message = response.message;
  return decision;
}

}  // namespace

HttpBusinessDecision map_mission_pause_response(
    const DownstreamResponseState state,
    const DownstreamResponse& response) {
  switch (state) {
    case DownstreamResponseState::kUnavailable:
      return fixedDecision(
          503,
          "MISSION_SERVICE_UNAVAILABLE",
          "mission pause service is unavailable");
    case DownstreamResponseState::kTimeout:
      return fixedDecision(
          504,
          "MISSION_SERVICE_TIMEOUT",
          "mission pause service did not respond before the deadline");
    case DownstreamResponseState::kResponded:
      return respondedDecision(response, 200);
  }
  return fixedDecision(
      500,
      "MISSION_SERVICE_RESPONSE_INVALID",
      "mission pause service response state is invalid");
}

HttpBusinessDecision map_execute_plan_response(
    const DownstreamResponseState state,
    const DownstreamResponse& response) {
  switch (state) {
    case DownstreamResponseState::kUnavailable:
      return fixedDecision(
          503,
          "MODELING_SERVICE_UNAVAILABLE",
          "modeling execute-plan service is unavailable");
    case DownstreamResponseState::kTimeout:
      return fixedDecision(
          504,
          "MODELING_SERVICE_TIMEOUT",
          "modeling execute-plan service did not respond before the deadline");
    case DownstreamResponseState::kResponded:
      return respondedDecision(response, 202);
  }
  return fixedDecision(
      500,
      "MODELING_SERVICE_RESPONSE_INVALID",
      "modeling execute-plan service response state is invalid");
}

}  // namespace http
}  // namespace cleanbot
