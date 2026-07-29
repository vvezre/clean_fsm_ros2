#pragma once

#include <string>

namespace cleanbot {
namespace http {

enum class DownstreamResponseState {
  kUnavailable,
  kTimeout,
  kResponded,
};

struct DownstreamResponse {
  bool accepted{false};
  std::string code;
  std::string message;
};

struct HttpBusinessDecision {
  int status_code{500};
  bool success{false};
  std::string code;
  std::string message;
};

HttpBusinessDecision map_mission_pause_response(
    DownstreamResponseState state,
    const DownstreamResponse& response);

HttpBusinessDecision map_execute_plan_response(
    DownstreamResponseState state,
    const DownstreamResponse& response);

}  // namespace http
}  // namespace cleanbot
