/*
 * 文件作用：服务响应映射实现：把ROS2服务结果转换为HTTP响应语义。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_http/service_response_mapping.hpp"

namespace cleanbot {
namespace http {
namespace {

// 构造不依赖下游响应内容的固定 HTTP 业务决策。
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

// 将下游业务接受标志映射为成功状态码或 409 冲突。
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

// 将任务暂停服务的不可用、超时或正常响应映射为 HTTP 结果。
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

// 将计划执行服务的不可用、超时或正常响应映射为 HTTP 结果。
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
