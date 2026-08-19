#pragma once

#include <string>

// 文件作用：声明下游 ROS 服务结果到 HTTP 业务状态码的映射规则。
namespace cleanbot {
namespace http {

// 调用下游服务后可观察到的响应状态。
enum class DownstreamResponseState {
  kUnavailable,
  kTimeout,
  kResponded,
};

// 下游服务返回的业务接受标志和说明信息。
struct DownstreamResponse {
  bool accepted{false};
  std::string code;
  std::string message;
};

// 对外 HTTP 响应所需的状态码与业务结果。
struct HttpBusinessDecision {
  int status_code{500};
  bool success{false};
  std::string code;
  std::string message;
};

// 将任务暂停服务结果映射为 HTTP 业务决策。
HttpBusinessDecision map_mission_pause_response(
    DownstreamResponseState state,
    const DownstreamResponse& response);

// 将模型计划执行服务结果映射为 HTTP 业务决策。
HttpBusinessDecision map_execute_plan_response(
    DownstreamResponseState state,
    const DownstreamResponse& response);

}  // namespace http
}  // namespace cleanbot
