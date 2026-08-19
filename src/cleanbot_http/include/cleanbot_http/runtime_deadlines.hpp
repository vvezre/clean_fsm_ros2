#pragma once

#include <chrono>

// 文件作用：集中约束 HTTP 网络读写与下游 ROS 服务等待的运行期超时。
namespace cleanbot {
namespace http {

// Beast 分别对 socket 读写使用的操作超时，不代表端到端请求时延上限。
inline constexpr std::chrono::milliseconds kHttpIoOperationDeadline{3000};
// 等待下游 ROS 服务响应的最长时间，必须早于 HTTP I/O 超时。
inline constexpr std::chrono::milliseconds kRosServiceResponseDeadline{2000};

static_assert(
    kRosServiceResponseDeadline < kHttpIoOperationDeadline,
    "the downstream ROS wait must remain shorter than one HTTP I/O deadline");

}  // namespace http
}  // namespace cleanbot
