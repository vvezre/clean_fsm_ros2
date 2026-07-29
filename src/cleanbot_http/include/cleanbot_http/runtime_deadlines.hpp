#pragma once

#include <chrono>

namespace cleanbot {
namespace http {

// Beast applies this deadline independently to socket read and write
// operations. It is not an end-to-end request latency guarantee.
inline constexpr std::chrono::milliseconds kHttpIoOperationDeadline{3000};
inline constexpr std::chrono::milliseconds kRosServiceResponseDeadline{2000};

static_assert(
    kRosServiceResponseDeadline < kHttpIoOperationDeadline,
    "the downstream ROS wait must remain shorter than one HTTP I/O deadline");

}  // namespace http
}  // namespace cleanbot
