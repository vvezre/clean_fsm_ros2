/*
 * 文件作用：RTK有效性实现：判断定位质量、固定解和数据新鲜度。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_rtk/rtk_validity.hpp"

#include <cmath>

namespace cleanbot {
namespace rtk {

// 综合连接、坐标、航向、固定解质量和新鲜度判断 RTK 是否可用于控制。
bool is_fixed_valid(const RtkValidityInput& input) {
  return input.serial_connected &&
      input.coordinate_valid &&
      input.center_valid &&
      input.heading_valid &&
      input.fix_quality == 4u &&
      std::isfinite(input.gga_age_sec) &&
      std::isfinite(input.max_gga_age_sec) &&
      input.gga_age_sec >= 0.0 &&
      input.max_gga_age_sec >= 0.0 &&
      input.gga_age_sec <= input.max_gga_age_sec;
}

// 判断 RTK 样本过期状态变化时是否需要发布状态心跳。
bool should_publish_freshness_heartbeat(
    const bool has_sample,
    const bool serial_connected,
    const double gga_age_sec,
    const double max_gga_age_sec) {
  if (!has_sample || !serial_connected) {
    return true;
  }
  if (!std::isfinite(gga_age_sec) || !std::isfinite(max_gga_age_sec) ||
      gga_age_sec < 0.0 || max_gga_age_sec < 0.0) {
    return true;
  }
  return gga_age_sec > max_gga_age_sec;
}

}  // namespace rtk
}  // namespace cleanbot
