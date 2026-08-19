#ifndef CLEANBOT_RTK__RTK_VALIDITY_HPP_
#define CLEANBOT_RTK__RTK_VALIDITY_HPP_

#include <cstdint>

// 文件作用：声明 RTK 定位有效性和新鲜度心跳发布条件的纯判定函数。
namespace cleanbot {
namespace rtk {

// 判断 RTK 固定解是否可供车辆控制使用的输入条件。
struct RtkValidityInput {
  bool serial_connected{false};
  bool coordinate_valid{false};
  bool center_valid{false};
  bool heading_valid{false};
  std::uint8_t fix_quality{0u};
  double gga_age_sec{-1.0};
  double max_gga_age_sec{2.0};
};

// 判断串口、坐标、航向、固定解质量和消息年龄是否均满足使用条件。
bool is_fixed_valid(const RtkValidityInput& input);
// 判断虽无新样本但仍需发布状态变化心跳。
bool should_publish_freshness_heartbeat(
    bool has_sample,
    bool serial_connected,
    double gga_age_sec,
    double max_gga_age_sec);

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__RTK_VALIDITY_HPP_
