#ifndef CLEANBOT_RTK__RTK_VALIDITY_HPP_
#define CLEANBOT_RTK__RTK_VALIDITY_HPP_

#include <cstdint>

namespace cleanbot {
namespace rtk {

struct RtkValidityInput {
  bool serial_connected{false};
  bool coordinate_valid{false};
  bool center_valid{false};
  bool heading_valid{false};
  std::uint8_t fix_quality{0u};
  double gga_age_sec{-1.0};
  double max_gga_age_sec{2.0};
};

bool is_fixed_valid(const RtkValidityInput& input);
bool should_publish_freshness_heartbeat(
    bool has_sample,
    bool serial_connected,
    double gga_age_sec,
    double max_gga_age_sec);

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__RTK_VALIDITY_HPP_
