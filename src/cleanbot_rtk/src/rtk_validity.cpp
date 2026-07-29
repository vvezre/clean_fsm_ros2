#include "cleanbot_rtk/rtk_validity.hpp"

#include <cmath>

namespace cleanbot {
namespace rtk {

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
