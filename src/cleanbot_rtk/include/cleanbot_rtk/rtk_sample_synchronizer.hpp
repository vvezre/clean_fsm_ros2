#ifndef CLEANBOT_RTK__RTK_SAMPLE_SYNCHRONIZER_HPP_
#define CLEANBOT_RTK__RTK_SAMPLE_SYNCHRONIZER_HPP_

#include "cleanbot_rtk/nmea_parser.hpp"

namespace cleanbot {
namespace rtk {

struct RtkSample {
  GgaData gga;
  double heading_deg{0.0};
  double heading_age_sec{0.0};
  double gga_age_sec{0.0};
  bool heading_valid{false};
};

struct SynchronizeResult {
  bool produced{false};
  bool stale{false};
  RtkSample sample;
};

class RtkSampleSynchronizer {
 public:
  RtkSampleSynchronizer(
      double sync_threshold_sec = 0.5,
      double nmea_max_age_sec = 2.0,
      double heading_max_age_sec = 0.5);

  void observe_heading(const HeadingData& heading, double receive_time_sec);
  SynchronizeResult observe_gga(
      const GgaData& gga,
      double receive_time_sec) const;
  void reset();

 private:
  static double utcDifference(double left, double right);
  double sync_threshold_sec_;
  double nmea_max_age_sec_;
  double heading_max_age_sec_;
  HeadingData latest_heading_;
  double latest_heading_received_at_{0.0};
  bool has_heading_{false};
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__RTK_SAMPLE_SYNCHRONIZER_HPP_
