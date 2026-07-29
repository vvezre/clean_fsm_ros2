#include "cleanbot_rtk/rtk_sample_synchronizer.hpp"

#include <cmath>

namespace cleanbot {
namespace rtk {

RtkSampleSynchronizer::RtkSampleSynchronizer(
    const double sync_threshold_sec,
    const double nmea_max_age_sec,
    const double heading_max_age_sec)
    : sync_threshold_sec_(sync_threshold_sec < 0.0 ? 0.0 : sync_threshold_sec),
      nmea_max_age_sec_(nmea_max_age_sec < 0.0 ? 0.0 : nmea_max_age_sec),
      heading_max_age_sec_(heading_max_age_sec < 0.0 ? 0.0 : heading_max_age_sec) {}

void RtkSampleSynchronizer::observe_heading(
    const HeadingData& heading, const double receive_time_sec) {
  latest_heading_ = heading;
  latest_heading_received_at_ = receive_time_sec;
  has_heading_ = true;
}

SynchronizeResult RtkSampleSynchronizer::observe_gga(
    const GgaData& gga,
    const double receive_time_sec) const {
  SynchronizeResult result;
  result.sample.gga = gga;
  result.sample.gga_age_sec = 0.0;
  result.produced = true;
  if (!has_heading_) {
    return result;
  }
  result.sample.heading_age_sec = receive_time_sec - latest_heading_received_at_;
  if (result.sample.heading_age_sec < 0.0) {
    result.sample.heading_age_sec = 0.0;
  }
  if (result.sample.heading_age_sec > heading_max_age_sec_) {
    return result;
  }
  if (latest_heading_.utc_seconds >= 0.0 && gga.utc_seconds >= 0.0 &&
      utcDifference(latest_heading_.utc_seconds, gga.utc_seconds) > sync_threshold_sec_) {
    return result;
  }

  result.sample.heading_deg = latest_heading_.heading_deg;
  result.sample.heading_valid = true;
  return result;
}

void RtkSampleSynchronizer::reset() {
  latest_heading_ = HeadingData();
  latest_heading_received_at_ = 0.0;
  has_heading_ = false;
}

double RtkSampleSynchronizer::utcDifference(const double left, const double right) {
  double difference = std::abs(left - right);
  if (difference > 43200.0) {
    difference = 86400.0 - difference;
  }
  return std::abs(difference);
}

}  // namespace rtk
}  // namespace cleanbot
