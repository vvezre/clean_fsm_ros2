#ifndef CLEANBOT_CONTROL__TRACKING_CORE_HPP_
#define CLEANBOT_CONTROL__TRACKING_CORE_HPP_

#include <cstdint>
#include <string>

namespace cleanbot {
namespace control {

struct TrackingParameters {
  double process_noise{0.2};
  double measurement_noise{1.0};
  double max_dt{1.0};
  double heading_gain{10.0};
  double cte_gain{1000.0};
  double short_range_heading_limit_deg{5.0};
  std::int32_t max_z_speed{15000};
  double target_tolerance_m{0.03};
  double overshoot_cte_tolerance_m{0.30};
};

struct FilteredRtkPoint {
  double raw_lat{0.0};
  double raw_lon{0.0};
  double lat{0.0};
  double lon{0.0};
  double timestamp{0.0};
  bool filtered{false};
};

struct TrackingCommand {
  double raw_lat{0.0};
  double raw_lon{0.0};
  double filtered_lat{0.0};
  double filtered_lon{0.0};
  double distance_to_target_m{0.0};
  double signed_remaining_m{0.0};
  double cte_m{0.0};
  double heading_error_deg{0.0};
  std::int32_t z_speed{0};
  std::string source;
};

double normalize_heading_delta(double target_heading, double current_heading);
bool should_finish_point_to_point(
    double distance_to_target,
    double signed_remaining,
    double cte,
    double target_tolerance_m = 0.03,
    double cte_tolerance_m = 0.30);

class RtkKalmanFilter2D {
 public:
  explicit RtkKalmanFilter2D(const TrackingParameters& parameters = TrackingParameters());
  void reset();
  FilteredRtkPoint update(double lat, double lon, double timestamp);

 private:
  TrackingParameters parameters_;
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double last_timestamp_{0.0};
  double state_[4]{};
  double covariance_[4][4]{};
  bool initialized_{false};
};

class StraightLinePController {
 public:
  explicit StraightLinePController(
      const TrackingParameters& parameters = TrackingParameters());

  TrackingCommand compute(
      double start_lat,
      double start_lon,
      double end_lat,
      double end_lon,
      double current_lat,
      double current_lon,
      double vehicle_heading,
      double target_heading,
      double raw_lat,
      double raw_lon) const;

 private:
  TrackingParameters parameters_;
};

TrackingCommand build_tracking_command(
    RtkKalmanFilter2D& filter,
    const StraightLinePController& controller,
    double start_lat,
    double start_lon,
    double end_lat,
    double end_lon,
    double current_lat,
    double current_lon,
    double vehicle_heading,
    double target_heading,
    double timestamp);

}  // namespace control
}  // namespace cleanbot

#endif  // CLEANBOT_CONTROL__TRACKING_CORE_HPP_
