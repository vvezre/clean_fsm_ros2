#include "cleanbot_mission/waypoint_plan.hpp"

#include <cmath>

namespace cleanbot {
namespace mission {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSamePointToleranceDeg = 1e-12;

double normalize_heading_deg(const double heading) {
  double normalized = std::fmod(heading, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  return normalized;
}

}  // namespace

WaypointSequence::WaypointSequence(
    const std::size_t waypoint_count,
    const bool loop,
    const std::uint32_t loop_count)
    : waypoint_count_(waypoint_count),
      loop_(loop),
      loop_count_(loop_count) {
  if (waypoint_count_ == 0u) {
    error_code_ = "WAYPOINTS_EMPTY";
  } else if (loop_ && waypoint_count_ < 2u) {
    error_code_ = "WAYPOINT_LOOP_REQUIRES_TWO_POINTS";
  }
}

bool WaypointSequence::valid() const {
  return error_code_.empty();
}

const std::string& WaypointSequence::error_code() const {
  return error_code_;
}

bool WaypointSequence::next(WaypointTarget& target) {
  if (!valid()) {
    return false;
  }

  if (!loop_) {
    if (next_index_ >= waypoint_count_) {
      return false;
    }
    target.waypoint_index = next_index_++;
    target.completed_loop = 0u;
    return true;
  }

  if (!initial_target_emitted_) {
    initial_target_emitted_ = true;
    next_index_ = 1u;
    target.waypoint_index = 0u;
    target.completed_loop = 0u;
    return true;
  }

  if (loop_count_ > 0u && completed_loop_ >= loop_count_) {
    return false;
  }
  if (next_index_ < waypoint_count_) {
    target.waypoint_index = next_index_++;
    target.completed_loop = completed_loop_;
    return true;
  }

  ++completed_loop_;
  next_index_ = 1u;
  target.waypoint_index = 0u;
  target.completed_loop = completed_loop_;
  return true;
}

bool validate_waypoint(const double lat, const double lon) {
  return std::isfinite(lat) && std::isfinite(lon) &&
      lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

double calculate_heading_deg(
    const double start_lat,
    const double start_lon,
    const double end_lat,
    const double end_lon) {
  const double delta_lat = end_lat - start_lat;
  const double delta_lon = end_lon - start_lon;
  if (std::abs(delta_lat) <= kSamePointToleranceDeg &&
      std::abs(delta_lon) <= kSamePointToleranceDeg) {
    return 0.0;
  }

  const double mean_lat_rad = (start_lat + end_lat) * 0.5 * kPi / 180.0;
  const double north = delta_lat;
  const double east = delta_lon * std::cos(mean_lat_rad);
  return normalize_heading_deg(std::atan2(east, north) * 180.0 / kPi);
}

double normalize_turn_angle_deg(
    const double target_heading,
    const double current_heading) {
  double delta = std::fmod(target_heading - current_heading + 180.0, 360.0);
  if (delta < 0.0) {
    delta += 360.0;
  }
  return delta - 180.0;
}

WaypointSegmentPlan build_waypoint_segment(
    const double start_lat,
    const double start_lon,
    const double current_heading,
    const double end_lat,
    const double end_lon) {
  WaypointSegmentPlan segment;
  segment.start_lat = start_lat;
  segment.start_lon = start_lon;
  segment.end_lat = end_lat;
  segment.end_lon = end_lon;

  const bool same_point =
      std::abs(end_lat - start_lat) <= kSamePointToleranceDeg &&
      std::abs(end_lon - start_lon) <= kSamePointToleranceDeg;
  segment.heading_deg = same_point
      ? normalize_heading_deg(current_heading)
      : calculate_heading_deg(start_lat, start_lon, end_lat, end_lon);
  segment.turn_angle_deg = same_point
      ? 0.0
      : normalize_turn_angle_deg(segment.heading_deg, current_heading);
  return segment;
}

}  // namespace mission
}  // namespace cleanbot
