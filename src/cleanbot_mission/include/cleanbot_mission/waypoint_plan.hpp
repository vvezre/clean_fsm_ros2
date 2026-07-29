#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace cleanbot {
namespace mission {

struct WaypointTarget {
  std::size_t waypoint_index{0u};
  std::uint32_t completed_loop{0u};
};

struct WaypointSegmentPlan {
  double start_lat{0.0};
  double start_lon{0.0};
  double end_lat{0.0};
  double end_lon{0.0};
  double heading_deg{0.0};
  double turn_angle_deg{0.0};
};

// Produces the same closed-loop order as the legacy Python implementation:
// approach P0 once, then repeat P1 ... Pn -> P0 for every completed loop.
class WaypointSequence {
 public:
  WaypointSequence(
      std::size_t waypoint_count,
      bool loop,
      std::uint32_t loop_count);

  bool valid() const;
  const std::string& error_code() const;
  bool next(WaypointTarget& target);

 private:
  std::size_t waypoint_count_{0u};
  bool loop_{false};
  std::uint32_t loop_count_{0u};
  bool initial_target_emitted_{false};
  std::size_t next_index_{0u};
  std::uint32_t completed_loop_{0u};
  std::string error_code_;
};

bool validate_waypoint(double lat, double lon);
double calculate_heading_deg(
    double start_lat,
    double start_lon,
    double end_lat,
    double end_lon);
double normalize_turn_angle_deg(double target_heading, double current_heading);
WaypointSegmentPlan build_waypoint_segment(
    double start_lat,
    double start_lon,
    double current_heading,
    double end_lat,
    double end_lon);

}  // namespace mission
}  // namespace cleanbot
