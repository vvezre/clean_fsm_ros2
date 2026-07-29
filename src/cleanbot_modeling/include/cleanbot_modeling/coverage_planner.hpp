#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "cleanbot_modeling/model_types.hpp"

namespace cleanbot {
namespace modeling {

struct CoverageLane {
  std::string id;
  Point2d start;
  Point2d end;
  double heading_deg{0.0};
  double offset_cm{0.0};
  double length_cm{0.0};
};

struct CoverageResult {
  bool success{false};
  std::string code;
  std::string message;
  std::size_t natural_lane_count{0u};
  double lane_spacing_cm{0.0};
  double actual_overlap_cm{0.0};
  std::vector<CoverageLane> lanes;
};

CoverageResult plan_coverage(
    const std::vector<Point2d>& polygon,
    double sweep_angle_deg,
    double brush_width_cm,
    double minimum_overlap_cm);

}  // namespace modeling
}  // namespace cleanbot
