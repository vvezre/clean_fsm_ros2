#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "cleanbot_modeling/model_types.hpp"

// 文件作用：声明基于多边形区域生成平行清扫覆盖线的规划接口。
namespace cleanbot {
namespace modeling {

// 一条覆盖清扫线的端点、航向、偏移与长度。
struct CoverageLane {
  std::string id;
  Point2d start;
  Point2d end;
  double heading_deg{0.0};
  double offset_cm{0.0};
  double length_cm{0.0};
};

// 覆盖规划的状态、实际重叠量和生成车道集合。
struct CoverageResult {
  bool success{false};
  std::string code;
  std::string message;
  std::size_t natural_lane_count{0u};
  double lane_spacing_cm{0.0};
  double actual_overlap_cm{0.0};
  std::vector<CoverageLane> lanes;
};

// 根据区域、扫掠角、刷盘宽度和最小重叠量生成覆盖车道。
CoverageResult plan_coverage(
    const std::vector<Point2d>& polygon,
    double sweep_angle_deg,
    double brush_width_cm,
    double minimum_overlap_cm);

}  // namespace modeling
}  // namespace cleanbot
