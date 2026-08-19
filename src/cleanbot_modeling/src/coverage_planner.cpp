/*
 * 文件作用：覆盖规划实现：根据区域边界生成清扫覆盖路径。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_modeling/coverage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "cleanbot_modeling/geometry.hpp"

namespace cleanbot {
namespace modeling {
namespace coverage_planner_detail {

constexpr double kCoveragePi = 3.14159265358979323846;
constexpr double kCoverageEpsilon = 1e-7;
constexpr std::size_t kMaximumParityExpansion = 20u;

struct Axis {
  double direction_x{0.0};
  double direction_y{1.0};
  double normal_x{1.0};
  double normal_y{0.0};
};

// 根据扫掠航向构造车道方向轴和法向轴。
Axis make_axis(const double heading_deg) {
  const double radians =
      normalize_heading_deg(heading_deg) * kCoveragePi / 180.0;
  Axis axis;
  axis.direction_x = std::sin(radians);
  axis.direction_y = std::cos(radians);
  axis.normal_x = std::cos(radians);
  axis.normal_y = -std::sin(radians);
  return axis;
}

// 计算点在指定轴方向上的标量投影。
double projection(
    const Point2d& point,
    const double x,
    const double y) {
  return point.x_cm * x + point.y_cm * y;
}

// 在覆盖规划容差内判断两个局部坐标点是否重合。
bool same_point(const Point2d& left, const Point2d& right) {
  return point_distance_cm(left, right) <= 1e-5;
}

// 仅在不存在等价点时追加交点，避免顶点重复计数。
void append_unique(std::vector<Point2d>& points, const Point2d& candidate) {
  const bool exists = std::any_of(
      points.begin(), points.end(),
      [&candidate](const Point2d& existing) {
        return same_point(existing, candidate);
      });
  if (!exists) {
    points.push_back(candidate);
  }
}

// 计算指定偏移扫掠线与区域多边形边界的全部唯一交点。
std::vector<Point2d> line_intersections(
    const std::vector<Point2d>& polygon,
    const Axis& axis,
    const double offset) {
  std::vector<Point2d> intersections;
  for (std::size_t index = 0u; index < polygon.size(); ++index) {
    const Point2d& start = polygon[index];
    const Point2d& end = polygon[(index + 1u) % polygon.size()];
    const double start_value =
        projection(start, axis.normal_x, axis.normal_y) - offset;
    const double end_value =
        projection(end, axis.normal_x, axis.normal_y) - offset;

    if (std::abs(start_value) <= kCoverageEpsilon) {
      append_unique(intersections, start);
    }
    if (std::abs(end_value) <= kCoverageEpsilon) {
      append_unique(intersections, end);
    }

    const double denominator = start_value - end_value;
    if (std::abs(denominator) <= kCoverageEpsilon) {
      continue;
    }
    const double ratio = start_value / denominator;
    if (ratio < -kCoverageEpsilon || ratio > 1.0 + kCoverageEpsilon) {
      continue;
    }
    Point2d point;
    point.x_cm = start.x_cm + (end.x_cm - start.x_cm) * ratio;
    point.y_cm = start.y_cm + (end.y_cm - start.y_cm) * ratio;
    append_unique(intersections, point);
  }

  std::sort(
      intersections.begin(), intersections.end(),
      // 排序谓词作用：按当前几何或序号规则稳定比较两个候选元素。
      [&axis](const Point2d& left, const Point2d& right) {
        return projection(left, axis.direction_x, axis.direction_y) <
            projection(right, axis.direction_x, axis.direction_y);
      });
  return intersections;
}

// 按偏移位置生成区域内成对交点构成的覆盖车道。
std::vector<CoverageLane> build_lanes(
    const std::vector<Point2d>& polygon,
    const Axis& axis,
    const double heading_deg,
    const double first_offset,
    const double spacing,
    const std::size_t line_count) {
  std::vector<CoverageLane> lanes;
  for (std::size_t line_index = 0u; line_index < line_count; ++line_index) {
    const double offset =
        first_offset + spacing * static_cast<double>(line_index);
    const auto intersections = line_intersections(polygon, axis, offset);
    for (std::size_t index = 0u; index + 1u < intersections.size(); index += 2u) {
      const Point2d& start = intersections[index];
      const Point2d& end = intersections[index + 1u];
      const double length = point_distance_cm(start, end);
      if (length <= kCoverageEpsilon) {
        continue;
      }
      CoverageLane lane;
      lane.id = "lane-" + std::to_string(lanes.size() + 1u);
      lane.start = start;
      lane.end = end;
      lane.heading_deg = normalize_heading_deg(heading_deg);
      lane.offset_cm = offset;
      lane.length_cm = length;
      lanes.push_back(lane);
    }
  }
  return lanes;
}

// 构造覆盖规划失败结果和稳定业务错误码。
CoverageResult coverage_failure(
    const std::string& code,
    const std::string& message) {
  CoverageResult result;
  result.code = code;
  result.message = message;
  return result;
}

}  // namespace coverage_planner_detail

using namespace coverage_planner_detail;

// 校验区域与刷盘参数，计算间距、重叠量和全部平行覆盖车道。
CoverageResult plan_coverage(
    const std::vector<Point2d>& polygon,
    const double sweep_angle_deg,
    const double brush_width_cm,
    const double minimum_overlap_cm) {
  if (!is_simple_polygon(polygon)) {
    return coverage_failure(
        "POLYGON_INVALID", "coverage polygon is invalid");
  }
  if (!std::isfinite(sweep_angle_deg) ||
      !std::isfinite(brush_width_cm) ||
      !std::isfinite(minimum_overlap_cm) ||
      brush_width_cm <= 0.0 ||
      minimum_overlap_cm < 0.0 ||
      minimum_overlap_cm >= brush_width_cm) {
    return coverage_failure(
        "LANE_SPACING_INVALID",
        "brush width and overlap do not produce a positive lane spacing");
  }

  const Axis axis = make_axis(sweep_angle_deg);
  double minimum_offset = std::numeric_limits<double>::infinity();
  double maximum_offset = -std::numeric_limits<double>::infinity();
  for (const auto& point : polygon) {
    const double offset = projection(point, axis.normal_x, axis.normal_y);
    minimum_offset = std::min(minimum_offset, offset);
    maximum_offset = std::max(maximum_offset, offset);
  }
  const double width = maximum_offset - minimum_offset;
  if (!std::isfinite(width) || width <= kCoverageEpsilon) {
    return coverage_failure(
        "POLYGON_WIDTH_INVALID", "coverage width is zero");
  }

  const double maximum_spacing = brush_width_cm - minimum_overlap_cm;
  double first_offset = 0.0;
  double centre_span = 0.0;
  std::size_t natural_count = 0u;

  if (width <= brush_width_cm) {
    natural_count = 1u;
    centre_span = std::min(maximum_spacing, width * 0.5);
    if (centre_span <= kCoverageEpsilon) {
      return coverage_failure(
          "DISTINCT_LANES_UNAVAILABLE",
          "area is too narrow for two distinct cleaning lanes");
    }
    first_offset =
        (minimum_offset + maximum_offset) * 0.5 - centre_span * 0.5;
  } else {
    centre_span = width - brush_width_cm;
    natural_count =
        static_cast<std::size_t>(
            std::ceil(centre_span / maximum_spacing)) +
        1u;
    natural_count = std::max<std::size_t>(natural_count, 2u);
    first_offset = minimum_offset + brush_width_cm * 0.5;
  }

  std::size_t line_count = std::max<std::size_t>(natural_count, 2u);
  if (line_count % 2u != 0u) {
    ++line_count;
  }

  const std::size_t maximum_line_count =
      line_count + kMaximumParityExpansion;
  for (; line_count <= maximum_line_count; line_count += 2u) {
    const double spacing =
        centre_span / static_cast<double>(line_count - 1u);
    if (!std::isfinite(spacing) || spacing <= 0.0 ||
        spacing > maximum_spacing + kCoverageEpsilon) {
      continue;
    }
    auto lanes = build_lanes(
        polygon,
        axis,
        sweep_angle_deg,
        first_offset,
        spacing,
        line_count);
    if (lanes.empty() || lanes.size() % 2u != 0u) {
      continue;
    }

    bool distinct = true;
    for (std::size_t left = 0u; left < lanes.size() && distinct; ++left) {
      for (std::size_t right = left + 1u; right < lanes.size(); ++right) {
        const bool same_direction =
            same_point(lanes[left].start, lanes[right].start) &&
            same_point(lanes[left].end, lanes[right].end);
        const bool reverse_direction =
            same_point(lanes[left].start, lanes[right].end) &&
            same_point(lanes[left].end, lanes[right].start);
        if (same_direction || reverse_direction) {
          distinct = false;
          break;
        }
      }
    }
    if (!distinct) {
      continue;
    }

    CoverageResult result;
    result.success = true;
    result.code = "OK";
    result.message = "coverage lanes generated";
    result.natural_lane_count = natural_count;
    result.lane_spacing_cm = spacing;
    result.actual_overlap_cm = brush_width_cm - spacing;
    result.lanes = std::move(lanes);
    return result;
  }

  return coverage_failure(
      "EVEN_LANE_LAYOUT_UNAVAILABLE",
      "unable to generate distinct even cleaning lanes");
}

}  // namespace modeling
}  // namespace cleanbot
