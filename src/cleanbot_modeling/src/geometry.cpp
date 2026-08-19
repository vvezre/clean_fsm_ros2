/*
 * 文件作用：几何工具实现：提供点、线段、多边形和角度计算。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_modeling/geometry.hpp"

#include <cmath>

namespace cleanbot {
namespace modeling {
namespace geometry_detail {

constexpr double kGeometryEarthRadiusM = 6371000.0;
constexpr double kGeometryPi = 3.14159265358979323846;
constexpr double kGeometryEpsilon = 1e-9;

// 计算三点有向面积，用于判断转向方向和共线关系。
double geometry_orientation(
    const Point2d& a,
    const Point2d& b,
    const Point2d& c) {
  return (b.x_cm - a.x_cm) * (c.y_cm - a.y_cm) -
      (b.y_cm - a.y_cm) * (c.x_cm - a.x_cm);
}

// 判断共线点是否位于线段包围范围内。
bool geometry_on_segment(
    const Point2d& a,
    const Point2d& b,
    const Point2d& point) {
  return point.x_cm >= std::min(a.x_cm, b.x_cm) - kGeometryEpsilon &&
      point.x_cm <= std::max(a.x_cm, b.x_cm) + kGeometryEpsilon &&
      point.y_cm >= std::min(a.y_cm, b.y_cm) - kGeometryEpsilon &&
      point.y_cm <= std::max(a.y_cm, b.y_cm) + kGeometryEpsilon &&
      std::abs(geometry_orientation(a, b, point)) <= kGeometryEpsilon;
}

// 使用方向测试和共线特例判断两条线段是否相交。
bool geometry_segments_intersect(
    const Point2d& a,
    const Point2d& b,
    const Point2d& c,
    const Point2d& d) {
  const double ab_c = geometry_orientation(a, b, c);
  const double ab_d = geometry_orientation(a, b, d);
  const double cd_a = geometry_orientation(c, d, a);
  const double cd_b = geometry_orientation(c, d, b);

  if (((ab_c > kGeometryEpsilon && ab_d < -kGeometryEpsilon) ||
       (ab_c < -kGeometryEpsilon && ab_d > kGeometryEpsilon)) &&
      ((cd_a > kGeometryEpsilon && cd_b < -kGeometryEpsilon) ||
       (cd_a < -kGeometryEpsilon && cd_b > kGeometryEpsilon))) {
    return true;
  }
  return (std::abs(ab_c) <= kGeometryEpsilon &&
          geometry_on_segment(a, b, c)) ||
      (std::abs(ab_d) <= kGeometryEpsilon &&
       geometry_on_segment(a, b, d)) ||
      (std::abs(cd_a) <= kGeometryEpsilon &&
       geometry_on_segment(c, d, a)) ||
      (std::abs(cd_b) <= kGeometryEpsilon &&
       geometry_on_segment(c, d, b));
}

}  // namespace geometry_detail

using namespace geometry_detail;

// 将经纬度投影到以指定原点为基准的局部厘米坐标。
Point2d lat_lon_to_local_cm(
    const double origin_lat,
    const double origin_lon,
    const double lat,
    const double lon) {
  const double mean_lat =
      (origin_lat + lat) * 0.5 * kGeometryPi / 180.0;
  Point2d point;
  point.x_cm =
      (lon - origin_lon) * kGeometryPi / 180.0 *
      kGeometryEarthRadiusM * std::cos(mean_lat) * 100.0;
  point.y_cm =
      (lat - origin_lat) * kGeometryPi / 180.0 *
      kGeometryEarthRadiusM * 100.0;
  return point;
}

// 必要时建立模型原点，并为采集点写入统一局部坐标。
bool set_model_point_local_coordinates(
    CleaningModel& model,
    ModelPoint& point) {
  if (!std::isfinite(point.lat) || !std::isfinite(point.lon) ||
      point.lat < -90.0 || point.lat > 90.0 ||
      point.lon < -180.0 || point.lon > 180.0) {
    return false;
  }
  if (!model.origin_valid) {
    model.origin_lat = point.lat;
    model.origin_lon = point.lon;
    model.origin_valid = true;
  }
  const auto local = lat_lon_to_local_cm(
      model.origin_lat,
      model.origin_lon,
      point.lat,
      point.lon);
  point.x_cm = local.x_cm;
  point.y_cm = local.y_cm;
  return true;
}

// 将局部厘米坐标反投影为经纬度。
void local_cm_to_lat_lon(
    const double origin_lat,
    const double origin_lon,
    const Point2d& origin,
    const Point2d& point,
    double& lat,
    double& lon) {
  const double north_m = (point.y_cm - origin.y_cm) / 100.0;
  const double east_m = (point.x_cm - origin.x_cm) / 100.0;
  lat = origin_lat +
      north_m / kGeometryEarthRadiusM * 180.0 / kGeometryPi;
  const double mean_lat =
      (origin_lat + lat) * 0.5 * kGeometryPi / 180.0;
  lon = origin_lon +
      east_m / (kGeometryEarthRadiusM * std::cos(mean_lat)) *
      180.0 / kGeometryPi;
}

// 校验多边形点数、有限性、非零面积和边界无自交。
bool is_simple_polygon(const std::vector<Point2d>& polygon) {
  if (polygon.size() < 3u) {
    return false;
  }
  const std::size_t count = polygon.size();
  for (std::size_t index = 0u; index < count; ++index) {
    if (!std::isfinite(polygon[index].x_cm) ||
        !std::isfinite(polygon[index].y_cm) ||
        point_distance_cm(polygon[index], polygon[(index + 1u) % count]) <=
            kGeometryEpsilon) {
      return false;
    }
  }

  for (std::size_t left = 0u; left < count; ++left) {
    const std::size_t left_next = (left + 1u) % count;
    for (std::size_t right = left + 1u; right < count; ++right) {
      const std::size_t right_next = (right + 1u) % count;
      if (left == right || left_next == right || right_next == left) {
        continue;
      }
      if (left == 0u && right_next == 0u) {
        continue;
      }
      if (geometry_segments_intersect(
              polygon[left],
              polygon[left_next],
              polygon[right],
              polygon[right_next])) {
        return false;
      }
    }
  }
  return true;
}

// 计算两个局部坐标点之间的欧氏距离（厘米）。
double point_distance_cm(const Point2d& start, const Point2d& end) {
  return std::hypot(end.x_cm - start.x_cm, end.y_cm - start.y_cm);
}

// 将任意航向角归一化到零至三百六十度范围。
double normalize_heading_deg(const double heading_deg) {
  double normalized = std::fmod(heading_deg, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  return normalized;
}

// 计算局部平面中起点指向终点的航向角。
double heading_from_points_deg(const Point2d& start, const Point2d& end) {
  return normalize_heading_deg(
      std::atan2(end.x_cm - start.x_cm, end.y_cm - start.y_cm) *
      180.0 / kGeometryPi);
}

// 将目标航向相对当前航向的转角归一化为最短有符号角。
double normalize_turn_deg(
    const double target_heading_deg,
    const double current_heading_deg) {
  double delta =
      std::fmod(target_heading_deg - current_heading_deg + 180.0, 360.0);
  if (delta < 0.0) {
    delta += 360.0;
  }
  return delta - 180.0;
}

}  // namespace modeling
}  // namespace cleanbot
