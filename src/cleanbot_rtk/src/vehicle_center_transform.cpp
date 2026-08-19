/*
 * 文件作用：车辆中心变换实现：把天线测量点转换到车辆参考中心。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_rtk/vehicle_center_transform.hpp"

#include <cmath>

namespace cleanbot {
namespace rtk {

namespace {
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kPi = 3.14159265358979323846;
}  // namespace

// 根据天线坐标、航向和安装偏移计算车辆几何中心。
CenterPoint VehicleCenterTransform::compute(
    const double raw_lat,
    const double raw_lon,
    const double heading_deg,
    const double along_heading_m,
    const double right_m) const {
  if (!std::isfinite(raw_lat) || !std::isfinite(raw_lon) ||
      !std::isfinite(heading_deg) || !std::isfinite(along_heading_m) ||
      !std::isfinite(right_m) || std::abs(raw_lat) > 90.0 ||
      std::abs(raw_lon) > 180.0) {
    return CenterPoint();
  }

  double heading = std::fmod(heading_deg, 360.0);
  if (heading < 0.0) {
    heading += 360.0;
  }
  const auto longitudinal = destination(
      raw_lat, raw_lon, heading, along_heading_m);
  if (!longitudinal.valid) {
    return CenterPoint();
  }
  return destination(
      longitudinal.lat,
      longitudinal.lon,
      std::fmod(heading + 90.0, 360.0),
      right_m);
}

// 在球面上从起点沿方位角和距离推导目标坐标。
CenterPoint VehicleCenterTransform::destination(
    const double lat,
    const double lon,
    const double bearing_deg,
    const double distance_m) {
  const double lat1 = lat * kPi / 180.0;
  const double lon1 = lon * kPi / 180.0;
  const double bearing = bearing_deg * kPi / 180.0;
  const double angular_distance = distance_m / kEarthRadiusM;
  const double lat2 = std::asin(
      std::sin(lat1) * std::cos(angular_distance) +
      std::cos(lat1) * std::sin(angular_distance) * std::cos(bearing));
  double lon2 = lon1 + std::atan2(
      std::sin(bearing) * std::sin(angular_distance) * std::cos(lat1),
      std::cos(angular_distance) - std::sin(lat1) * std::sin(lat2));
  lon2 = std::fmod(lon2 + 3.0 * kPi, 2.0 * kPi) - kPi;

  CenterPoint point;
  point.lat = round8(lat2 * 180.0 / kPi);
  point.lon = round8(lon2 * 180.0 / kPi);
  point.valid = std::isfinite(point.lat) && std::isfinite(point.lon);
  return point;
}

// 将坐标值按接口精度保留至小数点后八位。
double VehicleCenterTransform::round8(const double value) {
  return std::round(value * 100000000.0) / 100000000.0;
}

}  // namespace rtk
}  // namespace cleanbot
