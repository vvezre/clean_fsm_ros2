/*
 * 文件作用：点采样实现：在输入区域内生成满足间距约束的采样点。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_modeling/point_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cleanbot {
namespace modeling {
namespace point_sampler_detail {

constexpr double kSamplerEarthRadiusM = 6371000.0;
constexpr double kSamplerPi = 3.14159265358979323846;
constexpr double kMaximumGgaAgeSec = 2.0;

// 将角度转换为采样距离计算使用的弧度。
double sampler_radians(const double degrees) {
  return degrees * kSamplerPi / 180.0;
}

// 使用半正矢公式计算两个 RTK 样本之间的地表距离。
double sampler_distance_m(
    const double lat_a,
    const double lon_a,
    const double lat_b,
    const double lon_b) {
  const double lat1 = sampler_radians(lat_a);
  const double lat2 = sampler_radians(lat_b);
  const double delta_lat = sampler_radians(lat_b - lat_a);
  const double delta_lon = sampler_radians(lon_b - lon_a);
  const double sin_lat = std::sin(delta_lat * 0.5);
  const double sin_lon = std::sin(delta_lon * 0.5);
  const double haversine =
      sin_lat * sin_lat +
      std::cos(lat1) * std::cos(lat2) * sin_lon * sin_lon;
  const double bounded = std::max(0.0, std::min(1.0, haversine));
  return kSamplerEarthRadiusM * 2.0 *
      std::atan2(std::sqrt(bounded), std::sqrt(1.0 - bounded));
}

// 构造模型点采样失败结果和业务说明。
SampleResult sample_failure(
    const std::string& code,
    const std::string& message) {
  SampleResult result;
  result.code = code;
  result.message = message;
  return result;
}

// 判断样本的固定解、中心坐标和经纬度是否可用于建模。
bool sample_coordinate_valid(const RtkSample& sample) {
  return std::isfinite(sample.lat) && std::isfinite(sample.lon) &&
      sample.lat >= -90.0 && sample.lat <= 90.0 &&
      sample.lon >= -180.0 && sample.lon <= 180.0;
}

}  // namespace point_sampler_detail

using namespace point_sampler_detail;

// 校验样本质量与离散半径，并计算平均坐标和圆周平均航向。
SampleResult sample_point(
    const std::vector<RtkSample>& samples,
    const std::size_t minimum_samples,
    const double maximum_radius_m) {
  if (minimum_samples == 0u || samples.size() < minimum_samples) {
    return sample_failure(
        "SAMPLE_COUNT_INSUFFICIENT",
        "not enough RTK samples were collected");
  }
  if (!std::isfinite(maximum_radius_m) || maximum_radius_m <= 0.0) {
    return sample_failure(
        "SAMPLE_RADIUS_INVALID",
        "maximum RTK sample radius must be positive");
  }

  double latitude_sum = 0.0;
  double longitude_sum = 0.0;
  double heading_sin_sum = 0.0;
  double heading_cos_sum = 0.0;
  std::size_t heading_count = 0u;

  for (const auto& sample : samples) {
    if (!sample.fixed_valid || !sample.center_valid ||
        sample.fix_quality != 4u) {
      return sample_failure(
          "RTK_NOT_FIXED", "RTK fixed vehicle-centre data is required");
    }
    if (!std::isfinite(sample.gga_age_sec) ||
        sample.gga_age_sec < 0.0 ||
        sample.gga_age_sec > kMaximumGgaAgeSec) {
      return sample_failure("RTK_STALE", "RTK GGA data is stale");
    }
    if (!sample.vehicle_static) {
      return sample_failure(
          "VEHICLE_NOT_STATIC", "vehicle must remain stationary");
    }
    if (!sample_coordinate_valid(sample)) {
      return sample_failure(
          "RTK_LOCATION_INVALID", "RTK coordinate is invalid");
    }

    latitude_sum += sample.lat;
    longitude_sum += sample.lon;
    if (sample.heading_valid && std::isfinite(sample.heading_deg)) {
      const double heading = sampler_radians(sample.heading_deg);
      heading_sin_sum += std::sin(heading);
      heading_cos_sum += std::cos(heading);
      ++heading_count;
    }
  }

  const double count = static_cast<double>(samples.size());
  const double mean_lat = latitude_sum / count;
  const double mean_lon = longitude_sum / count;
  double radius_m = 0.0;
  for (const auto& sample : samples) {
    radius_m = std::max(
        radius_m,
        sampler_distance_m(mean_lat, mean_lon, sample.lat, sample.lon));
  }
  if (radius_m > maximum_radius_m) {
    return sample_failure(
        "RTK_SAMPLE_UNSTABLE",
        "RTK sample radius exceeds the configured maximum");
  }

  SampleResult result;
  result.success = true;
  result.code = "OK";
  result.message = "RTK point sampled";
  result.point.lat = mean_lat;
  result.point.lon = mean_lon;
  result.point.sample_count = samples.size();
  result.point.sample_radius_m = radius_m;
  result.point.fix_quality = samples.back().fix_quality;
  result.point.gga_age_sec = samples.back().gga_age_sec;
  result.point.source = "rtk_mean";
  if (heading_count > 0u) {
    double heading_deg =
        std::atan2(heading_sin_sum, heading_cos_sum) *
        180.0 / kSamplerPi;
    if (heading_deg < 0.0) {
      heading_deg += 360.0;
    }
    result.point.heading_deg = heading_deg;
    result.point.heading_valid = true;
  }
  return result;
}

}  // namespace modeling
}  // namespace cleanbot
