/*
 * 文件作用：跟踪核心实现：根据目标线段和车辆状态计算跟踪控制量。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_control/tracking_core.hpp"

#include <algorithm>
#include <cmath>

namespace cleanbot {
namespace control {

namespace {
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

// 将角度转换为三角函数计算需要的弧度。
double degreesToRadians(const double value) { return value * kPi / 180.0; }
// 将弧度转换回对外使用的角度。
double radiansToDegrees(const double value) { return value * 180.0 / kPi; }

// 以起点为原点将经纬度近似投影到局部米坐标。
void latLonToLocal(
    const double origin_lat,
    const double origin_lon,
    const double lat,
    const double lon,
    double& x,
    double& y) {
  const double lat0 = degreesToRadians(origin_lat);
  x = degreesToRadians(lon - origin_lon) * kEarthRadiusM * std::cos(lat0);
  y = degreesToRadians(lat - origin_lat) * kEarthRadiusM;
}

// 将局部米坐标反投影为经纬度。
void localToLatLon(
    const double origin_lat,
    const double origin_lon,
    const double x,
    const double y,
    double& lat,
    double& lon) {
  lat = origin_lat + radiansToDegrees(y / kEarthRadiusM);
  const double scale = kEarthRadiusM * std::cos(degreesToRadians(origin_lat));
  lon = origin_lon + radiansToDegrees(x / scale);
}
}  // namespace

// 返回目标航向相对当前航向的最短有符号角差。
double normalize_heading_delta(
    const double target_heading, const double current_heading) {
  double delta = std::fmod(target_heading - current_heading + 180.0, 360.0);
  if (delta < 0.0) {
    delta += 360.0;
  }
  return delta - 180.0;
}

// 依据距离、投影剩余量和横向误差判定目标点是否完成。
bool should_finish_point_to_point(
    const double distance_to_target,
    const double signed_remaining,
    const double cte,
    const double target_tolerance_m,
    const double cte_tolerance_m) {
  if (std::isfinite(distance_to_target) && distance_to_target <= target_tolerance_m) {
    return true;
  }
  return std::isfinite(signed_remaining) && std::isfinite(cte) &&
      signed_remaining <= 0.0 && std::abs(cte) <= cte_tolerance_m;
}

// 保存卡尔曼滤波过程噪声、观测噪声和时间步长限制。
RtkKalmanFilter2D::RtkKalmanFilter2D(const TrackingParameters& parameters)
    : parameters_(parameters) {}

// 清空状态向量、协方差和局部坐标原点。
void RtkKalmanFilter2D::reset() {
  origin_lat_ = 0.0;
  origin_lon_ = 0.0;
  last_timestamp_ = 0.0;
  for (int row = 0; row < 4; ++row) {
    state_[row] = 0.0;
    for (int column = 0; column < 4; ++column) {
      covariance_[row][column] = 0.0;
    }
  }
  initialized_ = false;
}

// 执行二维位置/速度卡尔曼预测和观测更新，输出滤波坐标。
FilteredRtkPoint RtkKalmanFilter2D::update(
    const double lat, const double lon, const double timestamp) {
  FilteredRtkPoint result;
  result.raw_lat = lat;
  result.raw_lon = lon;
  result.lat = lat;
  result.lon = lon;
  result.timestamp = timestamp;
  if (!initialized_) {
    origin_lat_ = lat;
    origin_lon_ = lon;
    state_[0] = state_[1] = state_[2] = state_[3] = 0.0;
    covariance_[0][0] = parameters_.measurement_noise;
    covariance_[1][1] = parameters_.measurement_noise;
    covariance_[2][2] = 1.0;
    covariance_[3][3] = 1.0;
    last_timestamp_ = timestamp;
    initialized_ = true;
    return result;
  }

  double dt = timestamp - last_timestamp_;
  if (dt <= 0.0) {
    dt = 0.05;
  }
  dt = std::min(dt, std::max(0.05, parameters_.max_dt));
  last_timestamp_ = timestamp;

  double measurement_x = 0.0;
  double measurement_y = 0.0;
  latLonToLocal(origin_lat_, origin_lon_, lat, lon, measurement_x, measurement_y);

  const double f[4][4] = {
      {1.0, 0.0, dt, 0.0},
      {0.0, 1.0, 0.0, dt},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0}};
  double predicted_state[4]{};
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      predicted_state[row] += f[row][column] * state_[column];
    }
  }

  double fp[4][4]{};
  double predicted_covariance[4][4]{};
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      for (int inner = 0; inner < 4; ++inner) {
        fp[row][column] += f[row][inner] * covariance_[inner][column];
      }
    }
  }
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      for (int inner = 0; inner < 4; ++inner) {
        predicted_covariance[row][column] += fp[row][inner] * f[column][inner];
      }
    }
  }
  predicted_covariance[0][0] += parameters_.process_noise * dt * dt;
  predicted_covariance[1][1] += parameters_.process_noise * dt * dt;
  predicted_covariance[2][2] += parameters_.process_noise * dt;
  predicted_covariance[3][3] += parameters_.process_noise * dt;

  const double s00 = predicted_covariance[0][0] + parameters_.measurement_noise;
  const double s01 = predicted_covariance[0][1];
  const double s10 = predicted_covariance[1][0];
  const double s11 = predicted_covariance[1][1] + parameters_.measurement_noise;
  double determinant = s00 * s11 - s01 * s10;
  if (std::abs(determinant) < 1e-9) {
    determinant = determinant < 0.0 ? -1e-9 : 1e-9;
  }
  const double inv_s[2][2] = {
      {s11 / determinant, -s01 / determinant},
      {-s10 / determinant, s00 / determinant}};
  double kalman_gain[4][2]{};
  for (int row = 0; row < 4; ++row) {
    kalman_gain[row][0] =
        predicted_covariance[row][0] * inv_s[0][0] +
        predicted_covariance[row][1] * inv_s[1][0];
    kalman_gain[row][1] =
        predicted_covariance[row][0] * inv_s[0][1] +
        predicted_covariance[row][1] * inv_s[1][1];
  }

  const double innovation[2] = {
      measurement_x - predicted_state[0],
      measurement_y - predicted_state[1]};
  for (int row = 0; row < 4; ++row) {
    state_[row] = predicted_state[row] +
        kalman_gain[row][0] * innovation[0] +
        kalman_gain[row][1] * innovation[1];
  }
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      covariance_[row][column] = predicted_covariance[row][column] -
          kalman_gain[row][0] * predicted_covariance[0][column] -
          kalman_gain[row][1] * predicted_covariance[1][column];
    }
  }

  localToLatLon(origin_lat_, origin_lon_, state_[0], state_[1], result.lat, result.lon);
  result.filtered = true;
  return result;
}

// 保存直线 P 控制器的航向、横向误差增益和速度限幅。
StraightLinePController::StraightLinePController(const TrackingParameters& parameters)
    : parameters_(parameters) {}

// 计算相对目标线的横向误差、航向误差和限幅后的转向速度。
TrackingCommand StraightLinePController::compute(
    const double start_lat,
    const double start_lon,
    const double end_lat,
    const double end_lon,
    const double current_lat,
    const double current_lon,
    const double vehicle_heading,
    const double target_heading,
    const double raw_lat,
    const double raw_lon) const {
  TrackingCommand result;
  result.raw_lat = raw_lat;
  result.raw_lon = raw_lon;
  result.filtered_lat = current_lat;
  result.filtered_lon = current_lon;
  result.source = "straight_line_p_control";

  double end_x = 0.0;
  double end_y = 0.0;
  double current_x = 0.0;
  double current_y = 0.0;
  latLonToLocal(start_lat, start_lon, end_lat, end_lon, end_x, end_y);
  latLonToLocal(start_lat, start_lon, current_lat, current_lon, current_x, current_y);
  const double path_length = std::hypot(end_x, end_y);
  if (path_length <= 1e-6) {
    result.source = "straight_line_p_degenerate_path";
    return result;
  }

  const double unit_x = end_x / path_length;
  const double unit_y = end_y / path_length;
  const double projection = current_x * unit_x + current_y * unit_y;
  result.cte_m = unit_y * current_x - unit_x * current_y;
  result.signed_remaining_m = path_length - projection;
  result.distance_to_target_m = std::hypot(end_x - current_x, end_y - current_y);
  result.heading_error_deg = normalize_heading_delta(target_heading, vehicle_heading);
  if (result.distance_to_target_m < 1.0) {
    const double limit = std::abs(parameters_.short_range_heading_limit_deg);
    result.heading_error_deg = std::max(
        -limit, std::min(limit, result.heading_error_deg));
  }
  const double output = result.heading_error_deg * parameters_.heading_gain -
      result.cte_m * parameters_.cte_gain;
  const auto rounded = static_cast<std::int32_t>(std::llround(output));
  result.z_speed = std::max(
      -std::abs(parameters_.max_z_speed),
      std::min(std::abs(parameters_.max_z_speed), rounded));
  return result;
}

// 先滤波 RTK 坐标，再计算一次完整的直线跟踪指令。
TrackingCommand build_tracking_command(
    RtkKalmanFilter2D& filter,
    const StraightLinePController& controller,
    const double start_lat,
    const double start_lon,
    const double end_lat,
    const double end_lon,
    const double current_lat,
    const double current_lon,
    const double vehicle_heading,
    const double target_heading,
    const double timestamp) {
  const auto filtered = filter.update(current_lat, current_lon, timestamp);
  auto result = controller.compute(
      start_lat,
      start_lon,
      end_lat,
      end_lon,
      filtered.lat,
      filtered.lon,
      vehicle_heading,
      target_heading,
      current_lat,
      current_lon);
  result.source = "kalman_p_control";
  return result;
}

}  // namespace control
}  // namespace cleanbot
