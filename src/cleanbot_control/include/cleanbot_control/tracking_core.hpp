#ifndef CLEANBOT_CONTROL__TRACKING_CORE_HPP_
#define CLEANBOT_CONTROL__TRACKING_CORE_HPP_

#include <cstdint>
#include <string>

// 文件作用：声明 RTK 点位滤波、直线跟踪与跟踪指令生成的无 ROS 核心算法。
namespace cleanbot {
namespace control {

// RTK 滤波和直线跟踪控制器的可调参数。
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

// 记录原始与滤波后 RTK 坐标及其时间戳。
struct FilteredRtkPoint {
  double raw_lat{0.0};
  double raw_lon{0.0};
  double lat{0.0};
  double lon{0.0};
  double timestamp{0.0};
  bool filtered{false};
};

// 直线跟踪算法计算出的几何误差和转向控制结果。
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

// 将两个航向角的差值归一化到最短旋转方向。
double normalize_heading_delta(double target_heading, double current_heading);
// 根据距离、剩余投影距离和横向误差判断是否到达目标点。
bool should_finish_point_to_point(
    double distance_to_target,
    double signed_remaining,
    double cte,
    double target_tolerance_m = 0.03,
    double cte_tolerance_m = 0.30);

class RtkKalmanFilter2D {
 public:
  // 使用给定噪声参数创建二维 RTK 卡尔曼滤波器。
  explicit RtkKalmanFilter2D(const TrackingParameters& parameters = TrackingParameters());
  // 清除滤波器状态，下一次更新将重新初始化原点。
  void reset();
  // 输入新的经纬度观测值并返回滤波后的坐标。
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
  // 使用给定增益和限幅参数创建直线 P 控制器。
  explicit StraightLinePController(
      const TrackingParameters& parameters = TrackingParameters());

  // 计算当前位置相对目标直线的误差和转向速度指令。
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

// 组合滤波器和控制器，生成一次完整的直线跟踪指令。
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
