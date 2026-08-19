#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// 文件作用：声明航点循环顺序、航向计算和直线段计划生成工具。
namespace cleanbot {
namespace mission {

// 一次航点序列推进后需要到达的航点及已完成循环次数。
struct WaypointTarget {
  std::size_t waypoint_index{0u};
  std::uint32_t completed_loop{0u};
};

// 两个航点间直线段的起终坐标、航向和转向角。
struct WaypointSegmentPlan {
  double start_lat{0.0};
  double start_lon{0.0};
  double end_lat{0.0};
  double end_lon{0.0};
  double heading_deg{0.0};
  double turn_angle_deg{0.0};
};

// 生成与旧版 Python 相同的闭环顺序：先到达一次 P0，再重复 P1...Pn→P0。
class WaypointSequence {
 public:
  // 使用航点数量、是否闭环和循环次数创建序列生成器。
  WaypointSequence(
      std::size_t waypoint_count,
      bool loop,
      std::uint32_t loop_count);

  // 返回输入配置是否可生成有效序列。
  bool valid() const;
  // 返回配置无效时的稳定错误码。
  const std::string& error_code() const;
  // 取出下一个目标航点；序列耗尽时返回 false。
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

// 校验经纬度是否处于可用地理范围。
bool validate_waypoint(double lat, double lon);
// 计算由起点指向终点的地理航向角。
double calculate_heading_deg(
    double start_lat,
    double start_lon,
    double end_lat,
    double end_lon);
// 将目标与当前航向的差归一化为最短转向角。
double normalize_turn_angle_deg(double target_heading, double current_heading);
// 根据起点、当前航向和终点生成一段航点计划。
WaypointSegmentPlan build_waypoint_segment(
    double start_lat,
    double start_lon,
    double current_heading,
    double end_lat,
    double end_lon);

}  // namespace mission
}  // namespace cleanbot
