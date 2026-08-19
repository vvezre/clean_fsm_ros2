#pragma once

#include <vector>

#include "cleanbot_modeling/model_types.hpp"

// 文件作用：声明模型经纬度与局部厘米坐标、航向和多边形的几何计算工具。
namespace cleanbot {
namespace modeling {

// 将经纬度投影到以模型原点为基准的局部厘米坐标。
Point2d lat_lon_to_local_cm(
    double origin_lat,
    double origin_lon,
    double lat,
    double lon);

// 首个有效点建立模型原点，并计算当前点在统一局部坐标系中的坐标。
bool set_model_point_local_coordinates(
    CleaningModel& model,
    ModelPoint& point);

// 将局部厘米坐标反投影为经纬度。
void local_cm_to_lat_lon(
    double origin_lat,
    double origin_lon,
    const Point2d& origin,
    const Point2d& point,
    double& lat,
    double& lon);

// 判断多边形是否无自交且满足简单多边形条件。
bool is_simple_polygon(const std::vector<Point2d>& polygon);
// 计算两个局部坐标点之间的直线距离（厘米）。
double point_distance_cm(const Point2d& start, const Point2d& end);
// 将航向角归一化到标准范围。
double normalize_heading_deg(double heading_deg);
// 计算由起点指向终点的局部平面航向角。
double heading_from_points_deg(const Point2d& start, const Point2d& end);
// 计算并归一化目标航向相对当前航向的最短转向角。
double normalize_turn_deg(double target_heading_deg, double current_heading_deg);

}  // namespace modeling
}  // namespace cleanbot
