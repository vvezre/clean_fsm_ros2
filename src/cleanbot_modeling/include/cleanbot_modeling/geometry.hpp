#pragma once

#include <vector>

#include "cleanbot_modeling/model_types.hpp"

namespace cleanbot {
namespace modeling {

Point2d lat_lon_to_local_cm(
    double origin_lat,
    double origin_lon,
    double lat,
    double lon);

// 第一个有效点建立模型原点，后续所有区域组的点都投影到同一局部坐标系。
bool set_model_point_local_coordinates(
    CleaningModel& model,
    ModelPoint& point);

void local_cm_to_lat_lon(
    double origin_lat,
    double origin_lon,
    const Point2d& origin,
    const Point2d& point,
    double& lat,
    double& lon);

bool is_simple_polygon(const std::vector<Point2d>& polygon);
double point_distance_cm(const Point2d& start, const Point2d& end);
double normalize_heading_deg(double heading_deg);
double heading_from_points_deg(const Point2d& start, const Point2d& end);
double normalize_turn_deg(double target_heading_deg, double current_heading_deg);

}  // namespace modeling
}  // namespace cleanbot
