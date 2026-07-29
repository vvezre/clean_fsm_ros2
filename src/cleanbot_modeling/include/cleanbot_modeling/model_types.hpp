#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cleanbot {
namespace modeling {

struct Point2d {
  double x_cm{0.0};
  double y_cm{0.0};
};

struct ModelPoint {
  std::string id;
  std::size_t sequence{0u};
  double lat{0.0};
  double lon{0.0};
  double x_cm{0.0};
  double y_cm{0.0};
  double heading_deg{0.0};
  bool heading_valid{false};
  // 用户采集时指定的原始类型，只允许boundary或connection。
  std::string capture_type{"boundary"};
  std::string role{"unknown"};
  std::vector<std::string> roles;
  std::size_t sample_count{0u};
  double sample_radius_m{0.0};
  std::uint8_t fix_quality{0u};
  double gga_age_sec{0.0};
  std::string source{"rtk_mean"};
};

struct ModelSubArea {
  std::string id;
  std::string name;
  std::vector<std::string> point_ids;
  bool confirmed{false};
};

struct ModelConnector {
  std::string id;
  std::string type{"sub_area_connector"};
  std::string start_point_id;
  std::string end_point_id;
  std::string from_sub_area_id;
  std::string to_sub_area_id;
  double length_cm{0.0};
  bool confirmed{false};
};

struct ModelGroup {
  std::string id;
  std::string name;
  std::uint32_t area_number{1u};
  std::string sweep_mode{"auto"};
  double sweep_angle_deg{0.0};
  std::string recognition_status{"unrecognized"};
  std::string recognition_message;
  double recognition_confidence{0.0};
  std::vector<ModelPoint> points;
  std::vector<ModelSubArea> sub_areas;
  std::vector<ModelConnector> connectors;
};

struct CleaningModel {
  std::string id;
  std::string name;
  std::uint64_t version{0u};
  std::string status{"draft"};
  // 整个模型唯一的局部坐标原点；所有区域组的x_cm/y_cm都相对它计算。
  double origin_lat{0.0};
  double origin_lon{0.0};
  bool origin_valid{false};
  bool recognition_confirmed{false};
  bool preview_confirmed{false};
  std::vector<ModelGroup> groups;
};

struct PlanSegment {
  std::size_t index{0u};
  std::string id;
  std::uint8_t segment_type{0u};
  std::string group_id;
  std::string sub_area_id;
  std::string source_lane_id;
  Point2d start;
  Point2d end;
  double start_lat{0.0};
  double start_lon{0.0};
  double end_lat{0.0};
  double end_lon{0.0};
  double heading_deg{0.0};
  double turn_angle_deg{0.0};
  std::int32_t speed{0};
  std::uint8_t mode{0u};
  bool brush_enabled{false};
};

struct CleaningPlan {
  std::string id;
  std::string model_id;
  std::uint64_t model_version{0u};
  std::string plan_hash;
  std::uint64_t generated_at{0u};
  double brush_width_cm{0.0};
  double minimum_overlap_cm{0.0};
  double actual_overlap_cm{0.0};
  std::size_t cleaning_lane_count{0u};
  std::size_t transfer_segment_count{0u};
  double total_length_cm{0.0};
  bool preview_confirmed{false};
  std::vector<PlanSegment> segments;
};

}  // namespace modeling
}  // namespace cleanbot
