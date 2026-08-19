/*
 * 文件作用：任务计划构建实现：把几何区域转换为可执行任务段。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_modeling/task_plan_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "cleanbot_modeling/coverage_planner.hpp"
#include "cleanbot_modeling/geometry.hpp"

namespace cleanbot {
namespace modeling {
namespace task_plan_builder_detail {

constexpr std::uint8_t kPlanSegmentCleaning = 1u;
constexpr std::uint8_t kPlanSegmentTransfer = 2u;
constexpr double kPlanPointEpsilonCm = 1e-5;

// 构造任务计划生成失败结果和稳定错误码。
TaskPlanResult plan_failure(
    const std::string& code,
    const std::string& message) {
  TaskPlanResult result;
  result.code = code;
  result.message = message;
  return result;
}

// 按点标识在区域组中查找模型点。
const ModelPoint* find_point(
    const ModelGroup& group,
    const std::string& point_id) {
  const auto found = std::find_if(
      group.points.begin(), group.points.end(),
      // 筛选谓词作用：检查当前元素是否符合查找、确认或删除条件。
      [&point_id](const ModelPoint& point) {
        return point.id == point_id;
      });
  return found == group.points.end() ? nullptr : &(*found);
}

// 在计划拼接容差内判断两个局部坐标是否相同。
bool same_plan_point(const Point2d& left, const Point2d& right) {
  return point_distance_cm(left, right) <= kPlanPointEpsilonCm;
}

// 根据区域边界主方向自动选择覆盖扫掠角。
double automatic_sweep_angle(
    const ModelGroup& group,
    const ModelSubArea& area) {
  if (area.point_ids.size() < 2u) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto* first = find_point(group, area.point_ids[0]);
  const auto* second = find_point(group, area.point_ids[1]);
  if (first == nullptr || second == nullptr) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return normalize_heading_deg(
      heading_from_points_deg(
          {first->x_cm, first->y_cm},
          {second->x_cm, second->y_cm}) +
      90.0);
}

// 查找连接指定两个子区域的已确认连接段。
const ModelConnector* find_connector(
    const ModelGroup& group,
    const std::string& from_sub_area_id,
    const std::string& to_sub_area_id) {
  const auto found = std::find_if(
      group.connectors.begin(), group.connectors.end(),
      // 筛选谓词作用：检查当前元素是否符合查找、确认或删除条件。
      [&from_sub_area_id, &to_sub_area_id](const ModelConnector& connector) {
        return connector.from_sub_area_id == from_sub_area_id &&
            connector.to_sub_area_id == to_sub_area_id;
      });
  return found == group.connectors.end() ? nullptr : &(*found);
}

// 根据连接关系确定所有子区域的可连续访问顺序。
bool resolve_sub_area_chain(
    const ModelGroup& group,
    std::vector<std::size_t>& ordered_indexes,
    std::string& error_code,
    std::string& error_message) {
  const std::size_t count = group.sub_areas.size();
  ordered_indexes.clear();
  if (count == 1u) {
    if (!group.connectors.empty()) {
      error_code = "CONNECTOR_GRAPH_INVALID";
      error_message = "a single sub-area cannot have a connector";
      return false;
    }
    ordered_indexes.push_back(0u);
    return true;
  }
  if (group.connectors.size() != count - 1u) {
    error_code = "SUB_AREA_CONNECTOR_MISSING";
    error_message = "every adjacent sub-area pair needs one confirmed connector";
    return false;
  }

  std::map<std::string, std::size_t> indexes;
  for (std::size_t index = 0u; index < count; ++index) {
    const auto& area = group.sub_areas[index];
    if (area.id.empty() || !indexes.emplace(area.id, index).second) {
      error_code = "SUB_AREA_ID_INVALID";
      error_message = "sub-area ids must be unique and non-empty";
      return false;
    }
  }
  std::vector<std::size_t> outgoing(count, count);
  std::vector<std::size_t> incoming(count, 0u);
  for (const auto& connector : group.connectors) {
    if (!connector.confirmed) {
      error_code = "SUB_AREA_CONNECTOR_NOT_CONFIRMED";
      error_message = "all sub-area connectors must be confirmed";
      return false;
    }
    const auto from = indexes.find(connector.from_sub_area_id);
    const auto to = indexes.find(connector.to_sub_area_id);
    if (from == indexes.end() || to == indexes.end() || from->second == to->second) {
      error_code = "CONNECTOR_SUB_AREA_MISSING";
      error_message = "connector references invalid sub-area ids";
      return false;
    }
    if (outgoing[from->second] != count || incoming[to->second] != 0u) {
      error_code = "CONNECTOR_GRAPH_INVALID";
      error_message = "sub-area connector graph contains a branch";
      return false;
    }
    outgoing[from->second] = to->second;
    ++incoming[to->second];
  }

  std::size_t root = count;
  for (std::size_t index = 0u; index < count; ++index) {
    if (incoming[index] == 0u) {
      if (root != count) {
        error_code = "CONNECTOR_GRAPH_INVALID";
        error_message = "sub-area connector graph must have one chain root";
        return false;
      }
      root = index;
    }
  }
  if (root == count) {
    error_code = "CONNECTOR_GRAPH_INVALID";
    error_message = "sub-area connector graph contains a cycle";
    return false;
  }
  std::vector<bool> visited(count, false);
  std::size_t current = root;
  while (current != count) {
    if (visited[current]) {
      error_code = "CONNECTOR_GRAPH_INVALID";
      error_message = "sub-area connector graph contains a cycle";
      return false;
    }
    visited[current] = true;
    ordered_indexes.push_back(current);
    current = outgoing[current];
  }
  if (ordered_indexes.size() != count) {
    error_code = "CONNECTOR_GRAPH_INVALID";
    error_message = "sub-area connector graph is not a complete chain";
    return false;
  }
  return true;
}

// 从模型显式原点或有效采集点确定地理坐标原点。
bool find_origin(
    const CleaningModel& model,
    ModelPoint& origin) {
  if (!model.origin_valid ||
      !std::isfinite(model.origin_lat) || !std::isfinite(model.origin_lon) ||
      model.origin_lat < -90.0 || model.origin_lat > 90.0 ||
      model.origin_lon < -180.0 || model.origin_lon > 180.0) {
    return false;
  }
  origin.lat = model.origin_lat;
  origin.lon = model.origin_lon;
  origin.x_cm = 0.0;
  origin.y_cm = 0.0;
  return true;
}

// 将计划段局部坐标转换并填充起终点经纬度。
void fill_geo(
    PlanSegment& segment,
    const ModelPoint& origin) {
  const Point2d origin_xy{origin.x_cm, origin.y_cm};
  local_cm_to_lat_lon(
      origin.lat,
      origin.lon,
      origin_xy,
      segment.start,
      segment.start_lat,
      segment.start_lon);
  local_cm_to_lat_lon(
      origin.lat,
      origin.lon,
      origin_xy,
      segment.end,
      segment.end_lat,
      segment.end_lon);
}

// 构造带索引、类型、速度和几何信息的基础计划段。
PlanSegment make_segment(
    const std::uint8_t type,
    const std::string& group_id,
    const std::string& sub_area_id,
    const std::string& lane_id,
    const Point2d& start,
    const Point2d& end,
    const std::int32_t speed,
    const ModelPoint& origin,
    const double previous_heading,
    const std::size_t index) {
  PlanSegment segment;
  segment.index = index;
  segment.id = (type == kPlanSegmentCleaning ? "clean-" : "transfer-") +
      std::to_string(index + 1u);
  segment.segment_type = type;
  segment.group_id = group_id;
  segment.sub_area_id = sub_area_id;
  segment.source_lane_id = lane_id;
  segment.start = start;
  segment.end = end;
  segment.heading_deg = heading_from_points_deg(start, end);
  segment.turn_angle_deg =
      normalize_turn_deg(segment.heading_deg, previous_heading);
  segment.speed = speed;
  segment.mode = type;
  segment.brush_enabled = type == kPlanSegmentCleaning;
  fill_geo(segment, origin);
  return segment;
}

// 将影响计划内容的字段序列化为稳定指纹文本。
std::string plan_fingerprint(const CleaningPlan& plan) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(8)
         << plan.model_id << '|' << plan.model_version << '|'
         << plan.brush_width_cm << '|' << plan.minimum_overlap_cm;
  for (const auto& segment : plan.segments) {
    stream << '|' << static_cast<int>(segment.segment_type)
           << '|' << segment.group_id
           << '|' << segment.sub_area_id
           << '|' << segment.source_lane_id
           << '|' << segment.start.x_cm
           << '|' << segment.start.y_cm
           << '|' << segment.end.x_cm
           << '|' << segment.end.y_cm
           << '|' << segment.speed;
  }
  return stream.str();
}

// 使用 FNV-1a 计算计划指纹的十六进制哈希。
std::string fnv1a_hex(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ull;
  for (const unsigned char character : value) {
    hash ^= static_cast<std::uint64_t>(character);
    hash *= 1099511628211ull;
  }
  std::ostringstream stream;
  stream << std::hex << std::setw(16) << std::setfill('0') << hash;
  return stream.str();
}

}  // namespace task_plan_builder_detail

using namespace task_plan_builder_detail;

// 校验正式模型，生成转场、连接和清扫段并计算计划摘要。
TaskPlanResult build_task_plan(
    const CleaningModel& model,
    const double brush_width_cm,
    const double minimum_overlap_cm,
    const std::int32_t speed) {
  if (!model.recognition_confirmed) {
    return plan_failure(
        "MODEL_NOT_CONFIRMED",
        "model recognition must be confirmed before planning");
  }
  if (model.id.empty() || model.groups.empty()) {
    return plan_failure("MODEL_EMPTY", "model has no executable groups");
  }
  if (speed <= 0) {
    return plan_failure("PLAN_SPEED_INVALID", "plan speed must be positive");
  }

  ModelPoint origin;
  if (!find_origin(model, origin)) {
    return plan_failure(
        "MODEL_ORIGIN_MISSING",
        "model has no valid RTK origin");
  }

  CleaningPlan plan;
  plan.model_id = model.id;
  plan.model_version = model.version;
  plan.brush_width_cm = brush_width_cm;
  plan.minimum_overlap_cm = minimum_overlap_cm;
  plan.actual_overlap_cm = std::numeric_limits<double>::infinity();

  bool has_previous = false;
  Point2d previous_end;
  double previous_heading =
      origin.heading_valid ? origin.heading_deg : 0.0;
  std::string previous_group_id;
  std::string previous_area_id;

  const auto append_transfer = [&plan, &previous_end, &previous_heading,
                                speed, &origin](
      const std::string& group_id,
      const std::string& area_id,
      const Point2d& end) {
    if (same_plan_point(previous_end, end)) {
      return;
    }
    auto transfer = make_segment(
        kPlanSegmentTransfer,
        group_id,
        area_id,
        "",
        previous_end,
        end,
        speed,
        origin,
        previous_heading,
        plan.segments.size());
    previous_heading = transfer.heading_deg;
    plan.total_length_cm += point_distance_cm(transfer.start, transfer.end);
    plan.segments.push_back(std::move(transfer));
    ++plan.transfer_segment_count;
  };

  for (const auto& group : model.groups) {
    if (group.sub_areas.empty()) {
      return plan_failure(
          "SUB_AREA_MISSING",
          "model group has no recognized sub-area");
    }
    std::vector<std::size_t> area_order;
    std::string chain_error_code;
    std::string chain_error_message;
    if (!resolve_sub_area_chain(
            group,
            area_order,
            chain_error_code,
            chain_error_message)) {
      return plan_failure(chain_error_code, chain_error_message);
    }
    for (const auto area_index : area_order) {
      const auto& area = group.sub_areas[area_index];
      if (!area.confirmed) {
        return plan_failure(
            "SUB_AREA_NOT_CONFIRMED",
            "all sub-areas must be confirmed");
      }

      std::vector<Point2d> polygon;
      polygon.reserve(area.point_ids.size());
      for (const auto& point_id : area.point_ids) {
        const auto* point = find_point(group, point_id);
        if (point == nullptr) {
          return plan_failure(
              "SUB_AREA_POINT_MISSING",
              "sub-area references a missing point");
        }
        polygon.push_back({point->x_cm, point->y_cm});
      }

      const double sweep_angle =
          group.sweep_mode == "manual"
          ? normalize_heading_deg(group.sweep_angle_deg)
          : automatic_sweep_angle(group, area);
      if (!std::isfinite(sweep_angle)) {
        return plan_failure(
            "SWEEP_ANGLE_INVALID",
            "unable to determine cleaning direction");
      }

      const auto coverage = plan_coverage(
          polygon,
          sweep_angle,
          brush_width_cm,
          minimum_overlap_cm);
      if (!coverage.success) {
        return plan_failure(coverage.code, coverage.message);
      }
      plan.actual_overlap_cm =
          std::min(plan.actual_overlap_cm, coverage.actual_overlap_cm);

      for (std::size_t lane_index = 0u;
           lane_index < coverage.lanes.size();
           ++lane_index) {
        Point2d lane_start = coverage.lanes[lane_index].start;
        Point2d lane_end = coverage.lanes[lane_index].end;
        if (lane_index % 2u != 0u) {
          std::swap(lane_start, lane_end);
        }

        if (has_previous && previous_area_id == area.id) {
          append_transfer(group.id, area.id, lane_start);
        } else if (has_previous) {
          if (previous_group_id != group.id) {
            return plan_failure(
                "GROUP_CONNECTOR_MISSING",
                "different area groups require an explicit group connector");
          }
          const auto* connector = find_connector(
              group, previous_area_id, area.id);
          if (connector == nullptr || !connector->confirmed) {
            return plan_failure(
                "SUB_AREA_CONNECTOR_MISSING",
                "adjacent sub-areas require a confirmed connector");
          }
          const auto* connector_start =
              find_point(group, connector->start_point_id);
          const auto* connector_end =
              find_point(group, connector->end_point_id);
          if (connector_start == nullptr || connector_end == nullptr) {
            return plan_failure(
                "CONNECTOR_POINT_MISSING",
                "connector references a missing point");
          }
          append_transfer(
              group.id,
              previous_area_id,
              {connector_start->x_cm, connector_start->y_cm});
          const Point2d connector_start_xy{
              connector_start->x_cm, connector_start->y_cm};
          const Point2d connector_end_xy{
              connector_end->x_cm, connector_end->y_cm};
          if (!same_plan_point(connector_start_xy, connector_end_xy)) {
            auto connector_segment = make_segment(
                kPlanSegmentTransfer,
                group.id,
                previous_area_id,
                "",
                connector_start_xy,
                connector_end_xy,
                speed,
                origin,
                previous_heading,
                plan.segments.size());
            previous_heading = connector_segment.heading_deg;
            plan.total_length_cm += point_distance_cm(
                connector_segment.start, connector_segment.end);
            plan.segments.push_back(std::move(connector_segment));
            ++plan.transfer_segment_count;
          }
          previous_end = connector_end_xy;
          append_transfer(group.id, area.id, lane_start);
        }

        auto cleaning = make_segment(
            kPlanSegmentCleaning,
            group.id,
            area.id,
            coverage.lanes[lane_index].id,
            lane_start,
            lane_end,
            speed,
            origin,
            previous_heading,
            plan.segments.size());
        previous_heading = cleaning.heading_deg;
        previous_end = cleaning.end;
        has_previous = true;
        plan.total_length_cm +=
            point_distance_cm(cleaning.start, cleaning.end);
        plan.segments.push_back(std::move(cleaning));
        ++plan.cleaning_lane_count;
        previous_group_id = group.id;
        previous_area_id = area.id;
      }
    }
  }

  if (plan.cleaning_lane_count == 0u ||
      plan.cleaning_lane_count % 2u != 0u) {
    return plan_failure(
        "EVEN_CLEANING_LANES_REQUIRED",
        "generated cleaning lane count is not positive and even");
  }

  plan.plan_hash = fnv1a_hex(plan_fingerprint(plan));
  plan.id = model.id + "-v" + std::to_string(model.version) +
      "-" + plan.plan_hash.substr(0u, 8u);

  TaskPlanResult result;
  result.success = true;
  result.code = "OK";
  result.message = "cleaning plan generated";
  result.plan = std::move(plan);
  return result;
}

}  // namespace modeling
}  // namespace cleanbot
