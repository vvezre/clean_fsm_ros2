#include "cleanbot_modeling/region_recognizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "cleanbot_modeling/geometry.hpp"

namespace cleanbot {
namespace modeling {
namespace region_recognizer_detail {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

struct BoundaryCandidate {
  std::vector<std::size_t> indexes;
  bool reordered{false};
  double confidence{1.0};
};

Point2d recognition_xy(const ModelPoint& point) {
  return {point.x_cm, point.y_cm};
}

double cross(const Point2d& a, const Point2d& b, const Point2d& c) {
  return (b.x_cm - a.x_cm) * (c.y_cm - a.y_cm) -
      (b.y_cm - a.y_cm) * (c.x_cm - a.x_cm);
}

double polygon_area_cm2(const std::vector<Point2d>& polygon) {
  double twice_area = 0.0;
  for (std::size_t index = 0u; index < polygon.size(); ++index) {
    const auto& current = polygon[index];
    const auto& next = polygon[(index + 1u) % polygon.size()];
    twice_area += current.x_cm * next.y_cm - next.x_cm * current.y_cm;
  }
  return std::abs(twice_area) * 0.5;
}

std::vector<Point2d> polygon_from_indexes(
    const ModelGroup& group,
    const std::vector<std::size_t>& indexes) {
  std::vector<Point2d> polygon;
  polygon.reserve(indexes.size());
  for (const auto index : indexes) {
    polygon.push_back(recognition_xy(group.points[index]));
  }
  return polygon;
}

double signed_turn_deg(
    const Point2d& previous,
    const Point2d& current,
    const Point2d& next) {
  const double incoming_x = current.x_cm - previous.x_cm;
  const double incoming_y = current.y_cm - previous.y_cm;
  const double outgoing_x = next.x_cm - current.x_cm;
  const double outgoing_y = next.y_cm - current.y_cm;
  return std::atan2(
      incoming_x * outgoing_y - incoming_y * outgoing_x,
      incoming_x * outgoing_x + incoming_y * outgoing_y) *
      180.0 / kPi;
}

double point_to_segment_distance_cm(
    const Point2d& point,
    const Point2d& start,
    const Point2d& end) {
  const double dx = end.x_cm - start.x_cm;
  const double dy = end.y_cm - start.y_cm;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= kEpsilon) {
    return point_distance_cm(point, start);
  }
  const double projection = std::max(
      0.0,
      std::min(
          1.0,
          ((point.x_cm - start.x_cm) * dx +
           (point.y_cm - start.y_cm) * dy) /
              length_squared));
  const Point2d closest{
      start.x_cm + projection * dx,
      start.y_cm + projection * dy};
  return point_distance_cm(point, closest);
}

double point_to_polygon_distance_cm(
    const Point2d& point,
    const std::vector<Point2d>& polygon) {
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0u; index < polygon.size(); ++index) {
    distance = std::min(
        distance,
        point_to_segment_distance_cm(
            point,
            polygon[index],
            polygon[(index + 1u) % polygon.size()]));
  }
  return distance;
}

bool point_inside_polygon(
    const Point2d& point,
    const std::vector<Point2d>& polygon) {
  bool inside = false;
  for (std::size_t current = 0u, previous = polygon.size() - 1u;
       current < polygon.size();
       previous = current++) {
    const auto& a = polygon[current];
    const auto& b = polygon[previous];
    const bool crosses =
        ((a.y_cm > point.y_cm) != (b.y_cm > point.y_cm)) &&
        (point.x_cm <
         (b.x_cm - a.x_cm) * (point.y_cm - a.y_cm) /
                 (b.y_cm - a.y_cm) +
             a.x_cm);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

std::size_t convex_hull_vertex_count(const std::vector<Point2d>& points) {
  if (points.size() <= 2u) {
    return points.size();
  }
  std::vector<Point2d> sorted = points;
  std::sort(
      sorted.begin(), sorted.end(),
      [](const Point2d& left, const Point2d& right) {
        if (left.x_cm != right.x_cm) {
          return left.x_cm < right.x_cm;
        }
        return left.y_cm < right.y_cm;
      });
  std::vector<Point2d> hull;
  hull.reserve(sorted.size() * 2u);
  for (const auto& point : sorted) {
    while (hull.size() >= 2u &&
           cross(hull[hull.size() - 2u], hull.back(), point) <= kEpsilon) {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const std::size_t lower_size = hull.size();
  for (auto iterator = sorted.rbegin() + 1u;
       iterator != sorted.rend(); ++iterator) {
    while (hull.size() > lower_size &&
           cross(hull[hull.size() - 2u], hull.back(), *iterator) <= kEpsilon) {
      hull.pop_back();
    }
    hull.push_back(*iterator);
  }
  if (!hull.empty()) {
    hull.pop_back();
  }
  return hull.size();
}

bool reorder_boundary_by_geometry(
    const ModelGroup& group,
    BoundaryCandidate& candidate) {
  Point2d centroid;
  for (const auto index : candidate.indexes) {
    centroid.x_cm += group.points[index].x_cm;
    centroid.y_cm += group.points[index].y_cm;
  }
  centroid.x_cm /= static_cast<double>(candidate.indexes.size());
  centroid.y_cm /= static_cast<double>(candidate.indexes.size());

  std::sort(
      candidate.indexes.begin(), candidate.indexes.end(),
      [&group, &centroid](const std::size_t left, const std::size_t right) {
        const auto& left_point = group.points[left];
        const auto& right_point = group.points[right];
        const double left_angle = std::atan2(
            left_point.y_cm - centroid.y_cm,
            left_point.x_cm - centroid.x_cm);
        const double right_angle = std::atan2(
            right_point.y_cm - centroid.y_cm,
            right_point.x_cm - centroid.x_cm);
        if (std::abs(left_angle - right_angle) > kEpsilon) {
          return left_angle < right_angle;
        }
        return point_distance_cm(centroid, recognition_xy(left_point)) <
            point_distance_cm(centroid, recognition_xy(right_point));
      });
  const auto polygon = polygon_from_indexes(group, candidate.indexes);
  if (!is_simple_polygon(polygon)) {
    return false;
  }
  candidate.reordered = true;
  candidate.confidence =
      convex_hull_vertex_count(polygon) == polygon.size() ? 0.98 : 0.75;
  return true;
}

bool has_duplicate_points(
    const ModelGroup& group,
    const BoundaryCandidate& candidate,
    const double tolerance_cm) {
  for (std::size_t left = 0u; left < candidate.indexes.size(); ++left) {
    for (std::size_t right = left + 1u;
         right < candidate.indexes.size(); ++right) {
      if (point_distance_cm(
              recognition_xy(group.points[candidate.indexes[left]]),
              recognition_xy(group.points[candidate.indexes[right]])) <=
          tolerance_cm) {
        return true;
      }
    }
  }
  return false;
}

RecognitionResult recognition_error_result(
    const ModelGroup& group,
    const std::string& code,
    const std::string& message) {
  RecognitionResult result;
  result.code = code;
  result.message = message;
  result.status = "invalid";
  result.needs_confirmation = true;
  result.group = group;
  result.group.sub_areas.clear();
  result.group.connectors.clear();
  result.group.recognition_status = result.status;
  result.group.recognition_message = message;
  result.group.recognition_confidence = 0.0;
  return result;
}

bool segment_crosses_polygon(
    const Point2d& start,
    const Point2d& end,
    const std::vector<Point2d>& polygon) {
  for (int step = 1; step < 20; ++step) {
    const double ratio = static_cast<double>(step) / 20.0;
    const Point2d sample{
        start.x_cm + (end.x_cm - start.x_cm) * ratio,
        start.y_cm + (end.y_cm - start.y_cm) * ratio};
    if (point_inside_polygon(sample, polygon)) {
      return true;
    }
  }
  return false;
}

}  // namespace region_recognizer_detail

using namespace region_recognizer_detail;

RecognitionResult recognize_group(
    const ModelGroup& input,
    const RecognitionOptions& options) {
  ModelGroup group = input;
  std::sort(
      group.points.begin(), group.points.end(),
      [](const ModelPoint& left, const ModelPoint& right) {
        return left.sequence < right.sequence;
      });
  group.sub_areas.clear();
  group.connectors.clear();

  for (auto& point : group.points) {
    if (point.capture_type.empty()) {
      point.capture_type = "boundary";
    }
    if (point.capture_type != "boundary" &&
        point.capture_type != "connection") {
      return recognition_error_result(
          group,
          "CAPTURE_TYPE_INVALID",
          "point capture type must be boundary or connection");
    }
    point.roles.clear();
    if (point.capture_type == "connection") {
      point.role = "connection_point";
      point.roles.push_back("sub_area_connector");
    } else {
      point.role = "unknown";
    }
  }

  std::vector<BoundaryCandidate> boundaries;
  std::vector<std::pair<std::size_t, std::size_t>> connector_pairs;
  std::size_t cursor = 0u;
  while (cursor < group.points.size()) {
    BoundaryCandidate boundary;
    while (cursor < group.points.size() &&
           group.points[cursor].capture_type == "boundary") {
      boundary.indexes.push_back(cursor++);
    }
    if (boundary.indexes.size() < 4u) {
      return recognition_error_result(
          group,
          "INSUFFICIENT_BOUNDARY_POINTS",
          "each sub-area requires at least four boundary points");
    }
    boundaries.push_back(std::move(boundary));
    if (cursor == group.points.size()) {
      break;
    }

    const std::size_t connection_start = cursor;
    while (cursor < group.points.size() &&
           group.points[cursor].capture_type == "connection") {
      ++cursor;
    }
    if (cursor - connection_start != 2u) {
      return recognition_error_result(
          group,
          "CONNECTION_POINTS_UNPAIRED",
          "exactly two consecutive connection points are required");
    }
    if (cursor == group.points.size()) {
      return recognition_error_result(
          group,
          "CONNECTION_WITHOUT_ADJACENT_AREA",
          "connection points require a following sub-area");
    }
    connector_pairs.emplace_back(connection_start, connection_start + 1u);
  }

  if (boundaries.empty()) {
    return recognition_error_result(
        group,
        "INSUFFICIENT_BOUNDARY_POINTS",
        "at least one sub-area is required");
  }
  if (connector_pairs.size() + 1u != boundaries.size()) {
    return recognition_error_result(
        group,
        "CONNECTION_WITHOUT_ADJACENT_AREA",
        "each connection pair must join adjacent sub-areas");
  }

  double overall_confidence = 1.0;
  for (std::size_t boundary_index = 0u;
       boundary_index < boundaries.size(); ++boundary_index) {
    auto& boundary = boundaries[boundary_index];
    if (has_duplicate_points(
            group, boundary, options.duplicate_tolerance_cm)) {
      return recognition_error_result(
          group,
          "DUPLICATE_BOUNDARY_POINT",
          "boundary contains duplicate or near-duplicate points");
    }
    auto polygon = polygon_from_indexes(group, boundary.indexes);
    if (!is_simple_polygon(polygon)) {
      if (!reorder_boundary_by_geometry(group, boundary)) {
        return recognition_error_result(
            group,
            "POLYGON_SELF_INTERSECTS",
            "boundary cannot be restored to a simple polygon");
      }
      polygon = polygon_from_indexes(group, boundary.indexes);
    }
    if (polygon_area_cm2(polygon) < options.minimum_area_cm2) {
      return recognition_error_result(
          group,
          "POLYGON_AREA_TOO_SMALL",
          "boundary polygon area is below the configured minimum");
    }
    overall_confidence = std::min(overall_confidence, boundary.confidence);

    ModelSubArea area;
    area.id = group.id + "-sa" + std::to_string(boundary_index + 1u);
    area.name = "sub-area " + std::to_string(boundary_index + 1u);
    for (std::size_t index = 0u; index < boundary.indexes.size(); ++index) {
      const auto point_index = boundary.indexes[index];
      auto& point = group.points[point_index];
      const auto& previous = polygon[
          (index + polygon.size() - 1u) % polygon.size()];
      const auto& current = polygon[index];
      const auto& next = polygon[(index + 1u) % polygon.size()];
      const double turn = std::abs(signed_turn_deg(previous, current, next));
      point.role = turn < options.assist_turn_threshold_deg
          ? "boundary_assist"
          : "boundary_corner";
      area.point_ids.push_back(point.id);
    }
    group.sub_areas.push_back(std::move(area));
  }

  for (std::size_t connector_index = 0u;
       connector_index < connector_pairs.size(); ++connector_index) {
    const auto pair = connector_pairs[connector_index];
    const auto start = recognition_xy(group.points[pair.first]);
    const auto end = recognition_xy(group.points[pair.second]);
    const double length = point_distance_cm(start, end);
    if (length < options.minimum_connector_length_cm) {
      return recognition_error_result(
          group,
          "CONNECTOR_TOO_SHORT",
          "connector length is below the configured minimum");
    }
    const auto from_polygon = polygon_from_indexes(
        group, boundaries[connector_index].indexes);
    const auto to_polygon = polygon_from_indexes(
        group, boundaries[connector_index + 1u].indexes);
    if (point_to_polygon_distance_cm(start, from_polygon) >
            options.maximum_connector_endpoint_distance_cm ||
        point_to_polygon_distance_cm(end, to_polygon) >
            options.maximum_connector_endpoint_distance_cm) {
      return recognition_error_result(
          group,
          "CONNECTOR_ENDPOINT_TOO_FAR",
          "connector endpoints are not close to adjacent sub-areas");
    }
    for (std::size_t third = 0u; third < boundaries.size(); ++third) {
      if (third == connector_index || third == connector_index + 1u) {
        continue;
      }
      if (segment_crosses_polygon(
              start,
              end,
              polygon_from_indexes(group, boundaries[third].indexes))) {
        return recognition_error_result(
            group,
            "CONNECTOR_CROSSES_SUB_AREA",
            "connector crosses an unrelated sub-area");
      }
    }

    ModelConnector connector;
    connector.id = group.id + "-c" + std::to_string(connector_index + 1u);
    connector.start_point_id = group.points[pair.first].id;
    connector.end_point_id = group.points[pair.second].id;
    connector.from_sub_area_id = group.sub_areas[connector_index].id;
    connector.to_sub_area_id = group.sub_areas[connector_index + 1u].id;
    connector.length_cm = length;
    group.connectors.push_back(std::move(connector));
  }

  const bool confirmed =
      overall_confidence >= options.unordered_auto_confirm_confidence;
  for (auto& area : group.sub_areas) {
    area.confirmed = confirmed;
  }
  for (auto& connector : group.connectors) {
    connector.confirmed = confirmed;
  }
  group.recognition_status = confirmed ? "recognized" : "needs_confirmation";
  group.recognition_message = confirmed
      ? "sub-areas and connectors recognized"
      : "boundary candidate requires manual confirmation";
  group.recognition_confidence = overall_confidence;

  RecognitionResult result;
  result.success = true;
  result.code = confirmed ? "OK" : "NEEDS_CONFIRMATION";
  result.message = group.recognition_message;
  result.status = group.recognition_status;
  result.needs_confirmation = !confirmed;
  result.group = std::move(group);
  return result;
}

}  // namespace modeling
}  // namespace cleanbot
