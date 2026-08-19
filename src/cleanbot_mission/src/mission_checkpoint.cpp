/*
 * 文件作用：任务检查点实现：保存和恢复任务执行位置及相关状态。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_mission/mission_checkpoint.hpp"

#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <system_error>
#include <utility>

#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

#include <nlohmann/json.hpp>

namespace cleanbot {
namespace mission {
namespace {

using Json = nlohmann::json;

// 判断经纬度是否为有限数且位于合法地理范围内。
bool valid_coordinate(const double lat, const double lon) {
  return std::isfinite(lat) && std::isfinite(lon) &&
      lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

// 在调用方提供错误缓冲区时写入说明，并统一返回失败。
bool set_error(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

// 校验检查点标识、任务类型、进度范围以及全部几何数据。
bool validate_record(const MissionCheckpointRecord& record, std::string* error) {
  if (record.run_id.empty()) {
    return set_error(error, "checkpoint run_id is empty");
  }
  if (record.mission_kind != "cleaning" && record.mission_kind != "return_home") {
    return set_error(error, "checkpoint mission_kind is unsupported");
  }
  if (record.total_segments == 0u ||
      record.segments.size() != record.total_segments) {
    return set_error(error, "checkpoint segment count does not match the plan");
  }
  if (record.next_segment > record.total_segments) {
    return set_error(error, "checkpoint next_segment is out of range");
  }
  if (record.completed_segments > record.total_segments) {
    return set_error(error, "checkpoint completed_segments is out of range");
  }
  if (!record.loop && record.total_loops != 1u) {
    return set_error(error, "non-loop checkpoint must have total_loops=1");
  }
  if (record.total_loops != 0u && record.current_loop >= record.total_loops) {
    return set_error(error, "checkpoint current_loop is out of range");
  }
  if (record.target_valid && !valid_coordinate(record.target_lat, record.target_lon)) {
    return set_error(error, "checkpoint target coordinate is invalid");
  }
  if (record.mission_kind == "return_home" && !record.target_valid) {
    return set_error(error, "return-home checkpoint target is missing");
  }
  if (record.home_valid && !valid_coordinate(record.home_lat, record.home_lon)) {
    return set_error(error, "checkpoint home coordinate is invalid");
  }
  for (const auto& segment : record.segments) {
    if (!valid_coordinate(segment.start_lat, segment.start_lon) ||
        !valid_coordinate(segment.end_lat, segment.end_lon) ||
        !std::isfinite(segment.heading_deg) ||
        !std::isfinite(segment.turn_angle_deg) ||
        !std::isfinite(segment.turn_back_length) ||
        !std::isfinite(segment.back_length)) {
      return set_error(error, "checkpoint contains an invalid segment geometry");
    }
  }
  if (record.total_waypoints != record.waypoints.size()) {
    return set_error(error, "checkpoint waypoint count does not match the plan");
  }
  for (const auto& waypoint : record.waypoints) {
    if (!valid_coordinate(waypoint.lat, waypoint.lon)) {
      return set_error(error, "checkpoint contains an invalid waypoint coordinate");
    }
  }
  return true;
}

// 将临时检查点文件强制同步到磁盘，降低掉电后丢失风险。
bool sync_file_to_disk(const std::filesystem::path& path, std::string* error) {
#if defined(_WIN32)
  const int fd = ::_wopen(path.c_str(), _O_RDONLY | _O_BINARY);
#else
  const int fd = ::open(path.c_str(), O_RDONLY);
#endif
  if (fd < 0) {
    return set_error(error, std::string("failed to open checkpoint for sync: ") + std::strerror(errno));
  }

  int result = 0;
#if defined(_WIN32)
  result = ::_commit(fd);
  ::_close(fd);
#else
  result = ::fsync(fd);
  ::close(fd);
#endif
  if (result != 0) {
    return set_error(error, std::string("failed to sync checkpoint: ") + std::strerror(errno));
  }
  return true;
}

// 以平台原子替换操作将完整临时文件发布为正式检查点。
bool atomic_replace_file(
    const std::filesystem::path& temporary,
    const std::filesystem::path& destination,
    std::string* error) {
#if defined(_WIN32)
  if (!::MoveFileExW(
          temporary.c_str(), destination.c_str(),
          MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
    return set_error(error, "failed to atomically replace checkpoint file");
  }
#else
  if (::rename(temporary.c_str(), destination.c_str()) != 0) {
    return set_error(error, std::string("failed to atomically replace checkpoint file: ") + std::strerror(errno));
  }
  if (!destination.parent_path().empty()) {
    const int directory_fd = ::open(destination.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
    if (directory_fd >= 0) {
      ::fsync(directory_fd);
      ::close(directory_fd);
    }
  }
#endif
  return true;
}

// 将任务段消息序列化为检查点 JSON 对象。
Json task_segment_to_json(const cleanbot_interfaces::msg::TaskSegment& segment) {
  return Json{
      {"index", segment.index},
      {"id", segment.id},
      {"segment_type", static_cast<int>(segment.segment_type)},
      {"group_id", segment.group_id},
      {"sub_area_id", segment.sub_area_id},
      {"source_lane_id", segment.source_lane_id},
      {"start_lat", segment.start_lat},
      {"start_lon", segment.start_lon},
      {"end_lat", segment.end_lat},
      {"end_lon", segment.end_lon},
      {"heading_deg", segment.heading_deg},
      {"turn_angle_deg", segment.turn_angle_deg},
      {"speed", segment.speed},
      {"mode", static_cast<int>(segment.mode)},
      {"turn_back_length", segment.turn_back_length},
      {"back_length", segment.back_length},
  };
}

// 从检查点 JSON 对象恢复任务段消息。
cleanbot_interfaces::msg::TaskSegment task_segment_from_json(const Json& json) {
  cleanbot_interfaces::msg::TaskSegment segment;
  segment.index = json.value("index", 0u);
  segment.id = json.value("id", "");
  segment.segment_type = static_cast<std::uint8_t>(json.value("segment_type", 0));
  segment.group_id = json.value("group_id", "");
  segment.sub_area_id = json.value("sub_area_id", "");
  segment.source_lane_id = json.value("source_lane_id", "");
  segment.start_lat = json.value("start_lat", 0.0);
  segment.start_lon = json.value("start_lon", 0.0);
  segment.end_lat = json.value("end_lat", 0.0);
  segment.end_lon = json.value("end_lon", 0.0);
  segment.heading_deg = json.value("heading_deg", 0.0);
  segment.turn_angle_deg = json.value("turn_angle_deg", 0.0);
  segment.speed = json.value("speed", 0);
  segment.mode = static_cast<std::uint8_t>(json.value("mode", 0));
  segment.turn_back_length = json.value("turn_back_length", 0.0);
  segment.back_length = json.value("back_length", 0.0);
  return segment;
}

// 将航点消息序列化为检查点 JSON 对象。
Json waypoint_to_json(const cleanbot_interfaces::msg::Waypoint& waypoint) {
  return Json{
      {"lat", waypoint.lat},
      {"lon", waypoint.lon},
      {"id", waypoint.id},
  };
}

// 从检查点 JSON 对象恢复航点消息。
cleanbot_interfaces::msg::Waypoint waypoint_from_json(const Json& json) {
  cleanbot_interfaces::msg::Waypoint waypoint;
  waypoint.lat = json.value("lat", 0.0);
  waypoint.lon = json.value("lon", 0.0);
  waypoint.id = json.value("id", "");
  return waypoint;
}

// 将完整任务执行记录转换为稳定字段名的 JSON 数据。
Json record_to_json(const MissionCheckpointRecord& record) {
  Json json;
  json["runId"] = record.run_id;
  json["missionKind"] = record.mission_kind;
  json["taskId"] = record.task_id;
  json["modelId"] = record.model_id;
  json["modelVersion"] = record.model_version;
  json["planId"] = record.plan_id;
  json["planHash"] = record.plan_hash;
  json["loop"] = record.loop;
  json["loopCount"] = record.loop_count;
  json["currentLoop"] = record.current_loop;
  json["totalLoops"] = record.total_loops;
  json["nextSegment"] = record.next_segment;
  json["totalSegments"] = record.total_segments;
  json["completedSegments"] = record.completed_segments;
  json["nextWaypoint"] = record.next_waypoint;
  json["totalWaypoints"] = record.total_waypoints;
  json["completedWaypoints"] = record.completed_waypoints;
  json["brushSpeed"] = record.brush_speed;
  json["targetLat"] = record.target_lat;
  json["targetLon"] = record.target_lon;
  json["targetValid"] = record.target_valid;
  json["dockAfterArrival"] = record.dock_after_arrival;
  json["homeLat"] = record.home_lat;
  json["homeLon"] = record.home_lon;
  json["homeValid"] = record.home_valid;
  json["createdAtMs"] = record.created_at_ms;
  json["updatedAtMs"] = record.updated_at_ms;
  json["state"] = record.state;
  json["segments"] = Json::array();
  for (const auto& segment : record.segments) {
    json["segments"].push_back(task_segment_to_json(segment));
  }
  json["waypoints"] = Json::array();
  for (const auto& waypoint : record.waypoints) {
    json["waypoints"].push_back(waypoint_to_json(waypoint));
  }
  return json;
}

// 从 JSON 数据恢复完整任务执行记录及其任务段和航点。
MissionCheckpointRecord record_from_json(const Json& json) {
  MissionCheckpointRecord record;
  record.run_id = json.value("runId", "");
  record.mission_kind = json.value("missionKind", "");
  record.task_id = json.value("taskId", "");
  record.model_id = json.value("modelId", "");
  record.model_version = json.value("modelVersion", std::uint64_t{0u});
  record.plan_id = json.value("planId", "");
  record.plan_hash = json.value("planHash", "");
  record.loop = json.value("loop", false);
  record.loop_count = json.value("loopCount", 0u);
  record.current_loop = json.value("currentLoop", 0u);
  record.total_loops = json.value("totalLoops", 0u);
  record.next_segment = json.value("nextSegment", 0u);
  record.total_segments = json.value("totalSegments", 0u);
  record.completed_segments = json.value("completedSegments", 0u);
  record.next_waypoint = json.value("nextWaypoint", 0u);
  record.total_waypoints = json.value("totalWaypoints", 0u);
  record.completed_waypoints = json.value("completedWaypoints", 0u);
  record.brush_speed = json.value("brushSpeed", 0);
  record.target_lat = json.value("targetLat", 0.0);
  record.target_lon = json.value("targetLon", 0.0);
  record.target_valid = json.value("targetValid", false);
  record.dock_after_arrival = json.value("dockAfterArrival", false);
  record.home_lat = json.value("homeLat", 0.0);
  record.home_lon = json.value("homeLon", 0.0);
  record.home_valid = json.value("homeValid", false);
  record.created_at_ms = json.value("createdAtMs", 0ll);
  record.updated_at_ms = json.value("updatedAtMs", 0ll);
  record.state = json.value("state", "");
  if (json.contains("segments") && json["segments"].is_array()) {
    for (const auto& item : json["segments"]) {
      record.segments.push_back(task_segment_from_json(item));
    }
  }
  if (json.contains("waypoints") && json["waypoints"].is_array()) {
    for (const auto& item : json["waypoints"]) {
      record.waypoints.push_back(waypoint_from_json(item));
    }
  }
  return record;
}

}  // namespace

// 保存检查点文件路径，后续读写均使用该固定位置。
MissionCheckpointStore::MissionCheckpointStore(std::filesystem::path checkpoint_path)
    : checkpoint_path_(std::move(checkpoint_path)) {}

// 返回当前检查点文件的配置路径。
const std::filesystem::path& MissionCheckpointStore::checkpoint_path() const {
  return checkpoint_path_;
}

// 返回用于检查点创建和更新时间的 Unix 毫秒时间戳。
std::int64_t MissionCheckpointStore::now_milliseconds() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

// 校验并写入临时文件，同步落盘后原子替换正式检查点。
bool MissionCheckpointStore::save(
    const MissionCheckpointRecord& record,
    std::string* error) const {
  if (!validate_record(record, error)) {
    return false;
  }

  auto serializable = record;
  serializable.updated_at_ms = now_milliseconds();
  if (serializable.created_at_ms == 0) {
    serializable.created_at_ms = serializable.updated_at_ms;
  }

  const Json root{
      {"schemaVersion", 1u},
      {"checkpoint", record_to_json(serializable)},
  };

  std::error_code fs_error;
  if (!checkpoint_path_.parent_path().empty()) {
    std::filesystem::create_directories(checkpoint_path_.parent_path(), fs_error);
    if (fs_error) {
      if (error != nullptr) {
        *error = fs_error.message();
      }
      return false;
    }
  }

  const auto tmp_path = checkpoint_path_.string() + ".tmp";
  {
    std::ofstream output(tmp_path, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
      if (error != nullptr) {
        *error = "failed to open checkpoint temp file";
      }
      return false;
    }
    output << root.dump(2) << '\n';
    output.flush();
    if (!output.good()) {
      if (error != nullptr) {
        *error = "failed to write checkpoint temp file";
      }
      return false;
    }
  }

  if (!sync_file_to_disk(tmp_path, error) ||
      !atomic_replace_file(tmp_path, checkpoint_path_, error)) {
    std::filesystem::remove(tmp_path, fs_error);
    return false;
  }
  return true;
}

// 读取、解析并校验最近检查点；文件不存在或无效时返回空值。
std::optional<MissionCheckpointRecord> MissionCheckpointStore::load_latest(
    std::string* error) const {
  std::error_code fs_error;
  if (!std::filesystem::exists(checkpoint_path_, fs_error)) {
    return std::nullopt;
  }
  std::ifstream input(checkpoint_path_, std::ios::binary);
  if (!input.is_open()) {
    if (error != nullptr) {
      *error = "failed to open checkpoint file";
    }
    return std::nullopt;
  }

  const Json root = Json::parse(input, nullptr, false);
  if (root.is_discarded()) {
    if (error != nullptr) {
      *error = "checkpoint file is not valid JSON";
    }
    return std::nullopt;
  }
  try {
    if (root.value("schemaVersion", 0u) != 1u ||
        !root.contains("checkpoint") || !root.at("checkpoint").is_object()) {
      if (error != nullptr) {
        *error = "checkpoint schema is unsupported";
      }
      return std::nullopt;
    }
    auto record = record_from_json(root.at("checkpoint"));
    if (!validate_record(record, error)) {
      return std::nullopt;
    }
    return record;
  } catch (const std::exception& exception) {
    if (error != nullptr) {
      *error = std::string("checkpoint fields have invalid types: ") + exception.what();
    }
    return std::nullopt;
  }
}

// 删除任务完成或取消后不再需要的检查点文件。
bool MissionCheckpointStore::clear(std::string* error) const {
  std::error_code fs_error;
  if (!std::filesystem::exists(checkpoint_path_, fs_error)) {
    return true;
  }
  if (!std::filesystem::remove(checkpoint_path_, fs_error)) {
    if (error != nullptr) {
      *error = fs_error ? fs_error.message() : "failed to clear checkpoint file";
    }
    return false;
  }
  return true;
}

}  // namespace mission
}  // namespace cleanbot
