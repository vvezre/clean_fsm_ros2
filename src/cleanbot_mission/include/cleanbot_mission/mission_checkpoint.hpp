#ifndef CLEANBOT_MISSION__MISSION_CHECKPOINT_HPP_
#define CLEANBOT_MISSION__MISSION_CHECKPOINT_HPP_

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "cleanbot_interfaces/msg/task_segment.hpp"
#include "cleanbot_interfaces/msg/waypoint.hpp"

// 文件作用：声明任务执行进度检查点的序列化、恢复和清理接口。
namespace cleanbot {
namespace mission {

// 可持久化的任务执行进度、计划和目标点记录。
struct MissionCheckpointRecord {
  std::string run_id;
  std::string mission_kind;
  std::string task_id;
  std::string model_id;
  std::uint64_t model_version{0u};
  std::string plan_id;
  std::string plan_hash;
  bool loop{false};
  std::uint32_t loop_count{0u};
  std::uint32_t current_loop{0u};
  std::uint32_t total_loops{0u};
  std::uint32_t next_segment{0u};
  std::uint32_t total_segments{0u};
  std::uint32_t completed_segments{0u};
  std::uint32_t next_waypoint{0u};
  std::uint32_t total_waypoints{0u};
  std::uint32_t completed_waypoints{0u};
  std::int32_t brush_speed{0};
  double target_lat{0.0};
  double target_lon{0.0};
  bool target_valid{false};
  bool dock_after_arrival{false};
  double home_lat{0.0};
  double home_lon{0.0};
  bool home_valid{false};
  std::int64_t created_at_ms{0};
  std::int64_t updated_at_ms{0};
  std::string state;
  std::vector<cleanbot_interfaces::msg::TaskSegment> segments;
  std::vector<cleanbot_interfaces::msg::Waypoint> waypoints;
};

class MissionCheckpointStore {
 public:
  // 使用指定检查点文件路径创建存储对象。
  explicit MissionCheckpointStore(std::filesystem::path checkpoint_path);

  // 返回实际使用的检查点文件路径。
  const std::filesystem::path& checkpoint_path() const;
  // 保存当前任务检查点；失败信息可通过 error 返回。
  bool save(const MissionCheckpointRecord& record, std::string* error = nullptr) const;
  // 读取最近一次检查点；不存在或错误时返回空值。
  std::optional<MissionCheckpointRecord> load_latest(std::string* error = nullptr) const;
  // 删除已完成或已取消任务的检查点。
  bool clear(std::string* error = nullptr) const;

 private:
  std::filesystem::path checkpoint_path_;
  // 取得用于记录创建和更新时间的当前毫秒时间戳。
  static std::int64_t now_milliseconds();
};

}  // namespace mission
}  // namespace cleanbot

#endif  // CLEANBOT_MISSION__MISSION_CHECKPOINT_HPP_
