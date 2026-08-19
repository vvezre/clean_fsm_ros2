// 文件作用：定义配置持久化仓库的抽象接口和读写结果数据结构。
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "cleanbot_config/config_registry.hpp"

namespace cleanbot {
namespace config {

struct RepositoryStatus {
  // 仓库健康性、可写性、模式版本和数据修订号快照。
  bool healthy{false};
  bool writable{false};
  std::string code;
  std::string message;
  std::uint32_t schema_version{0u};
  std::uint64_t revision{0u};
};

struct WriteResult {
  // 一次批量配置写入的结果、变更键及是否需要重启。
  bool success{false};
  std::string code;
  std::string message;
  std::uint64_t revision{0u};
  bool restart_required{false};
  std::vector<std::string> changed_keys;
};

class ConfigRepository {
 public:
  // 虚析构函数，允许通过抽象接口安全释放具体仓库。
  virtual ~ConfigRepository() = default;

  // 打开存储并按注册表完成模式初始化、迁移和默认值填充。
  virtual RepositoryStatus open_and_initialize(const ConfigRegistry& registry) = 0;
  // 读取仓库中全部当前配置值。
  virtual std::map<std::string, std::string> read_all() const = 0;
  // 校验并持久化一组配置值，记录更新来源和修订号。
  virtual WriteResult write_values(
      const std::map<std::string, std::string>& values,
      const std::string& updated_by,
      const ConfigRegistry& registry) = 0;
  // 关闭底层存储资源和数据库连接。
  virtual void close() = 0;
};

}  // namespace config
}  // namespace cleanbot
