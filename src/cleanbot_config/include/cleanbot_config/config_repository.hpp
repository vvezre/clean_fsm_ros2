#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "cleanbot_config/config_registry.hpp"

namespace cleanbot {
namespace config {

struct RepositoryStatus {
  bool healthy{false};
  bool writable{false};
  std::string code;
  std::string message;
  std::uint32_t schema_version{0u};
  std::uint64_t revision{0u};
};

struct WriteResult {
  bool success{false};
  std::string code;
  std::string message;
  std::uint64_t revision{0u};
  bool restart_required{false};
  std::vector<std::string> changed_keys;
};

class ConfigRepository {
 public:
  virtual ~ConfigRepository() = default;

  virtual RepositoryStatus open_and_initialize(const ConfigRegistry& registry) = 0;
  virtual std::map<std::string, std::string> read_all() const = 0;
  virtual WriteResult write_values(
      const std::map<std::string, std::string>& values,
      const std::string& updated_by,
      const ConfigRegistry& registry) = 0;
  virtual void close() = 0;
};

}  // namespace config
}  // namespace cleanbot
