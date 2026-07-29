#pragma once

#include <cstdint>
#include <string>

#include "cleanbot_config/config_repository.hpp"

struct sqlite3;

namespace cleanbot {
namespace config {

class SqliteConfigRepository : public ConfigRepository {
 public:
  explicit SqliteConfigRepository(
      std::string database_path,
      std::uint64_t minimum_free_space_bytes = 64u * 1024u * 1024u);
  ~SqliteConfigRepository() override;

  RepositoryStatus open_and_initialize(const ConfigRegistry& registry) override;
  std::map<std::string, std::string> read_all() const override;
  WriteResult write_values(
      const std::map<std::string, std::string>& values,
      const std::string& updated_by,
      const ConfigRegistry& registry) override;
  void close() override;

 private:
  bool execute(const std::string& sql, std::string& error) const;
  bool create_schema(const ConfigRegistry& registry, std::string& error);
  bool insert_default(
      const ConfigDefinition& definition,
      std::string& error);
  bool read_revision(std::uint64_t& revision, std::string& error) const;
  bool quick_check(std::string& error) const;

  std::string database_path_;
  std::uint64_t minimum_free_space_bytes_{0u};
  sqlite3* database_{nullptr};
  std::uint64_t revision_{0u};
};

}  // namespace config
}  // namespace cleanbot
