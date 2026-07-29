#pragma once

#include <map>
#include <string>
#include <vector>

namespace cleanbot {
namespace config {

enum class ConfigValueType {
  kString,
  kInteger,
  kDouble,
  kBoolean,
};

enum class ApplyPolicy {
  kRestart,
  kImmediate,
};

struct ConfigDefinition {
  std::string key;
  ConfigValueType value_type{ConfigValueType::kString};
  bool required{false};
  bool has_default{false};
  std::string default_value;
  bool has_range{false};
  double minimum{0.0};
  double maximum{0.0};
  bool sensitive{false};
  ApplyPolicy apply_policy{ApplyPolicy::kRestart};
};

struct ValidationResult {
  bool valid{false};
  std::string code;
  std::string message;
};

class ConfigRegistry {
 public:
  ConfigRegistry();

  const ConfigDefinition* find(const std::string& key) const;
  const std::map<std::string, ConfigDefinition>& definitions() const;
  std::map<std::string, std::string> default_values() const;
  ValidationResult validate_value(
      const std::string& key,
      const std::string& value) const;
  std::vector<std::string> missing_required(
      const std::map<std::string, std::string>& values) const;

 private:
  void add(const ConfigDefinition& definition);

  std::map<std::string, ConfigDefinition> definitions_;
};

}  // namespace config
}  // namespace cleanbot
