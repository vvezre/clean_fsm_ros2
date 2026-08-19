// 文件作用：声明配置定义注册表、类型校验规则和默认值生成接口。
#pragma once

#include <map>
#include <string>
#include <vector>

namespace cleanbot {
namespace config {

enum class ConfigValueType {
  // 配置值可使用的四种基础数据类型。
  kString,
  kInteger,
  kDouble,
  kBoolean,
};

enum class ApplyPolicy {
  // 配置变更生效策略：立即生效或要求节点重启。
  kRestart,
  kImmediate,
};

struct ConfigDefinition {
  // 单个配置键的类型、默认值、范围、敏感性和生效策略。
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
  // 配置校验结果；失败时携带稳定错误码和可读说明。
  bool valid{false};
  std::string code;
  std::string message;
};

class ConfigRegistry {
 public:
  // 构造并注册系统支持的全部配置键定义。
  ConfigRegistry();

  // 按键查找配置定义；不存在时返回空指针。
  const ConfigDefinition* find(const std::string& key) const;
  // 返回完整配置定义表，供服务和存储层遍历。
  const std::map<std::string, ConfigDefinition>& definitions() const;
  // 生成所有带默认值配置键的键值表。
  std::map<std::string, std::string> default_values() const;
  // 按定义校验指定键和值的类型、范围和必填约束。
  ValidationResult validate_value(
      const std::string& key,
      const std::string& value) const;
  // 返回当前键值表中缺失的全部必填配置键。
  std::vector<std::string> missing_required(
      const std::map<std::string, std::string>& values) const;

 private:
  // 向注册表写入一个配置定义；内部用于构造期集中注册。
  void add(const ConfigDefinition& definition);

  std::map<std::string, ConfigDefinition> definitions_;
};

}  // namespace config
}  // namespace cleanbot
