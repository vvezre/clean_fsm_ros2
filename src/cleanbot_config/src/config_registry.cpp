/*
 * 文件作用：配置注册表实现：集中定义配置键、类型、默认值和取值范围。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_config/config_registry.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>

namespace cleanbot {
namespace config {
namespace {

// 构造一条配置键的类型、默认值、范围和应用策略定义。
ConfigDefinition make_definition(
    const std::string& key,
    const ConfigValueType type,
    const bool required,
    const std::string& default_value,
    const bool has_default,
    const double minimum = 0.0,
    const double maximum = 0.0,
    const bool has_range = false,
    const bool sensitive = false,
    const ApplyPolicy policy = ApplyPolicy::kRestart) {
  ConfigDefinition definition;
  definition.key = key;
  definition.value_type = type;
  definition.required = required;
  definition.has_default = has_default;
  definition.default_value = default_value;
  definition.has_range = has_range;
  definition.minimum = minimum;
  definition.maximum = maximum;
  definition.sensitive = sensitive;
  definition.apply_policy = policy;
  return definition;
}

// 返回转为小写后的副本，用于大小写无关的布尔值比较。
std::string lower_copy(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](const unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
  return value;
}

// 判断配置键不存在或其文本值为空。
bool is_missing(
    const std::map<std::string, std::string>& values,
    const std::string& key) {
  const auto found = values.find(key);
  return found == values.end() || found->second.empty();
}

// 严格解析整数文本，并以 double 形式返回以便统一范围校验。
bool parse_integer(const std::string& value, double& parsed) {
  if (value.empty()) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const auto integer = std::strtoll(value.c_str(), &end, 10);
  if (errno != 0 || end == value.c_str() || *end != '\0') {
    return false;
  }
  parsed = static_cast<double>(integer);
  return true;
}

// 严格解析有限浮点数文本。
bool parse_double(const std::string& value, double& parsed) {
  if (value.empty()) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  parsed = std::strtod(value.c_str(), &end);
  return errno == 0 && end != value.c_str() && *end == '\0' &&
      std::isfinite(parsed);
}

}  // namespace

// 注册系统全部配置键的默认值、合法范围和热更新策略。
ConfigRegistry::ConfigRegistry() {
  const auto string_required = [this](const std::string& key) {
    add(make_definition(key, ConfigValueType::kString, true, "", false));
  };
  const auto integer_default = [this](
      const std::string& key, const std::string& value,
      const double minimum, const double maximum,
      const ApplyPolicy policy = ApplyPolicy::kRestart) {
    add(make_definition(
        key, ConfigValueType::kInteger, false, value, true,
        minimum, maximum, true, false, policy));
  };
  const auto double_default = [this](
      const std::string& key, const std::string& value,
      const double minimum, const double maximum) {
    add(make_definition(
        key, ConfigValueType::kDouble, false, value, true,
        minimum, maximum, true));
  };
  const auto boolean_default = [this](
      const std::string& key, const std::string& value) {
    add(make_definition(key, ConfigValueType::kBoolean, false, value, true));
  };

  string_required("hardware.lower_machine_port");
  integer_default("hardware.lower_machine_baudrate", "115200", 1, 3000000);
  integer_default("hardware.command_repeat_count", "5", 1, 10);
  // 仅为兼容已有SQLite配置库保留；旧Python串口协议不会读取或使用这些ACK参数。
  integer_default("hardware.command_ack_timeout_ms", "200", 1, 10000);
  integer_default("hardware.command_max_retries", "3", 0, 20);
  integer_default("hardware.command_history_size", "256", 1, 4096);

  string_required("rtk.port");
  integer_default("rtk.baudrate", "115200", 1, 3000000);
  boolean_default("rtk.save_config_on_connect", "false");
  double_default("rtk.nmea_sync_threshold_sec", "0.5", 0.0, 10.0);
  double_default("rtk.nmea_max_age_sec", "2.0", 0.0, 60.0);
  double_default("rtk.heading_max_age_sec", "0.5", 0.0, 60.0);
  add(make_definition(
      "rtk.center_offset_along_heading_m", ConfigValueType::kDouble,
      true, "", false, -10.0, 10.0, true));
  add(make_definition(
      "rtk.center_offset_right_m", ConfigValueType::kDouble,
      true, "", false, -10.0, 10.0, true));

  boolean_default("ntrip.enabled", "false");
  add(make_definition("ntrip.host", ConfigValueType::kString, false, "", true));
  integer_default("ntrip.port", "2101", 1, 65535);
  add(make_definition("ntrip.mountpoint", ConfigValueType::kString, false, "", true));
  add(make_definition("ntrip.username", ConfigValueType::kString, false, "", true));
  add(make_definition(
      "ntrip.password", ConfigValueType::kString, false, "", true,
      0.0, 0.0, false, true));
  double_default("ntrip.gga_interval_sec", "5.0", 0.1, 3600.0);
  double_default("ntrip.connect_timeout_sec", "5.0", 0.1, 300.0);
  double_default("ntrip.reconnect_interval_sec", "5.0", 0.1, 3600.0);
  double_default("ntrip.rtcm_timeout_sec", "15.0", 0.1, 3600.0);

  integer_default(
      "motion.base_forward_speed", "350", 50, 600,
      ApplyPolicy::kImmediate);
  integer_default(
      "motion.manual_max_speed", "350", 50, 600,
      ApplyPolicy::kImmediate);

  // 兼容旧Python服务的本机HTTP入口。监听地址和端口属于启动参数，
  // 修改后需要重启HTTP节点；手动最大速度仍可立即生效。
  boolean_default("http.enabled", "true");
  add(make_definition(
      "http.listen_address", ConfigValueType::kString,
      false, "0.0.0.0", true));
  integer_default("http.port", "7899", 1, 65535);

  // 云平台控制入口默认关闭。启用后 broker 地址和设备编号必须明确配置，
  // 避免机器人误订阅到其他设备的控制主题。
  boolean_default("cloud.mqtt.enabled", "false");
  add(make_definition(
      "cloud.mqtt.host", ConfigValueType::kString, false, "", true));
  integer_default("cloud.mqtt.port", "1883", 1, 65535);
  add(make_definition(
      "cloud.mqtt.username", ConfigValueType::kString, false, "", true));
  add(make_definition(
      "cloud.mqtt.password", ConfigValueType::kString, false, "", true,
      0.0, 0.0, false, true));
  integer_default("cloud.mqtt.keepalive_sec", "60", 5, 3600);
  integer_default("cloud.mqtt.qos", "1", 0, 2);
  double_default("cloud.command_max_age_sec", "2.0", 0.1, 60.0);
  add(make_definition(
      "device.company_code", ConfigValueType::kString,
      false, "ZTZN-PVC", true));
  add(make_definition(
      "device.product_model", ConfigValueType::kString,
      false, "-T01", true));
  add(make_definition(
      "device.product_id", ConfigValueType::kString, false, "", true));

  double_default("tracking.process_noise", "0.2", 0.000001, 1000000.0);
  double_default("tracking.measurement_noise", "1.0", 0.000001, 1000000.0);
  double_default("tracking.max_dt", "1.0", 0.001, 60.0);
  double_default("tracking.heading_gain", "10.0", 0.0, 1000000.0);
  double_default("tracking.cte_gain", "1000.0", 0.0, 1000000.0);
  double_default("tracking.short_range_heading_limit_deg", "5.0", 0.0, 180.0);
  integer_default("tracking.max_z_speed", "15000", 0, 1000000);
  double_default("tracking.target_tolerance_m", "0.03", 0.0, 100.0);
  double_default("tracking.overshoot_cte_tolerance_m", "0.30", 0.0, 100.0);

  integer_default("control.manual_command_lease_ms", "500", 1, 60000);
  integer_default("control.mission_command_lease_ms", "1000", 1, 60000);
  integer_default("control.vision_command_lease_ms", "500", 1, 60000);

  double_default("mission.rtk_recovery_timeout_sec", "30.0", 0.1, 3600.0);
  double_default("mission.command_completion_timeout_sec", "20.0", 0.1, 3600.0);
  integer_default("mission.turn_heartbeat_ms", "250", 50, 60000);
  integer_default("mission.turn_z_speed", "0", -1000000, 1000000);
  double_default("mission.turn_angle_tolerance_deg", "0.5", 0.0, 180.0);
  double_default("mission.rtk_freshness_timeout_sec", "2.0", 0.1, 60.0);
  double_default("mission.hardware_freshness_timeout_sec", "1.0", 0.1, 60.0);
  integer_default("mission.edge_debounce_ms", "200", 1, 60000);
  double_default("mission.edge_target_tolerance_m", "0.10", 0.0, 100.0);
  double_default("mission.low_battery_threshold_percent", "0.0", 0.0, 100.0);
  add(make_definition(
      "mission.checkpoint_path", ConfigValueType::kString,
      false, "/var/lib/cleanbot/runtime/mission_checkpoint.json", true));

  add(make_definition(
      "modeling.database_path", ConfigValueType::kString,
      false, "/var/lib/cleanbot/modeling.db", true));
  double_default("modeling.brush_width_cm", "116.0", 1.0, 1000.0);
  double_default("modeling.minimum_overlap_cm", "10.0", 0.1, 999.0);
  integer_default("modeling.sample_count", "10", 1, 1000);
  double_default(
      "modeling.maximum_sample_radius_m", "0.05", 0.001, 10.0);
  double_default(
      "modeling.recognition.duplicate_tolerance_cm", "5.0", 0.001, 1000.0);
  double_default(
      "modeling.recognition.minimum_area_cm2", "10000.0", 1.0, 1000000000.0);
  double_default(
      "modeling.recognition.assist_turn_threshold_deg", "20.0", 0.1, 90.0);
  double_default(
      "modeling.recognition.minimum_connector_length_cm", "50.0", 0.1, 1000000.0);
  double_default(
      "modeling.recognition.maximum_connector_endpoint_distance_cm",
      "100.0", 0.1, 1000000.0);
  double_default(
      "modeling.recognition.unordered_auto_confirm_confidence",
      "0.85", 0.0, 1.0);
}

// 按键名查找配置定义；未知键返回空指针。
const ConfigDefinition* ConfigRegistry::find(const std::string& key) const {
  const auto found = definitions_.find(key);
  return found == definitions_.end() ? nullptr : &found->second;
}

// 返回只读的全部配置定义。
const std::map<std::string, ConfigDefinition>& ConfigRegistry::definitions() const {
  return definitions_;
}

// 收集所有声明了默认值的配置键和值。
std::map<std::string, std::string> ConfigRegistry::default_values() const {
  std::map<std::string, std::string> values;
  for (const auto& item : definitions_) {
    if (item.second.has_default) {
      values[item.first] = item.second.default_value;
    }
  }
  return values;
}

// 校验指定键和值的存在性、类型和范围。
ValidationResult ConfigRegistry::validate_value(
    const std::string& key,
    const std::string& value) const {
  const auto* definition = find(key);
  if (definition == nullptr) {
    return {false, "CONFIG_KEY_UNKNOWN", "unknown configuration key: " + key};
  }
  if (value.empty() && definition->required) {
    return {false, "CONFIG_VALUE_REQUIRED", "configuration value is required: " + key};
  }
  if (value.empty() && definition->value_type == ConfigValueType::kString) {
    return {true, "OK", ""};
  }

  double numeric_value = 0.0;
  bool parsed = true;
  switch (definition->value_type) {
    case ConfigValueType::kString:
      break;
    case ConfigValueType::kInteger:
      parsed = parse_integer(value, numeric_value);
      break;
    case ConfigValueType::kDouble:
      parsed = parse_double(value, numeric_value);
      break;
    case ConfigValueType::kBoolean: {
      const auto normalized = lower_copy(value);
      parsed = normalized == "true" || normalized == "false" ||
          normalized == "1" || normalized == "0";
      break;
    }
  }
  if (!parsed) {
    return {false, "CONFIG_VALUE_INVALID", "invalid value for configuration key: " + key};
  }

  if (definition->has_range &&
      (numeric_value < definition->minimum || numeric_value > definition->maximum)) {
    return {false, "CONFIG_VALUE_OUT_OF_RANGE", "configuration value is out of range: " + key};
  }
  return {true, "OK", ""};
}

// 列出必填配置及按功能开关动态要求的缺失键。
std::vector<std::string> ConfigRegistry::missing_required(
    const std::map<std::string, std::string>& values) const {
  std::vector<std::string> missing;
  for (const auto& item : definitions_) {
    if (item.second.required && is_missing(values, item.first)) {
      missing.push_back(item.first);
    }
  }

  const auto enabled = values.find("ntrip.enabled");
  if (enabled != values.end()) {
    const auto normalized = lower_copy(enabled->second);
    if (normalized == "true" || normalized == "1") {
      for (const auto& key : {
          "ntrip.host", "ntrip.mountpoint", "ntrip.username", "ntrip.password"}) {
        if (is_missing(values, key)) {
          missing.push_back(key);
        }
      }
    }
  }

  const auto cloud_enabled = values.find("cloud.mqtt.enabled");
  if (cloud_enabled != values.end()) {
    const auto normalized = lower_copy(cloud_enabled->second);
    if (normalized == "true" || normalized == "1") {
      for (const auto& key : {"cloud.mqtt.host", "device.product_id"}) {
        if (is_missing(values, key)) {
          missing.push_back(key);
        }
      }
    }
  }
  return missing;
}

// 将一项定义按键名写入注册表。
void ConfigRegistry::add(const ConfigDefinition& definition) {
  definitions_[definition.key] = definition;
}

}  // namespace config
}  // namespace cleanbot
