/*
 * 文件作用：云消息编解码实现：在JSON消息和内部命令结构之间转换。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_gateway/cloud_message_codec.hpp"

#include <cstdint>
#include <exception>
#include <utility>

#include <nlohmann/json.hpp>

namespace cleanbot {
namespace gateway {
namespace {

using Json = nlohmann::json;

// 安全读取 JSON 字符串字段，字段不存在或类型错误时返回空串。
std::string string_value(const Json& object, const char* key) {
  const auto found = object.find(key);
  return found != object.end() && found->is_string()
      ? found->get<std::string>()
      : std::string();
}

// 安全读取 JSON 数值字段，字段不存在或类型错误时返回零。
double number_value(const Json& object, const char* key) {
  const auto found = object.find(key);
  return found != object.end() && found->is_number()
      ? found->get<double>()
      : 0.0;
}

// 使用设备身份和时间戳构造云端消息的统一外层结构。
Json envelope(
    const CloudIdentity& identity,
    const std::int64_t timestamp_sec,
    Json data) {
  return Json{
      {"company_code", identity.company_code},
      {"product_model", identity.product_model},
      {"product_id", identity.product_id},
      {"timestamp", timestamp_sec},
      {"data", std::move(data)},
  };
}

}  // namespace

// 保存当前机器人身份，用于来信校验和回执封装。
CloudMessageCodec::CloudMessageCodec(CloudIdentity identity)
    : identity_(std::move(identity)) {}

// 解析云端 JSON，校验设备身份和命令字段并生成内部命令。
CloudDecodeResult CloudMessageCodec::decode(
    const std::string& payload,
    const bool retained) const {
  CloudDecodeResult result;
  try {
    const auto root = Json::parse(payload);
    if (!root.is_object()) {
      result.code = "MESSAGE_NOT_OBJECT";
      result.message = "cloud message must be a JSON object";
      return result;
    }

    const auto company_code = string_value(root, "company_code");
    const auto product_model = string_value(root, "product_model");
    const auto product_id = string_value(root, "product_id");
    if (company_code != identity_.company_code ||
        product_model != identity_.product_model ||
        product_id != identity_.product_id) {
      result.code = "DEVICE_IDENTITY_MISMATCH";
      result.message = "cloud message identity does not match this robot";
      return result;
    }

    const auto data_found = root.find("data");
    if (data_found == root.end() || !data_found->is_object()) {
      result.code = "MESSAGE_DATA_REQUIRED";
      result.message = "cloud message data object is required";
      return result;
    }
    const auto& data = *data_found;
    result.command.command_id = string_value(data, "command_id");
    result.command.trace_id = string_value(data, "trace_id");
    result.command.command = string_value(data, "command");
    result.command.timestamp_sec = root.value("timestamp", std::int64_t{0});
    result.command.retained = retained;

    if (result.command.command_id.empty()) {
      result.code = "COMMAND_ID_REQUIRED";
      result.message = "command_id is required";
      return result;
    }
    if (result.command.command.empty()) {
      result.code = "COMMAND_REQUIRED";
      result.message = "command is required";
      return result;
    }

    const auto params_found = data.find("params");
    if (params_found != data.end() && params_found->is_object()) {
      const auto distance = params_found->find("distance");
      const auto dir_x = params_found->find("dirX");
      const auto dir_y = params_found->find("dirY");
      result.command.has_joystick_params =
          distance != params_found->end() && distance->is_number() &&
          dir_x != params_found->end() && dir_x->is_number() &&
          dir_y != params_found->end() && dir_y->is_number();
      if (result.command.has_joystick_params) {
        result.command.distance = number_value(*params_found, "distance");
        result.command.dir_x = number_value(*params_found, "dirX");
        result.command.dir_y = number_value(*params_found, "dirY");
      }
    }
    result.success = true;
    result.code = "OK";
    result.message = "cloud message decoded";
    return result;
  } catch (const std::exception& exception) {
    result.code = "MESSAGE_JSON_INVALID";
    result.message = exception.what();
    return result;
  }
}

// 编码命令已接收确认消息。
std::string CloudMessageCodec::encode_ack(
    const CloudCommandInput& command,
    const std::string& status,
    const std::int64_t timestamp_sec) const {
  return envelope(
      identity_,
      timestamp_sec,
      Json{
          {"type", "ack"},
          {"command_id", command.command_id},
          {"trace_id", command.trace_id},
          {"command", command.command},
          {"status", status},
          {"timestamp", timestamp_sec},
      }).dump();
}

// 编码命令最终执行结果及业务错误信息。
std::string CloudMessageCodec::encode_result(
    const CloudCommandInput& command,
    const bool success,
    const std::string& code,
    const std::string& message,
    const std::int64_t timestamp_sec) const {
  return envelope(
      identity_,
      timestamp_sec,
      Json{
          {"type", "command_result"},
          {"command_id", command.command_id},
          {"trace_id", command.trace_id},
          {"command", command.command},
          {"result", Json{
              {"success", success},
              {"code", code},
              {"message", message},
          }},
          {"timestamp", timestamp_sec},
      }).dump();
}

}  // namespace gateway
}  // namespace cleanbot
