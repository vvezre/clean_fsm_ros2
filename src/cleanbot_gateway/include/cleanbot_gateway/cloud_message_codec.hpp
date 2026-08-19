#pragma once

#include <string>

#include "cleanbot_gateway/cloud_command.hpp"

// 文件作用：声明云端 JSON 控制消息的编解码接口及设备身份信息。
namespace cleanbot {
namespace gateway {

// 用于校验来信和生成回执的云端设备身份。
struct CloudIdentity {
  std::string company_code;
  std::string product_model;
  std::string product_id;
};

// 云端消息解码后的状态、失败原因和命令内容。
struct CloudDecodeResult {
  bool success{false};
  std::string code;
  std::string message;
  CloudCommandInput command;
};

class CloudMessageCodec {
 public:
  // 使用当前设备的云端身份创建消息编解码器。
  explicit CloudMessageCodec(CloudIdentity identity);

  // 将云端 JSON 载荷解码为内部命令，并保留保留消息标志。
  CloudDecodeResult decode(
      const std::string& payload,
      bool retained) const;
  // 为已接收的云端命令生成确认回执 JSON。
  std::string encode_ack(
      const CloudCommandInput& command,
      const std::string& status,
      std::int64_t timestamp_sec) const;
  // 为执行完成的云端命令生成结果通知 JSON。
  std::string encode_result(
      const CloudCommandInput& command,
      bool success,
      const std::string& code,
      const std::string& message,
      std::int64_t timestamp_sec) const;

 private:
  CloudIdentity identity_;
};

}  // namespace gateway
}  // namespace cleanbot
