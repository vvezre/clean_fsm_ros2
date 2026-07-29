#pragma once

#include <string>

#include "cleanbot_gateway/cloud_command.hpp"

namespace cleanbot {
namespace gateway {

struct CloudIdentity {
  std::string company_code;
  std::string product_model;
  std::string product_id;
};

struct CloudDecodeResult {
  bool success{false};
  std::string code;
  std::string message;
  CloudCommandInput command;
};

class CloudMessageCodec {
 public:
  explicit CloudMessageCodec(CloudIdentity identity);

  CloudDecodeResult decode(
      const std::string& payload,
      bool retained) const;
  std::string encode_ack(
      const CloudCommandInput& command,
      const std::string& status,
      std::int64_t timestamp_sec) const;
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
