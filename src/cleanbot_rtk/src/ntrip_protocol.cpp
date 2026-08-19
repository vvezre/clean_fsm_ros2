/*
 * 文件作用：NTRIP协议实现：生成请求头并解析服务端响应状态。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_rtk/ntrip_protocol.hpp"

#include <sstream>

namespace cleanbot {
namespace rtk {

namespace {

// 使用标准 Base64 字母表编码 NTRIP Basic 认证凭据。
std::string base64Encode(const std::string& input) {
  static const char alphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  std::uint32_t accumulator = 0u;
  int bits = -6;
  for (const unsigned char value : input) {
    accumulator = (accumulator << 8u) | value;
    bits += 8;
    while (bits >= 0) {
      output.push_back(alphabet[(accumulator >> bits) & 0x3fu]);
      bits -= 6;
    }
  }
  if (bits > -6) {
    output.push_back(alphabet[(accumulator << 8u >> (bits + 8)) & 0x3fu]);
  }
  while (output.size() % 4u != 0u) {
    output.push_back('=');
  }
  return output;
}

// 不依赖额外库判断字符串是否以指定前缀开头。
bool startsWith(const std::string& value, const std::string& prefix) {
  if (value.size() < prefix.size()) {
    return false;
  }
  for (std::size_t index = 0u; index < prefix.size(); ++index) {
    if (value[index] != prefix[index]) {
      return false;
    }
  }
  return true;
}

// 以逐字符比较方式判断字符串是否包含目标文本。
bool containsText(const std::string& value, const std::string& target) {
  if (target.empty() || value.size() < target.size()) {
    return false;
  }
  for (std::size_t start = 0u; start + target.size() <= value.size(); ++start) {
    bool matches = true;
    for (std::size_t index = 0u; index < target.size(); ++index) {
      if (value[start + index] != target[index]) {
        matches = false;
        break;
      }
    }
    if (matches) {
      return true;
    }
  }
  return false;
}

}  // namespace

// 判断启用 NTRIP 后建立连接所需的主机、挂载点和认证信息是否齐全。
bool NtripConfig::complete() const {
  return enabled && !host.empty() && port > 0u && !mountpoint.empty() &&
      !username.empty() && !password.empty();
}

// 规范化挂载点并生成带 Basic 认证的 NTRIP HTTP 请求头。
std::string build_ntrip_request(const NtripConfig& config) {
  std::size_t mount_start = 0u;
  while (mount_start < config.mountpoint.size() && config.mountpoint[mount_start] == '/') {
    ++mount_start;
  }
  const std::string mountpoint = config.mountpoint.substr(mount_start);
  const std::string credentials = base64Encode(config.username + ":" + config.password);
  std::ostringstream request;
  request << "GET /" << mountpoint << " HTTP/1.0\r\n"
          << "Host: " << config.host << ':' << config.port << "\r\n"
          << "User-Agent: NTRIP cleanbot-ros2/1.0\r\n"
          << "Accept: */*\r\n"
          << "Connection: keep-alive\r\n"
          << "Authorization: Basic " << credentials << "\r\n\r\n";
  return request.str();
}

// 设置可接受的最大协议响应头长度。
NtripResponseParser::NtripResponseParser(const std::size_t max_header_size)
    : max_header_size_(max_header_size == 0u ? 1u : max_header_size) {}

// 累积网络字节，解析响应头并分离可转发的 RTCM 载荷。
NtripResponseResult NtripResponseParser::append(
    const std::vector<std::uint8_t>& bytes) {
  NtripResponseResult result;
  if (header_complete_) {
    result.header_complete = true;
    result.accepted = accepted_;
    result.error = error_;
    if (accepted_) {
      result.rtcm_bytes = bytes;
    }
    return result;
  }
  if (!error_.empty()) {
    result.error = error_;
    return result;
  }

  buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
  std::size_t header_end = buffer_.size();
  bool header_found = false;
  for (std::size_t index = 0u; index + 3u < buffer_.size(); ++index) {
    if (buffer_[index] == '\r' && buffer_[index + 1u] == '\n' &&
        buffer_[index + 2u] == '\r' && buffer_[index + 3u] == '\n') {
      header_end = index + 4u;
      header_found = true;
      break;
    }
  }
  if (!header_found) {
    if (buffer_.size() > max_header_size_) {
      error_ = "ntrip_header_too_large";
      result.error = error_;
    }
    return result;
  }
  if (header_end > max_header_size_) {
    error_ = "ntrip_header_too_large";
    result.error = error_;
    return result;
  }

  std::size_t first_line_end = 0u;
  while (first_line_end < header_end && buffer_[first_line_end] != '\r' &&
         buffer_[first_line_end] != '\n') {
    ++first_line_end;
  }
  const std::string first_line(buffer_.begin(), buffer_.begin() +
      static_cast<std::ptrdiff_t>(first_line_end));
  accepted_ = responseAccepted(first_line);
  header_complete_ = true;
  if (!accepted_) {
    error_ = "ntrip_response_rejected";
  }

  result.header_complete = true;
  result.accepted = accepted_;
  result.error = error_;
  if (accepted_ && header_end < buffer_.size()) {
    result.rtcm_bytes.assign(
        buffer_.begin() + static_cast<std::ptrdiff_t>(header_end), buffer_.end());
  }
  buffer_.clear();
  return result;
}

// 重置响应头解析状态，供重连后的新会话使用。
void NtripResponseParser::reset() {
  buffer_.clear();
  header_complete_ = false;
  accepted_ = false;
  error_.clear();
}

// 判断响应首行是否表示 NTRIP 服务端已接受请求。
bool NtripResponseParser::responseAccepted(const std::string& first_line) {
  if (startsWith(first_line, "ICY 200")) {
    return true;
  }
  return startsWith(first_line, "HTTP/") && containsText(first_line, " 200");
}

}  // namespace rtk
}  // namespace cleanbot
