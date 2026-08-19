#ifndef CLEANBOT_RTK__NTRIP_PROTOCOL_HPP_
#define CLEANBOT_RTK__NTRIP_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// 文件作用：声明 NTRIP 请求生成和响应头、RTCM 载荷流解析工具。
namespace cleanbot {
namespace rtk {

// NTRIP 挂载点连接、认证、心跳和重连参数。
struct NtripConfig {
  bool enabled{false};
  std::string host;
  std::uint16_t port{2101u};
  std::string mountpoint;
  std::string username;
  std::string password;
  double gga_interval_sec{5.0};
  double connect_timeout_sec{5.0};
  double reconnect_interval_sec{5.0};
  double rtcm_timeout_sec{15.0};

  // 返回连接配置是否具备建立 NTRIP 会话的必要字段。
  bool complete() const;
};

// 根据连接配置生成 NTRIP HTTP 风格请求。
std::string build_ntrip_request(const NtripConfig& config);

// NTRIP 响应头解析状态及随后的 RTCM 字节。
struct NtripResponseResult {
  bool header_complete{false};
  bool accepted{false};
  std::string error;
  std::vector<std::uint8_t> rtcm_bytes;
};

class NtripResponseParser {
 public:
  // 创建并设置可接受的最大响应头长度。
  explicit NtripResponseParser(std::size_t max_header_size = 8192u);

  // 追加网络字节，解析响应头并输出可转发的 RTCM 数据。
  NtripResponseResult append(const std::vector<std::uint8_t>& bytes);
  // 清空已有响应头和错误状态，准备建立新连接。
  void reset();

 private:
  // 判断响应首行是否表示服务端接受挂载点请求。
  static bool responseAccepted(const std::string& first_line);

  std::size_t max_header_size_;
  std::vector<std::uint8_t> buffer_;
  bool header_complete_{false};
  bool accepted_{false};
  std::string error_;
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__NTRIP_PROTOCOL_HPP_
