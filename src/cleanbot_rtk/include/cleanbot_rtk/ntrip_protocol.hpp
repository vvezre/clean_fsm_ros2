#ifndef CLEANBOT_RTK__NTRIP_PROTOCOL_HPP_
#define CLEANBOT_RTK__NTRIP_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cleanbot {
namespace rtk {

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

  bool complete() const;
};

std::string build_ntrip_request(const NtripConfig& config);

struct NtripResponseResult {
  bool header_complete{false};
  bool accepted{false};
  std::string error;
  std::vector<std::uint8_t> rtcm_bytes;
};

class NtripResponseParser {
 public:
  explicit NtripResponseParser(std::size_t max_header_size = 8192u);

  NtripResponseResult append(const std::vector<std::uint8_t>& bytes);
  void reset();

 private:
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
