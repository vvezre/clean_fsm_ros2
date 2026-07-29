#ifndef CLEANBOT_RTK__NTRIP_CLIENT_HPP_
#define CLEANBOT_RTK__NTRIP_CLIENT_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

#include "cleanbot_rtk/ntrip_protocol.hpp"

namespace cleanbot {
namespace rtk {

struct NtripStatus {
  bool enabled{false};
  bool configured{false};
  bool connected{false};
  double last_rtcm_monotonic_sec{-1.0};
  std::string last_error;
};

class NtripClient {
 public:
  using RtcmCallback = std::function<void(const std::vector<std::uint8_t>&)>;

  NtripClient(NtripConfig config, RtcmCallback rtcm_callback);
  ~NtripClient();

  NtripClient(const NtripClient&) = delete;
  NtripClient& operator=(const NtripClient&) = delete;

  void start();
  void stop();
  void updateGga(const std::string& gga_sentence);
  NtripStatus status() const;

 private:
  using Tcp = boost::asio::ip::tcp;

  void beginResolve();
  void beginConnect(Tcp::resolver::iterator endpoints);
  void sendRequest();
  void beginRead();
  void handleIncoming(const std::vector<std::uint8_t>& bytes);
  void enqueueWrite(const std::string& text);
  void beginWrite();
  void scheduleGga();
  void armRtcmTimeout();
  void handleFailure(const std::string& detail);
  void scheduleReconnect();
  void closeSocket();
  void setStatus(bool connected, const std::string& error);

  static long milliseconds(double seconds);
  static double monotonicSeconds();

  static constexpr std::size_t kReadBufferSize = 4096u;
  static constexpr std::size_t kMaxWriteQueueSize = 16u;

  NtripConfig config_;
  RtcmCallback rtcm_callback_;
  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::io_service::work> work_;
  Tcp::resolver resolver_;
  Tcp::socket socket_;
  boost::asio::deadline_timer connect_timer_;
  boost::asio::deadline_timer reconnect_timer_;
  boost::asio::deadline_timer gga_timer_;
  boost::asio::deadline_timer rtcm_timer_;
  std::thread io_thread_;
  std::array<std::uint8_t, kReadBufferSize> read_buffer_{};
  std::deque<std::string> write_queue_;
  NtripResponseParser response_parser_;
  std::atomic<bool> started_{false};
  std::atomic<bool> stopping_{false};
  bool streaming_{false};
  bool write_in_progress_{false};
  bool reconnect_pending_{false};
  std::string latest_gga_;
  mutable std::mutex status_mutex_;
  NtripStatus status_;
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__NTRIP_CLIENT_HPP_
