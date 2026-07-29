#ifndef CLEANBOT_RTK__RTK_SERIAL_HPP_
#define CLEANBOT_RTK__RTK_SERIAL_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

namespace cleanbot {
namespace rtk {

class RtkSerial {
 public:
  using DataCallback = std::function<void(const std::vector<std::uint8_t>&)>;
  using ConnectionCallback = std::function<void(bool, const std::string&)>;

  RtkSerial(
      std::string port_name,
      unsigned int baudrate,
      DataCallback data_callback,
      ConnectionCallback connection_callback,
      bool save_config_on_connect = false);
  ~RtkSerial();

  RtkSerial(const RtkSerial&) = delete;
  RtkSerial& operator=(const RtkSerial&) = delete;

  void start();
  void stop();
  void enqueueWrite(const std::vector<std::uint8_t>& bytes);
  bool connected() const;

 private:
  void openSerial();
  void configureReceiver();
  void beginRead();
  void onRead(const boost::system::error_code& error, std::size_t bytes_transferred);
  void enqueueWriteOnIo(const std::vector<std::uint8_t>& bytes);
  void beginWrite();
  void onWrite(const boost::system::error_code& error);
  void handleTransportError(const std::string& operation, const boost::system::error_code& error);
  void scheduleReconnect();
  void notifyConnection(bool connected, const std::string& detail);

  static constexpr std::size_t kReadBufferSize = 1024u;
  static constexpr std::size_t kMaxWriteQueueSize = 128u;

  std::string port_name_;
  unsigned int baudrate_;
  DataCallback data_callback_;
  ConnectionCallback connection_callback_;
  bool save_config_on_connect_{false};
  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::io_service::work> work_;
  boost::asio::serial_port serial_port_;
  boost::asio::deadline_timer reconnect_timer_;
  std::thread io_thread_;
  std::array<std::uint8_t, kReadBufferSize> read_buffer_{};
  std::deque<std::shared_ptr<std::vector<std::uint8_t>>> write_queue_;
  std::atomic<bool> started_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> connected_{false};
  bool write_in_progress_{false};
  bool reconnect_pending_{false};
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__RTK_SERIAL_HPP_
