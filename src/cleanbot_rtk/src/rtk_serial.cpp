#include "cleanbot_rtk/rtk_serial.hpp"

#include <iterator>
#include <utility>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace cleanbot {
namespace rtk {

RtkSerial::RtkSerial(
    std::string port_name,
    const unsigned int baudrate,
    DataCallback data_callback,
    ConnectionCallback connection_callback,
    const bool save_config_on_connect)
    : port_name_(std::move(port_name)),
      baudrate_(baudrate),
      data_callback_(std::move(data_callback)),
      connection_callback_(std::move(connection_callback)),
      save_config_on_connect_(save_config_on_connect),
      serial_port_(io_service_),
      reconnect_timer_(io_service_) {}

RtkSerial::~RtkSerial() { stop(); }

void RtkSerial::start() {
  if (started_.exchange(true)) {
    return;
  }
  stopping_.store(false);
  work_.reset(new boost::asio::io_service::work(io_service_));
  io_service_.post([this]() { openSerial(); });
  io_thread_ = std::thread([this]() { io_service_.run(); });
}

void RtkSerial::stop() {
  if (!started_.exchange(false)) {
    return;
  }
  stopping_.store(true);
  io_service_.post([this]() {
    boost::system::error_code ignored;
    reconnect_timer_.cancel(ignored);
    serial_port_.cancel(ignored);
    serial_port_.close(ignored);
    write_queue_.clear();
    write_in_progress_ = false;
    reconnect_pending_ = false;
    notifyConnection(false, "RTK serial stopped");
  });
  work_.reset();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  io_service_.reset();
}

void RtkSerial::enqueueWrite(const std::vector<std::uint8_t>& bytes) {
  if (!started_.load() || stopping_.load() || bytes.empty()) {
    return;
  }
  io_service_.post([this, bytes]() { enqueueWriteOnIo(bytes); });
}

bool RtkSerial::connected() const { return connected_.load(); }

void RtkSerial::openSerial() {
  if (stopping_.load() || serial_port_.is_open()) {
    return;
  }
  reconnect_pending_ = false;
  boost::system::error_code error;
  serial_port_.open(port_name_, error);
  if (error) {
    handleTransportError("open", error);
    return;
  }
  serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_), error);
  if (!error) {
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8u), error);
  }
  if (!error) {
    serial_port_.set_option(
        boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), error);
  }
  if (!error) {
    serial_port_.set_option(
        boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one), error);
  }
  if (!error) {
    serial_port_.set_option(
        boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none), error);
  }
  if (error) {
    handleTransportError("configure", error);
    return;
  }

  write_queue_.clear();
  write_in_progress_ = false;
  notifyConnection(true, "RTK serial connected: " + port_name_);
  configureReceiver();
  beginRead();
}

void RtkSerial::configureReceiver() {
  const std::string commands[] = {
      "unlog\r\n",
      "gngga 0.1\r\n",
      "gphpr 0.1\r\n",
  };
  for (const auto& command : commands) {
    enqueueWriteOnIo(std::vector<std::uint8_t>(command.begin(), command.end()));
  }
  if (save_config_on_connect_) {
    const std::string save_command = "saveconfig\r\n";
    enqueueWriteOnIo(std::vector<std::uint8_t>(save_command.begin(), save_command.end()));
  }
}

void RtkSerial::beginRead() {
  if (stopping_.load() || !serial_port_.is_open()) {
    return;
  }
  serial_port_.async_read_some(
      boost::asio::buffer(read_buffer_),
      [this](const boost::system::error_code& error, const std::size_t bytes_transferred) {
        onRead(error, bytes_transferred);
      });
}

void RtkSerial::onRead(
    const boost::system::error_code& error, const std::size_t bytes_transferred) {
  if (error) {
    if (!stopping_.load()) {
      handleTransportError("read", error);
    }
    return;
  }
  if (bytes_transferred > 0u && data_callback_) {
    data_callback_(std::vector<std::uint8_t>(
        read_buffer_.begin(),
        read_buffer_.begin() + static_cast<std::ptrdiff_t>(bytes_transferred)));
  }
  beginRead();
}

void RtkSerial::enqueueWriteOnIo(const std::vector<std::uint8_t>& bytes) {
  if (!connected_.load() || !serial_port_.is_open()) {
    return;
  }
  if (write_queue_.size() >= kMaxWriteQueueSize) {
    if (write_in_progress_ && !write_queue_.empty()) {
      write_queue_.erase(std::next(write_queue_.begin()), write_queue_.end());
    } else {
      write_queue_.clear();
    }
  }
  write_queue_.push_back(std::make_shared<std::vector<std::uint8_t>>(bytes));
  if (!write_in_progress_) {
    beginWrite();
  }
}

void RtkSerial::beginWrite() {
  if (write_queue_.empty() || !serial_port_.is_open()) {
    write_in_progress_ = false;
    return;
  }
  write_in_progress_ = true;
  const auto active_frame = write_queue_.front();
  boost::asio::async_write(
      serial_port_, boost::asio::buffer(*active_frame),
      [this, active_frame](const boost::system::error_code& error, std::size_t) {
        onWrite(error);
      });
}

void RtkSerial::onWrite(const boost::system::error_code& error) {
  if (error) {
    if (!stopping_.load()) {
      handleTransportError("write", error);
    }
    return;
  }
  if (!write_queue_.empty()) {
    write_queue_.pop_front();
  }
  write_in_progress_ = false;
  beginWrite();
}

void RtkSerial::handleTransportError(
    const std::string& operation, const boost::system::error_code& error) {
  boost::system::error_code ignored;
  serial_port_.cancel(ignored);
  serial_port_.close(ignored);
  write_queue_.clear();
  write_in_progress_ = false;
  notifyConnection(false, operation + " failed: " + error.message());
  scheduleReconnect();
}

void RtkSerial::scheduleReconnect() {
  if (stopping_.load() || reconnect_pending_) {
    return;
  }
  reconnect_pending_ = true;
  reconnect_timer_.expires_from_now(boost::posix_time::seconds(1));
  reconnect_timer_.async_wait([this](const boost::system::error_code& error) {
    reconnect_pending_ = false;
    if (!error && !stopping_.load()) {
      openSerial();
    }
  });
}

void RtkSerial::notifyConnection(
    const bool connected, const std::string& detail) {
  connected_.store(connected);
  if (connection_callback_) {
    connection_callback_(connected, detail);
  }
}

}  // namespace rtk
}  // namespace cleanbot
