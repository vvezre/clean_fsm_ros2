#include "cleanbot_rtk/ntrip_client.hpp"

#include <chrono>
#include <utility>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace cleanbot {
namespace rtk {

NtripClient::NtripClient(NtripConfig config, RtcmCallback rtcm_callback)
    : config_(std::move(config)),
      rtcm_callback_(std::move(rtcm_callback)),
      resolver_(io_service_),
      socket_(io_service_),
      connect_timer_(io_service_),
      reconnect_timer_(io_service_),
      gga_timer_(io_service_),
      rtcm_timer_(io_service_) {
  status_.enabled = config_.enabled;
  status_.configured = config_.complete();
}

NtripClient::~NtripClient() {
  stop();
}

void NtripClient::start() {
  if (started_.exchange(true)) {
    return;
  }
  stopping_ = false;
  io_service_.reset();
  work_.reset(new boost::asio::io_service::work(io_service_));
  io_thread_ = std::thread([this]() { io_service_.run(); });
  if (!config_.enabled) {
    return;
  }
  if (!config_.complete()) {
    setStatus(false, "ntrip_config_incomplete");
    return;
  }
  io_service_.post([this]() { beginResolve(); });
}

void NtripClient::stop() {
  if (!started_.exchange(false)) {
    return;
  }
  stopping_ = true;
  io_service_.post([this]() {
    boost::system::error_code ignored;
    resolver_.cancel();
    connect_timer_.cancel(ignored);
    reconnect_timer_.cancel(ignored);
    gga_timer_.cancel(ignored);
    rtcm_timer_.cancel(ignored);
    closeSocket();
    work_.reset();
  });
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  setStatus(false, "");
}

void NtripClient::updateGga(const std::string& gga_sentence) {
  if (!started_ || gga_sentence.empty()) {
    return;
  }
  io_service_.post([this, gga_sentence]() { latest_gga_ = gga_sentence; });
}

NtripStatus NtripClient::status() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_;
}

void NtripClient::beginResolve() {
  if (stopping_ || reconnect_pending_) {
    return;
  }
  response_parser_.reset();
  streaming_ = false;
  write_queue_.clear();
  write_in_progress_ = false;

  connect_timer_.expires_from_now(
      boost::posix_time::milliseconds(milliseconds(config_.connect_timeout_sec)));
  connect_timer_.async_wait([this](const boost::system::error_code& error) {
    if (!error && !stopping_) {
      handleFailure("ntrip_connect_timeout");
    }
  });

  Tcp::resolver::query query(config_.host, std::to_string(config_.port));
  resolver_.async_resolve(query,
      [this](const boost::system::error_code& error, Tcp::resolver::iterator endpoints) {
        if (stopping_ || reconnect_pending_) {
          return;
        }
        if (error) {
          handleFailure("ntrip_resolve_failed:" + error.message());
          return;
        }
        beginConnect(endpoints);
      });
}

void NtripClient::beginConnect(Tcp::resolver::iterator endpoints) {
  closeSocket();
  boost::asio::async_connect(socket_, endpoints,
      [this](const boost::system::error_code& error, Tcp::resolver::iterator) {
        if (stopping_ || reconnect_pending_) {
          return;
        }
        if (error) {
          handleFailure("ntrip_connect_failed:" + error.message());
          return;
        }
        boost::system::error_code ignored;
        connect_timer_.cancel(ignored);
        sendRequest();
        beginRead();
      });
}

void NtripClient::sendRequest() {
  enqueueWrite(build_ntrip_request(config_));
}

void NtripClient::beginRead() {
  socket_.async_read_some(boost::asio::buffer(read_buffer_),
      [this](const boost::system::error_code& error, const std::size_t length) {
        if (stopping_ || reconnect_pending_) {
          return;
        }
        if (error) {
          handleFailure("ntrip_read_failed:" + error.message());
          return;
        }
        handleIncoming(std::vector<std::uint8_t>(
            read_buffer_.begin(), read_buffer_.begin() + length));
        if (!reconnect_pending_) {
          beginRead();
        }
      });
}

void NtripClient::handleIncoming(const std::vector<std::uint8_t>& bytes) {
  const NtripResponseResult result = response_parser_.append(bytes);
  if (!result.error.empty()) {
    handleFailure(result.error);
    return;
  }
  if (result.header_complete && result.accepted && !streaming_) {
    streaming_ = true;
    setStatus(true, "");
    scheduleGga();
    armRtcmTimeout();
  }
  if (!result.rtcm_bytes.empty()) {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_.last_rtcm_monotonic_sec = monotonicSeconds();
    }
    armRtcmTimeout();
    if (rtcm_callback_) {
      rtcm_callback_(result.rtcm_bytes);
    }
  }
}

void NtripClient::enqueueWrite(const std::string& text) {
  if (text.empty() || !socket_.is_open()) {
    return;
  }
  if (write_queue_.size() >= kMaxWriteQueueSize) {
    write_queue_.pop_back();
  }
  write_queue_.push_back(text);
  if (!write_in_progress_) {
    beginWrite();
  }
}

void NtripClient::beginWrite() {
  if (write_queue_.empty() || stopping_ || reconnect_pending_) {
    write_in_progress_ = false;
    return;
  }
  write_in_progress_ = true;
  boost::asio::async_write(socket_, boost::asio::buffer(write_queue_.front()),
      [this](const boost::system::error_code& error, std::size_t) {
        if (stopping_ || reconnect_pending_) {
          return;
        }
        if (error) {
          handleFailure("ntrip_write_failed:" + error.message());
          return;
        }
        write_queue_.pop_front();
        beginWrite();
      });
}

void NtripClient::scheduleGga() {
  gga_timer_.expires_from_now(
      boost::posix_time::milliseconds(milliseconds(config_.gga_interval_sec)));
  gga_timer_.async_wait([this](const boost::system::error_code& error) {
    if (error || stopping_ || reconnect_pending_ || !streaming_) {
      return;
    }
    if (!latest_gga_.empty()) {
      std::string sentence = latest_gga_;
      if (sentence.size() < 2u || sentence.substr(sentence.size() - 2u) != "\r\n") {
        sentence += "\r\n";
      }
      enqueueWrite(sentence);
    }
    scheduleGga();
  });
}

void NtripClient::armRtcmTimeout() {
  rtcm_timer_.expires_from_now(
      boost::posix_time::milliseconds(milliseconds(config_.rtcm_timeout_sec)));
  rtcm_timer_.async_wait([this](const boost::system::error_code& error) {
    if (!error && !stopping_ && streaming_) {
      handleFailure("ntrip_rtcm_timeout");
    }
  });
}

void NtripClient::handleFailure(const std::string& detail) {
  if (stopping_ || reconnect_pending_) {
    return;
  }
  setStatus(false, detail);
  streaming_ = false;
  boost::system::error_code ignored;
  connect_timer_.cancel(ignored);
  gga_timer_.cancel(ignored);
  rtcm_timer_.cancel(ignored);
  resolver_.cancel();
  closeSocket();
  write_queue_.clear();
  write_in_progress_ = false;
  scheduleReconnect();
}

void NtripClient::scheduleReconnect() {
  if (stopping_ || reconnect_pending_) {
    return;
  }
  reconnect_pending_ = true;
  reconnect_timer_.expires_from_now(
      boost::posix_time::milliseconds(milliseconds(config_.reconnect_interval_sec)));
  reconnect_timer_.async_wait([this](const boost::system::error_code& error) {
    reconnect_pending_ = false;
    if (!error && !stopping_) {
      beginResolve();
    }
  });
}

void NtripClient::closeSocket() {
  boost::system::error_code ignored;
  socket_.cancel(ignored);
  socket_.shutdown(Tcp::socket::shutdown_both, ignored);
  socket_.close(ignored);
}

void NtripClient::setStatus(const bool connected, const std::string& error) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_.enabled = config_.enabled;
  status_.configured = config_.complete();
  status_.connected = connected;
  status_.last_error = error;
}

long NtripClient::milliseconds(const double seconds) {
  const double bounded = seconds > 0.001 ? seconds : 0.001;
  return static_cast<long>(bounded * 1000.0);
}

double NtripClient::monotonicSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace rtk
}  // namespace cleanbot
