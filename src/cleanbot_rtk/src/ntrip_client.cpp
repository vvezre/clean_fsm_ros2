/*
 * 文件作用：NTRIP客户端实现：建立差分数据连接并转发RTCM字节流。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_rtk/ntrip_client.hpp"

#include <chrono>
#include <utility>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace cleanbot {
namespace rtk {

// 保存 NTRIP 参数和 RTCM 回调，并初始化网络 I/O 对象。
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

// 析构时停止重连、定时器和网络线程。
NtripClient::~NtripClient() {
  stop();
}

// 启动网络线程，并在配置完整时开始域名解析。
void NtripClient::start() {
  if (started_.exchange(true)) {
    return;
  }
  stopping_ = false;
  io_service_.reset();
  work_.reset(new boost::asio::io_service::work(io_service_));
  // 线程入口作用：运行当前节点的 I/O 事件循环，直到收到停止请求。
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

// 停止所有异步网络操作并等待后台线程退出。
void NtripClient::stop() {
  if (!started_.exchange(false)) {
    return;
  }
  stopping_ = true;
  // 异步任务作用：在 I/O 线程中串行执行当前状态变更或资源操作。
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

// 在线程安全的 I/O 队列中更新周期上行的 GGA 语句。
void NtripClient::updateGga(const std::string& gga_sentence) {
  if (!started_ || gga_sentence.empty()) {
    return;
  }
  io_service_.post([this, gga_sentence]() { latest_gga_ = gga_sentence; });
}

// 在线程保护下返回当前连接和 RTCM 新鲜度状态。
NtripStatus NtripClient::status() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_;
}

// 异步解析 NTRIP 服务主机名。
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
  // 定时回调作用：处理定时器到期事件，并执行超时检查或重连操作。
  connect_timer_.async_wait([this](const boost::system::error_code& error) {
    if (!error && !stopping_) {
      handleFailure("ntrip_connect_timeout");
    }
  });

  Tcp::resolver::query query(config_.host, std::to_string(config_.port));
  resolver_.async_resolve(query,
      // 异步回调作用：处理主机名解析结果，并继续建立 NTRIP 连接。
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

// 对解析到的端点发起带超时保护的 TCP 连接。
void NtripClient::beginConnect(Tcp::resolver::iterator endpoints) {
  closeSocket();
  boost::asio::async_connect(socket_, endpoints,
      // 异步回调作用：处理网络连接结果，成功后发送请求，失败时安排重连。
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

// 生成并排队发送带认证信息的 NTRIP 请求。
void NtripClient::sendRequest() {
  enqueueWrite(build_ntrip_request(config_));
}

// 持续异步读取响应头和 RTCM 字节流。
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

// 解析接收到的协议字节，并将有效 RTCM 载荷交给上层。
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

// 限制队列容量后加入待发送的协议文本。
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

// 异步发送写队列中的下一条请求或 GGA 语句。
void NtripClient::beginWrite() {
  if (write_queue_.empty() || stopping_ || reconnect_pending_) {
    write_in_progress_ = false;
    return;
  }
  write_in_progress_ = true;
  boost::asio::async_write(socket_, boost::asio::buffer(write_queue_.front()),
      // 异步回调作用：处理写入完成事件，释放当前帧并继续发送队列。
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

// 按配置周期安排最新 GGA 语句上行。
void NtripClient::scheduleGga() {
  gga_timer_.expires_from_now(
      boost::posix_time::milliseconds(milliseconds(config_.gga_interval_sec)));
  // 定时回调作用：处理定时器到期事件，并执行超时检查或重连操作。
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

// 启动 RTCM 新鲜度定时器，超时则触发故障恢复。
void NtripClient::armRtcmTimeout() {
  rtcm_timer_.expires_from_now(
      boost::posix_time::milliseconds(milliseconds(config_.rtcm_timeout_sec)));
  // 定时回调作用：处理定时器到期事件，并执行超时检查或重连操作。
  rtcm_timer_.async_wait([this](const boost::system::error_code& error) {
    if (!error && !stopping_ && streaming_) {
      handleFailure("ntrip_rtcm_timeout");
    }
  });
}

// 记录网络或协议失败，关闭当前连接并安排重连。
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

// 避免重复重连，并在设定间隔后重新解析服务器。
void NtripClient::scheduleReconnect() {
  if (stopping_ || reconnect_pending_) {
    return;
  }
  reconnect_pending_ = true;
  reconnect_timer_.expires_from_now(
      boost::posix_time::milliseconds(milliseconds(config_.reconnect_interval_sec)));
  // 定时回调作用：处理定时器到期事件，并执行超时检查或重连操作。
  reconnect_timer_.async_wait([this](const boost::system::error_code& error) {
    reconnect_pending_ = false;
    if (!error && !stopping_) {
      beginResolve();
    }
  });
}

// 取消解析和 socket I/O，释放当前连接资源。
void NtripClient::closeSocket() {
  boost::system::error_code ignored;
  socket_.cancel(ignored);
  socket_.shutdown(Tcp::socket::shutdown_both, ignored);
  socket_.close(ignored);
}

// 在线程保护下更新对外状态快照。
void NtripClient::setStatus(const bool connected, const std::string& error) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_.enabled = config_.enabled;
  status_.configured = config_.complete();
  status_.connected = connected;
  status_.last_error = error;
}

// 将秒数安全转换为 Asio 定时器使用的毫秒数。
long NtripClient::milliseconds(const double seconds) {
  const double bounded = seconds > 0.001 ? seconds : 0.001;
  return static_cast<long>(bounded * 1000.0);
}

// 返回不受系统时间校正影响的单调秒计时。
double NtripClient::monotonicSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace rtk
}  // namespace cleanbot
