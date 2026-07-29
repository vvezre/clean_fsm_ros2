#include "cleanbot_hardware/lower_machine_serial.hpp"

#include <algorithm>
#include <utility>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace cleanbot {
namespace hardware {

// 保存串口配置和上层回调，并让串口、重连定时器和帧缓冲绑定同一个I/O上下文。
LowerMachineSerial::LowerMachineSerial(
    std::string port_name,
    const unsigned int baudrate,
    FrameCallback frame_callback,
    ConnectionCallback connection_callback)
    : port_name_(std::move(port_name)),
      baudrate_(baudrate),
      frame_callback_(std::move(frame_callback)),
      connection_callback_(std::move(connection_callback)),
      serial_port_(io_service_),
      reconnect_timer_(io_service_),
      frame_buffer_(512u) {}

// 析构时统一调用stop，保证异步任务和工作线程不会在对象销毁后继续访问成员。
LowerMachineSerial::~LowerMachineSerial() { stop(); }

// 只允许启动一次：保持I/O上下文存活，在专用线程中异步打开串口并处理全部事件。
void LowerMachineSerial::start() {
  if (started_.exchange(true)) {
    return;
  }
  stopping_.store(false);
  work_.reset(new boost::asio::io_service::work(io_service_));
  io_service_.post([this]() { openSerial(); });
  io_thread_ = std::thread([this]() { io_service_.run(); });
}

// 在I/O线程中取消定时器和串口操作，清理缓存后等待线程安全退出。
void LowerMachineSerial::stop() {
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
    frame_buffer_.clear();
    notifyConnection(false, "serial transport stopped");
  });
  work_.reset();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  io_service_.reset();
}

// 从任意调用线程接收待发帧，并投递到串口I/O线程执行真正的队列操作。
void LowerMachineSerial::send(
    const std::vector<std::uint8_t>& frame,
    const WritePriority priority,
    const std::string& coalescing_key,
    SendCallback callback) {
  if (!started_.load() || stopping_.load() || frame.empty()) {
    if (callback) {
      callback(SendEvent::kTransportOffline, "serial_transport_not_running");
    }
    return;
  }
  io_service_.post([this, frame, priority, coalescing_key, callback]() {
    enqueueWrite(frame, priority, coalescing_key, callback);
  });
}

// 通过原子变量返回连接状态，允许ROS2线程无锁查询。
bool LowerMachineSerial::connected() const { return connected_.load(); }

// 打开并配置8N1、无流控串口；成功后清空旧残帧、报告连接并启动读取。
void LowerMachineSerial::openSerial() {
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
            boost::asio::serial_port_base::stop_bits::one),
        error);
  }
  if (!error) {
    serial_port_.set_option(
        boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none),
        error);
  }
  if (error) {
    handleTransportError("configure", error);
    return;
  }

  frame_buffer_.clear();
  notifyConnection(true, "serial connected: " + port_name_);
  beginRead();
}

// 提交一次异步读取；回调结束后由onRead继续提交下一次读取，形成持续监听。
void LowerMachineSerial::beginRead() {
  if (stopping_.load() || !serial_port_.is_open()) {
    return;
  }
  serial_port_.async_read_some(
      boost::asio::buffer(read_buffer_),
      [this](const boost::system::error_code& error, const std::size_t bytes_transferred) {
        onRead(error, bytes_transferred);
      });
}

// 把本次读取字节追加到帧缓冲，循环提取完整帧并交给节点注册的FrameCallback。
void LowerMachineSerial::onRead(
    const boost::system::error_code& error, const std::size_t bytes_transferred) {
  if (error) {
    if (!stopping_.load()) {
      handleTransportError("read", error);
    }
    return;
  }

  if (bytes_transferred > 0u) {
    frame_buffer_.append(std::vector<std::uint8_t>(
        read_buffer_.begin(),
        read_buffer_.begin() + static_cast<std::ptrdiff_t>(bytes_transferred)));
    std::vector<std::uint8_t> frame;
    while (frame_buffer_.pop(frame)) {
      if (frame_callback_) {
        frame_callback_(frame);
      }
    }
  }
  beginRead();
}

// 在I/O线程内执行优先级入队；若当前没有写操作，立即启动队首发送。
void LowerMachineSerial::enqueueWrite(
    const std::vector<std::uint8_t>& frame,
    const WritePriority priority,
    const std::string& coalescing_key,
    SendCallback callback) {
  if (!connected_.load() || !serial_port_.is_open()) {
    if (callback) {
      callback(SendEvent::kTransportOffline, "serial_transport_offline");
    }
    return;
  }

  WriteQueueCallback queue_callback;
  if (callback) {
    queue_callback = [callback](const WriteQueueEvent event) {
      if (event == WriteQueueEvent::kWritten) {
        callback(SendEvent::kWritten, "serial_write_completed");
      } else if (event == WriteQueueEvent::kSuperseded) {
        callback(SendEvent::kSuperseded, "queue_item_superseded");
      } else {
        callback(SendEvent::kSuperseded, "queue_item_evicted");
      }
    };
  }
  if (!write_queue_.enqueue(
          frame, priority, coalescing_key, write_in_progress_, queue_callback)) {
    if (callback) {
      callback(SendEvent::kQueueRejected, "write_queue_full");
    }
    return;
  }
  if (callback) {
    callback(SendEvent::kQueued, "write_queue_accepted");
  }
  if (!write_in_progress_) {
    beginWrite();
  }
}

// 保存队首共享句柄并启动async_write，确保整个协议帧一次完整写出且内存有效。
void LowerMachineSerial::beginWrite() {
  if (write_queue_.empty() || !serial_port_.is_open()) {
    write_in_progress_ = false;
    return;
  }
  write_in_progress_ = true;
  const auto active_frame = write_queue_.front_handle();
  boost::asio::async_write(
      serial_port_, boost::asio::buffer(*active_frame),
      [this, active_frame](const boost::system::error_code& error, std::size_t) {
        onWrite(error);
      });
}

// 写成功后删除队首并继续下一帧；写失败则进入统一传输错误和重连流程。
void LowerMachineSerial::onWrite(const boost::system::error_code& error) {
  if (error) {
    if (!stopping_.load()) {
      handleTransportError("write", error);
    }
    return;
  }

  if (!write_queue_.empty()) {
    const auto callback = write_queue_.front_callback();
    if (callback) {
      callback(WriteQueueEvent::kWritten);
    }
    write_queue_.pop_front();
  }
  write_in_progress_ = false;
  beginWrite();
}

// 关闭故障串口、清除旧收发数据、发布离线状态并安排延迟重连。
void LowerMachineSerial::handleTransportError(
    const std::string& operation, const boost::system::error_code& error) {
  boost::system::error_code ignored;
  serial_port_.cancel(ignored);
  serial_port_.close(ignored);
  write_queue_.clear();
  write_in_progress_ = false;
  frame_buffer_.clear();
  notifyConnection(false, operation + " failed: " + error.message());
  scheduleReconnect();
}

// 一秒后尝试重新打开串口；reconnect_pending_保证同一时刻只存在一个重连任务。
void LowerMachineSerial::scheduleReconnect() {
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

// 原子更新连接标志，并把连接变化及原因回调给LowerMachineNode。
void LowerMachineSerial::notifyConnection(
    const bool connected, const std::string& detail) {
  connected_.store(connected);
  if (connection_callback_) {
    connection_callback_(connected, detail);
  }
}

}  // namespace hardware
}  // namespace cleanbot
