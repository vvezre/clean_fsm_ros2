#ifndef CLEANBOT_HARDWARE__LOWER_MACHINE_SERIAL_HPP_
#define CLEANBOT_HARDWARE__LOWER_MACHINE_SERIAL_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

#include "cleanbot_hardware/frame_buffer.hpp"
#include "cleanbot_hardware/write_queue.hpp"

namespace cleanbot {
namespace hardware {

enum class SendEvent : std::uint8_t {
  kQueued = 0,
  kWritten = 1,
  kQueueRejected = 2,
  kTransportOffline = 3,
  kSuperseded = 4,
};

class LowerMachineSerial {
 public:
  /// 完整串口帧回调：每解析出一帧状态或ACK数据时通知上层节点。
  using FrameCallback = std::function<void(const std::vector<std::uint8_t>&)>;
  /// 连接状态回调：串口连接或断开时将状态和原因通知上层节点。
  using ConnectionCallback = std::function<void(bool, const std::string&)>;
  /// 单帧发送事件回调：报告真实入队、写入、拒绝、离线或被新命令替换状态。
  using SendCallback = std::function<void(SendEvent, const std::string&)>;

  /// 创建下位机串口传输对象，保存端口参数及上层回调，但尚不启动I/O线程。
  LowerMachineSerial(
      std::string port_name,
      unsigned int baudrate,
      FrameCallback frame_callback,
      ConnectionCallback connection_callback);
  /// 销毁串口对象前停止异步任务、关闭端口并回收I/O线程。
  ~LowerMachineSerial();

  /// 禁止复制串口对象，确保物理串口、I/O线程和异步回调始终只有一个所有者。
  LowerMachineSerial(const LowerMachineSerial&) = delete;
  /// 禁止复制赋值，避免两个对象同时管理同一串口和I/O上下文。
  LowerMachineSerial& operator=(const LowerMachineSerial&) = delete;

  /// 启动Boost.Asio I/O线程并异步打开下位机串口。
  void start();
  /// 停止重连和读写任务，关闭串口并等待I/O线程退出。
  void stop();
  /// 将完整协议帧按指定优先级加入异步发送队列；合并键可淘汰尚未发送的旧连续命令。
  void send(
      const std::vector<std::uint8_t>& frame,
      WritePriority priority = WritePriority::kFinite,
      const std::string& coalescing_key = "",
      SendCallback callback = SendCallback());
  /// 线程安全地返回串口当前是否已经打开并可用。
  bool connected() const;

 private:
  /// 在I/O线程中打开串口、设置通信参数，并开始第一轮异步读取。
  void openSerial();
  /// 提交一次async_read_some，等待串口返回任意数量的新字节。
  void beginRead();
  /// 处理异步读取结果，将字节交给FrameBuffer并逐帧回调上层。
  void onRead(const boost::system::error_code& error, std::size_t bytes_transferred);
  /// 在I/O线程中按优先级和合并规则将待发帧加入发送队列。
  void enqueueWrite(
      const std::vector<std::uint8_t>& frame,
      WritePriority priority,
      const std::string& coalescing_key,
      SendCallback callback);
  /// 当没有其他写操作进行时，发送优先级队列头部的一帧数据。
  void beginWrite();
  /// 处理异步写入结果，成功后弹出队首并继续发送下一帧。
  void onWrite(const boost::system::error_code& error);
  /// 统一处理打开、读取或写入异常，关闭端口、清理队列并安排重连。
  void handleTransportError(const std::string& operation, const boost::system::error_code& error);
  /// 创建延时重连任务，避免串口故障时进行无间隔的忙循环重试。
  void scheduleReconnect();
  /// 更新原子连接标志，并通过回调向上层报告连接状态变化。
  void notifyConnection(bool connected, const std::string& detail);

  static constexpr std::size_t kReadBufferSize = 256u;
  static constexpr std::size_t kMaxWriteQueueSize = 32u;

  std::string port_name_;
  unsigned int baudrate_;
  FrameCallback frame_callback_;
  ConnectionCallback connection_callback_;
  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::io_service::work> work_;
  boost::asio::serial_port serial_port_;
  boost::asio::deadline_timer reconnect_timer_;
  std::thread io_thread_;
  std::array<std::uint8_t, kReadBufferSize> read_buffer_{};
  FrameBuffer frame_buffer_;
  PriorityWriteQueue write_queue_{kMaxWriteQueueSize};
  std::atomic<bool> started_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> connected_{false};
  bool write_in_progress_{false};
  bool reconnect_pending_{false};
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__LOWER_MACHINE_SERIAL_HPP_
