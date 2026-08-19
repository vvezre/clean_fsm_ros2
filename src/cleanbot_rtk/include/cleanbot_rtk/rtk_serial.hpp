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

// 文件作用：声明 RTK 接收机串口的异步收发、自动重连和接收机配置流程。
namespace cleanbot {
namespace rtk {

class RtkSerial {
 public:
  // 收到串口原始字节时的回调类型。
  using DataCallback = std::function<void(const std::vector<std::uint8_t>&)>;
  // 串口连接状态变化时的回调类型。
  using ConnectionCallback = std::function<void(bool, const std::string&)>;

  // 使用串口参数、数据回调和连接状态回调创建串口对象。
  RtkSerial(
      std::string port_name,
      unsigned int baudrate,
      DataCallback data_callback,
      ConnectionCallback connection_callback,
      bool save_config_on_connect = false);
  // 析构时停止 I/O 线程并释放串口资源。
  ~RtkSerial();

  // 禁止复制串口对象，确保接收机端口和 I/O 线程保持唯一所有者。
  RtkSerial(const RtkSerial&) = delete;
  // 禁止复制赋值，避免异步串口资源被重复管理。
  RtkSerial& operator=(const RtkSerial&) = delete;

  // 启动串口打开、接收和自动重连流程。
  void start();
  // 停止 I/O 线程，取消串口读写。
  void stop();
  // 将 RTCM 等字节排队写入接收机串口。
  void enqueueWrite(const std::vector<std::uint8_t>& bytes);
  // 返回当前串口是否已成功连接。
  bool connected() const;

 private:
  // 尝试打开并配置串口设备。
  void openSerial();
  // 向已连接接收机写入必要配置命令。
  void configureReceiver();
  // 投递下一次异步串口读取。
  void beginRead();
  // 处理串口读取完成、字节分发和错误。
  void onRead(const boost::system::error_code& error, std::size_t bytes_transferred);
  // 在 I/O 线程中将写入请求加入队列。
  void enqueueWriteOnIo(const std::vector<std::uint8_t>& bytes);
  // 开始发送队列中的下一帧数据。
  void beginWrite();
  // 处理一次异步串口写完成事件。
  void onWrite(const boost::system::error_code& error);
  // 处理传输错误并切换到重连流程。
  void handleTransportError(const std::string& operation, const boost::system::error_code& error);
  // 安排延迟后重新打开串口。
  void scheduleReconnect();
  // 通过回调通知上层串口连接状态变化。
  void notifyConnection(bool connected, const std::string& detail);

  // 单次读取大小和排队写入帧数上限。
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
