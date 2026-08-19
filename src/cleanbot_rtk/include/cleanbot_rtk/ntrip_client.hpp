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

// 文件作用：声明 NTRIP 差分数据客户端的异步连接、GGA 上行、重连与状态管理。
namespace cleanbot {
namespace rtk {

// NTRIP 客户端对外暴露的启用、配置、连接和 RTCM 新鲜度状态。
struct NtripStatus {
  bool enabled{false};
  bool configured{false};
  bool connected{false};
  double last_rtcm_monotonic_sec{-1.0};
  std::string last_error;
};

class NtripClient {
 public:
  // 接收到有效 RTCM 数据时的回调类型。
  using RtcmCallback = std::function<void(const std::vector<std::uint8_t>&)>;

  // 使用连接配置和 RTCM 回调创建客户端。
  NtripClient(NtripConfig config, RtcmCallback rtcm_callback);
  // 析构时停止网络线程和所有异步操作。
  ~NtripClient();

  // 禁止复制客户端，确保 socket、定时器和网络线程只有一个所有者。
  NtripClient(const NtripClient&) = delete;
  // 禁止复制赋值，避免异步回调引用被覆盖的网络资源。
  NtripClient& operator=(const NtripClient&) = delete;

  // 启动 NTRIP 解析、连接和重连循环。
  void start();
  // 停止网络线程并关闭连接。
  void stop();
  // 更新将按周期发送到服务端的最新 GGA 语句。
  void updateGga(const std::string& gga_sentence);
  // 返回线程安全的当前连接状态快照。
  NtripStatus status() const;

 private:
  // TCP 协议类型别名。
  using Tcp = boost::asio::ip::tcp;

  // 异步解析 NTRIP 主机名。
  void beginResolve();
  // 使用解析出的端点建立 TCP 连接。
  void beginConnect(Tcp::resolver::iterator endpoints);
  // 发送包含认证信息的 NTRIP 请求。
  void sendRequest();
  // 开始异步读取 NTRIP 响应和 RTCM 流。
  void beginRead();
  // 解析网络输入并将合法 RTCM 数据交给回调。
  void handleIncoming(const std::vector<std::uint8_t>& bytes);
  // 将 GGA 或协议文本加入待发送队列。
  void enqueueWrite(const std::string& text);
  // 发送待发送队列中的下一条文本。
  void beginWrite();
  // 设置下一次周期性 GGA 上行定时器。
  void scheduleGga();
  // 设置 RTCM 数据新鲜度超时定时器。
  void armRtcmTimeout();
  // 处理解析、连接或传输失败并更新状态。
  void handleFailure(const std::string& detail);
  // 安排下一次延时重连。
  void scheduleReconnect();
  // 关闭当前 socket，取消进行中的 I/O。
  void closeSocket();
  // 在线程安全状态对象中更新连接标志和错误信息。
  void setStatus(bool connected, const std::string& error);

  // 将秒数转换为 Asio 定时器使用的毫秒数。
  static long milliseconds(double seconds);
  // 返回单调时钟秒数，用于 RTCM 新鲜度计算。
  static double monotonicSeconds();

  // 单次网络读取缓冲区和待发送文本队列的容量上限。
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
