#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>

#include "cleanbot_http/http_session.hpp"

// 文件作用：声明基于 Boost.Asio 的异步 HTTP 监听器和连接生命周期管理逻辑。
namespace cleanbot {
namespace http {

// 异步 HTTP 监听器。构造时完成端口绑定，绑定失败直接抛出异常。
class HttpServer : public std::enable_shared_from_this<HttpServer> {
 public:
  // 单个请求的路由回调类型。
  using RequestHandler = HttpSession::RequestHandler;
  // 网络运行错误的上报回调类型。
  using ErrorHandler = std::function<void(const std::string& message)>;

  // 绑定监听地址和端口，并设置请求处理、错误处理和超时策略。
  HttpServer(
      boost::asio::io_context& io_context,
      const std::string& listen_address,
      std::uint16_t port,
      RequestHandler request_handler,
      ErrorHandler error_handler = ErrorHandler(),
      std::chrono::milliseconds request_timeout =
          std::chrono::milliseconds(1000));
  // 析构时确保异步监听器和会话得到停止。
  ~HttpServer();

  // 开始异步接受新的 TCP 连接。
  void start();
  // 停止监听并关闭所有活动会话。
  void stop();
  // 返回实际绑定的端口，端口为 0 时由系统自动分配。
  std::uint16_t port() const;

 private:
  // 保存活动会话的弱引用集合，避免服务器与会话循环持有。
  using SessionSet = std::set<
      std::weak_ptr<HttpSession>,
      std::owner_less<std::weak_ptr<HttpSession>>>;

  // 投递下一次异步 accept 操作。
  void acceptNext();
  // 处理 accept 结果，创建会话或上报网络错误。
  void onAccept(
      const std::shared_ptr<boost::asio::ip::tcp::socket>& socket,
      const boost::system::error_code& error);
  // 从活动会话集合中移除已关闭会话。
  void removeSession(const std::shared_ptr<HttpSession>& session);
  // 在 Asio 执行器线程内完成停止操作。
  void stopOnExecutor();
  // 将可读错误信息交给上层错误处理器。
  void reportError(const std::string& message) const;

  boost::asio::io_context& io_context_;
  boost::asio::strand<boost::asio::io_context::executor_type> strand_;
  boost::asio::ip::tcp::acceptor acceptor_;
  RequestHandler request_handler_;
  ErrorHandler error_handler_;
  std::chrono::milliseconds request_timeout_;
  std::uint16_t port_{0u};
  SessionSet sessions_;
  std::mutex stop_mutex_;
  std::atomic<bool> started_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> stopped_{false};
};

}  // namespace http
}  // namespace cleanbot
