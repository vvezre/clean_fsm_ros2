#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include "cleanbot_http/http_control_router.hpp"

// 文件作用：声明单条 HTTP TCP 连接的异步读写、超时和关闭控制。
namespace cleanbot {
namespace http {

// 管理一个 HTTP TCP 连接。读取、路由、响应和超时均通过 Asio 异步执行。
class HttpSession : public std::enable_shared_from_this<HttpSession> {
 public:
  // 将 HTTP 方法和路径转换成业务响应的回调类型。
  using RequestHandler = std::function<HttpControlResult(
      const std::string& method,
      const std::string& target)>;
  // 会话关闭时通知服务器回收引用的回调类型。
  using CloseHandler =
      std::function<void(const std::shared_ptr<HttpSession>& session)>;

  // 使用已接受的 socket 和各类回调创建会话。
  HttpSession(
      boost::asio::ip::tcp::socket socket,
      RequestHandler request_handler,
      CloseHandler close_handler,
      std::chrono::milliseconds request_timeout);

  // 启动请求读取流程。
  void start();
  // 主动停止本会话并释放连接资源。
  void stop();

 private:
  // 处理异步读取完成事件。
  void onRead(boost::beast::error_code error, std::size_t bytes_transferred);
  // 将路由结果转换为 HTTP 响应并异步写出。
  void sendResponse(const HttpControlResult& result);
  // 处理异步写出完成事件。
  void onWrite(boost::beast::error_code error, std::size_t bytes_transferred);
  // 关闭底层连接，并只通知关闭回调一次。
  void close();

  boost::beast::tcp_stream stream_;
  boost::beast::flat_buffer buffer_;
  boost::beast::http::request<boost::beast::http::string_body> request_;
  boost::beast::http::response<boost::beast::http::string_body> response_;
  RequestHandler request_handler_;
  CloseHandler close_handler_;
  std::chrono::milliseconds request_timeout_;
  bool closed_{false};
};

}  // namespace http
}  // namespace cleanbot
