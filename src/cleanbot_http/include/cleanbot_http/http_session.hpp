#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include "cleanbot_http/http_control_router.hpp"

namespace cleanbot {
namespace http {

// 管理一个 HTTP TCP 连接。读取、路由、响应和超时均通过 Asio 异步执行。
class HttpSession : public std::enable_shared_from_this<HttpSession> {
 public:
  using RequestHandler = std::function<HttpControlResult(
      const std::string& method,
      const std::string& target)>;
  using CloseHandler =
      std::function<void(const std::shared_ptr<HttpSession>& session)>;

  HttpSession(
      boost::asio::ip::tcp::socket socket,
      RequestHandler request_handler,
      CloseHandler close_handler,
      std::chrono::milliseconds request_timeout);

  void start();
  void stop();

 private:
  void onRead(boost::beast::error_code error, std::size_t bytes_transferred);
  void sendResponse(const HttpControlResult& result);
  void onWrite(boost::beast::error_code error, std::size_t bytes_transferred);
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
