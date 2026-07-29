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

namespace cleanbot {
namespace http {

// 异步 HTTP 监听器。构造时完成端口绑定，绑定失败直接抛出异常。
class HttpServer : public std::enable_shared_from_this<HttpServer> {
 public:
  using RequestHandler = HttpSession::RequestHandler;
  using ErrorHandler = std::function<void(const std::string& message)>;

  HttpServer(
      boost::asio::io_context& io_context,
      const std::string& listen_address,
      std::uint16_t port,
      RequestHandler request_handler,
      ErrorHandler error_handler = ErrorHandler(),
      std::chrono::milliseconds request_timeout =
          std::chrono::milliseconds(1000));
  ~HttpServer();

  void start();
  void stop();
  std::uint16_t port() const;

 private:
  using SessionSet = std::set<
      std::weak_ptr<HttpSession>,
      std::owner_less<std::weak_ptr<HttpSession>>>;

  void acceptNext();
  void onAccept(
      const std::shared_ptr<boost::asio::ip::tcp::socket>& socket,
      const boost::system::error_code& error);
  void removeSession(const std::shared_ptr<HttpSession>& session);
  void stopOnExecutor();
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
