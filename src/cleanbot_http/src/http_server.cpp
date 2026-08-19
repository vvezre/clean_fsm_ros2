/*
 * 文件作用：HTTP服务器实现：监听端口、接收连接并管理会话生命周期。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_http/http_server.hpp"

#include <future>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <boost/asio/error.hpp>
#include <boost/asio/post.hpp>

namespace cleanbot {
namespace http {

namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace {

// 将监听端点操作和 Boost 错误信息组合为启动异常。
std::runtime_error endpoint_error(
    const std::string& operation,
    const std::string& address,
    const std::uint16_t port,
    const boost::system::error_code& error) {
  std::ostringstream message;
  message << "HTTP " << operation << " failed for "
          << address << ':' << port << ": " << error.message();
  return std::runtime_error(message.str());
}

}  // namespace

// 创建 acceptor，校验地址并完成监听端口绑定。
HttpServer::HttpServer(
    asio::io_context& io_context,
    const std::string& listen_address,
    const std::uint16_t port,
    RequestHandler request_handler,
    ErrorHandler error_handler,
    const std::chrono::milliseconds request_timeout)
    : io_context_(io_context),
      strand_(asio::make_strand(io_context)),
      acceptor_(strand_),
      request_handler_(std::move(request_handler)),
      error_handler_(std::move(error_handler)),
      request_timeout_(request_timeout) {
  boost::system::error_code error;
  const auto address = asio::ip::make_address(listen_address, error);
  if (error) {
    throw endpoint_error("address parsing", listen_address, port, error);
  }
  const tcp::endpoint endpoint(address, port);

  acceptor_.open(endpoint.protocol(), error);
  if (error) {
    throw endpoint_error("open", listen_address, port, error);
  }
  acceptor_.set_option(asio::socket_base::reuse_address(true), error);
  if (error) {
    throw endpoint_error("set_option", listen_address, port, error);
  }
  acceptor_.bind(endpoint, error);
  if (error) {
    throw endpoint_error("bind", listen_address, port, error);
  }
  acceptor_.listen(asio::socket_base::max_listen_connections, error);
  if (error) {
    throw endpoint_error("listen", listen_address, port, error);
  }
  const auto local_endpoint = acceptor_.local_endpoint(error);
  if (error) {
    throw endpoint_error("local_endpoint", listen_address, port, error);
  }
  port_ = local_endpoint.port();
}

// 析构时保证监听器和活动会话均已停止。
HttpServer::~HttpServer() {
  boost::system::error_code ignored;
  acceptor_.cancel(ignored);
  acceptor_.close(ignored);
}

// 在 Asio 执行器上启动首次异步连接接受操作。
void HttpServer::start() {
  if (started_.exchange(true) || stopping_.load()) {
    return;
  }
  asio::post(
      strand_,
      // 异步任务作用：在 I/O 线程中串行执行当前状态变更或资源操作。
      [self = shared_from_this()]() {
        self->acceptNext();
      });
}

// 线程安全地停止服务器，并等待执行器完成资源关闭。
void HttpServer::stop() {
  std::lock_guard<std::mutex> stop_lock(stop_mutex_);
  if (stopped_.load()) {
    return;
  }
  stopping_.store(true);

  if (!started_.load() || io_context_.stopped()) {
    stopOnExecutor();
    return;
  }
  if (strand_.running_in_this_thread()) {
    stopOnExecutor();
    return;
  }

  auto completed = std::make_shared<std::promise<void>>();
  auto future = completed->get_future();
  asio::post(
      strand_,
      // 异步任务作用：在 I/O 线程中串行执行当前状态变更或资源操作。
      [self = shared_from_this(), completed]() {
        self->stopOnExecutor();
        completed->set_value();
      });
  if (future.wait_for(std::chrono::seconds(2)) !=
      std::future_status::ready) {
    throw std::runtime_error("HTTP server stop timed out");
  }
}

// 返回 acceptor 实际绑定的本地端口。
std::uint16_t HttpServer::port() const {
  return port_;
}

// 创建新 socket 并投递下一次异步 accept。
void HttpServer::acceptNext() {
  if (stopping_.load() || !acceptor_.is_open()) {
    return;
  }
  auto socket = std::make_shared<tcp::socket>(strand_);
  acceptor_.async_accept(
      *socket,
      [self = shared_from_this(), socket](
          const boost::system::error_code error) {
        self->onAccept(socket, error);
      });
}

// 处理连接接受结果，创建会话后继续接受下一连接。
void HttpServer::onAccept(
    const std::shared_ptr<tcp::socket>& socket,
    const boost::system::error_code& error) {
  if (error) {
    if (!stopping_.load() && error != asio::error::operation_aborted) {
      reportError("HTTP accept failed: " + error.message());
      acceptNext();
    }
    return;
  }

  std::weak_ptr<HttpServer> weak_server = shared_from_this();
  auto session = std::make_shared<HttpSession>(
      std::move(*socket),
      request_handler_,
      // 匿名函数作用：封装当前局部回调或判定逻辑，供调用方在本作用域内执行。
      [weak_server](const std::shared_ptr<HttpSession>& closed_session) {
        if (const auto server = weak_server.lock()) {
          server->removeSession(closed_session);
        }
      },
      request_timeout_);
  sessions_.insert(session);
  session->start();
  acceptNext();
}

// 从活动集合中移除已关闭会话的弱引用。
void HttpServer::removeSession(
    const std::shared_ptr<HttpSession>& session) {
  sessions_.erase(std::weak_ptr<HttpSession>(session));
}

// 在执行器线程关闭 acceptor，并停止全部活动会话。
void HttpServer::stopOnExecutor() {
  if (stopped_.exchange(true)) {
    return;
  }
  stopping_.store(true);

  boost::system::error_code ignored;
  acceptor_.cancel(ignored);
  acceptor_.close(ignored);

  std::vector<std::shared_ptr<HttpSession>> active_sessions;
  active_sessions.reserve(sessions_.size());
  for (const auto& weak_session : sessions_) {
    if (const auto session = weak_session.lock()) {
      active_sessions.push_back(session);
    }
  }
  sessions_.clear();
  for (const auto& session : active_sessions) {
    session->stop();
  }
}

// 将网络错误交给上层回调，未配置回调时保持静默。
void HttpServer::reportError(const std::string& message) const {
  if (error_handler_) {
    error_handler_(message);
  }
}

}  // namespace http
}  // namespace cleanbot
