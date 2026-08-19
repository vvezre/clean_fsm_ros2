/*
 * 文件作用：HTTP会话实现：解析请求、生成响应并处理连接读写。
 * 说明：本文件只负责本模块的实现逻辑，输入输出和线程约束以对应头文件为准。
 */
#include "cleanbot_http/http_session.hpp"

#include <utility>

#include <boost/asio/error.hpp>

namespace cleanbot {
namespace http {

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace beast_http = boost::beast::http;
using tcp = asio::ip::tcp;

namespace {

// 构造请求处理器异常时使用的固定 500 JSON 响应。
HttpControlResult internal_error() {
  HttpControlResult result;
  result.status_code = 500;
  result.body =
      "{\"success\":false,\"code\":\"HTTP_HANDLER_ERROR\","
      "\"message\":\"HTTP request handler failed\"}";
  return result;
}

}  // namespace

// 接管已连接 socket，并保存路由、关闭回调和单次请求超时。
HttpSession::HttpSession(
    tcp::socket socket,
    RequestHandler request_handler,
    CloseHandler close_handler,
    const std::chrono::milliseconds request_timeout)
    : stream_(std::move(socket)),
      request_handler_(std::move(request_handler)),
      close_handler_(std::move(close_handler)),
      request_timeout_(request_timeout) {}

// 设置读取期限并异步读取一条完整 HTTP 请求。
void HttpSession::start() {
  if (closed_) {
    return;
  }
  stream_.expires_after(request_timeout_);
  beast_http::async_read(
      stream_,
      buffer_,
      request_,
      [self = shared_from_this()](
          const beast::error_code error,
          const std::size_t bytes_transferred) {
        self->onRead(error, bytes_transferred);
      });
}

// 主动终止当前连接并触发统一关闭流程。
void HttpSession::stop() {
  close();
}

// 处理读取结果，将方法和路径交给路由器并捕获所有异常。
void HttpSession::onRead(
    const beast::error_code error,
    const std::size_t) {
  if (error) {
    close();
    return;
  }

  HttpControlResult result;
  try {
    const std::string method(
        request_.method_string().data(),
        request_.method_string().size());
    const std::string target(
        request_.target().data(),
        request_.target().size());
    result = request_handler_ ? request_handler_(method, target) : internal_error();
  } catch (...) {
    result = internal_error();
  }
  sendResponse(result);
}

// 将业务结果组装为禁用缓存和跨域允许的 HTTP 响应并异步发送。
void HttpSession::sendResponse(const HttpControlResult& result) {
  response_ = beast_http::response<beast_http::string_body>{
      static_cast<beast_http::status>(result.status_code),
      request_.version()};
  response_.set(beast_http::field::server, "cleanbot-http");
  response_.set(beast_http::field::content_type, result.content_type);
  response_.set("Access-Control-Allow-Origin", "*");
  response_.set("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  response_.set("Access-Control-Allow-Headers", "Content-Type");
  response_.set(beast_http::field::cache_control, "no-store");
  response_.keep_alive(false);
  response_.body() = result.body;
  response_.prepare_payload();

  stream_.expires_after(request_timeout_);
  beast_http::async_write(
      stream_,
      response_,
      [self = shared_from_this()](
          const beast::error_code error,
          const std::size_t bytes_transferred) {
        self->onWrite(error, bytes_transferred);
      });
}

// 响应发送结束后关闭一次性 HTTP 连接。
void HttpSession::onWrite(
    const beast::error_code,
    const std::size_t) {
  close();
}

// 幂等取消读写、关闭 socket，并通知服务器移除会话。
void HttpSession::close() {
  if (closed_) {
    return;
  }
  closed_ = true;

  beast::error_code ignored;
  stream_.socket().cancel(ignored);
  stream_.socket().shutdown(tcp::socket::shutdown_both, ignored);
  stream_.socket().close(ignored);

  auto close_handler = std::move(close_handler_);
  if (close_handler) {
    close_handler(shared_from_this());
  }
}

}  // namespace http
}  // namespace cleanbot
