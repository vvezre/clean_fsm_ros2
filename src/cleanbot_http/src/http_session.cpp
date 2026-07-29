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

HttpControlResult internal_error() {
  HttpControlResult result;
  result.status_code = 500;
  result.body =
      "{\"success\":false,\"code\":\"HTTP_HANDLER_ERROR\","
      "\"message\":\"HTTP request handler failed\"}";
  return result;
}

}  // namespace

HttpSession::HttpSession(
    tcp::socket socket,
    RequestHandler request_handler,
    CloseHandler close_handler,
    const std::chrono::milliseconds request_timeout)
    : stream_(std::move(socket)),
      request_handler_(std::move(request_handler)),
      close_handler_(std::move(close_handler)),
      request_timeout_(request_timeout) {}

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

void HttpSession::stop() {
  close();
}

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

void HttpSession::onWrite(
    const beast::error_code,
    const std::size_t) {
  close();
}

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
