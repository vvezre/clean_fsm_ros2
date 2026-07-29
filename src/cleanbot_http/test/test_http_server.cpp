#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include "cleanbot_http/http_server.hpp"

namespace {

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace beast_http = boost::beast::http;
using tcp = asio::ip::tcp;

cleanbot::http::HttpControlResult successResult(
    const std::string&,
    const std::string&) {
  cleanbot::http::HttpControlResult result;
  result.status_code = 200;
  result.body = "1";
  result.content_type = "text/plain; charset=utf-8";
  return result;
}

class RunningServer {
 public:
  explicit RunningServer(
      const std::chrono::milliseconds timeout =
          std::chrono::milliseconds(100))
      : work_guard_(asio::make_work_guard(io_context_)),
        server_(std::make_shared<cleanbot::http::HttpServer>(
            io_context_,
            "127.0.0.1",
            0u,
            successResult,
            cleanbot::http::HttpServer::ErrorHandler(),
            timeout)) {
    server_->start();
    thread_ = std::thread([this]() { io_context_.run(); });
  }

  ~RunningServer() {
    stop();
  }

  void stop() {
    if (stopped_) {
      return;
    }
    stopped_ = true;
    server_->stop();
    work_guard_.reset();
    io_context_.stop();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  std::uint16_t port() const {
    return server_->port();
  }

 private:
  asio::io_context io_context_;
  asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
  std::shared_ptr<cleanbot::http::HttpServer> server_;
  std::thread thread_;
  bool stopped_{false};
};

tcp::socket connectTo(const std::uint16_t port, asio::io_context& io_context) {
  tcp::socket socket(io_context);
  socket.connect(
      tcp::endpoint(asio::ip::make_address("127.0.0.1"), port));
  return socket;
}

beast_http::response<beast_http::string_body> sendCompleteRequest(
    const std::uint16_t port,
    const std::string& target,
    const beast_http::verb method = beast_http::verb::get) {
  asio::io_context io_context;
  beast::tcp_stream stream(io_context);
  stream.connect(
      tcp::endpoint(asio::ip::make_address("127.0.0.1"), port));
  beast_http::request<beast_http::empty_body> request{
      method, target, 11};
  request.set(beast_http::field::host, "127.0.0.1");
  beast_http::write(stream, request);

  beast::flat_buffer buffer;
  beast_http::response<beast_http::string_body> response;
  beast_http::read(stream, buffer, response);
  return response;
}

bool waitForPeerClose(
    tcp::socket& socket,
    const std::chrono::milliseconds timeout) {
  socket.non_blocking(true);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  char value = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    beast::error_code error;
    const auto bytes = socket.read_some(asio::buffer(&value, 1u), error);
    if (bytes == 0u &&
        (error == asio::error::eof ||
         error == asio::error::connection_reset ||
         error == asio::error::operation_aborted)) {
      return true;
    }
    if (!error && bytes > 0u) {
      continue;
    }
    if (error != asio::error::would_block &&
        error != asio::error::try_again) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

TEST(HttpServer, PartialHeaderTimesOutAndClosesConnection) {
  RunningServer server;
  asio::io_context client_io;
  auto socket = connectTo(server.port(), client_io);
  asio::write(
      socket,
      asio::buffer(
          std::string("GET /vehicle/parking HTTP/1.1\r\nHost:")));

  EXPECT_TRUE(waitForPeerClose(socket, std::chrono::milliseconds(1000)));
}

TEST(HttpServer, PartialClientDoesNotBlockAnotherRequest) {
  RunningServer server(std::chrono::milliseconds(500));
  asio::io_context client_io;
  auto partial = connectTo(server.port(), client_io);
  asio::write(
      partial,
      asio::buffer(
          std::string("GET /vehicle/parking HTTP/1.1\r\nHost:")));

  const auto response =
      sendCompleteRequest(server.port(), "/vehicle/parking");
  EXPECT_EQ(response.result_int(), 200);
  EXPECT_EQ(response.body(), "1");
}

TEST(HttpServer, PostResponseAdvertisesPostCorsMethod) {
  RunningServer server;

  const auto response = sendCompleteRequest(
      server.port(),
      "/api/v1/mission/pause",
      beast_http::verb::post);

  const auto methods = response["Access-Control-Allow-Methods"];
  EXPECT_NE(methods.find("POST"), beast::string_view::npos);
}

TEST(HttpServer, StopClosesActiveSessionAndReturns) {
  RunningServer server(std::chrono::seconds(5));
  asio::io_context client_io;
  auto socket = connectTo(server.port(), client_io);
  asio::write(
      socket,
      asio::buffer(
          std::string("GET /vehicle/parking HTTP/1.1\r\nHost:")));

  const auto started_at = std::chrono::steady_clock::now();
  server.stop();
  const auto elapsed = std::chrono::steady_clock::now() - started_at;
  EXPECT_LT(elapsed, std::chrono::seconds(2));
}

TEST(HttpServer, PortConflictThrowsDuringConstruction) {
  asio::io_context first_io;
  auto first = std::make_shared<cleanbot::http::HttpServer>(
      first_io, "127.0.0.1", 0u, successResult);
  const auto occupied_port = first->port();

  asio::io_context second_io;
  EXPECT_THROW(
      std::make_shared<cleanbot::http::HttpServer>(
          second_io, "127.0.0.1", occupied_port, successResult),
      std::runtime_error);
}

}  // namespace
