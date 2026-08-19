/*
 * 文件作用：车辆手动控制、状态查询和任务业务接口的 C++ HTTP/ROS2 入口。
 *
 * HttpServer/HttpSession 负责异步网络通信；本节点只负责读取配置、解析控制请求，
 * 并在 HTTP、ROS2 Service 与 Topic 之间适配。命令仲裁和下位机串口仍由原节点负责。
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>

#include "cleanbot_common/qos_profiles.hpp"
#include "cleanbot_config/config_client.hpp"
#include "cleanbot_http/http_control_router.hpp"
#include "cleanbot_http/http_server.hpp"
#include "cleanbot_http/runtime_deadlines.hpp"
#include "cleanbot_http/service_response_mapping.hpp"
#include "cleanbot_http/user_control_request.hpp"
#include "cleanbot_http/vehicle_state_json.hpp"
#include "cleanbot_interfaces/msg/vehicle_command.hpp"
#include "cleanbot_interfaces/msg/vehicle_state.hpp"
#include "cleanbot_interfaces/srv/emergency_stop.hpp"
#include "cleanbot_interfaces/srv/execute_model_plan.hpp"
#include "cleanbot_interfaces/srv/manual_control.hpp"
#include "cleanbot_interfaces/srv/set_mission_pause.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cleanbot {
namespace http {

namespace asio = boost::asio;

class HttpGatewayNode : public rclcpp::Node {
 public:
  // 构造 HTTP 网关节点，连接 ROS2 接口并准备网络服务运行环境。
  HttpGatewayNode() : Node("http_gateway_node") {
    manual_publisher_ = create_publisher<VehicleCommand>(
        "/control/manual_cmd", common::latest_command_qos());
    emergency_publisher_ = create_publisher<VehicleCommand>(
        "/control/emergency_cmd", common::latest_command_qos());
    vehicle_state_subscription_ =
        create_subscription<cleanbot_interfaces::msg::VehicleState>(
        "/vehicle/state",
        common::latched_status_qos(),
        [this](
            const cleanbot_interfaces::msg::VehicleState::SharedPtr state) {
          std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
          latest_vehicle_state_ = *state;
          has_vehicle_state_ = true;
        });
    mission_pause_client_ =
        create_client<cleanbot_interfaces::srv::SetMissionPause>(
        "/mission/set_pause");
    execute_plan_client_ =
        create_client<cleanbot_interfaces::srv::ExecuteModelPlan>(
        "/modeling/execute_plan");
    manual_service_ =
        create_service<cleanbot_interfaces::srv::ManualControl>(
        "/control/manual",
        std::bind(
            &HttpGatewayNode::onManualControl,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    emergency_service_ =
        create_service<cleanbot_interfaces::srv::EmergencyStop>(
        "/control/emergency_stop",
        std::bind(
            &HttpGatewayNode::onEmergencyStop,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    config_client_ = std::make_unique<config::ConfigClient>(
        this,
        std::vector<std::string>{
            "http.enabled",
            "http.listen_address",
            "http.port",
            "motion.manual_max_speed",
            "tracking.max_z_speed",
        },
        false,
        // 配置回调作用：接收最新配置快照，并刷新本节点对应的运行参数。
        [this](const config::ConfigSnapshot& snapshot, const bool initial) {
          configure(snapshot, initial);
        });
  }

  // 析构节点时停止 HTTP 服务，避免后台网络线程继续访问已释放资源。
  ~HttpGatewayNode() override {
    stopServer();
  }

 private:
  using VehicleCommand = cleanbot_interfaces::msg::VehicleCommand;
  using WorkGuard =
      asio::executor_work_guard<asio::io_context::executor_type>;

  // 读取 HTTP 和手动控制参数；首次配置决定是否启动服务端。
  void configure(const config::ConfigSnapshot& snapshot, const bool initial) {
    cleanbot::control::JoystickParameters parameters;
    parameters.max_linear_speed = static_cast<std::int32_t>(
        snapshot.get_integer("motion.manual_max_speed"));
    manual_max_speed_.store(parameters.max_linear_speed);
    maximum_z_speed_.store(static_cast<std::int32_t>(
        snapshot.get_integer("tracking.max_z_speed")));
    control_ready_.store(true);
    {
      std::lock_guard<std::mutex> lock(router_mutex_);
      if (router_) {
        // 只更新摇杆映射参数，保留已接收的会话序号，避免旧命令重新变为有效。
        router_->updateParameters(parameters);
      } else {
        router_ = std::make_unique<HttpControlRouter>(parameters);
      }
    }

    if (!initial) {
      RCLCPP_INFO(
          get_logger(),
          "HTTP manual maximum speed updated to %d and is effective immediately",
          parameters.max_linear_speed);
      return;
    }

    if (!snapshot.get_boolean("http.enabled")) {
      RCLCPP_WARN(
          get_logger(),
          "HTTP control gateway is disabled; ROS2 internal control remains available");
      return;
    }

    startServer(
        snapshot.get_string("http.listen_address"),
        static_cast<std::uint16_t>(snapshot.get_integer("http.port")));
  }

  // 手动控制接口：先做边界校验，再把合格请求转成 ROS2 手动命令。
  void onManualControl(
      const std::shared_ptr<cleanbot_interfaces::srv::ManualControl::Request> request,
      std::shared_ptr<cleanbot_interfaces::srv::ManualControl::Response> response) {
    if (!control_ready_.load()) {
      response->accepted = false;
      response->code = "CONFIG_NOT_READY";
      response->message = "control configuration is not ready";
      return;
    }

    const auto decision = build_manual_command(
        request->source,
        request->x_speed,
        request->z_speed,
        request->brake,
        manual_max_speed_.load(),
        maximum_z_speed_.load());
    response->accepted = decision.accepted;
    response->code = decision.code;
    response->message = decision.message;
    if (decision.accepted) {
      publishUserControl(
          decision.command, VehicleCommand::PRIORITY_MANUAL);
    }
  }

  // 急停接口：只接受紧急语义，成功后直接发急停命令。
  void onEmergencyStop(
      const std::shared_ptr<cleanbot_interfaces::srv::EmergencyStop::Request> request,
      std::shared_ptr<cleanbot_interfaces::srv::EmergencyStop::Response> response) {
    const auto decision = build_emergency_command(
        request->source, request->reason);
    response->success = decision.accepted;
    response->code = decision.code;
    response->message = decision.message;
    if (decision.accepted) {
      publishUserControl(
          decision.command, VehicleCommand::PRIORITY_EMERGENCY);
    }
  }

  // 给 HTTP 请求补齐 ROS2 侧字段，并按优先级发布到对应控制主题。
  void publishUserControl(
      const UserControlCommand& input,
      const std::uint8_t priority) {
    VehicleCommand command;
    command.stamp = now();
    command.command_id = next_command_id_.fetch_add(1u);
    command.request_id = command.command_id;
    command.source = input.source;
    command.priority = priority;
    command.active = input.active;
    command.operator_intent = input.operator_intent;
    command.status = input.brake ? 0u : 1u;
    command.x_speed = input.x_speed;
    command.z_speed = input.z_speed;
    command.brake = input.brake;
    if (priority == VehicleCommand::PRIORITY_EMERGENCY) {
      emergency_publisher_->publish(command);
    } else {
      manual_publisher_->publish(command);
    }
  }

  // 启动异步 HTTP 服务线程；监听失败就让节点初始化失败，避免半可用状态。
  void startServer(const std::string& address, const std::uint16_t port) {
    if (server_thread_.joinable()) {
      RCLCPP_WARN(get_logger(), "HTTP control gateway is already running");
      return;
    }

    io_context_.restart();
    work_guard_ = std::make_unique<WorkGuard>(
        asio::make_work_guard(io_context_));
    try {
      server_ = std::make_shared<HttpServer>(
          io_context_,
          address,
          port,
          [this](
              const std::string& method,
              const std::string& target) {
            return handleRequest(method, target);
          },
          // 匿名函数作用：封装当前局部回调或判定逻辑，供调用方在本作用域内执行。
          [this](const std::string& message) {
            RCLCPP_WARN(get_logger(), "%s", message.c_str());
          },
          kHttpIoOperationDeadline);
      server_->start();
      // 线程入口作用：运行当前节点的 I/O 事件循环，直到收到停止请求。
      server_thread_ = std::thread([this]() { io_context_.run(); });
    } catch (const std::exception& exception) {
      work_guard_.reset();
      server_.reset();
      io_context_.stop();
      RCLCPP_FATAL(
          get_logger(),
          "HTTP control gateway failed to listen on %s:%u: %s",
          address.c_str(),
          static_cast<unsigned int>(port),
          exception.what());
      throw;
    }

    RCLCPP_INFO(
        get_logger(),
        "HTTP control gateway listening on %s:%u",
        address.c_str(),
        static_cast<unsigned int>(server_->port()));
  }

  // 关闭 acceptor 和 io_context，并等待线程退出，确保析构时没有悬挂回调。
  void stopServer() {
    if (server_) {
      try {
        // 先关闭 acceptor 和活动连接，再停止 io_context，保证退出任务能被执行。
        server_->stop();
      } catch (const std::exception& exception) {
        RCLCPP_ERROR(
            get_logger(),
            "HTTP control gateway stop failed: %s",
            exception.what());
      }
    }
    work_guard_.reset();
    io_context_.stop();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    server_.reset();
  }

  // 路由层：先做 HTTP 业务判定，再决定是返回 JSON、转 ROS2 Service，还是发控制命令。
  HttpControlResult handleRequest(
      const std::string& method,
      const std::string& target) {
    HttpControlResult result;
    {
      // 序号检查、序号更新和路由结果生成必须处于同一临界区。
      std::lock_guard<std::mutex> lock(router_mutex_);
      if (!router_) {
        result.status_code = 503;
        result.body =
            "{\"success\":false,\"code\":\"GATEWAY_NOT_READY\","
            "\"message\":\"HTTP control gateway is not ready\"}";
      } else {
        result = router_->route(method, target);
      }
    }

    if (result.action == HttpControlAction::kVehicleState) {
      return vehicleStateResponse();
    }
    if (result.action == HttpControlAction::kMissionPause) {
      return missionPauseResponse(true);
    }
    if (result.action == HttpControlAction::kMissionResume) {
      return missionPauseResponse(false);
    }
    if (result.action == HttpControlAction::kExecuteModelPlan) {
      return executePlanResponse(result.plan_id, result.brush_speed);
    }

    // 旧控制路由保持“发布即响应”，不把发布Topic解释为下位机执行完成。
    publishControl(result);
    return result;
  }

  // 将服务映射决策转换为 HTTP JSON 响应。
  static HttpControlResult jsonResponse(
      const int status_code,
      const bool success,
      const std::string& code,
      const std::string& message) {
    HttpControlResult result;
    result.status_code = status_code;
    result.content_type = "application/json; charset=utf-8";
    result.body = business_response_json(success, code, message);
    return result;
  }

  // 使用显式状态码和业务字段构造 HTTP JSON 响应。
  static HttpControlResult jsonResponse(
      const HttpBusinessDecision& decision) {
    return jsonResponse(
        decision.status_code,
        decision.success,
        decision.code,
        decision.message);
  }

  // 直接把最近一次车辆状态转成 JSON 返回给前端。
  HttpControlResult vehicleStateResponse() {
    cleanbot_interfaces::msg::VehicleState state;
    {
      std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
      if (!has_vehicle_state_) {
        return jsonResponse(
            503,
            false,
            "VEHICLE_STATE_UNAVAILABLE",
            "vehicle state has not been received");
      }
      state = latest_vehicle_state_;
    }

    VehicleStateJsonData data;
    data.stamp_sec = state.stamp.sec;
    data.stamp_nanosec = state.stamp.nanosec;
    data.control_state = state.control_state;
    data.health_state = state.health_state;
    data.fault_state = state.fault_state;
    data.current_action = state.current_action;
    data.message = state.message;
    data.start_ready = state.start_ready;
    data.parking = state.parking;
    data.cleaning = state.cleaning;
    data.rtk_fixed = state.rtk_fixed;
    data.in_garage = state.in_garage;
    data.battery_percent = state.battery_percent;
    data.current_segment = state.current_segment;
    data.total_segments = state.total_segments;

    HttpControlResult result;
    result.status_code = 200;
    result.content_type = "application/json; charset=utf-8";
    result.body = vehicle_state_json(data);
    return result;
  }

  // 暂停/恢复走 ROS2 Service，同步等待结果并映射成 HTTP 响应。
  HttpControlResult missionPauseResponse(const bool pause) {
    if (!mission_pause_client_->wait_for_service(
            std::chrono::seconds(0))) {
      return jsonResponse(map_mission_pause_response(
          DownstreamResponseState::kUnavailable,
          DownstreamResponse()));
    }

    auto request =
        std::make_shared<cleanbot_interfaces::srv::SetMissionPause::Request>();
    request->pause = pause;
    auto future = mission_pause_client_->async_send_request(request);
    if (future.wait_for(kRosServiceResponseDeadline) !=
        std::future_status::ready) {
      mission_pause_client_->remove_pending_request(future);
      return jsonResponse(map_mission_pause_response(
          DownstreamResponseState::kTimeout,
          DownstreamResponse()));
    }
    const auto response = future.get();
    DownstreamResponse downstream;
    downstream.accepted = response->success;
    downstream.code = response->code;
    downstream.message = response->message;
    return jsonResponse(map_mission_pause_response(
        DownstreamResponseState::kResponded, downstream));
  }

  // 执行计划请求先打到建模节点的 Service，再把返回结果映射成 HTTP 响应。
  HttpControlResult executePlanResponse(
      const std::string& plan_id,
      const std::int32_t brush_speed) {
    if (!execute_plan_client_->wait_for_service(
            std::chrono::seconds(0))) {
      return jsonResponse(map_execute_plan_response(
          DownstreamResponseState::kUnavailable,
          DownstreamResponse()));
    }

    auto request =
        std::make_shared<cleanbot_interfaces::srv::ExecuteModelPlan::Request>();
    request->plan_id = plan_id;
    request->brush_speed = brush_speed;
    auto future = execute_plan_client_->async_send_request(request);
    if (future.wait_for(kRosServiceResponseDeadline) !=
        std::future_status::ready) {
      execute_plan_client_->remove_pending_request(future);
      return jsonResponse(map_execute_plan_response(
          DownstreamResponseState::kTimeout,
          DownstreamResponse()));
    }
    const auto response = future.get();
    DownstreamResponse downstream;
    downstream.accepted = response->accepted;
    downstream.code = response->code;
    downstream.message = response->message;
    return jsonResponse(map_execute_plan_response(
        DownstreamResponseState::kResponded, downstream));
  }

  // 旧版控制路径：把 HTTP 结果翻译成最终控制主题，兼容现有前端请求。
  void publishControl(const HttpControlResult& result) {
    if (result.action != HttpControlAction::kManual &&
        result.action != HttpControlAction::kEmergencyStop) {
      return;
    }

    if (result.action == HttpControlAction::kEmergencyStop) {
      const auto decision = build_emergency_command(
          "http_emergency_stop", "legacy HTTP parking request");
      if (decision.accepted) {
        publishUserControl(
            decision.command, VehicleCommand::PRIORITY_EMERGENCY);
      }
      return;
    }

    VehicleCommand command;
    command.stamp = now();
    command.command_id = next_command_id_.fetch_add(1u);
    command.request_id = command.command_id;
    command.active = true;
    command.operator_intent = true;
    command.status = result.brake ? 0u : 1u;
    command.brake = result.brake;

    command.source = "http_joystick";
    command.priority = VehicleCommand::PRIORITY_MANUAL;
    command.x_speed = result.x_speed;
    command.steering_offset = result.steering_offset;
    manual_publisher_->publish(command);
  }

  std::unique_ptr<config::ConfigClient> config_client_;
  std::unique_ptr<HttpControlRouter> router_;
  std::mutex router_mutex_;
  std::mutex vehicle_state_mutex_;
  bool has_vehicle_state_{false};
  cleanbot_interfaces::msg::VehicleState latest_vehicle_state_;

  asio::io_context io_context_;
  std::unique_ptr<WorkGuard> work_guard_;
  std::shared_ptr<HttpServer> server_;
  std::thread server_thread_;

  std::atomic<std::uint64_t> next_command_id_{1u};
  std::atomic<bool> control_ready_{false};
  std::atomic<std::int32_t> manual_max_speed_{350};
  std::atomic<std::int32_t> maximum_z_speed_{15000};
  rclcpp::Publisher<VehicleCommand>::SharedPtr manual_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr emergency_publisher_;
  rclcpp::Subscription<cleanbot_interfaces::msg::VehicleState>::SharedPtr
      vehicle_state_subscription_;
  rclcpp::Client<cleanbot_interfaces::srv::SetMissionPause>::SharedPtr
      mission_pause_client_;
  rclcpp::Client<cleanbot_interfaces::srv::ExecuteModelPlan>::SharedPtr
      execute_plan_client_;
  rclcpp::Service<cleanbot_interfaces::srv::ManualControl>::SharedPtr
      manual_service_;
  rclcpp::Service<cleanbot_interfaces::srv::EmergencyStop>::SharedPtr
      emergency_service_;
};

}  // namespace http
}  // namespace cleanbot

// 初始化 ROS 2，运行 HTTP 网关节点，并在退出前停止网络资源。
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    const auto node = std::make_shared<cleanbot::http::HttpGatewayNode>();
    rclcpp::spin(node);
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(
        rclcpp::get_logger("http_gateway_node"),
        "HTTP gateway process exiting: %s",
        exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
