#pragma once

#include <cstdint>
#include <string>

// 文件作用：声明车辆状态和通用业务结果到 HTTP JSON 文本的序列化接口。
namespace cleanbot {
namespace http {

// 生成车辆状态 JSON 所需的各项运行数据。
struct VehicleStateJsonData {
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0u};
  std::string control_state;
  std::string health_state;
  std::string fault_state;
  std::string current_action;
  std::string message;
  bool start_ready{false};
  bool parking{true};
  bool cleaning{false};
  bool rtk_fixed{false};
  bool in_garage{false};
  double battery_percent{-1.0};
  std::uint32_t current_segment{0u};
  std::uint32_t total_segments{0u};
};

// 将车辆运行数据编码为对外 HTTP 状态 JSON。
std::string vehicle_state_json(const VehicleStateJsonData& state);

// 将业务成败、编码、消息和可选数据包装为统一 JSON 响应。
std::string business_response_json(
    bool success,
    const std::string& code,
    const std::string& message,
    const std::string& data_json = "{}");

}  // namespace http
}  // namespace cleanbot
