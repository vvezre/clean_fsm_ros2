#pragma once

#include <cstdint>
#include <string>

// 文件作用：声明来自用户控制接口的人工和急停指令构造与限幅规则。
namespace cleanbot {
namespace http {

// 可交给控制仲裁器的简化用户控制命令。
struct UserControlCommand {
  std::string source;
  bool active{true};
  bool operator_intent{true};
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  bool brake{true};
};

// 用户控制命令构造后的接受状态和失败原因。
struct UserControlDecision {
  bool accepted{false};
  std::string code;
  std::string message;
  UserControlCommand command;
};

// 校验限幅后构造人工驾驶命令。
UserControlDecision build_manual_command(
    const std::string& source,
    std::int32_t x_speed,
    std::int32_t z_speed,
    bool brake,
    std::int32_t maximum_x_speed,
    std::int32_t maximum_z_speed);

// 构造带来源和原因信息的软件急停命令。
UserControlDecision build_emergency_command(
    const std::string& source,
    const std::string& reason);

}  // namespace http
}  // namespace cleanbot
