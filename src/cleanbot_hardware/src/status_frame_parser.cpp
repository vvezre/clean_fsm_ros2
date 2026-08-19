/*
 * 文件作用：校验并解析旧下位机固定23字节状态帧。
 *
 * 输入必须已经由FrameBuffer按固定长度提取；这里再次验证0x7B帧头和0x7D帧尾。
 * 输出供LowerMachineNode发布/hardware/status并生成0xBB动作完成事件。
 * 旧状态帧没有当前实现可验证的XOR和命令序号，不能把解析成功当成独立ACK。
 */
#include "cleanbot_hardware/status_frame_parser.hpp"

#include <cmath>

namespace cleanbot {
namespace hardware {

namespace {
// 23字节是唯一接受的状态格式；旧14字节短完成帧已明确不再支持。
constexpr std::uint8_t kFrameStart = 0x7b;
constexpr std::uint8_t kFrameEnd = 0x7d;
constexpr std::uint8_t kFinished = 0xbb;
constexpr std::size_t kFrameLength = 23u;
}  // namespace

/*
 * 解析旧下位机的固定23字节状态帧。数组下标与旧协议字段对应如下：
 *
 * 字节   长度  字段                         解析规则
 * 0      1     帧头                         固定0x7B
 * 1      1     当前状态                     status
 * 2      1     上电状态                     power_on
 * 3      1     硬件状态                     hardware_state
 * 4..5   2     直行速度                     int16、大端序
 * 6..7   2     转向速度                     int16、大端序
 * 8      1     滚刷速度                     原始字节
 * 9      1     边缘状态                     0x00表示edge_clear
 * 10     1     电池百分比                   原始整数百分比
 * 11     1     air状态                      原始字节
 * 12     1     移动完成                     0xBB表示完成
 * 13     1     旋转完成                     0xBB表示完成
 * 14..15 2     电池组电压                   uint16、大端序、单位0.01V
 * 16..17 2     角度                         uint16、大端序
 * 18..19 2     里程计原始值                 uint16、大端序
 * 20..21 2     未定义                       当前实现忽略
 * 22     1     帧尾                         固定0x7D
 *
 * 旧状态帧没有可用的独立XOR和命令序号。解析成功只说明帧结构有效，
 * 不能证明它是某条ROS2命令的精确ACK。失败不抛异常，只返回稳定错误码。
 */
StatusParseResult StatusFrameParser::parse(
    const std::vector<std::uint8_t>& frame) const {
  StatusParseResult result;
  if (frame.empty() || frame.front() != kFrameStart) {
    result.error = "frame_start_missing";
    return result;
  }
  if (frame.size() != kFrameLength) {
    result.error = "frame_length_invalid";
    return result;
  }
  if (frame.back() != kFrameEnd) {
    result.error = "frame_end_missing";
    return result;
  }

  result.status.full_frame = true;
  // 字节1..3：当前命令/模式、上电状态和下位机自身状态。
  result.status.status = frame[1];
  result.status.power_on = frame[2];
  result.status.hardware_state = frame[3];
  // 字节4..7：同时保留原始无符号值和按二进制补码解释的有符号速度。
  result.status.x_speed_raw = readU16(frame, 4u);
  result.status.z_speed_raw = readU16(frame, 6u);
  result.status.x_speed = readI16(frame, 4u);
  result.status.z_speed = readI16(frame, 6u);
  // 字节8..11：滚刷、边缘、电量百分比和旧air状态。
  result.status.brush_speed = frame[8];
  result.status.edge_clear = frame[9] == 0x00;
  result.status.battery_percent = static_cast<double>(frame[10]);
  result.status.air_state = frame[11];
  // 字节12/13只有等于0xBB才表示移动/旋转完成。
  result.status.move_finished = frame[12] == kFinished;
  result.status.rotate_finished = frame[13] == kFinished;
  // 电压原始值单位为0.01V，ROS2状态按0.1V精度发布。
  result.status.pack_voltage =
      std::round(static_cast<double>(readU16(frame, 14u)) * 0.01 * 10.0) / 10.0;
  result.status.angle = readU16(frame, 16u);
  result.status.odometer = readU16(frame, 18u);
  // 字节20/21在已部署旧Python协议中未定义，不推测或复用为命令序号。
  result.status.command_sequence = 0u;
  result.parsed = true;
  return result;
}

// 将大端uint16按二进制补码还原为int16，避免依赖实现定义的强制转换。
std::int16_t StatusFrameParser::readI16(
    const std::vector<std::uint8_t>& frame, const std::size_t index) {
  const std::uint16_t raw = readU16(frame, index);
  const std::int32_t signed_value = raw >= 0x8000u
      ? static_cast<std::int32_t>(raw) - 0x10000
      : static_cast<std::int32_t>(raw);
  return static_cast<std::int16_t>(signed_value);
}

// 从指定偏移读取一个大端无符号16位值；调用方保证偏移位于23字节帧内。
std::uint16_t StatusFrameParser::readU16(
    const std::vector<std::uint8_t>& frame, const std::size_t index) {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(frame[index]) << 8u) |
      static_cast<std::uint16_t>(frame[index + 1u]));
}

}  // namespace hardware
}  // namespace cleanbot
