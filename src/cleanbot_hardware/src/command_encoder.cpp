/*
 * 文件作用：实现旧Python下位机19字节命令帧编码。
 *
 * 普通帧：[0]=0x7B，[1..16]=控制字段，[17]=字节0..16的XOR，[18]=0x7D。
 * 特殊帧：初始化航向占用字节17/18；E0/EA/FA/FB保持旧布局且不计算普通XOR。
 * 本文件不发送串口，也不判断下位机是否执行成功。
 */
#include "cleanbot_hardware/command_encoder.hpp"

#include <algorithm>
#include <cmath>

namespace cleanbot {
namespace hardware {

namespace {
// 固定边界和校验位置来自已部署的旧Python协议，不能按新协议重新排列。
constexpr std::uint8_t kCommandFrameStart = 0x7b;
constexpr std::uint8_t kCommandFrameEnd = 0x7d;
constexpr std::uint8_t kStopStatus = 0x00;
constexpr std::uint8_t kChargeStatus = 0x05;
constexpr std::size_t kChecksumIndex = 17u;
constexpr std::size_t kChecksumInputLength = 17u;
}  // namespace

/*
 * 编码旧下位机的普通19字节命令帧。下面的数组下标就是下位机协议字段位置：
 *
 * 字节   长度  字段                         编码规则
 * 0      1     帧头                         固定0x7B
 * 1      1     命令类型                     刹车0x00、充电0x05或status
 * 2      1     上电标志                     power_on
 * 3      1     硬件控制标志                 hardware_control
 * 4..5   2     直行速度                     int16、大端序
 * 6..7   2     转向速度                     int16、大端序
 * 8..9   2     目标距离                     uint16、大端序、最大50000
 * 10     1     滚刷速度                     int8补码、范围[-100, 100]
 * 11..12 2     纠偏量                       int16、大端序
 * 13..14 2     目标旋转量                   int16、大端序
 * 15..16 2     航向角                       uint16、大端序、单位0.01度
 * 17     1     XOR校验                      字节0..16逐字节异或
 * 18     1     帧尾                         固定0x7D
 *
 * 返回值不是ROS2消息，而是可以直接写入旧下位机串口的协议字节。
 * 刹车和充电语义优先覆盖调用方提供的status。
 */
std::vector<std::uint8_t> CommandEncoder::encode(const CommandFields& fields) const {
  std::vector<std::uint8_t> frame(kCommandLength, 0u);
  frame[0] = kCommandFrameStart;
  frame[1] = fields.brake ? kStopStatus :
      (fields.charge ? kChargeStatus : fields.status);
  frame[2] = fields.power_on;
  frame[3] = fields.hardware_control;

  if (!fields.brake) {
    // 字节4..16均按旧协议大端序写入；刹车时这些运动字段保持为0。
    writeI16(frame, 4u, fields.x_speed);
    writeI16(frame, 6u, fields.z_speed);
    writeU16(frame, 8u, std::min(fields.target_distance, 50000));

    // 滚刷字段只有一个字节，旧协议用int8补码表达[-100, 100]。
    const auto brush = std::max(-100, std::min(100, fields.brush_speed));
    frame[10] = static_cast<std::uint8_t>(static_cast<std::int8_t>(brush));
    writeI16(frame, 11u, fields.steering_offset);
    writeI16(frame, 13u, fields.target_rotation);

    // 航向统一成[0, 360)后换算为0.01度，360.00度回绕为0。
    double normalized_heading = std::fmod(fields.heading_deg, 360.0);
    if (normalized_heading < 0.0) {
      normalized_heading += 360.0;
    }
    const auto heading_centidegrees =
        static_cast<std::int32_t>(std::round(normalized_heading * 100.0)) % 36000;
    writeU16(frame, 15u, heading_centidegrees);
  }

  // 普通帧最后写入XOR和固定帧尾，确保校验涵盖最终控制字段。
  frame[kChecksumIndex] = checksum(frame);
  frame[18] = kCommandFrameEnd;
  return frame;
}

// 保留旧sendInitHeading布局：航向直接占据最后两个字节，因此没有普通XOR和0x7D帧尾。
// 当前节点入口尚未调用该方法；保留它是为了协议兼容和后续显式接入。
std::vector<std::uint8_t> CommandEncoder::encodeInitHeading(const double heading_deg) const {
  std::vector<std::uint8_t> frame(kCommandLength, 0u);
  frame[0] = kCommandFrameStart;

  double normalized_heading = std::fmod(heading_deg, 360.0);
  if (normalized_heading < 0.0) {
    normalized_heading += 360.0;
  }
  const auto heading_centidegrees =
      static_cast<std::int32_t>(std::round(normalized_heading * 100.0)) % 36000;
  writeU16(frame, 17u, heading_centidegrees);
  return frame;
}

// 保留旧模式控制命令布局：只设置帧头、命令类型、上电位和帧尾，字节17保持0。
std::vector<std::uint8_t> CommandEncoder::encodeLegacyControl(
    const std::uint8_t command_type, const std::uint8_t power_on) const {
  std::vector<std::uint8_t> frame(kCommandLength, 0u);
  frame[0] = kCommandFrameStart;
  frame[1] = command_type;
  frame[2] = power_on;
  frame[18] = kCommandFrameEnd;
  return frame;
}

// 把同一完整帧无间隔拼接成一个写入批次；默认次数由配置中心提供。
std::vector<std::uint8_t> CommandEncoder::repeat(
    const std::vector<std::uint8_t>& frame, const std::size_t repeat_count) const {
  std::vector<std::uint8_t> burst;
  burst.reserve(frame.size() * repeat_count);
  for (std::size_t attempt = 0u; attempt < repeat_count; ++attempt) {
    burst.insert(burst.end(), frame.begin(), frame.end());
  }
  return burst;
}

// 将有符号值截断为int16，并按大端序写入其二进制补码。
void CommandEncoder::writeI16(
    std::vector<std::uint8_t>& frame, const std::size_t index, const std::int32_t value) {
  const auto clamped = std::max(-32768, std::min(32767, value));
  const auto encoded = static_cast<std::uint16_t>(static_cast<std::int16_t>(clamped));
  frame[index] = static_cast<std::uint8_t>((encoded >> 8u) & 0xffu);
  frame[index + 1u] = static_cast<std::uint8_t>(encoded & 0xffu);
}

// 将数值截断为uint16范围并按大端序写入。
void CommandEncoder::writeU16(
    std::vector<std::uint8_t>& frame, const std::size_t index, const std::int32_t value) {
  const auto clamped = std::max(0, std::min(65535, value));
  const auto encoded = static_cast<std::uint16_t>(clamped);
  frame[index] = static_cast<std::uint8_t>((encoded >> 8u) & 0xffu);
  frame[index + 1u] = static_cast<std::uint8_t>(encoded & 0xffu);
}

// 对字节0..16逐字节异或；特殊命令不调用此方法。
std::uint8_t CommandEncoder::checksum(const std::vector<std::uint8_t>& frame) {
  std::uint8_t value = 0u;
  for (std::size_t index = 0u; index < kChecksumInputLength; ++index) {
    value = static_cast<std::uint8_t>(value ^ frame[index]);
  }
  return value;
}

}  // namespace hardware
}  // namespace cleanbot
