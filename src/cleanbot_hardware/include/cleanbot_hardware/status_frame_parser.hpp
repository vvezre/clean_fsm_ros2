/*
 * 文件作用：声明旧下位机固定23字节状态帧的解析结果和解析器。
 *
 * 输入：FrameBuffer提取出的单个23字节候选帧。
 * 输出：结构化硬件状态或稳定的解析错误码。
 * 边界：旧状态帧没有当前实现可用的XOR字段，只校验帧头、长度和固定帧尾。
 */
#ifndef CLEANBOT_HARDWARE__STATUS_FRAME_PARSER_HPP_
#define CLEANBOT_HARDWARE__STATUS_FRAME_PARSER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace cleanbot {
namespace hardware {

// 与旧状态帧一一对应的内部数据；节点会再转换成HardwareStatus ROS2消息。
struct HardwareStatusPatch {
  bool full_frame{false};
  std::uint8_t status{0};
  std::uint8_t power_on{0};
  std::uint8_t hardware_state{0};
  std::uint16_t x_speed_raw{0};
  std::uint16_t z_speed_raw{0};
  std::int16_t x_speed{0};
  std::int16_t z_speed{0};
  std::uint8_t brush_speed{0};
  bool edge_clear{false};
  double battery_percent{0.0};
  std::uint8_t air_state{0};
  // 第12/13字节等于0xBB时分别表示移动/旋转完成。
  bool move_finished{false};
  bool rotate_finished{false};
  double pack_voltage{0.0};
  std::uint16_t angle{0};
  std::uint16_t odometer{0};
  // 仅为兼容ROS2消息保留；旧状态帧没有命令序号，因此解析结果固定为0。
  std::uint16_t command_sequence{0};
};

// 解析成功时parsed为true且status有效；失败时error给出机器可读原因。
struct StatusParseResult {
  bool parsed{false};
  std::string error;
  HardwareStatusPatch status;
};

// 无状态解析器，可重复用于每个完整候选帧。
class StatusFrameParser {
 public:
  // 校验并解析一帧固定23字节的旧协议状态数据。
  StatusParseResult parse(const std::vector<std::uint8_t>& frame) const;

 private:
  // 从指定偏移读取两个字节，并按旧协议的大端序解释为无符号整数。
  static std::uint16_t readU16(
      const std::vector<std::uint8_t>& frame, std::size_t index);

  // 从指定偏移读取两个字节，先按大端序读取，再按二进制补码解释为有符号整数。
  static std::int16_t readI16(
      const std::vector<std::uint8_t>& frame, std::size_t index);
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__STATUS_FRAME_PARSER_HPP_
