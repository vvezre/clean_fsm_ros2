/*
 * 文件作用：声明旧Python下位机19字节命令帧的字段模型和编码器。
 *
 * 输入：ROS2节点整理后的速度、距离、转角、航向、滚刷、刹车和充电字段。
 * 输出：可直接交给串口传输层的字节帧或重复发送批次。
 * 边界：这里只编码协议，不访问串口；旧协议不传输ROS2命令序号。
 */
#ifndef CLEANBOT_HARDWARE__COMMAND_ENCODER_HPP_
#define CLEANBOT_HARDWARE__COMMAND_ENCODER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cleanbot {
namespace hardware {

// 普通命令的中间字段模型；数值范围会在编码时收敛到旧协议可表达范围。
struct CommandFields {
  // 旧协议控制头字段。
  std::uint8_t status{0};
  std::uint8_t power_on{1};
  std::uint8_t hardware_control{0};
  // 运动、执行器和有限动作字段。
  std::int32_t x_speed{0};
  std::int32_t z_speed{0};
  std::int32_t steering_offset{0};
  std::int32_t brush_speed{0};
  std::int32_t target_distance{0};
  std::int32_t target_rotation{0};
  // 角度编码前会归一化到[0, 360)，并转换为0.01度单位。
  double heading_deg{0.0};
  // 仅供ROS2侧关联业务命令；旧串口协议不传输sequence。
  std::uint16_t sequence{0};
  // 安全和充电语义优先覆盖普通status字段。
  bool brake{false};
  bool charge{false};
};

// 纯协议编码器：无共享状态，可在节点和单元测试中直接创建使用。
class CommandEncoder {
 public:
  // 旧Python协议的普通命令固定为19字节。
  static constexpr std::size_t kCommandLength = 19u;

  // 编码运动、定距、转向、滚刷和停止命令：字节17为0..16异或值，字节18为0x7D。
  std::vector<std::uint8_t> encode(const CommandFields& fields) const;

  // 编码旧Python初始化航向命令：字节17/18保存航向值，不再表示XOR和帧尾。
  std::vector<std::uint8_t> encodeInitHeading(double heading_deg) const;

  // 编码E0/EA/FA/FB特殊命令：字节17保持0，字节18保持0x7D，不重新计算普通XOR。
  std::vector<std::uint8_t> encodeLegacyControl(
      std::uint8_t command_type, std::uint8_t power_on = 1u) const;

  // 将同一帧连续复制为一次串口写入批次，保持旧Python重复发送机制。
  std::vector<std::uint8_t> repeat(
      const std::vector<std::uint8_t>& frame, std::size_t repeat_count) const;

 private:
  // 以大端序写入有符号16位值，越界值先截断到int16范围。
  static void writeI16(
      std::vector<std::uint8_t>& frame, std::size_t index, std::int32_t value);
  // 以大端序写入无符号16位值，负数归零，过大值截断为65535。
  static void writeU16(
      std::vector<std::uint8_t>& frame, std::size_t index, std::int32_t value);
  // 对普通帧字节0..16执行逐字节XOR，结果写入字节17。
  static std::uint8_t checksum(const std::vector<std::uint8_t>& frame);
};

}  // namespace hardware
}  // namespace cleanbot

#endif  // CLEANBOT_HARDWARE__COMMAND_ENCODER_HPP_
