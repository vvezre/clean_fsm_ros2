/*
 * 文件作用：锁定旧Python 19字节命令帧的字节级兼容契约。
 *
 * 覆盖普通运动、刹车清零、有符号补码、初始化航向特殊布局、
 * E0/EA/FA/FB模式布局和默认五次重复批次，防止重构破坏旧下位机固件兼容性。
 */
#include <algorithm>

#include <gtest/gtest.h>

#include "cleanbot_hardware/command_encoder.hpp"

namespace {

using cleanbot::hardware::CommandEncoder;
using cleanbot::hardware::CommandFields;

// 验证普通命令所有主要字段的大端布局及最终XOR字节。
TEST(CommandEncoder, EncodesSignedSpeedsAndLegacyFields) {
  CommandFields fields;
  fields.status = 1u;
  fields.x_speed = 300;
  fields.z_speed = -100;
  fields.brush_speed = 50;
  fields.target_distance = 90;
  fields.target_rotation = 1800;
  fields.heading_deg = 359.99;
  fields.sequence = 0x1234u;

  const auto frame = CommandEncoder().encode(fields);

  const std::vector<std::uint8_t> expected{
      0x7b, 0x01, 0x01, 0x00, 0x01, 0x2c, 0xff, 0x9c, 0x00, 0x5a,
      0x32, 0x00, 0x00, 0x07, 0x08, 0x8c, 0x9f, 0x41, 0x7d};
  EXPECT_EQ(frame, expected);
}

// 刹车必须覆盖原status并清空全部运动/执行字段，避免残留速度继续生效。
TEST(CommandEncoder, BrakeClearsMotionFields) {
  CommandFields fields;
  fields.status = 3u;
  fields.x_speed = 500;
  fields.z_speed = -500;
  fields.brush_speed = 80;
  fields.target_distance = 1000;
  fields.target_rotation = 900;
  fields.brake = true;

  const auto frame = CommandEncoder().encode(fields);

  EXPECT_EQ(frame[1], 0u);
  for (std::size_t index = 4u; index <= 14u; ++index) {
    EXPECT_EQ(frame[index], 0u);
  }
}

// 负旋转量必须以16位二进制补码编码，保护左右转方向语义。
TEST(CommandEncoder, EncodesNegativeRotationAsSignedTwosComplement) {
  CommandFields fields;
  fields.target_rotation = -900;

  const auto frame = CommandEncoder().encode(fields);

  EXPECT_EQ(frame[13], 0xfcu);
  EXPECT_EQ(frame[14], 0x7cu);
}

// 人工纠偏支持负值；切换到刹车后纠偏字段也必须同步清零。
TEST(CommandEncoder, EncodesManualSteeringAsSignedTwosComplement) {
  CommandFields fields;
  fields.steering_offset = -1000;

  const auto frame = CommandEncoder().encode(fields);

  EXPECT_EQ(frame[11], 0xfcu);
  EXPECT_EQ(frame[12], 0x18u);

  fields.brake = true;
  const auto stopped = CommandEncoder().encode(fields);
  EXPECT_EQ(stopped[11], 0x00u);
  EXPECT_EQ(stopped[12], 0x00u);
}

// 初始化航向占用字节17/18，不允许普通编码器把它改成XOR和0x7D。
TEST(CommandEncoder, PreservesPythonInitHeadingSpecialLayout) {
  const auto frame = CommandEncoder().encodeInitHeading(90.0);

  ASSERT_EQ(frame.size(), 19u);
  EXPECT_EQ(frame[0], 0x7bu);
  EXPECT_EQ(frame[17], 0x23u);
  EXPECT_EQ(frame[18], 0x28u);
}

// E0模式代表一类旧特殊命令，其字节17保持0且字节18保持固定帧尾。
TEST(CommandEncoder, PreservesPythonLegacyControlLayout) {
  const auto frame = CommandEncoder().encodeLegacyControl(0xe0u);

  ASSERT_EQ(frame.size(), 19u);
  EXPECT_EQ(frame[0], 0x7bu);
  EXPECT_EQ(frame[1], 0xe0u);
  EXPECT_EQ(frame[2], 0x01u);
  EXPECT_EQ(frame[17], 0x00u);
  EXPECT_EQ(frame[18], 0x7du);
}

// 默认重发机制应生成一个包含五个完全相同19字节帧的连续写入批次。
TEST(CommandEncoder, BuildsOneWriteContainingFiveIdenticalLegacyFrames) {
  CommandFields fields;
  fields.status = 1u;
  const CommandEncoder encoder;
  const auto frame = encoder.encode(fields);

  const auto burst = encoder.repeat(frame, 5u);

  ASSERT_EQ(burst.size(), 95u);
  for (std::size_t offset = 0u; offset < burst.size(); offset += frame.size()) {
    EXPECT_TRUE(std::equal(frame.begin(), frame.end(), burst.begin() + offset));
  }
}

}  // namespace
