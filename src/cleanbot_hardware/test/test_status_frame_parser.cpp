/*
 * 文件作用：验证旧下位机23字节状态帧的结构校验和字段解析。
 *
 * 测试明确拒绝已废弃的14字节短完成帧，并锁定大端速度、电压、里程和
 * 第12/13字节0xBB完成标志；旧帧没有命令序号，解析结果必须保持0。
 */
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "cleanbot_hardware/status_frame_parser.hpp"

namespace {

using cleanbot::hardware::StatusFrameParser;

// 用完整样例同时验证有符号速度、0xBB完成位、电压缩放和未定义序号边界。
TEST(StatusFrameParser, ParsesFullStatusUsingLegacyBigEndianFields) {
  const std::vector<std::uint8_t> frame = {
      0x7b, 0x01, 0x01, 0x02, 0x01, 0x2c, 0xff, 0x9c, 0x32, 0x00, 0x64,
      0x01, 0xbb, 0x00, 0x14, 0x1e, 0x00, 0x5a, 0x04, 0xd2, 0x12, 0x34, 0x7d};

  const auto result = StatusFrameParser().parse(frame);

  ASSERT_TRUE(result.parsed);
  EXPECT_TRUE(result.status.full_frame);
  EXPECT_EQ(result.status.status, 1);
  EXPECT_EQ(result.status.power_on, 1);
  EXPECT_EQ(result.status.hardware_state, 2);
  EXPECT_EQ(result.status.x_speed, 300);
  EXPECT_EQ(result.status.z_speed, -100);
  EXPECT_EQ(result.status.brush_speed, 50);
  EXPECT_TRUE(result.status.edge_clear);
  EXPECT_DOUBLE_EQ(result.status.battery_percent, 100.0);
  EXPECT_EQ(result.status.air_state, 1);
  EXPECT_TRUE(result.status.move_finished);
  EXPECT_FALSE(result.status.rotate_finished);
  EXPECT_DOUBLE_EQ(result.status.pack_voltage, 51.5);
  EXPECT_EQ(result.status.angle, 90);
  EXPECT_EQ(result.status.odometer, 1234);
  EXPECT_EQ(result.status.command_sequence, 0u);
}

// 旧14字节短完成帧已从当前契约移除，不能被误当成有效状态。
TEST(StatusFrameParser, RejectsFormerShortFinishFrame) {
  const std::vector<std::uint8_t> frame = {
      0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0xbb, 0x00};

  const auto result = StatusFrameParser().parse(frame);

  EXPECT_FALSE(result.parsed);
  EXPECT_EQ(result.error, "frame_length_invalid");
}

// 缺少0x7B帧头或不足23字节时返回稳定错误，不读取越界字段。
TEST(StatusFrameParser, RejectsUnsynchronizedAndTooShortFrames) {
  const auto unsynchronized = StatusFrameParser().parse({0x00, 0x01, 0x02, 0x03});
  const auto too_short = StatusFrameParser().parse({0x7b, 0x01, 0x02});

  EXPECT_FALSE(unsynchronized.parsed);
  EXPECT_EQ(unsynchronized.error, "frame_start_missing");
  EXPECT_FALSE(too_short.parsed);
  EXPECT_EQ(too_short.error, "frame_length_invalid");
}

// 即使长度正确，字节22不是0x7D也必须拒绝。
TEST(StatusFrameParser, RejectsAFrameWhoseFixedTailIsInvalid) {
  std::vector<std::uint8_t> frame = {
      0x7b, 0x01, 0x01, 0x02, 0x01, 0x2c, 0xff, 0x9c, 0x32, 0x00, 0x64,
      0x01, 0xbb, 0x00, 0x14, 0x1e, 0x00, 0x5a, 0x04, 0xd2, 0x00, 0x00, 0x00};

  const auto result = StatusFrameParser().parse(frame);

  EXPECT_FALSE(result.parsed);
  EXPECT_EQ(result.error, "frame_end_missing");
}

}  // namespace
