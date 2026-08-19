// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include <gtest/gtest.h>

#include "cleanbot_control/joystick_mapper.hpp"

// 测试目的：验证 JoystickMapper.MapsFullForwardAndRelease 场景的行为、状态变化和边界条件。
TEST(JoystickMapper, MapsFullForwardAndRelease) {
  cleanbot::control::JoystickMapper mapper;
  EXPECT_EQ(mapper.map(0.0, 1.0).x_speed, 350);
  EXPECT_TRUE(mapper.map(0.0, 0.0).brake);
}
