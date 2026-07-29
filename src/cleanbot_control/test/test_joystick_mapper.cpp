#include <gtest/gtest.h>

#include "cleanbot_control/joystick_mapper.hpp"

TEST(JoystickMapper, MapsFullForwardAndRelease) {
  cleanbot::control::JoystickMapper mapper;
  EXPECT_EQ(mapper.map(0.0, 1.0).x_speed, 350);
  EXPECT_TRUE(mapper.map(0.0, 0.0).brake);
}
