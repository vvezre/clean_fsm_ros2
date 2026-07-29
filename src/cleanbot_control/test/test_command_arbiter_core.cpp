#include <gtest/gtest.h>

#include "cleanbot_control/command_arbiter_core.hpp"

TEST(CommandArbiterCore, EmergencyClearsBrushAndRequiresFreshIntent) {
  cleanbot::control::CommandArbiterCore arbiter;
  arbiter.set_brush(true, 60, true);
  cleanbot::control::ControlCommand emergency;
  emergency.active = true;
  emergency.operator_intent = true;
  emergency.command_id = 10u;
  emergency.brake = true;
  arbiter.update(cleanbot::control::CommandSource::kEmergency, emergency, 0u);
  EXPECT_TRUE(arbiter.software_stopped());
  EXPECT_EQ(arbiter.output(0u).brush_speed, 0);
}
