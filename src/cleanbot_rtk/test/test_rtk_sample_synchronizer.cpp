#include <gtest/gtest.h>

#include "cleanbot_rtk/rtk_sample_synchronizer.hpp"

namespace {

using cleanbot::rtk::GgaData;
using cleanbot::rtk::HeadingData;
using cleanbot::rtk::RtkSampleSynchronizer;

TEST(RtkSampleSynchronizer, PairsFreshUtcSamples) {
  RtkSampleSynchronizer synchronizer(0.5, 2.0, 0.5);
  HeadingData heading;
  heading.utc_seconds = 100.0;
  heading.heading_deg = 90.0;
  GgaData gga;
  gga.utc_seconds = 100.2;
  gga.position_valid = true;

  synchronizer.observe_heading(heading, 10.0);
  const auto result = synchronizer.observe_gga(gga, 10.1);

  ASSERT_TRUE(result.produced);
  EXPECT_TRUE(result.sample.heading_valid);
  EXPECT_DOUBLE_EQ(result.sample.heading_deg, 90.0);
}

TEST(RtkSampleSynchronizer, ExpiresHeading) {
  RtkSampleSynchronizer synchronizer(0.5, 2.0, 0.5);
  HeadingData heading;
  heading.utc_seconds = 100.0;
  heading.heading_deg = 90.0;
  GgaData gga;
  gga.utc_seconds = 100.2;
  gga.position_valid = true;

  synchronizer.observe_heading(heading, 10.0);
  const auto result = synchronizer.observe_gga(gga, 10.6);

  ASSERT_TRUE(result.produced);
  EXPECT_FALSE(result.sample.heading_valid);
}

}  // namespace
