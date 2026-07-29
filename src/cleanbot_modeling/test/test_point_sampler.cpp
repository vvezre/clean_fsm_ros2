#include <gtest/gtest.h>

#include <vector>

#include "cleanbot_modeling/point_sampler.hpp"

namespace {

std::vector<cleanbot::modeling::RtkSample> stable_samples() {
  std::vector<cleanbot::modeling::RtkSample> samples(10u);
  for (std::size_t index = 0u; index < samples.size(); ++index) {
    auto& sample = samples[index];
    sample.fixed_valid = true;
    sample.center_valid = true;
    sample.fix_quality = 4u;
    sample.gga_age_sec = 0.2;
    sample.vehicle_static = true;
    sample.lat = 31.2 + static_cast<double>(index) * 1e-9;
    sample.lon = 121.5 - static_cast<double>(index) * 1e-9;
    sample.heading_valid = true;
    sample.heading_deg = index % 2u == 0u ? 359.0 : 1.0;
  }
  return samples;
}

TEST(PointSampler, ProducesMeanForStableFixedSamples) {
  const auto result =
      cleanbot::modeling::sample_point(stable_samples(), 10u, 0.05);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.code, "OK");
  EXPECT_EQ(result.point.sample_count, 10u);
  EXPECT_LT(result.point.sample_radius_m, 0.05);
}

TEST(PointSampler, RejectsUnstableSamples) {
  auto samples = stable_samples();
  samples.back().lat += 1e-6;

  const auto result =
      cleanbot::modeling::sample_point(samples, 10u, 0.05);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "RTK_SAMPLE_UNSTABLE");
}

}  // namespace
