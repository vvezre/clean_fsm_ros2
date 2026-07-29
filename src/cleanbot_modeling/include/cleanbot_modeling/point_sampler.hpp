#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "cleanbot_modeling/model_types.hpp"

namespace cleanbot {
namespace modeling {

struct RtkSample {
  bool fixed_valid{false};
  bool center_valid{false};
  double lat{0.0};
  double lon{0.0};
  double heading_deg{0.0};
  bool heading_valid{false};
  double gga_age_sec{0.0};
  std::uint8_t fix_quality{0u};
  bool vehicle_static{false};
};

struct SampleResult {
  bool success{false};
  std::string code;
  std::string message;
  ModelPoint point;
};

SampleResult sample_point(
    const std::vector<RtkSample>& samples,
    std::size_t minimum_samples = 10u,
    double maximum_radius_m = 0.05);

}  // namespace modeling
}  // namespace cleanbot
