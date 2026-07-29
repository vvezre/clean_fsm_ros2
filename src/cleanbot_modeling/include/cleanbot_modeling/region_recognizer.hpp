#pragma once

#include <string>

#include "cleanbot_modeling/model_types.hpp"

namespace cleanbot {
namespace modeling {

struct RecognitionOptions {
  double duplicate_tolerance_cm{5.0};
  double minimum_area_cm2{10000.0};
  double assist_turn_threshold_deg{20.0};
  double minimum_connector_length_cm{50.0};
  double maximum_connector_endpoint_distance_cm{100.0};
  double unordered_auto_confirm_confidence{0.85};
};

struct RecognitionResult {
  bool success{false};
  std::string code;
  std::string message;
  std::string status;
  bool needs_confirmation{true};
  ModelGroup group;
};

RecognitionResult recognize_group(
    const ModelGroup& group,
    const RecognitionOptions& options = RecognitionOptions{});

}  // namespace modeling
}  // namespace cleanbot
