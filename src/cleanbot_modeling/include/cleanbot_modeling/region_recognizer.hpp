#pragma once

#include <string>

#include "cleanbot_modeling/model_types.hpp"

// 文件作用：声明模型区域的重复点清理、子区域识别和连接段辅助识别接口。
namespace cleanbot {
namespace modeling {

// 区域识别使用的几何容差、置信度和连接段长度阈值。
struct RecognitionOptions {
  double duplicate_tolerance_cm{5.0};
  double minimum_area_cm2{10000.0};
  double assist_turn_threshold_deg{20.0};
  double minimum_connector_length_cm{50.0};
  double maximum_connector_endpoint_distance_cm{100.0};
  double unordered_auto_confirm_confidence{0.85};
};

// 区域识别的成败、确认需求及已处理区域组。
struct RecognitionResult {
  bool success{false};
  std::string code;
  std::string message;
  std::string status;
  bool needs_confirmation{true};
  ModelGroup group;
};

// 识别一个区域组中的子区域和连接段，并给出确认建议。
RecognitionResult recognize_group(
    const ModelGroup& group,
    const RecognitionOptions& options = RecognitionOptions{});

}  // namespace modeling
}  // namespace cleanbot
