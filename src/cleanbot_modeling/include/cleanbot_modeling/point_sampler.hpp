#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "cleanbot_modeling/model_types.hpp"

// 文件作用：声明基于多次 RTK 样本校验和均值计算的模型点采集接口。
namespace cleanbot {
namespace modeling {

// 单次 RTK 采样的固定解、坐标、航向和车辆静止状态。
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

// 模型点采样的结果、失败原因和计算出的点位。
struct SampleResult {
  bool success{false};
  std::string code;
  std::string message;
  ModelPoint point;
};

// 校验样本数量、质量和离散半径后生成一个模型点。
SampleResult sample_point(
    const std::vector<RtkSample>& samples,
    std::size_t minimum_samples = 10u,
    double maximum_radius_m = 0.05);

}  // namespace modeling
}  // namespace cleanbot
