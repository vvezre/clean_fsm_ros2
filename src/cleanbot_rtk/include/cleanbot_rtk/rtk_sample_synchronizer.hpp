#ifndef CLEANBOT_RTK__RTK_SAMPLE_SYNCHRONIZER_HPP_
#define CLEANBOT_RTK__RTK_SAMPLE_SYNCHRONIZER_HPP_

#include "cleanbot_rtk/nmea_parser.hpp"

// 文件作用：声明 GGA 定位与航向消息的时间同步、过期判定和样本合成逻辑。
namespace cleanbot {
namespace rtk {

// 已按时间配对的定位与航向样本。
struct RtkSample {
  GgaData gga;
  double heading_deg{0.0};
  double heading_age_sec{0.0};
  double gga_age_sec{0.0};
  bool heading_valid{false};
};

// GGA 到达后尝试同步的产出状态。
struct SynchronizeResult {
  bool produced{false};
  bool stale{false};
  RtkSample sample;
};

class RtkSampleSynchronizer {
 public:
  // 设置定位与航向的允许时间差及各自最大有效年龄。
  RtkSampleSynchronizer(
      double sync_threshold_sec = 0.5,
      double nmea_max_age_sec = 2.0,
      double heading_max_age_sec = 0.5);

  // 记录最新航向句及其本地接收时间。
  void observe_heading(const HeadingData& heading, double receive_time_sec);
  // 以新的 GGA 句为主样本，尝试配对最新航向并返回结果。
  SynchronizeResult observe_gga(
      const GgaData& gga,
      double receive_time_sec) const;
  // 清除缓存的航向样本。
  void reset();

 private:
  // 计算跨越 UTC 午夜时仍正确的时间差。
  static double utcDifference(double left, double right);
  double sync_threshold_sec_;
  double nmea_max_age_sec_;
  double heading_max_age_sec_;
  HeadingData latest_heading_;
  double latest_heading_received_at_{0.0};
  bool has_heading_{false};
};

}  // namespace rtk
}  // namespace cleanbot

#endif  // CLEANBOT_RTK__RTK_SAMPLE_SYNCHRONIZER_HPP_
