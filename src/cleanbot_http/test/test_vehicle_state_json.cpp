// 文件作用：为对应模块的核心算法、协议处理和边界条件提供单元测试。
#include "cleanbot_http/vehicle_state_json.hpp"

#include <limits>
#include <locale>

#include <gtest/gtest.h>

namespace {

class CommaDecimalPoint : public std::numpunct<char> {
 protected:
// 辅助函数作用：为测试场景提供 do_decimal_point 所需的准备、执行或清理逻辑。
  char do_decimal_point() const override {
    return ',';
  }
};

class ScopedGlobalLocale {
 public:
// 辅助函数作用：为测试场景提供 ScopedGlobalLocale 所需的准备、执行或清理逻辑。
  explicit ScopedGlobalLocale(const std::locale& replacement)
      : previous_(std::locale()) {
    std::locale::global(replacement);
  }

  // 退出测试作用域时恢复进程原有区域设置，防止影响后续用例。
  ~ScopedGlobalLocale() {
    std::locale::global(previous_);
  }

 private:
  std::locale previous_;
};

}  // namespace

// 测试目的：验证 VehicleStateJson.EscapesStringsAndSerializesFiniteState 场景的行为、状态变化和边界条件。
TEST(VehicleStateJson, EscapesStringsAndSerializesFiniteState) {
  cleanbot::http::VehicleStateJsonData state;
  state.stamp_sec = 12;
  state.stamp_nanosec = 34u;
  state.control_state = "idle";
  state.health_state = "ready";
  state.fault_state = "NONE";
  state.current_action = "idle";
  state.message = "quoted \"message\"\nnext";
  state.start_ready = true;
  state.parking = true;
  state.rtk_fixed = true;
  state.battery_percent = 71.5;

  const auto json = cleanbot::http::vehicle_state_json(state);

  EXPECT_NE(json.find("\"success\":true"), std::string::npos);
  EXPECT_NE(json.find("quoted \\\"message\\\"\\nnext"), std::string::npos);
  EXPECT_NE(json.find("\"batteryPercent\":71.5"), std::string::npos);
  EXPECT_NE(json.find("\"startReady\":true"), std::string::npos);
  EXPECT_NE(json.find("\"stamp\":{\"sec\":12,\"nanosec\":34}"),
            std::string::npos);
}

// 测试目的：验证 VehicleStateJson.ReplacesNonFiniteBatteryWithUnknownSentinel 场景的行为、状态变化和边界条件。
TEST(VehicleStateJson, ReplacesNonFiniteBatteryWithUnknownSentinel) {
  cleanbot::http::VehicleStateJsonData state;
  state.battery_percent = std::numeric_limits<double>::infinity();

  const auto json = cleanbot::http::vehicle_state_json(state);

  EXPECT_NE(json.find("\"batteryPercent\":-1.0"), std::string::npos);
  EXPECT_EQ(json.find("inf"), std::string::npos);
}

// 测试目的：验证 VehicleStateJson.AlwaysUsesJsonDecimalPoint 场景的行为、状态变化和边界条件。
TEST(VehicleStateJson, AlwaysUsesJsonDecimalPoint) {
  const ScopedGlobalLocale locale_guard(
      std::locale(std::locale::classic(), new CommaDecimalPoint));
  cleanbot::http::VehicleStateJsonData state;
  state.battery_percent = 71.5;

  const auto json = cleanbot::http::vehicle_state_json(state);

  EXPECT_NE(json.find("\"batteryPercent\":71.5"), std::string::npos);
  EXPECT_EQ(json.find("\"batteryPercent\":71,5"), std::string::npos);
}

// 测试目的：验证 VehicleStateJson.BuildsEscapedBusinessResponse 场景的行为、状态变化和边界条件。
TEST(VehicleStateJson, BuildsEscapedBusinessResponse) {
  const auto json = cleanbot::http::business_response_json(
      false, "NO_ACTIVE_MISSION", "no \"active\" mission");

  EXPECT_NE(json.find("\"success\":false"), std::string::npos);
  EXPECT_NE(json.find("\"code\":\"NO_ACTIVE_MISSION\""),
            std::string::npos);
  EXPECT_NE(json.find("no \\\"active\\\" mission"), std::string::npos);
}
