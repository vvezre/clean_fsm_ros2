#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "cleanbot_gateway/cloud_message_codec.hpp"

namespace {

cleanbot::gateway::CloudIdentity identity() {
  return {"ZTZN-PVC", "-T01", "250001"};
}

TEST(CloudMessageCodec, DecodesExistingCloudEnvelope) {
  cleanbot::gateway::CloudMessageCodec codec(identity());
  const std::string payload = R"({
    "company_code":"ZTZN-PVC",
    "product_model":"-T01",
    "product_id":"250001",
    "timestamp":100,
    "data":{
      "command_id":"cmd_1",
      "trace_id":"trace_1",
      "command":"joystick_move",
      "params":{"distance":50,"dirX":0.25,"dirY":0.75}
    }
  })";

  const auto result = codec.decode(payload, false);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.command.command_id, "cmd_1");
  EXPECT_EQ(result.command.trace_id, "trace_1");
  EXPECT_EQ(result.command.command, "joystick_move");
  EXPECT_DOUBLE_EQ(result.command.distance, 50.0);
  EXPECT_DOUBLE_EQ(result.command.dir_x, 0.25);
  EXPECT_DOUBLE_EQ(result.command.dir_y, 0.75);
}

TEST(CloudMessageCodec, RejectsMessageForAnotherRobot) {
  cleanbot::gateway::CloudMessageCodec codec(identity());
  const auto result = codec.decode(R"({
    "company_code":"ZTZN-PVC",
    "product_model":"-T01",
    "product_id":"999999",
    "timestamp":100,
    "data":{"command_id":"cmd_1","command":"parking","params":{}}
  })", false);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "DEVICE_IDENTITY_MISMATCH");
}

TEST(CloudMessageCodec, EncodesAckAndCommandResultForCloudTracking) {
  cleanbot::gateway::CloudMessageCodec codec(identity());
  cleanbot::gateway::CloudCommandInput command;
  command.command_id = "cmd_1";
  command.trace_id = "trace_1";
  command.command = "parking";

  const auto ack = nlohmann::json::parse(codec.encode_ack(command, "accepted", 100));
  EXPECT_EQ(ack["data"]["type"], "ack");
  EXPECT_EQ(ack["data"]["command_id"], "cmd_1");
  EXPECT_EQ(ack["data"]["status"], "accepted");

  const auto result = nlohmann::json::parse(
      codec.encode_result(command, true, "OK", "stopped", 101));
  EXPECT_EQ(result["data"]["type"], "command_result");
  EXPECT_TRUE(result["data"]["result"]["success"]);
  EXPECT_EQ(result["data"]["result"]["code"], "OK");
}

}  // namespace
